//! Free-running 16-bit TimerA1 with compare alarms and a 64-bit Embassy clock.
//!
//! Counting never stops/reloads when an alarm changes. Overflow bookkeeping and
//! compare channel 1 share an IRQ handler. Logical TICK_HZ is independent of the
//! PCLK1/prescaler rate: fractional conversion retains its remainder, so it does
//! not accumulate rounding drift. No SysTick, DWT or periodic tick IRQ is used.
//! The driver must be serviced before a whole hardware counter wrap is missed;
//! at PCLK1=96 MHz and /256 this is 174.763 ms. Stop-mode timekeeping is not
//! provided; keep PCLK1 running and unchanged while this driver is in use.

use crate::{
    intc::{self, Line, Route},
    pac,
};
use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, Ordering},
};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time_driver::{Driver, TICK_HZ};
use embassy_time_queue_utils::Queue;

const BASE: usize = 0x4001_5000; // TimerA1, vendor SVD.
const OVF: u8 = 1 << 6;
const OVF_IE: u8 = 1 << 4;
static INITIALIZED: AtomicBool = AtomicBool::new(false);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitError {
    Clock,
    AlreadyInitialized,
    Routing(intc::IrqError),
}

/// MMIO is isolated so wrap races and late compare writes can be tested without
/// approximating them using the host clock.
trait Registers {
    fn count(&mut self) -> u16;
    fn overflow(&mut self) -> bool;
    fn clear_overflow(&mut self);
    fn disarm(&mut self);
    fn arm(&mut self, value: u16);
}
struct Mmio;
fn r16(offset: usize) -> u16 {
    unsafe { ((BASE + offset) as *const u16).read_volatile() }
}
fn w16(offset: usize, value: u16) {
    unsafe { ((BASE + offset) as *mut u16).write_volatile(value) }
}
fn r8(offset: usize) -> u8 {
    unsafe { ((BASE + offset) as *const u8).read_volatile() }
}
fn w8(offset: usize, value: u8) {
    unsafe { ((BASE + offset) as *mut u8).write_volatile(value) }
}
impl Registers for Mmio {
    fn count(&mut self) -> u16 {
        r16(0)
    }
    fn overflow(&mut self) -> bool {
        r8(0x81) & OVF != 0
    }
    fn clear_overflow(&mut self) {
        w8(0x81, OVF_IE);
    }
    fn disarm(&mut self) {
        w16(0x90, 0); // ICONR: disable comparison IRQs.
        w16(0x9c, 0); // STFLR: W0C, all channels exclusively owned here.
    }
    fn arm(&mut self, value: u16) {
        w16(0x40, value); // CMPAR1, unbuffered compare mode.
        w16(0x90, 1); // ICONR.ITEN1.
    }
}

#[derive(Default)]
struct Clock {
    hz: u32,
    epoch: u64,
    raw: u64,
    ticks: u64,
    fraction: u64,
    wraps: u32,
}
impl Clock {
    fn sample(&mut self, io: &mut impl Registers) -> u64 {
        // Read count before flag: if wrap races the read, account it and retry.
        // Clearing the flag here also makes now() safe before the pending IRQ
        // runs; that IRQ sees an already-cleared flag and cannot double-count.
        let raw = loop {
            let low = io.count();
            if io.overflow() {
                io.clear_overflow();
                self.epoch += 1 << 16;
                self.wraps = self.wraps.wrapping_add(1);
            } else {
                break self.epoch + u64::from(low);
            }
        };
        if self.hz != 0 {
            let scaled = (raw - self.raw) * TICK_HZ + self.fraction;
            self.ticks += scaled / u64::from(self.hz);
            self.fraction = scaled % u64::from(self.hz);
            self.raw = raw;
        }
        self.ticks
    }
    fn delta(&self, at: u64) -> Option<u16> {
        let ticks = at.checked_sub(self.ticks)?;
        // Long deadlines wait for an overflow IRQ before programming compare.
        // This guard also keeps the multiplication below the u64 limit.
        let available = (u64::from(u16::MAX) * TICK_HZ + self.fraction) / u64::from(self.hz);
        if ticks == 0 || ticks > available {
            return None;
        }
        let counts = (ticks * u64::from(self.hz) - self.fraction).div_ceil(TICK_HZ);
        u16::try_from(counts).ok()
    }
}

struct State {
    clock: Clock,
    queue: Queue,
    irqs: u32,
    _timer: Option<pac::Tmra1>,
    _routes: Option<[Route; 2]>,
}
impl State {
    const fn new() -> Self {
        Self {
            clock: Clock {
                hz: 0,
                epoch: 0,
                raw: 0,
                ticks: 0,
                fraction: 0,
                wraps: 0,
            },
            queue: Queue::new(),
            irqs: 0,
            _timer: None,
            _routes: None,
        }
    }
    fn rearm(&mut self, io: &mut impl Registers) {
        loop {
            let now = self.clock.sample(io);
            let at = self.queue.next_expiration(now);
            io.disarm();
            if at == u64::MAX {
                return;
            }
            let Some(delta) = self.clock.delta(at) else {
                return;
            };
            io.arm((self.clock.raw as u16).wrapping_add(delta));
            // A near deadline can pass while CMPAR/ICONR are being written.
            // Expire/rearm now, rather than waiting for another full wrap.
            if self.clock.sample(io) < at {
                return;
            }
        }
    }
}
pub struct TimerATimeDriver {
    state: Mutex<CriticalSectionRawMutex, RefCell<State>>,
}
impl Default for TimerATimeDriver {
    fn default() -> Self {
        Self::new()
    }
}
impl TimerATimeDriver {
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(RefCell::new(State::new())),
        }
    }
}
impl Driver for TimerATimeDriver {
    fn now(&self) -> u64 {
        self.state.lock(|s| s.borrow_mut().clock.sample(&mut Mmio))
    }
    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        self.state.lock(|s| {
            let mut state = s.borrow_mut();
            if state.queue.schedule_wake(at, waker) {
                state.rearm(&mut Mmio);
            }
        });
    }
}
embassy_time_driver::time_driver_impl!(static DRIVER:TimerATimeDriver=TimerATimeDriver::new());

fn configuration(pclk1: u32) -> Result<(u32, u8), InitError> {
    // Use the lowest rate up to /256 that still resolves logical ticks.
    // /256 is also used by the separately qualified TimerA PWM driver.
    for div in (0..=8).rev() {
        let divisor = 1 << div;
        if pclk1.is_multiple_of(divisor)
            && u64::from(pclk1 / divisor) >= TICK_HZ
            && pclk1 / divisor > 0
        {
            return Ok((pclk1 / divisor, (div as u8) << 4 | 3));
        }
    }
    Err(InitError::Clock)
}

/// Claim all of TimerA1 and two routable interrupt lines, forever. Install both
/// handlers to call [`on_interrupt`]. No pin mux/output is enabled. Set PCLK1
/// to its final value first. Use default-features=false to exclude the legacy
/// SysTick driver and select an embassy-time/tick-hz-* in the application.
///
/// # Safety
/// Caller owns TimerA1, its clock gate and all trigger/channel configuration,
/// and exclusively owns the supplied interrupt lines with matching handlers.
/// Clock/power register write access must already be unlocked by board startup.
pub unsafe fn init(
    timer: pac::Tmra1,
    pclk1: u32,
    overflow: Line,
    compare: Line,
    priority: u8,
) -> Result<(), InitError> {
    let (hz, run) = configuration(pclk1)?;
    if overflow == compare {
        return Err(InitError::Routing(intc::IrqError::LineTaken));
    }
    INITIALIZED
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .map_err(|_| InitError::AlreadyInitialized)?;
    let result = critical_section::with(|_| {
        // SAFETY: initialization owns the complete timer and unlocked clock gate.
        unsafe {
            let gate = 0x40048008 as *mut u32;
            gate.write_volatile(gate.read_volatile() & !(1 << 2));
        }
        w8(0x80, 2); // STOP and CKDIV=0 before CNTER, per errata 3.3.5.
        w8(0x81, 0);
        w16(0, 0);
        w16(4, u16::MAX);
        for offset in [0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c] {
            w16(offset, 0);
        }
        for channel in 0..8 {
            w16(0x100 + channel * 4, 0); // CCONR: compare, no capture trigger.
            w16(0x140 + channel * 4, 0); // PCONR: no outputs.
        }
        for channel in 0..4 {
            w16(0xc0 + channel * 8, 0);
        } // BCONR: no buffering.
        // SAFETY: caller transfers the vectors and has installed their handlers.
        let first = unsafe { Route::new(256, overflow, priority) }.map_err(InitError::Routing)?;
        let second = unsafe { Route::new(258, compare, priority) }.map_err(InitError::Routing)?;
        DRIVER.state.lock(|s| {
            let mut state = s.borrow_mut();
            state.clock.hz = hz;
            state._timer = Some(timer);
            state._routes = Some([first, second]);
        });
        w8(0x81, OVF_IE);
        w8(0x80, run);
        Ok(())
    });
    if result.is_err() {
        INITIALIZED.store(false, Ordering::Release);
    }
    result
}

/// Dispatch both the TimerA1 overflow and comparison vectors here.
pub fn on_interrupt() {
    DRIVER.state.lock(|s| {
        let mut state = s.borrow_mut();
        state.irqs = state.irqs.wrapping_add(1);
        state.rearm(&mut Mmio);
    });
}
/// Diagnostic snapshot: logical ticks, hardware counter rate, IRQ and wraps.
pub fn diagnostics() -> (u64, u32, u32, u32) {
    DRIVER.state.lock(|s| {
        let mut state = s.borrow_mut();
        let now = state.clock.sample(&mut Mmio);
        (now, state.clock.hz, state.irqs, state.clock.wraps)
    })
}

#[cfg(test)]
#[path = "time_driver_tmra/tests.rs"]
mod tests;
