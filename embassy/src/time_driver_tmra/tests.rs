extern crate std;
use super::*;
use std::{
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    },
    task::{Wake, Waker},
};

#[derive(Default)]
struct Model {
    raw: u64,
    flag: bool,
    armed: Option<u16>,
    late: u16,
    wrap_on_flag: bool,
}
impl Model {
    fn advance(&mut self, n: u64) {
        let old = self.raw;
        self.raw += n;
        if old >> 16 != self.raw >> 16 {
            self.flag = true;
        }
    }
}
impl Registers for Model {
    fn count(&mut self) -> u16 {
        self.raw as u16
    }
    fn overflow(&mut self) -> bool {
        if self.wrap_on_flag {
            self.wrap_on_flag = false;
            self.advance(1);
        }
        self.flag
    }
    fn clear_overflow(&mut self) {
        self.flag = false;
    }
    fn disarm(&mut self) {
        self.armed = None;
    }
    fn arm(&mut self, n: u16) {
        self.armed = Some(n);
        let late = self.late;
        self.late = 0;
        self.advance(u64::from(late));
    }
}
#[derive(Default)]
struct Counter(AtomicUsize);
impl Wake for Counter {
    fn wake(self: Arc<Self>) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
}
fn state() -> State {
    let mut s = State::new();
    s.clock.hz = TICK_HZ as u32;
    s
}

#[test]
fn prescale_is_checked_and_does_not_require_an_exact_logical_tick_divisor() {
    assert_eq!(configuration(96_000_000), Ok((375_000, 0x83)));
    assert_eq!(configuration(0), Err(InitError::Clock));
    assert_eq!(configuration(TICK_HZ as u32 - 1), Err(InitError::Clock));
}
#[test]
fn every_fraction_is_preserved_over_many_reads_and_wraps() {
    let mut c = Clock {
        hz: 375000,
        ..Clock::default()
    };
    let mut io = Model::default();
    for i in 0..50000 {
        io.advance([1, 2, 13, 127][i % 4]);
        assert_eq!(c.sample(&mut io), io.raw * TICK_HZ / 375000);
    }
    assert!(c.wraps > 10);
    assert_eq!(c.fraction, io.raw * TICK_HZ % 375000);
}
#[test]
fn pending_overflow_is_consumed_once_even_before_its_irq() {
    let mut c = Clock {
        hz: TICK_HZ as u32,
        ..Clock::default()
    };
    let mut io = Model::default();
    io.advance(65535);
    assert_eq!(c.sample(&mut io), 65535);
    io.wrap_on_flag = true;
    assert_eq!(c.sample(&mut io), 65536);
    assert_eq!(c.sample(&mut io), 65536);
    assert_eq!(c.wraps, 1);
    io.advance(7);
    assert_eq!(c.sample(&mut io), 65543);
}
#[test]
fn independent_deadlines_and_earlier_rescheduling_wake_once() {
    let mut s = state();
    let mut io = Model::default();
    let first = Arc::new(Counter::default());
    let second = Arc::new(Counter::default());
    s.queue.schedule_wake(10, &Waker::from(first.clone()));
    s.queue.schedule_wake(20, &Waker::from(second.clone()));
    s.rearm(&mut io);
    assert_eq!(io.armed, Some(10));
    s.queue.schedule_wake(5, &Waker::from(first.clone()));
    s.rearm(&mut io);
    assert_eq!(io.armed, Some(5));
    io.advance(5);
    s.rearm(&mut io);
    assert_eq!(first.0.load(Ordering::Relaxed), 1);
    assert_eq!(second.0.load(Ordering::Relaxed), 0);
    assert_eq!(io.armed, Some(20));
    io.advance(15);
    s.rearm(&mut io);
    assert_eq!(second.0.load(Ordering::Relaxed), 1);
    assert_eq!(io.armed, None);
}
#[test]
fn programming_race_expires_now_instead_of_waiting_a_wrap() {
    let mut s = state();
    let mut io = Model {
        late: 4,
        ..Model::default()
    };
    let w = Arc::new(Counter::default());
    s.queue.schedule_wake(2, &Waker::from(w.clone()));
    s.rearm(&mut io);
    assert_eq!(w.0.load(Ordering::Relaxed), 1);
    assert_eq!(io.armed, None);
}
#[test]
fn long_deadline_is_rearmed_from_overflow_and_compares_across_wrap() {
    let mut s = state();
    let mut io = Model::default();
    let w = Arc::new(Counter::default());
    s.queue.schedule_wake(70000, &Waker::from(w.clone()));
    s.rearm(&mut io);
    assert_eq!(io.armed, None);
    io.advance(65000);
    s.rearm(&mut io);
    assert_eq!(io.armed, Some(4464));
    io.advance(536);
    s.rearm(&mut io);
    assert_eq!(io.armed, Some(4464));
    io.advance(4464);
    s.rearm(&mut io);
    assert_eq!(w.0.load(Ordering::Relaxed), 1);
}
#[test]
fn compare_conversion_is_ceil_and_never_early() {
    let mut c = Clock {
        hz: 375000,
        ..Clock::default()
    };
    let mut io = Model::default();
    for _ in 0..100 {
        io.advance(73);
        c.sample(&mut io);
        let at = c.ticks + 33;
        let delta = c.delta(at).unwrap();
        assert!((io.raw + u64::from(delta)) * TICK_HZ / 375000 >= at);
        assert!((io.raw + u64::from(delta) - 1) * TICK_HZ / 375000 < at);
    }
}
