//! `stepper` (stepper.c), `endstop` (endstop.c) and `trsync` (trsync.c).
//! Timers are polled from `Mcu::task` like io.rs.
//! ponytail: polled timers; step jitter bounded by main-loop latency.
//! Move to a TIMG alarm ISR if Phase 3 jitter measurement demands it.

use crate::dict_defs::CLOCK_FREQ;
use crate::io::{Res, Timer, before};
use crate::mcu::Board;

/// Moves shared by all steppers (Klipper move_alloc pool), reported as move_count.
pub const MOVE_COUNT: usize = 1024;
const NIL: u16 = u16::MAX;

#[derive(Clone, Copy)]
struct Move {
    interval: u32,
    count: u16,
    add: i16,
    dir: bool,
    next: u16,
}

/// Fixed pool of moves; free list and per-stepper queues are index chains.
pub struct Pool {
    m: [Move; MOVE_COUNT],
    free: u16,
}

impl Pool {
    pub const fn new() -> Self {
        let mut m = [Move { interval: 0, count: 0, add: 0, dir: false, next: NIL }; MOVE_COUNT];
        let mut i = 0;
        while i + 1 < MOVE_COUNT {
            m[i].next = i as u16 + 1;
            i += 1;
        }
        Self { m, free: 0 }
    }
    fn alloc(&mut self, v: Move) -> Res<u16> {
        let i = self.free;
        if i == NIL {
            return Err("Move queue overflow");
        }
        self.free = self.m[i as usize].next;
        self.m[i as usize] = Move { next: NIL, ..v };
        Ok(i)
    }
    fn release(&mut self, i: u16) {
        self.m[i as usize].next = self.free;
        self.free = i;
    }
}

impl Default for Pool {
    fn default() -> Self {
        Self::new()
    }
}

const POSITION_BIAS: u32 = 0x4000_0000;
const LAST_DIR: u8 = 1 << 0;
const NEXT_DIR: u8 = 1 << 1;
const INVERT_STEP: u8 = 1 << 2;
const NEED_RESET: u8 = 1 << 3;
const SINGLE_SCHED: u8 = 1 << 4;

pub struct Stepper {
    pub t: Timer,
    interval: u32,
    add: i16,
    count: u32,
    next_step_time: u32,
    pulse: u32,
    step_pin: u32,
    dir_pin: u32,
    step_lvl: bool,
    dir_lvl: bool,
    position: u32,
    head: u16,
    tail: u16,
    flags: u8,
    /// trsync oid this stepper stops on (Klipper trsync_signal).
    pub stop_on: Option<u8>,
}

impl Stepper {
    pub fn new(b: &mut impl Board, step: u32, dir: u32, invert: i8, pulse: u32) -> Res<Self> {
        let flags = match invert {
            1.. => INVERT_STEP,
            ..0 => SINGLE_SCHED,
            0 => 0,
        };
        let step_lvl = flags & INVERT_STEP != 0;
        b.set_out(step, step_lvl)?;
        b.set_out(dir, false)?;
        Ok(Self {
            t: Timer::default(),
            interval: 0,
            add: 0,
            count: 0,
            next_step_time: 0,
            pulse,
            step_pin: step,
            dir_pin: dir,
            step_lvl,
            dir_lvl: false,
            position: POSITION_BIAS.wrapping_neg(),
            head: NIL,
            tail: NIL,
            flags,
            stop_on: None,
        })
    }

    fn single(&self) -> bool {
        self.flags & SINGLE_SCHED != 0
    }

    fn toggle_dir(&mut self, b: &mut impl Board) -> Res {
        self.dir_lvl = !self.dir_lvl;
        b.set_out(self.dir_pin, self.dir_lvl)
    }

    fn load_next(&mut self, b: &mut impl Board, pool: &mut Pool) -> Res {
        if self.head == NIL {
            self.count = 0;
            self.t.active = false;
            return Ok(());
        }
        let i = self.head;
        let m = pool.m[i as usize];
        self.head = m.next;
        if self.head == NIL {
            self.tail = NIL;
        }
        pool.release(i);

        let pos = if m.dir { self.position.wrapping_neg() } else { self.position };
        self.position = pos.wrapping_add(m.count as u32);
        self.add = m.add;
        self.interval = m.interval.wrapping_add(m.add as i32 as u32);
        let was_active = self.count != 0;
        let mut min_next = self.t.wake;
        self.next_step_time = self.next_step_time.wrapping_add(m.interval);
        self.t.wake = self.next_step_time;
        self.count = m.count as u32;
        if was_active && before(self.next_step_time, min_next) {
            let diff = self.next_step_time.wrapping_sub(min_next) as i32;
            if diff < -((CLOCK_FREQ / 1000) as i32) {
                return Err("Stepper too far in past");
            }
            self.t.wake = min_next;
        }
        if was_active && m.dir {
            if self.single() {
                while before(b.now(), min_next) {}
            }
            self.toggle_dir(b)?;
            min_next = b.now().wrapping_add(self.pulse);
            if before(self.t.wake, min_next) {
                self.t.wake = min_next;
            }
            return Ok(());
        }
        if m.dir {
            self.toggle_dir(b)?;
        }
        Ok(())
    }

    /// One step. Unlike Klipper's stepper_event_full, the unstep edge is done
    /// inline after `pulse` ticks: a timer event per edge doubled ISR load,
    /// and one 595 shift (~3 µs) already exceeds the TMC2209's 100 ns minimum.
    pub fn event(&mut self, b: &mut impl Board, pool: &mut Pool) -> Res {
        self.step_lvl = !self.step_lvl;
        b.set_out(self.step_pin, self.step_lvl)?;
        let t0 = b.now();
        if !self.single() {
            while b.now().wrapping_sub(t0) < self.pulse {}
            self.step_lvl = !self.step_lvl;
            b.set_out(self.step_pin, self.step_lvl)?;
        }
        let min_next = t0.wrapping_add(self.pulse);
        let count = self.count - 1;
        if count != 0 {
            self.next_step_time = self.next_step_time.wrapping_add(self.interval);
            self.interval = self.interval.wrapping_add(self.add as i32 as u32);
            self.count = count;
            self.t.wake = if before(self.next_step_time, min_next) { min_next } else { self.next_step_time };
            return Ok(());
        }
        self.t.wake = min_next;
        self.load_next(b, pool)
    }

    pub fn queue_step(&mut self, b: &mut impl Board, pool: &mut Pool, a: &[u32]) -> Res {
        let count = a[1] as u16;
        if count == 0 {
            return Err("Invalid count parameter");
        }
        let mut flags = self.flags;
        let dir = (flags & LAST_DIR != 0) != (flags & NEXT_DIR != 0);
        if dir {
            flags ^= LAST_DIR;
        }
        if self.count == 0 && flags & NEED_RESET != 0 {
            return Ok(());
        }
        let i = pool.alloc(Move { interval: a[0], count, add: a[2] as i16, dir, next: NIL })?;
        self.flags = flags;
        if self.tail == NIL {
            self.head = i;
        } else {
            pool.m[self.tail as usize].next = i;
        }
        self.tail = i;
        if self.count == 0 {
            self.load_next(b, pool)?;
            self.t.active = true;
        }
        Ok(())
    }

    pub fn set_next_dir(&mut self, dir: bool) {
        self.flags = (self.flags & !NEXT_DIR) | if dir { NEXT_DIR } else { 0 };
    }

    pub fn reset_clock(&mut self, clock: u32) -> Res {
        if self.count != 0 {
            return Err("Can't reset time when stepper active");
        }
        self.next_step_time = clock;
        self.t.wake = clock;
        self.flags &= !NEED_RESET;
        Ok(())
    }

    pub fn position(&self) -> i32 {
        let p = self.position.wrapping_sub(self.count);
        let p = if p & 0x8000_0000 != 0 { p.wrapping_neg() } else { p };
        p.wrapping_sub(POSITION_BIAS) as i32
    }

    /// stepper_stop: halt, drop queue, require reset_step_clock.
    pub fn stop(&mut self, b: &mut impl Board, pool: &mut Pool) {
        self.t = Timer::default();
        self.next_step_time = 0;
        self.position = (self.position() as u32).wrapping_add(POSITION_BIAS).wrapping_neg();
        self.count = 0;
        self.flags = (self.flags & (INVERT_STEP | SINGLE_SCHED)) | NEED_RESET;
        self.dir_lvl = false;
        let _ = b.set_out(self.dir_pin, false);
        if !self.single() {
            self.step_lvl = self.flags & INVERT_STEP != 0;
            let _ = b.set_out(self.step_pin, self.step_lvl);
        }
        while self.head != NIL {
            let i = self.head;
            self.head = pool.m[i as usize].next;
            pool.release(i);
        }
        self.tail = NIL;
    }
}

const PIN_HIGH: u8 = 1 << 0;
const HOMING: u8 = 1 << 1;

#[derive(Default)]
pub struct Endstop {
    pub t: Timer,
    pub pin: u32,
    rest: u32,
    sample_time: u32,
    pub nextwake: u32,
    ts: u8,
    pub flags: u8,
    sample_count: u8,
    trigger_count: u8,
    reason: u8,
    oversample: bool,
}

impl Endstop {
    pub fn new(b: &mut impl Board, pin: u32, pull_up: bool) -> Res<Self> {
        b.in_setup(pin, pull_up)?;
        Ok(Self { pin, ..Default::default() })
    }

    pub fn homing(&self) -> bool {
        self.flags & HOMING != 0
    }

    /// `endstop_home` args after oid; caller validated trsync oid a[5].
    pub fn home(&mut self, a: &[u32]) {
        self.t = Timer { active: false, wake: a[0] };
        self.sample_time = a[1];
        self.sample_count = a[2] as u8;
        if self.sample_count == 0 {
            self.flags = 0;
            return;
        }
        self.rest = a[3];
        self.oversample = false;
        self.trigger_count = self.sample_count;
        self.flags = HOMING | if a[4] != 0 { PIN_HIGH } else { 0 };
        self.ts = a[5] as u8;
        self.reason = a[6] as u8;
        self.t.active = true;
    }

    /// Returns `Some((trsync_oid, reason))` when the trigger fires.
    pub fn event(&mut self, b: &mut impl Board) -> Option<(u8, u8)> {
        let hit = b.get_in(self.pin) == (self.flags & PIN_HIGH != 0);
        if !self.oversample {
            let next = self.t.wake.wrapping_add(self.rest);
            if !hit {
                self.t.wake = next;
                return None;
            }
            self.nextwake = next;
            self.oversample = true;
        } else if !hit {
            self.oversample = false;
            self.t.wake = self.nextwake;
            self.trigger_count = self.sample_count;
            return None;
        }
        self.trigger_count -= 1;
        if self.trigger_count == 0 {
            self.t.active = false;
            return Some((self.ts, self.reason));
        }
        self.t.wake = self.t.wake.wrapping_add(self.sample_time);
        None
    }
}

pub const CAN_TRIGGER: u8 = 1 << 0;
pub const REPORT: u8 = 1 << 2;

#[derive(Default)]
pub struct Trsync {
    pub report: Timer,
    pub expire: Timer,
    report_ticks: u32,
    pub flags: u8,
    pub trigger_reason: u8,
    pub expire_reason: u8,
}

impl Trsync {
    /// Timers and flags only; caller clears steppers' `stop_on`.
    pub fn clear(&mut self) {
        *self = Self::default();
    }

    pub fn start(&mut self, a: &[u32]) {
        self.clear();
        self.flags = CAN_TRIGGER;
        self.report_ticks = a[1];
        self.report = Timer { active: a[1] != 0, wake: a[0] };
        self.expire_reason = a[2] as u8;
    }

    pub fn set_timeout(&mut self, clock: u32) {
        if self.flags & CAN_TRIGGER != 0 {
            self.expire = Timer { active: true, wake: clock };
        }
    }

    pub fn report_event(&mut self) {
        self.flags |= REPORT;
        self.report.wake = self.report.wake.wrapping_add(self.report_ticks);
    }
}
