//! `digital_out` (gpiocmds.c) and `analog_in` (adccmds.c) objects.
//! Timers are polled from the main loop (`Mcu::task`); fine for heater/fan
//! soft-PWM and ADC sampling. ponytail: move to ISR scheduler with steppers.

use crate::mcu::Board;

pub type Res<T = ()> = Result<T, &'static str>;

/// Klipper `timer_is_before` (wrap safe).
pub fn before(a: u32, b: u32) -> bool {
    (a.wrapping_sub(b) as i32) < 0
}

#[derive(Default)]
pub struct Timer {
    pub active: bool,
    pub wake: u32,
}

const MQ: usize = 8;

/// Per-object move queue of (clock, on_ticks).
#[derive(Default)]
struct Queue {
    buf: [(u32, u32); MQ],
    head: usize,
    len: usize,
}

impl Queue {
    fn push(&mut self, v: (u32, u32)) -> Res<bool> {
        if self.len == MQ {
            return Err("Move queue overflow");
        }
        self.buf[(self.head + self.len) % MQ] = v;
        self.len += 1;
        Ok(self.len == 1)
    }
    fn pop(&mut self) -> Option<(u32, u32)> {
        let v = self.first()?;
        self.head = (self.head + 1) % MQ;
        self.len -= 1;
        Some(v)
    }
    fn first(&self) -> Option<(u32, u32)> {
        (self.len > 0).then(|| self.buf[self.head])
    }
}

const ON: u8 = 1 << 0;
const TOGGLING: u8 = 1 << 1;
const CHECK_END: u8 = 1 << 2;
const DEFAULT_ON: u8 = 1 << 4;

#[derive(Default)]
pub struct DigitalOut {
    pub t: Timer,
    pin: u32,
    flags: u8,
    toggle_fn: bool,
    on_dur: u32,
    off_dur: u32,
    end_time: u32,
    max_dur: u32,
    cycle: u32,
    mq: Queue,
}

impl DigitalOut {
    pub fn new(b: &mut impl Board, pin: u32, value: bool, default: bool, max_dur: u32) -> Res<Self> {
        b.set_out(pin, value)?;
        let flags = if value { ON } else { 0 } | if default { DEFAULT_ON } else { 0 };
        Ok(Self { pin, flags, max_dur, ..Default::default() })
    }

    pub fn set_cycle(&mut self, ticks: u32) -> Res {
        if self.mq.len > 0 {
            return Err("Can not set soft pwm cycle ticks while updates pending");
        }
        self.cycle = ticks;
        Ok(())
    }

    pub fn queue(&mut self, clock: u32, on: u32) -> Res {
        if !self.mq.push((clock, on))? {
            return Ok(());
        }
        let flags = self.flags;
        if flags & CHECK_END != 0 && before(self.end_time, clock) {
            return Err("Scheduled digital out event will exceed max_duration");
        }
        self.end_time = clock;
        self.flags = flags | CHECK_END;
        if !(flags & TOGGLING != 0 && self.t.active && before(self.t.wake, clock)) {
            self.t = Timer { active: true, wake: clock };
            self.toggle_fn = false;
        }
        Ok(())
    }

    pub fn update(&mut self, b: &mut impl Board, now: u32, value: bool) -> Res {
        self.t.active = false;
        if self.mq.len > 0 {
            return Err("update_digital_out not valid with active queue");
        }
        b.set_out(self.pin, value)?;
        let def = self.flags & DEFAULT_ON;
        let on = if value { ON } else { 0 };
        self.flags = def | on;
        if value != (def != 0) && self.max_dur != 0 {
            self.end_time = now.wrapping_add(self.max_dur);
            self.t = Timer { active: true, wake: self.end_time };
            self.toggle_fn = false;
            self.flags |= CHECK_END;
        }
        Ok(())
    }

    /// Shutdown: drive default value, drop queue, stop timer.
    pub fn shutdown(&mut self, b: &mut impl Board) {
        let def = self.flags & DEFAULT_ON != 0;
        let _ = b.set_out(self.pin, def);
        self.flags = if def { ON | DEFAULT_ON } else { 0 };
        self.mq = Queue::default();
        self.t.active = false;
    }

    pub fn event(&mut self, b: &mut impl Board) -> Res {
        if self.toggle_fn { self.toggle(b) } else { self.load(b) }
    }

    fn toggle(&mut self, b: &mut impl Board) -> Res {
        self.flags ^= ON;
        let on = self.flags & ON != 0;
        b.set_out(self.pin, on)?;
        let mut w = self.t.wake.wrapping_add(if on { self.on_dur } else { self.off_dur });
        if self.flags & CHECK_END != 0 && !before(w, self.end_time) {
            self.toggle_fn = false;
            w = self.end_time;
        }
        self.t.wake = w;
        Ok(())
    }

    fn load(&mut self, b: &mut impl Board) -> Res {
        let Some((_, on)) = self.mq.pop() else {
            return Err("Missed scheduling of next digital out event");
        };
        let mut flags = if on != 0 { ON } else { 0 };
        b.set_out(self.pin, flags != 0)?;
        let wake = self.t.wake;
        let mut end = 0;
        if flags == 0 || on >= self.cycle {
            if (flags == 0) != (self.flags & DEFAULT_ON == 0) && self.max_dur != 0 {
                end = wake.wrapping_add(self.max_dur);
                flags |= CHECK_END;
            }
        } else {
            flags |= TOGGLING;
            if self.max_dur != 0 {
                end = wake.wrapping_add(self.max_dur);
                flags |= CHECK_END;
            }
        }
        if let Some((w, _)) = self.mq.first() {
            if flags & CHECK_END != 0 && before(end, w) {
                return Err("Scheduled digital out event will exceed max_duration");
            }
            end = w;
            flags |= CHECK_END;
        }
        self.end_time = end;
        self.flags = flags | (self.flags & DEFAULT_ON);
        if flags & TOGGLING == 0 {
            self.t.active = flags & CHECK_END != 0;
            self.t.wake = end;
            return Ok(());
        }
        let w = wake.wrapping_add(on);
        if flags & CHECK_END != 0 && !before(w, end) {
            self.t.wake = end;
            return Ok(());
        }
        self.toggle_fn = true;
        self.t.wake = w;
        self.on_dur = on;
        self.off_dur = self.cycle - on;
        Ok(())
    }
}

pub const ADC_DATA: usize = 48;

pub struct AnalogIn {
    pub t: Timer,
    pin: u32,
    rest: u32,
    sample_time: u32,
    pub next_begin: u32,
    value: u16,
    min: u16,
    max: u16,
    invalid: u8,
    range_check: u8,
    state: u8,
    sample_count: u8,
    bpr: u8,
    pub data: [u8; ADC_DATA],
    dc: u8,
}

impl AnalogIn {
    pub fn new(b: &mut impl Board, pin: u32) -> Res<Self> {
        b.adc_setup(pin)?;
        Ok(Self {
            t: Timer::default(),
            pin,
            rest: 0,
            sample_time: 0,
            next_begin: 0,
            value: 0,
            min: 0,
            max: 0,
            invalid: 0,
            range_check: 0,
            state: 1,
            sample_count: 0,
            bpr: 0,
            data: [0; ADC_DATA],
            dc: 0,
        })
    }

    /// `query_analog_in` args after oid.
    pub fn query(&mut self, a: &[u32]) -> Res {
        self.next_begin = a[0];
        self.t = Timer { active: false, wake: a[0] };
        self.sample_time = a[1];
        self.sample_count = a[2] as u8;
        self.state = self.sample_count + 1;
        self.rest = a[3];
        self.bpr = a[4] as u8;
        self.dc = 0;
        self.min = a[5] as u16;
        self.max = a[6] as u16;
        self.range_check = a[7] as u8;
        if self.sample_count == 0 {
            return Ok(());
        }
        if self.bpr as usize > ADC_DATA {
            return Err("Invalid analog_in bytes_per_report");
        }
        self.t.active = true;
        Ok(())
    }

    /// Returns Ok(Some(n)) when `data[..n]` should be reported.
    /// `check` false (already shut down) skips the range check.
    pub fn event(&mut self, b: &mut impl Board, check: bool) -> Res<Option<usize>> {
        let v = b.adc_read(self.pin);
        if self.state >= self.sample_count {
            self.state = 0;
            self.value = v;
        } else {
            self.value = self.value.wrapping_add(v);
        }
        self.state += 1;
        if self.state < self.sample_count {
            self.t.wake = self.t.wake.wrapping_add(self.sample_time);
            return Ok(None);
        }
        self.next_begin = self.next_begin.wrapping_add(self.rest);
        self.t.wake = self.next_begin;
        let value = self.value;
        let d = self.dc as usize;
        self.data[d..d + 2].copy_from_slice(&value.to_le_bytes());
        self.dc += 2;
        let mut out = None;
        if self.dc + 2 > self.bpr {
            out = Some(self.dc as usize);
            self.dc = 0;
        }
        if (self.min..=self.max).contains(&value) {
            self.invalid = 0;
        } else {
            self.invalid += 1;
            if self.invalid >= self.range_check {
                self.invalid = 0;
                if check {
                    return Err("ADC out of range");
                }
            }
        }
        Ok(out)
    }
}

const BF_NO_RETRANSMIT: u8 = 0x80;
const BF_PENDING: u8 = 0xff;
const BF_ACKED: u8 = 0xfe;

/// `buttons` (buttons.c): debounced polling of up to 8 inputs, reports
/// retransmitted until acked.
#[derive(Default)]
pub struct Buttons {
    pub t: Timer,
    pins: [u32; 8],
    count: u8,
    rest: u32,
    pressed: u8,
    last_pressed: u8,
    pub reports: [u8; 8],
    pub report_count: u8,
    pub ack_count: u8,
    retransmit_state: u8,
    retransmit_count: u8,
}

impl Buttons {
    pub fn new(count: u32) -> Res<Self> {
        if count > 8 {
            return Err("Max of 8 buttons");
        }
        Ok(Self { count: count as u8, ..Default::default() })
    }

    pub fn add(&mut self, b: &mut impl Board, pos: u32, pin: u32, pull_up: bool) -> Res {
        if pos >= self.count as u32 {
            return Err("Set button past maximum button count");
        }
        b.in_setup(pin, pull_up)?;
        self.pins[pos as usize] = pin;
        Ok(())
    }

    /// `buttons_query` args after oid.
    pub fn query(&mut self, a: &[u32]) -> Res {
        self.t = Timer { active: false, wake: a[0] };
        self.rest = a[1];
        self.pressed = a[3] as u8;
        self.last_pressed = a[3] as u8;
        self.ack_count = 0;
        self.report_count = 0;
        self.retransmit_state = BF_ACKED;
        self.retransmit_count = a[2] as u8;
        if self.retransmit_count >= BF_NO_RETRANSMIT {
            return Err("Invalid buttons retransmit count");
        }
        self.t.active = self.rest != 0;
        Ok(())
    }

    pub fn ack(&mut self, count: u8) {
        self.ack_count = self.ack_count.wrapping_add(count);
        if count >= self.report_count {
            self.report_count = 0;
            self.retransmit_state = BF_ACKED;
        } else {
            let n = (self.report_count - count) as usize;
            self.reports.copy_within(count as usize..count as usize + n, 0);
            self.report_count = n as u8;
        }
    }

    /// Poll pins; returns true when `buttons_state` should be sent now.
    pub fn event(&mut self, b: &mut impl Board) -> bool {
        let mut status = 0u8;
        for i in 0..self.count as usize {
            if b.get_in(self.pins[i]) {
                status |= 1 << i;
            }
        }
        let diff = status ^ self.pressed;
        let mut send = false;
        if diff != 0 {
            let debounced = !(status ^ self.last_pressed);
            if diff & debounced != 0 {
                self.pressed = (self.pressed & !debounced) | (status & debounced);
                if (self.report_count as usize) < self.reports.len() {
                    self.reports[self.report_count as usize] = self.pressed;
                    self.report_count += 1;
                    self.retransmit_state = BF_PENDING;
                }
            }
        }
        self.last_pressed = status;
        if self.retransmit_state & BF_NO_RETRANSMIT == 0 {
            // Wraps 0 -> 0xff (BF_PENDING): timeout, resend.
            self.retransmit_state = self.retransmit_state.wrapping_sub(1);
        }
        if self.retransmit_state == BF_PENDING && self.report_count > 0 {
            self.retransmit_state = self.retransmit_count;
            send = true;
        }
        self.t.wake = self.t.wake.wrapping_add(self.rest);
        send
    }
}
