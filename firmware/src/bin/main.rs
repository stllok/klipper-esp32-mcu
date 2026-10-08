//! Klipper MCU firmware for the JOKER board: base protocol on UART0 (CH340C),
//! digital_out on GPIO + 74HC595 chain, analog_in on ADC1.
//! Clock: TIMG0 T0 at APB/2 = 40 MHz (esp-hal time uses LACT, so T0 is free).
//! Core 1 runs timers (steps, endstops, PWM) in a tight poll loop; core 0
//! moves serial bytes and runs commands, locking the shared state per command.
#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::cell::RefCell;
use critical_section::Mutex;
use esp_backtrace as _;
use core::sync::atomic::{AtomicBool, Ordering};
use esp_hal::Blocking;
use esp_hal::analog::adc::{Adc, AdcCalLine, AdcConfig, AdcPin, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::time::Rate;
use esp_hal::gpio::{AnyPin, Flex, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::main;
use esp_hal::peripherals::{ADC1, GPIO32, GPIO33, GPIO34, GPIO36, GPIO39, TIMG0};
use esp_hal::system::{CpuControl, Stack};
use esp_hal::rtc_cntl::{Rtc, RwdtStage};
use esp_hal::time::Duration;
use esp_hal::uart::{Config, Uart};
use klipper_esp32_mcu::dict_defs::{CLOCK_FREQ, SERIAL_BAUD, SR_PIN_BASE, SR_PINS};
use klipper_esp32_mcu::io::{Res, before};
use klipper_esp32_mcu::{Frame, MESSAGE_HEADER_SIZE, MESSAGE_MAX, MESSAGE_TRAILER_SIZE, find_block};
use klipper_esp32_mcu::mcu::{Board, Mcu, parse};

esp_bootloader_esp_idf::esp_app_desc!();

/// Safe 595 word: drivers disabled (EN high on bits 8/9/10), heaters/fans off.
const SR_SAFE: u32 = (1 << 8) | (1 << 9) | (1 << 10);
/// Bits shifted per latch: 2× 74HC595 on the stock board.
/// ponytail: 16 until a +2/+5 expansion is fitted; then bump to 24/32.
const SR_BITS: u32 = 16;

/// 74HC595 chain bit-banged on GPIO25 (SHCP) / 26 (STCP) / 27 (DS).
/// ~16 bits in a few µs at 240 MHz; plenty for heaters/fans. Phase 3 may
/// move this to I2S if step timing needs it.
struct Sr<'d> {
    // Owned to keep the pins configured; push() drives them via registers.
    _clk: Output<'d>,
    _latch: Output<'d>,
    _data: Output<'d>,
    word: u32,
}

impl Sr<'_> {
    fn push(&mut self) {
        // MSB first: last bit shifted lands in Q0 of the first 595 (bit 0).
        // Raw W1TS/W1TC writes: ~4x faster than Output::set_*, matters per step edge.
        const CLK: u32 = 1 << 25;
        const LATCH: u32 = 1 << 26;
        const DATA: u32 = 1 << 27;
        let g = esp_hal::peripherals::GPIO::regs();
        let (set, clr) = (g.out_w1ts(), g.out_w1tc());
        unsafe {
            for i in (0..SR_BITS).rev() {
                if self.word >> i & 1 != 0 {
                    set.write(|w| w.bits(DATA));
                } else {
                    clr.write(|w| w.bits(DATA));
                }
                // Each APB write is ~50 ns; 74HC595 needs ~20 ns pulses.
                set.write(|w| w.bits(CLK));
                clr.write(|w| w.bits(CLK));
            }
            set.write(|w| w.bits(LATCH));
            clr.write(|w| w.bits(LATCH));
        }
    }
}

struct Adcs<'d> {
    adc: Adc<'d, ADC1<'d>, Blocking>,
    p32: AdcPin<GPIO32<'d>, ADC1<'d>, AdcCalLine<ADC1<'d>>>,
    p33: AdcPin<GPIO33<'d>, ADC1<'d>, AdcCalLine<ADC1<'d>>>,
    p34: AdcPin<GPIO34<'d>, ADC1<'d>, AdcCalLine<ADC1<'d>>>,
    p36: AdcPin<GPIO36<'d>, ADC1<'d>, AdcCalLine<ADC1<'d>>>,
    p39: AdcPin<GPIO39<'d>, ADC1<'d>, AdcCalLine<ADC1<'d>>>,
}

/// Outgoing bytes; drained to the UART FIFO by the main loop so the timer
/// ISR never waits on serial.
/// ponytail: on overflow bytes are dropped; the host retransmits.
struct Tx {
    buf: [u8; 1024],
    head: usize,
    len: usize,
}

struct Joker<'d> {
    uart: Uart<'d, Blocking>,
    tx: Tx,
    sr: Sr<'d>,
    adcs: Adcs<'d>,
    /// Bitmask of direct GPIOs already configured as outputs.
    outs: u64,
    /// One I2C write queued by `i2c_transfer`; core 0 sends it outside the
    /// lock right after the command, so a slow OLED write never stalls steps.
    i2c: Option<(u8, [u8; MESSAGE_MAX], usize)>,
}

/// GPIOs usable as plain outputs: not flash (6-11), UART0 (1, 3), 595 lines
/// (25-27), input-only (34-39) or nonexistent (20, 24, 28-31).
fn output_ok(pin: u32) -> bool {
    matches!(pin, 0 | 2 | 4 | 5 | 12..=19 | 21..=23 | 32 | 33)
}

impl Board for Joker<'_> {
    fn now(&mut self) -> u32 {
        clock()
    }
    fn write(&mut self, block: &[u8]) {
        let t = &mut self.tx;
        if t.len + block.len() > t.buf.len() {
            return;
        }
        for &c in block {
            t.buf[(t.head + t.len) % t.buf.len()] = c;
            t.len += 1;
        }
    }
    fn safe_outputs(&mut self) {
        self.sr.word = SR_SAFE;
        self.sr.push();
    }
    fn set_out(&mut self, pin: u32, value: bool) -> Res {
        if (SR_PIN_BASE..SR_PIN_BASE + SR_PINS).contains(&pin) {
            let bit = 1 << (pin - SR_PIN_BASE);
            let w = if value { self.sr.word | bit } else { self.sr.word & !bit };
            if w != self.sr.word {
                self.sr.word = w;
                self.sr.push();
            }
            return Ok(());
        }
        if !output_ok(pin) {
            return Err("Invalid output pin");
        }
        if self.outs & 1 << pin == 0 {
            // SAFETY: pin validated above and owned by nothing else here.
            // Flex has no Drop, so the config persists after it goes away.
            let mut f = Flex::new(unsafe { AnyPin::steal(pin as u8) });
            f.set_level(Level::from(value));
            f.apply_output_config(&OutputConfig::default());
            f.set_output_enable(true);
            self.outs |= 1 << pin;
            return Ok(());
        }
        // Already configured: write W1TS/W1TC directly (Flex::new would reset the pad).
        let g = esp_hal::peripherals::GPIO::regs();
        unsafe {
            match (pin < 32, value) {
                (true, true) => g.out_w1ts().write(|w| w.bits(1 << pin)),
                (true, false) => g.out_w1tc().write(|w| w.bits(1 << pin)),
                (false, true) => g.out1_w1ts().write(|w| w.bits(1 << (pin - 32))),
                (false, false) => g.out1_w1tc().write(|w| w.bits(1 << (pin - 32))),
            };
        }
        Ok(())
    }
    fn in_setup(&mut self, pin: u32, pull_up: bool) -> Res {
        // Any real GPIO except flash/UART0/595 lines; 34-39 have no pull-up.
        let ok = matches!(pin, 0 | 2 | 4 | 5 | 12..=19 | 21..=23 | 32..=36 | 39);
        if !ok || (pull_up && pin >= 34) {
            return Err("Invalid input pin");
        }
        // SAFETY: pin validated above. Flex has no Drop; config persists.
        let mut f = Flex::new(unsafe { AnyPin::steal(pin as u8) });
        f.set_output_enable(false);
        let pull = if pull_up { Pull::Up } else { Pull::None };
        f.apply_input_config(&InputConfig::default().with_pull(pull));
        f.set_input_enable(true);
        Ok(())
    }
    fn get_in(&mut self, pin: u32) -> bool {
        let g = esp_hal::peripherals::GPIO::regs();
        if pin < 32 {
            g.in_().read().bits() >> pin & 1 != 0
        } else {
            g.in1().read().bits() >> (pin - 32) & 1 != 0
        }
    }
    fn i2c_setup(&mut self, _rate: u32) -> Res {
        // ponytail: bus fixed at 400 kHz on GPIO21/22 from boot.
        Ok(())
    }
    fn i2c_write(&mut self, addr: u8, data: &[u8]) -> u8 {
        let mut buf = [0; MESSAGE_MAX];
        buf[..data.len()].copy_from_slice(data);
        self.i2c = Some((addr, buf, data.len()));
        0 // reported before the transfer; NACKs are not surfaced
    }
    fn adc_setup(&mut self, pin: u32) -> Res {
        match pin {
            32 | 33 | 34 | 36 | 39 => Ok(()),
            _ => Err("Invalid analog pin"),
        }
    }
    fn adc_read(&mut self, pin: u32) -> u16 {
        let a = &mut self.adcs;
        let r = match pin {
            32 => nb::block!(a.adc.read_oneshot(&mut a.p32)),
            33 => nb::block!(a.adc.read_oneshot(&mut a.p33)),
            34 => nb::block!(a.adc.read_oneshot(&mut a.p34)),
            36 => nb::block!(a.adc.read_oneshot(&mut a.p36)),
            39 => nb::block!(a.adc.read_oneshot(&mut a.p39)),
            _ => Ok(0),
        };
        r.unwrap_or(0)
    }
}

static STATE: Mutex<RefCell<Option<(Mcu, Joker<'static>)>>> = Mutex::new(RefCell::new(None));

fn with_state<R>(f: impl FnOnce(&mut Mcu, &mut Joker<'static>) -> R) -> R {
    critical_section::with(|cs| {
        let mut st = STATE.borrow_ref_mut(cs);
        let (m, b) = st.as_mut().unwrap();
        f(m, b)
    })
}

/// Read the 40 MHz clock. Lock-free: both cores may latch concurrently; a
/// racing latch only yields a slightly newer value.
#[esp_hal::ram]
fn clock() -> u32 {
    let t = TIMG0::regs().t(0);
    t.update().write(|w| unsafe { w.bits(1) });
    // ponytail: ESP32 latches LO on the APB write; no ready bit to poll.
    t.lo().read().bits()
}

/// Set by core 0 after commands so core 1 re-reads the next wake time.
static KICK: AtomicBool = AtomicBool::new(false);

/// Core 1: run due timers under the lock, then spin lock-free until the next
/// one (or a kick). Only lock contention with core 0 (one command) delays a step.
#[esp_hal::ram]
fn timer_core() -> ! {
    loop {
        // ponytail: 1 µs gap: closer timers run in the same locked pass.
        let wake = with_state(|m, b| m.service(b, 40));
        loop {
            if KICK.swap(false, Ordering::Acquire) {
                break;
            }
            if let Some(w) = wake
                && !before(clock(), w)
            {
                break;
            }
        }
    }
}

fn clock_init() {
    const APB_HZ: u32 = 80_000_000;
    let t = TIMG0::regs().t(0);
    t.config().write(|w| unsafe { w.bits(0) });
    t.loadlo().write(|w| unsafe { w.bits(0) });
    t.loadhi().write(|w| unsafe { w.bits(0) });
    t.load().write(|w| unsafe { w.bits(1) });
    t.config().write(|w| unsafe {
        w.divider().bits((APB_HZ / CLOCK_FREQ) as u16);
        w.increase().set_bit();
        // Reset value has AUTORELOAD=1; keep it off in case an alarm is ever used.
        w.autoreload().clear_bit();
        w.en().set_bit()
    });
}

#[main]
fn main() -> ! {
    // CPU 240 MHz; APB stays 80 MHz (CLOCK_FREQ relies on it).
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    // Safety first: 595 outputs are undefined until the first latch.
    let oc = OutputConfig::default();
    let mut sr = Sr {
        _clk: Output::new(p.GPIO25, Level::Low, oc),
        _latch: Output::new(p.GPIO26, Level::Low, oc),
        _data: Output::new(p.GPIO27, Level::Low, oc),
        word: SR_SAFE,
    };
    sr.push();

    let mut rtc = Rtc::new(p.RTC_TIMER);
    rtc.rwdt.set_timeout(RwdtStage::Stage0, Duration::from_millis(500));
    rtc.rwdt.enable();

    clock_init();

    // 11 dB: ~0.15..2.45 V usable range; line-fit calibration from eFuse vref,
    // reading in mV (ADC_MAX = 3300).
    let mut cfg = AdcConfig::new();
    let p32 = cfg.enable_pin_with_cal(p.GPIO32, Attenuation::_11dB);
    let p33 = cfg.enable_pin_with_cal(p.GPIO33, Attenuation::_11dB);
    let p34 = cfg.enable_pin_with_cal(p.GPIO34, Attenuation::_11dB);
    let p36 = cfg.enable_pin_with_cal(p.GPIO36, Attenuation::_11dB);
    let p39 = cfg.enable_pin_with_cal(p.GPIO39, Attenuation::_11dB);
    let adc = Adc::new(p.ADC1, cfg);

    // UART0 is the protocol port. Never log over it (esp-println shares it).
    let uart = Uart::new(p.UART0, Config::default().with_baudrate(SERIAL_BAUD))
        .unwrap()
        .with_rx(p.GPIO3)
        .with_tx(p.GPIO1);
    let tx = Tx { buf: [0; 1024], head: 0, len: 0 };
    // OLED header (SSD1306): I2C0, SDA GPIO21 / SCL GPIO22 (Arduino-ESP32 Wire defaults).
    let mut i2c = I2c::new(p.I2C0, I2cConfig::default().with_frequency(Rate::from_khz(400)))
        .unwrap()
        .with_sda(p.GPIO21)
        .with_scl(p.GPIO22);
    let mut board = Joker { uart, tx, sr, adcs: Adcs { adc, p32, p33, p34, p36, p39 }, outs: 0, i2c: None };
    let mut mcu = Mcu::new();
    mcu.start(&mut board);
    critical_section::with(|cs| STATE.borrow_ref_mut(cs).replace((mcu, board)));

    static mut STACK: Stack<8192> = Stack::new();
    let mut cpu = CpuControl::new(p.CPU_CTRL);
    // SAFETY: STACK is only ever handed to the app core, once.
    let stack = unsafe { &mut *core::ptr::addr_of_mut!(STACK) };
    let _core1 = cpu.start_app_core(stack, || timer_core()).unwrap();

    // Framing happens outside the lock; each command runs under it.
    let mut rx = [0u8; 2 * MESSAGE_MAX];
    let mut fill = 0;
    loop {
        rtc.rwdt.feed();
        if fill < rx.len() {
            let n = with_state(|_, b| b.uart.read_buffered(&mut rx[fill..]).unwrap_or(0));
            fill += n;
        }
        loop {
            match find_block(&rx[..fill]) {
                Frame::Incomplete => break,
                Frame::Skip(n) => {
                    rx.copy_within(n..fill, 0);
                    fill -= n;
                }
                Frame::Ok { len, seq } => {
                    if with_state(|m, _| m.accept(seq)) {
                        let mut p = &rx[MESSAGE_HEADER_SIZE..len - MESSAGE_TRAILER_SIZE];
                        while !p.is_empty() {
                            let (c, rest) = parse(p);
                            let w = with_state(|m, b| {
                                m.exec(&c, b);
                                b.i2c.take()
                            });
                            p = rest;
                            KICK.store(true, Ordering::Release);
                            if let Some((addr, buf, n)) = w {
                                let _ = i2c.write(addr, &buf[..n]);
                            }
                        }
                    }
                    with_state(|m, b| m.ack(b));
                    rx.copy_within(len..fill, 0);
                    fill -= len;
                }
            }
        }
        if fill == rx.len() {
            fill = 0; // ponytail: garbage-filled buffer dropped; host retransmits
        }
        with_state(|m, b| m.stats_task(b));
        with_state(|_, b| {
            let t = &mut b.tx;
            // uart.write spins while the FIFO is full; only feed it when there is
            // room, a few bytes per pass, to keep this critical section short.
            let end = (t.head + t.len).min(t.buf.len()).min(t.head + 4);
            if t.len == 0 || !b.uart.write_ready() {
                return;
            }
            if let Ok(n) = b.uart.write(&t.buf[t.head..end]) {
                t.head = (t.head + n) % t.buf.len();
                t.len -= n;
            }
        });
    }
}
