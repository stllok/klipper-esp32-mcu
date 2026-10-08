//! Host simulator: runs the MCU core over stdin/stdout so the real Klipper
//! host can talk to it through a PTY:
//!   socat PTY,link=/tmp/kmcu,raw,echo=0 EXEC:target/debug/examples/sim
use klipper_esp32_mcu::dict_defs::CLOCK_FREQ;
use klipper_esp32_mcu::mcu::{Board, Mcu};
use std::io::{Read, Write};
use std::sync::mpsc;
use std::time::Instant;

struct Sim(Instant, std::io::Stdout);
impl Board for Sim {
    fn now(&mut self) -> u32 {
        (self.0.elapsed().as_nanos() * CLOCK_FREQ as u128 / 1_000_000_000) as u32
    }
    fn write(&mut self, b: &[u8]) {
        self.1.write_all(b).unwrap();
        self.1.flush().unwrap();
    }
    fn set_out(&mut self, _pin: u32, _v: bool) -> klipper_esp32_mcu::io::Res {
        Ok(())
    }
    fn in_setup(&mut self, _pin: u32, _pull_up: bool) -> klipper_esp32_mcu::io::Res {
        Ok(())
    }
    fn adc_setup(&mut self, _pin: u32) -> klipper_esp32_mcu::io::Res {
        Ok(())
    }
    // ~25 C on a 100k NTC with 4.7k pull-up (mV of ADC_MAX=3300).
    fn adc_read(&mut self, _pin: u32) -> u16 {
        3150
    }
    fn i2c_setup(&mut self, _rate: u32) -> klipper_esp32_mcu::io::Res {
        Ok(())
    }
    fn i2c_write(&mut self, _addr: u8, _data: &[u8]) -> u8 {
        0
    }
    // Endstops read triggered so G28 completes instantly.
    fn get_in(&mut self, _pin: u32) -> bool {
        true
    }
}

fn main() {
    let (tx, rx) = mpsc::channel::<Vec<u8>>();
    std::thread::spawn(move || {
        let mut buf = [0u8; 64];
        let mut stdin = std::io::stdin();
        while let Ok(n) = stdin.read(&mut buf) {
            if n == 0 || tx.send(buf[..n].to_vec()).is_err() {
                break;
            }
        }
    });
    let mut b = Sim(Instant::now(), std::io::stdout());
    let mut mcu = Mcu::new();
    mcu.start(&mut b);
    let mut pending: Vec<u8> = Vec::new();
    loop {
        // Busy loop: step timers need sub-ms polling.
        match rx.try_recv() {
            Ok(d) => pending.extend(d),
            Err(mpsc::TryRecvError::Disconnected) => return,
            Err(_) => {}
        }
        while !pending.is_empty() {
            let space = mcu.rx_space();
            let n = space.len().min(pending.len());
            space[..n].copy_from_slice(&pending[..n]);
            pending.drain(..n);
            mcu.rx_commit(n, &mut b);
        }
        mcu.task(&mut b);
    }
}
