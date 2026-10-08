# Klipper MCU firmware for ESP32 (JOKER board) — Spec

Status: v0.5 (2026-10). Phases 0-2 done. Phase 3 code complete (step jitter < 2 µs met); bench test (STEPPER_BUZZ X/Y/Z/E with motors, G28 with real switches) pending, deferred to before the Phase 4 print.
Implementation language: Rust (`no_std`, `esp-hal` 1.2.x), not a fork of Klipper's C tree.

## 1. Goal

Run the JOKER (MagicMaker, ESP32) printer board as a Klipper MCU: Klipper host
(klippy on a Pi/PC) talks to it over USB serial (CH340C) using the standard
Klipper protocol, with no host-side patches required for the core feature set.

Non-goals (for now): WiFi/BT transport, LCD/encoder UI, SD card, Klipper
upstream merge, ADXL/accelerometer, CAN.

## 2. Research findings

### 2.1 Prior art
| Project | Approach | Takeaway |
|---|---|---|
| Klipper3d/klipper#199 (2018–2024, closed/locked) | Feature request | No official ESP32 port. Main objections: toolchain (ESP-IDF/FreeRTOS), timing jitter, limited GPIO. Upstream prefers minimal, close-to-hardware ports (cf. `src/rp2040`). |
| nikhil-robinson/klipper_esp32 (2024) | Klipper C on ESP-IDF | Commands work over UART; data dictionary hard-coded due to build system mismatch. |
| fermino/klipper-esp32-port (created 2025-11, last push 2026-03) | Klipper C fork, ESP-IDF HAL/LL only, "FreeRTOS out of the way" | Confirms viable path; explicitly no WiFi. |
| Marlin `HAL/ESP32` + `pins_MRR_ESPE.h` | I2S stepper stream into 74HC595 | Reference for JOKER's I2S shift-register output scheme. |

Sources: https://github.com/Klipper3d/klipper/issues/199,
https://github.com/fermino/klipper-esp32-port,
https://www.klipper3d.org/Protocol.html, https://www.klipper3d.org/MCU_Commands.html,
Klipper `docs/Code_Overview.md` ("Porting to a new micro-controller").

### 2.2 Why Rust/esp-hal instead of forking Klipper C
- Klipper's MCU build relies on its Kconfig/Makefile + `buildcommands.py` to
  generate the data dictionary; ESP-IDF's CMake fights this (the reason other
  ports hard-code the dictionary). In Rust we generate the dictionary in
  `build.rs` and own the whole image.
- `esp-hal` is bare-metal (no FreeRTOS), so timer ISR latency is
  deterministic — the exact thing the C ports struggle to achieve.
- Toolchain already present: `esp` rustup toolchain, `esp-hal 1.2.2`,
  `espflash 4.6.0` (template from `esp-generate`).

Trade-off: we must reimplement `sched.c`, `command.c`, `stepper.c`,
`gpiocmds.c`, `adccmds.c`, `endstop.c`, `trsync.c`, `basecmd.c` semantics.
They are small (~2–3 kLOC C) and fully specified by protocol + host code.

### 2.3 Klipper MCU contract (what the host expects)
- Framing: `<len><0x10|seq><payload><crc16 BE><0x7e>`, 5..64 bytes,
  CRC16-CCITT (Klipper variant, check value `0x6f91`), VLQ ints, strings as
  `<vlq len><bytes>`. Ack = empty block with seq = last_rx+1; nak = empty
  block with current expected seq. Implemented in `src/lib.rs`.
- Hard-coded: `identify_response offset=%u data=%.*s` id 0, `identify offset=%u count=%c` id 1.
  Everything else via zlib-compressed JSON data dictionary.
- Clock: 32-bit free-running counter, `CLOCK_FREQ` constant exported; host
  compares with `timer_is_before` semantics (wrap-safe).
- Required constants: `MCU`, `CLOCK_FREQ`, `SERIAL_BAUD`, `STATS_SUMSQ_BASE`,
  `ADC_MAX`, `PWM_MAX`, `RESERVE_PINS_serial`, `STEPPER_BOTH_EDGE` (optional).
- Required base commands: `get_uptime`, `get_clock`, `get_config`,
  `config_reset`, `finalize_config crc=%u`, `allocate_oids count=%c`,
  `emergency_stop`, `clear_shutdown`, `identify`, `stats` task output,
  `shutdown`/`is_shutdown` responses.
- Timing: tasks must not block >100 µs; timer callbacks must finish in a few µs;
  host retransmits if no ack in ~tens of ms.

## 3. Target hardware: JOKER (v2.0 schematic)

Files: `/mnt/ssd/data/git/magicmaker/ESP32主控板JOKER SPARROW/JOKER/`
(`原理图/joker2.0原理图.pdf`, `Firmware template固件模板/pins_MRR_ESPE.h`).
Board is an MRR ESPE derivative. Factory flash backup: `backup/factory-4MB.bin`.

- MCU: ESP32 (dual Xtensa LX6, 240 MHz, 4 MB flash), USB via CH340C on
  UART0 (GPIO1 TX / GPIO3 RX), auto-reset (DTR/RTS) circuit.
- Drivers: 4 sockets (X, Y, Z, E0), fitted with TMC2209 in standalone mode (no UART wired). Microsteps come from MS1/MS2 straps (TMC2209: 00=8, 01=32, 10=64, 11=16). JOKER strap state not yet checked; set `microsteps` to match.
- Outputs are NOT direct GPIO: 2× 74HC595 driven by I2S0 in serial mode.

### 3.1 Direct GPIO
| Function | GPIO | Notes |
|---|---|---|
| I2S_BCK (595 SHCP) | 25 | |
| I2S_WS (595 STCP/latch) | 26 | |
| I2S_DATA (595 DS) | 27 | |
| X / Y / Z endstop | 15 / 16 / 17 | GPIO15 is a strapping pin |
| Hotend thermistor | 36 (ADC1_CH0) | input-only, 4.7 k pull-up |
| Bed thermistor | 39 (ADC1_CH3) | input-only |
| Filament runout | 13 | |
| Servo / BLTouch | 14 | |
| SD card (SPI) | 23 MOSI, 19 MISO, 18 SCK, 5 CS | later |
| Encoder EN1/EN2/ENC | 2 / 4 / 12 | GPIO2/12 strapping; UI = non-goal |
| Expansion (+2/+5) | 32, 33, 34 (ADC1) | extra thermistor or MAX endstops |
| Spare UART1 | 21 / 22 | possible debug log port |

### 3.2 Shift-register outputs (Marlin pin 128+n = bit n)
| Bit | Signal | Bit | Signal |
|---|---|---|---|
| 0 | X_STEP | 8 | X+Y ENABLE (shared) |
| 1 | X_DIR | 9 | Z_ENABLE |
| 2 | Y_STEP | 10 | E0_ENABLE |
| 3 | Y_DIR | 11 | BEEPER |
| 4 | Z_STEP | 12 | HEATER_BED |
| 5 | Z_DIR | 13 | HEATER_E0 |
| 6 | E0_STEP | 14 | E0 auto fan (FAN1) |
| 7 | E0_DIR | 15 | Part fan (FAN0) |

Bits 16+ exist only with the JOKER+2 / +5 expansion (more 595s on the chain).
Bit order verified: MSB-first shift, 16 bits, lands bit 0 at Q0 of first 595 (fan sr15 + beeper sr11 confirmed).

## 4. Architecture

```
klippy (host) ──USB/CH340──► UART0 ─► rx ring ─► framer (lib.rs) ─► dispatcher
                                                                   │
                       timer ISR (TIMG0, 1 alarm) ◄── scheduler ◄──┘ (oids, queues)
                              │
               ┌──────────────┼─────────────────┐
         out_word (u32 shadow) ADC1 sampling    endstop/trsync
               │
           I2S0 ─► 74HC595 chain (step/dir/en/heaters/fans)
```

- Single core (PRO_CPU) for everything in v1. APP_CPU unused. Option later:
  move comms to APP_CPU.
- No RTOS, no allocator. Fixed-size pools for oids/move queues (Klipper
  `move_queue` style), sized from `allocate_oids`.
- Clock: TIMG0 **T0** (esp-hal's `time` uses the LACT timer, so T0 is free) (APB 80 MHz / divider 2 = 40 MHz, 32-bit low word of
  the 64-bit counter) → 25 ns resolution, wrap ≈107 s.
  Decision: **CLOCK_FREQ = 40_000_000**, read via TIMG0 LO register.
  (CCOUNT CPU cycle counter rejected: frequency changes with CPU clock and it
  is per-core.)
- Scheduler: sorted timer list + one TIMG alarm, same algorithm as Klipper
  `sched.c` (`sched_add_timer`, `TIMER_MIN_TRY_TICKS`, repeat-in-ISR loop).
  ISR and all timer code in IRAM (`#[ram]`).
- Comms: UART0 at 250000 baud (CH340C supports it; Klipper default). RX via
  UART FIFO interrupt into ring buffer; TX from task loop.

### 4.1 Shift-register output design (the hard part)
Klipper's stepper code issues a GPIO toggle per step at an exact clock. With
595s, every output change means shifting 16+ bits and latching.

Option A (v1, chosen): **I2S in "immediate/shadow" mode.** Keep a 32-bit
shadow word; on each step event, update the bit and push one 32-bit frame
through I2S0 TX FIFO (I2S clock ≈ 4–8 MHz → 4–8 µs per word incl. latch).
Step pulse: set STEP, next frame clear STEP (min pulse = one frame ≈ 4 µs;
TMC2209 needs ≥100 ns). Exports `STEPPER_STEP_BOTH_EDGE=1`, but the host only uses both-edge stepping when TMC UART enables dedge, which this board lacks.
Limits: total step rate ≈ 100–125 k events/s across all steppers (matches
Marlin/Grbl_ESP32 "125 kHz" figure). Enough for a Cartesian printer at
typical speeds; measure in Phase 3.

Option B (later, if A jitters): Marlin-style DMA stream — I2S DMA runs
continuously at fixed 4 µs ticks, scheduler fills the buffer ahead of time.
Deterministic but adds a fixed latency and buffer management. Only if
Phase 3 measurements demand it.

Heater/fan PWM on 595 bits: software PWM via scheduler (Klipper
`digital_out` with `cycle_ticks`), heaters at ~10 Hz is fine. Part fan PWM
limited to low frequency (≤ ~1 kHz); acceptable.

### 4.2 Pin naming exported to host
- Direct GPIO: `gpio0`..`gpio39` (enumeration `pin`, values 0..39).
- Shift-register bits: `sr0`..`sr31` (values 128..159, same numbering as Marlin).
- Firmware validates per command: input-only pins (34–39) reject output;
  flash pins (6–11) and UART0 (1, 3) reserved (`RESERVE_PINS_serial`).

### 4.3 Data dictionary generation
- Commands declared in Rust as a static table `(format_str, handler)`.
- `build.rs` emits JSON `{commands, responses, enumerations, config, version,
  build_versions}`, zlib-compresses (`miniz_oxide`, build-dependency only),
  embeds bytes via `include_bytes!`. Command ids assigned deterministically
  (identify_response=0, identify=1, others by sorted order).
- `identify offset count` serves slices of that blob.

## 5. Command set by milestone
| Milestone | Commands / responses |
|---|---|
| M1 identify | identify / identify_response, get_uptime, get_clock, clock, uptime, get_config, config, allocate_oids, finalize_config, config_reset, emergency_stop, clear_shutdown, shutdown, is_shutdown, stats, debug_* optional |
| M2 IO | config_digital_out, set_digital_out_pwm_cycle, queue_digital_out, update_digital_out, set_digital_out, config_analog_in, query_analog_in, analog_in_state |
| M3 motion | config_stepper, queue_step, set_next_step_dir, reset_step_clock, stepper_get_position, stepper_stop_on_trigger, config_endstop, endstop_home, endstop_query_state, endstop_state, config_trsync, trsync_start, trsync_set_timeout, trsync_trigger, trsync_state |
| M4 extras | buttons ✅, i2c_transfer (write-only) ✅, config_pwm_out (servo on GPIO14 via LEDC), spi (SD) — optional |

## 6. Safety requirements
- On boot and on `shutdown`/`emergency_stop`: shadow word = all heaters off,
  fans off, all drivers DISABLED (EN high, active-low on TMC2209), pushed before anything else.
- 595 outputs are undefined until the first I2S frame: first thing in `main`
  is pushing the safe word. Verify on hardware that heaters don't pulse at reset.
- `digital_out` max_duration enforced (host sets it for heaters); expiry → shutdown.
- ADC min/max range check → shutdown (Klipper `analog_in` semantics).
- Hardware watchdog (RWDT) enabled, fed from the main loop; timeout ≈ 500 ms.
- Never log on UART0 (shares the protocol port). Debug logs go to UART1
  (GPIO21/22) or are disabled.

## 7. Phased plan & acceptance criteria
| Phase | Deliverable | Done when |
|---|---|---|
| 0 ✅ | Repo layout, protocol primitives + host tests, firmware skeleton that frames + acks on UART0 | `cargo test` passes; `firmware` builds; board acks a hand-made block |
| 1 ✅ (scheduler deferred to Phase 2) | Clock, data dictionary, M1 commands | `klippy/console.py -b 250000 /dev/ttyUSB0` connects, `get_clock` works; klippy reaches "config" stage with an `[mcu]`-only config |
| 2 ✅ (595 bit-banged on GPIO25-27; I2S deferred to Phase 3 if needed; dout/ain timers polled in main loop) | 595 driver, digital_out, analog_in | Fan/heater toggled from klippy; `TEMPERATURE` readings sane (ADC1 calibration via eFuse) |
| 3 🟡 code complete, bench test pending (timers polled on core 1, commands locked per command on core 0; commands parsed outside the lock; measured 50k steps/s: p99 1.6-1.8 µs, max 5.4 µs in 80k edges) | Stepper, endstop, trsync | `STEPPER_BUZZ` OK on X/Y/Z/E; `G28` homes; logic-analyzer step jitter < 2 µs @ 50 k steps/s |
| 4 🟡 `config/printer-joker.cfg` from `/mnt/ssd/data/git/joker-marlin` (Sluban SSS 220: 220×220×140, BMG E387, SN04-N probe on gpio14, bed mesh). SSD1306 OLED via `i2c_transfer` on I2C0 (SDA21/SCL22, 400 kHz, written outside the state lock, write-only, status always SUCCESS) and encoder via `buttons` (gpio2/4/12). Sim dry run OK; on hardware klippy reaches Ready with the display configured. Real print pending | Print a calibration cube | Full print via Klipper with a JOKER `printer.cfg` in `config/` |

## 8. Repo layout
```
Cargo.toml          host-testable protocol crate (no_std), `cargo test`
build.rs            generates zlib data dictionary from src/dict_defs.rs
src/lib.rs          framing, CRC16, VLQ
src/dict_defs.rs    message table (id = index), constants, enums, static strings
src/mcu.rs          dispatcher + base commands behind `Board` trait (host-tested)
src/io.rs           digital_out (soft PWM, max_duration) + analog_in objects
src/motion.rs       stepper (stepper_event_full), endstop, trsync; shared 1024-move pool
examples/sim.rs     stdin/stdout simulator for running real klippy over a PTY
firmware/           esp-hal binary (xtensa), depends on crate by path
docs/SPEC.md        this file
backup/             factory flash dump (git-ignored)
```

## 9. Build / flash
```
cargo test                                   # host, repo root
cd firmware
export PATH=~/.rustup/toolchains/esp/xtensa-esp-elf/esp-15.2.0_20250920/xtensa-esp-elf/bin:$PATH
cargo build --release
espflash flash --chip esp32 -p /dev/ttyUSB0 target/xtensa-esp32-none-elf/release/firmware
```
Restore Marlin: `espflash write-bin -p /dev/ttyUSB0 0x0 ../backup/factory-4MB.bin`.
Simulator (no hardware):
```
cargo build --example sim
socat PTY,link=/tmp/kmcu,raw,echo=0 EXEC:target/debug/examples/sim &
python klippy/console.py -b 250000 /tmp/kmcu     # Klipper checkout, venv: greenlet cffi pyserial jinja2
```
JOKER's chip is ESP32 rev 1.1; esp-hal defaults to min rev 3.0, so
`firmware/.cargo/esp-config.toml` sets `ESP_HAL_CONFIG_MIN_CHIP_REVISION="100"`.
Note: `~/.cargo/config.toml` forces `-fuse-ld=mold`; `firmware/.cargo/config.toml`
overrides rustflags at target level so the xtensa linker is used.

## 10. Open questions / risks
0. analog_in reports calibrated mV (`ADC_MAX=3300`). ESP32 11 dB tops out
   ~3.1 V, so a 100k NTC at room temp (≈3.1 V) is near saturation; accuracy
   at printing temps is fine. Measured: both pins pinned at 0.951 (≈3139 mV
   ceiling) at room temp → reads constant ~27 °C below ~35 °C; open
   thermistor indistinguishable from cold.
1. ~~595 bit order~~ resolved (Phase 2 hardware test).
2. ADC1 on ESP32 is nonlinear; may need eFuse two-point calibration or a
   host-side `adc_temperature` table tuned for the board.
3. CH340C at 250000 baud: confirm error rate; fallback 500000/1000000 if
   bandwidth is short.
4. Stepper EN for X and Y is shared (bit 8) — Klipper must use one
   `enable_pin` for both; document in sample config.
5. Strapping pins (GPIO2, 12, 15) used by board signals; leave them as inputs
   until boot completes.
