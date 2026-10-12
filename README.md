# klipper-esp32-mcu

Klipper MCU firmware for the JOKER (MagicMaker ESP32) printer board, written in Rust (`no_std`, `esp-hal`). Talks to stock klippy over USB serial (CH340, 250000 baud). No host patches needed.

Only well tested on JOKER v2.0 (ESP32 rev 1.1, TMC2209 standalone). Other ESP32 boards need pin map changes and are untested.

Design, pin map, status and roadmap: [docs/SPEC.md](docs/SPEC.md).

## References

- Klipper protocol: https://www.klipper3d.org/Protocol.html
- Klipper MCU commands: https://www.klipper3d.org/MCU_Commands.html
- ESP32 port request (upstream): https://github.com/Klipper3d/klipper/issues/199
- Prior C port on ESP-IDF: https://github.com/fermino/klipper-esp32-port
- esp-hal: https://github.com/esp-rs/esp-hal

## Layout

| Path | What |
|---|---|
| `src/` | Host-testable protocol core (`mcu.rs` commands, `motion.rs` steppers, `io.rs` GPIO/ADC/595/buttons/I2C, `dict_defs.rs` message table) |
| `build.rs` | Generates zlib data dictionary from `src/dict_defs.rs` |
| `firmware/` | esp-hal binary (xtensa), depends on root crate by path |
| `examples/sim.rs` | Host simulator of the MCU over a PTY |
| `config/printer-joker.cfg` | Klipper printer config for JOKER (Sluban SSS 220) |
| `scripts/hwtest.sh` | Runs real klippy + Moonraker/Fluidd against the board |

## Build & flash

Needs the `esp` Rust toolchain (espup) and `espflash`.

```sh
cargo test                                  # host tests (repo root)

cd firmware
export PATH=~/.rustup/toolchains/esp/xtensa-esp-elf/esp-15.2.0_20250920/xtensa-esp-elf/bin:$PATH
cargo build --release
espflash flash --chip esp32 -p /dev/ttyUSB0 target/xtensa-esp32-none-elf/release/firmware
```

ESP32 rev 1.1 boards: `firmware/.cargo/esp-config.toml` sets `ESP_HAL_CONFIG_MIN_CHIP_REVISION="100"`, otherwise espflash refuses.

## Run

Simulator (no hardware):

```sh
cargo build --example sim
socat PTY,link=/tmp/kmcu,raw,echo=0 EXEC:target/debug/examples/sim &
python klippy/console.py -b 250000 /tmp/kmcu   # from a Klipper checkout
```

Hardware:

```sh
KLIPPER=/path/to/klipper scripts/hwtest.sh [timeout_s]
```

State lives in `.hwtest/` (gitignored). Send gcode with `echo CMD > .hwtest/printer`; log in `.hwtest/klippy.log`. After `SAVE_CONFIG`, copy the `#*#` block back into `config/printer-joker.cfg` (hwtest overwrites `.hwtest/printer.cfg` each run).

## Notes

- `CLOCK_FREQ` = 40 MHz (TIMG0).
- Message ids = indices into `MESSAGES` in `src/dict_defs.rs`.
- Not supported yet: WiFi/BT, SD card, CAN, accelerometer. See SPEC §1.
