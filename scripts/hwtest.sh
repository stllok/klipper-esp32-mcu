#!/usr/bin/env bash
# Hardware connection test: start real klippy against the board with
# config/printer-joker.cfg and wait until the MCU is configured.
# Usage: scripts/hwtest.sh [timeout_s]    (state lives in .hwtest/, gitignored)
set -euo pipefail
cd "$(dirname "$0")/.."
KLIPPER=${KLIPPER:-/mnt/ssd/data/git/klipper}
DIR=.hwtest
TIMEOUT=${1:-60}
mkdir -p "$DIR"

if [ ! -x "$DIR/venv/bin/python" ]; then
    python3 -m venv "$DIR/venv"
    "$DIR/venv/bin/pip" install -q greenlet cffi pyserial jinja2
fi

# Fluidd UI + moonraker (fetched once into .hwtest/, gitignored).
[ -d "$DIR/moonraker" ] || git clone -q --depth 1 https://github.com/Arksine/moonraker.git "$DIR/moonraker"
if [ ! -f "$DIR/fluidd/index.html" ]; then
    curl -sfL -o "$DIR/fluidd.zip" https://github.com/fluidd-core/fluidd/releases/latest/download/fluidd.zip
    mkdir -p "$DIR/fluidd" && unzip -qo "$DIR/fluidd.zip" -d "$DIR/fluidd"
fi
"$DIR/venv/bin/python" -c 'import tornado' 2>/dev/null ||
    "$DIR/venv/bin/pip" install -q -r "$DIR/moonraker/scripts/moonraker-requirements.txt"
mkdir -p "$DIR/data/config" "$DIR/data/logs"
cat > "$DIR/moonraker.conf" <<CONF
[server]
host: 0.0.0.0
port: 7125
klippy_uds_address: $PWD/$DIR/klippy.sock

[authorization]
trusted_clients:
    127.0.0.0/8
    ::1/128
    192.168.0.0/16
    10.0.0.0/8
cors_domains:
    *://localhost:8080
    *://127.0.0.1:8080
    *://*.local:8080

[octoprint_compat]
CONF

# Stop moonraker/fluidd left over from a previous run.
for f in moonraker.pid fluidd.pid; do
    if [ -f "$DIR/$f" ]; then kill -- -"$(cat "$DIR/$f")" 2>/dev/null || true; fi
done

# Stop a klippy left over from a previous run: pidfile holds its process group.
if [ -f "$DIR/klippy.pid" ]; then
    kill -- -"$(cat "$DIR/klippy.pid")" 2>/dev/null || true
    sleep 1
fi

cp config/printer-joker.cfg "$DIR/printer.cfg"
: > "$DIR/klippy.log"
# plain `sg` may be ast-grep; dialout is needed for the serial port.
setsid /usr/bin/sg dialout -c "exec '$DIR/venv/bin/python' '$KLIPPER/klippy/klippy.py' \
    '$DIR/printer.cfg' -I '$DIR/printer' -l '$DIR/klippy.log' \
    -a '$PWD/$DIR/klippy.sock'" </dev/null >/dev/null 2>&1 &
echo $! > "$DIR/klippy.pid"

setsid "$DIR/venv/bin/python" "$DIR/moonraker/moonraker/moonraker.py" \
    -d "$PWD/$DIR/data" -c "$PWD/$DIR/moonraker.conf" -l "$PWD/$DIR/moonraker.log" \
    </dev/null >/dev/null 2>&1 &
echo $! > "$DIR/moonraker.pid"
setsid "$DIR/venv/bin/python" scripts/fluidd_serve.py "$DIR/fluidd" 8080 </dev/null >/dev/null 2>&1 &
echo $! > "$DIR/fluidd.pid"

for _ in $(seq "$TIMEOUT"); do
    sleep 1
    if grep -q "Configured MCU 'mcu'" "$DIR/klippy.log"; then
        grep -m1 "Configured MCU 'mcu'" "$DIR/klippy.log"
        echo "Fluidd: http://localhost:8080 (moonraker :7125, log $DIR/moonraker.log)"
        echo "OK: klippy connected (still running; stop: kill -- -$(cat "$DIR/klippy.pid")," \
             "gcode pty $DIR/printer)"
        exit 0
    fi
    if grep -qE "^(Config error|Internal error)|MCU '.*' shutdown|Error " "$DIR/klippy.log"; then
        break
    fi
done
echo "FAIL: see $DIR/klippy.log" >&2
grep -E "error|Error|shutdown|Unable" "$DIR/klippy.log" | tail -5 >&2 || true
exit 1
