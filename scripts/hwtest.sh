#!/usr/bin/env bash
# Hardware connection test: start real klippy against the board with
# config/printer-joker.cfg and wait until the MCU is configured.
# Usage: scripts/hwtest.sh [timeout_s]    (state lives in .hwtest/, gitignored)
set -euo pipefail
cd "$(dirname "$0")/.."
DIR=.hwtest
TIMEOUT=${1:-60}
mkdir -p "$DIR"
MACOS=0; [ "$(uname -s)" = Darwin ] && MACOS=1
if [ "$MACOS" = 1 ]; then
    # macOS: Klipper cloned into .hwtest/. Moonraker runs without inotify (see conf).
    KLIPPER=${KLIPPER:-$PWD/$DIR/klipper}
    [ -d "$KLIPPER/klippy" ] || git clone -q --depth 1 https://github.com/Klipper3d/klipper.git "$KLIPPER"
    # chelper is Linux-only in 2 spots: prctl(PR_SET_NAME) and <linux/can.h>.
    # Patch the clone (idempotent; serial, not CAN, is used here).
    CH="$KLIPPER/klippy/chelper"
    grep -q __APPLE__ "$CH/pyhelper.c" || python3 - "$CH" <<'PY'
import sys; d = sys.argv[1]
def sub(f, a, b):
    p = f"{d}/{f}"; s = open(p).read(); assert a in s, (f, a)
    open(p, "w").write(s.replace(a, b, 1))
sub("pyhelper.c", "#include <sys/prctl.h>  // prctl",
    "#ifdef __APPLE__\n#include <pthread.h>\n#else\n#include <sys/prctl.h>\n#endif")
sub("pyhelper.c", "return prctl(PR_SET_NAME, name);",
    "#ifdef __APPLE__\n    return pthread_setname_np(name);\n#else\n"
    "    return prctl(PR_SET_NAME, name);\n#endif")
sub("serialqueue.c", "#include <linux/can.h> // // struct can_frame",
    "#ifdef __APPLE__\n#include <stdint.h>\nstruct can_frame { uint32_t can_id;"
    " uint8_t can_dlc; uint8_t __pad[3]; uint8_t data[8]; };\n"
    "#define CAN_EFF_FLAG 0x80000000U\n#else\n#include <linux/can.h>\n#endif")
PY
    # No setsid binary on macOS: start command in a new session via python.
    # exec: background subshell becomes the process, so $! == pid == pgid.
    setsid() { exec python3 -c 'import os,sys; os.setsid(); os.execvp(sys.argv[1], sys.argv[1:])' "$@"; }
else
    KLIPPER=${KLIPPER:-/mnt/ssd/data/git/klipper}
fi

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
# No inotify on macOS: disable moonraker's file system observer.
[ "$MACOS" = 0 ] || printf '\n[file_manager]\nfile_system_observer: none\n' >> "$DIR/moonraker.conf"

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
if [ "$MACOS" = 1 ]; then
    # /dev/serial/by-id is Linux-only; use SERIAL or first /dev/cu.usbserial-*.
    SERIAL=${SERIAL:-$(ls /dev/cu.usbserial-* /dev/cu.usbmodem* 2>/dev/null | head -1 || true)}
    [ -n "$SERIAL" ] || { echo "FAIL: no /dev/cu.usbserial-* (set SERIAL=)" >&2; exit 1; }
    sed -i '' "s|^serial:.*|serial: $SERIAL|" "$DIR/printer.cfg"
    setsid "$DIR/venv/bin/python" "$KLIPPER/klippy/klippy.py" \
        "$DIR/printer.cfg" -I "$DIR/printer" -l "$DIR/klippy.log" \
        -a "$PWD/$DIR/klippy.sock" </dev/null >/dev/null 2>&1 &
    echo $! > "$DIR/klippy.pid"
else
# plain `sg` may be ast-grep; dialout is needed for the serial port.
setsid /usr/bin/sg dialout -c "exec '$DIR/venv/bin/python' '$KLIPPER/klippy/klippy.py' \
    '$DIR/printer.cfg' -I '$DIR/printer' -l '$DIR/klippy.log' \
    -a '$PWD/$DIR/klippy.sock'" </dev/null >/dev/null 2>&1 &
echo $! > "$DIR/klippy.pid"
fi

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
