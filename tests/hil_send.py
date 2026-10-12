#!/usr/bin/env python3
"""HIL flight-trace sender: CSV trace -> Ek-7 SUT synthetic frames @10 Hz.

Trace CSV columns:
  t_s,press_pa,ax,ay,az,ex,ey,ez,valid
altitude is derived host-side from pressure against the trace-start P0
(same barometric relation as firmware). Streams via ST-LINK VCOM; the
firmware SUT path feeds the SAME estimator + state machine as flight.
Gyro columns are recorded for format completeness (Ek-7 SUT frames carry
no gyro fields, so Mahony is untouched by HIL).

Usage:
  python3 tests/hil_send.py tests/traces/ascent.csv [--port PATH] [--dry]
  python3 tests/hil_send.py --gen-ascent tests/traces/ascent.csv
"""
import argparse
import csv
import glob
import os
import struct
import sys
import termios
import time
import tty


def discover_port():
    c = sorted(glob.glob("/dev/serial/by-id/*STLink*"))
    if c:
        return c[0]
    c = sorted(glob.glob("/dev/ttyACM*"))
    return c[0] if c else None


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    a = termios.tcgetattr(fd)
    a[4] = a[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def baro_alt(p, p0):
    return 44330.0 * (1.0 - (p / p0) ** (1.0 / 5.255))


def synth_frame(alt, press_mb, ax, ay, az, ex=0.0, ey=0.0, ez=0.0):
    body = struct.pack(">8f", round(alt, 2), round(press_mb, 2),
                       round(ax, 2), round(ay, 2), round(az, 2),
                       round(ex, 2), round(ey, 2), round(ez, 2))
    return b"\xAB" + body + bytes([(sum(body) & 0xFF), 0x0D, 0x0A])


def gen_ascent(path):
    """Deterministic normal-ascent HIL trace (TEST thresholds)."""
    rows = []
    t = 0.0

    def add(dt, press, ax, ay, az, valid=1):
        nonlocal t
        n = int(round(dt / 0.1))
        for _ in range(n):
            rows.append((round(t, 1), press, ax, ay, az, 0.0, 0.0, 0.0,
                         valid))
            t = round(t + 0.1, 1)

    p0 = 100000.0
    add(2.0, p0, 0.0, 0.0, -9.81)                    # rest
    add(0.4, p0, 0.0, 0.0, -14.7)                    # launch spike 1.5 g
    for i in range(30):                              # climb to ~8 m
        add(0.1, p0 - (i + 1) * 3.0, 0.0, 0.0, -9.81)
    for i in range(20):                              # fast descent
        add(0.1, p0 - 90.0 + (i + 1) * 4.5, 0.0, 0.0, -9.81)
    add(0.6, p0, 0.0, 0.0, -1.0)                     # free-fall accel
    add(3.0, p0, 0.0, 0.0, -9.81)                    # rest on ground
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "press_pa", "ax", "ay", "az", "ex", "ey",
                    "ez", "valid"])
        w.writerows(rows)
    print("wrote %s (%d rows)" % (path, len(rows)))


class LiveHil:
    """Compact ANSI monitor for HIL runs (single serial owner).

    INPUT.* = PC-sent synthetic values. BOARD.* = STM32-decoded replies.
    Never mixed. Order check (launch<desc<drogue<main) preserved."""

    def __init__(self):
        self.n_tx = 0
        self.n_rx = 0
        self.words = []
        self.in_t = 0.0
        self.in_alt = 0.0
        self.in_vvel = 0.0
        self._buf = b""
        self._on = False
        self._last_draw = 0.0

    def start(self):
        sys.stdout.write("\033[2J\033[H\033[?25l")
        sys.stdout.flush()
        self._on = True

    def stop(self):
        if self._on:
            sys.stdout.write("\033[?25h\n")
            sys.stdout.flush()
            self._on = False

    def wait_ready(self, fd):
        """st-flash reset + WAIT_FOR_LAUNCH gate (stale-state guard)."""
        import subprocess
        try:
            subprocess.run(["st-flash", "reset"], timeout=20,
                           capture_output=True)
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False
        time.sleep(0.3)
        try:
            while os.read(fd, 4096):
                pass
        except (BlockingIOError, OSError):
            pass
        t0 = time.time()
        buf = b""
        while time.time() - t0 < 45:
            try:
                ch = os.read(fd, 4096)
                if ch:
                    buf += ch
                    if b"WAITING FOR LAUNCH" in buf:
                        return True
                else:
                    time.sleep(0.05)
            except BlockingIOError:
                time.sleep(0.05)
            except OSError:
                return False
        return False

    def poll(self, fd, i, in_alts):
        try:
            while True:
                ch = os.read(fd, 4096)
                if not ch:
                    break
                self._feed(ch)
        except BlockingIOError:
            pass
        except OSError:
            pass
        self.n_tx = i + 1
        if i < len(in_alts):
            self.in_alt = in_alts[i]
            self.in_vvel = ((in_alts[i] - in_alts[i - 1]) / 0.1
                            if i > 0 else 0.0)
            self.in_t = i * 0.1
        if time.monotonic() - self._last_draw > 0.2:
            self._last_draw = time.monotonic()
            self.draw()

    def _feed(self, chunk):
        data = self._buf + chunk
        i = 0
        while i + 6 <= len(data):
            if (data[i] == 0xAA and data[i + 4] == 0x0D
                    and data[i + 5] == 0x0A
                    and ((data[i + 1] + data[i + 2]) & 0xFF)
                    == data[i + 3]):
                self.words.append((data[i + 1] << 8) | data[i + 2])
                self.n_rx += 1
                i += 6
            else:
                i += 1
        self._buf = data[max(0, i - 5):]

    def phase(self):
        w = self.words[-1] if self.words else 0
        if w & 0x80:
            return "MAIN"
        if w & 0x20:
            return "DROGUE"
        if w & 0x10:
            return "APOGEE"
        if w & 0x01:
            return "LAUNCH"
        return "WAIT"

    def draw(self):
        if not self._on:
            return
        w = self.words[-1] if self.words else 0
        rows = [
            "SUT LIVE",
            "-----------------------------",
            "INPUT  T     %.1f s" % self.in_t,
            "INPUT  ALT   %.1f m" % self.in_alt,
            "INPUT  VVEL  %+.1f m/s" % self.in_vvel,
            "",
            "BOARD  STATE %s" % self.phase(),
            "BOARD  STATUS 0x%04X" % w,
            "BOARD  DROGUE %s  MAIN %s"
            % ("YES" if w & 0x20 else "NO",
               "YES" if w & 0x80 else "NO"),
            "-----------------------------",
            "Frames TX:%d RX:%d" % (self.n_tx, self.n_rx),
        ]
        sys.stdout.write("\033[H" + "\n".join(rows) + "\n")
        sys.stdout.flush()

    def finish(self, fd):
        t0 = time.time()
        while time.time() - t0 < 1.0:
            try:
                ch = os.read(fd, 4096)
                if ch:
                    self._feed(ch)
                else:
                    time.sleep(0.01)
            except (BlockingIOError, OSError):
                break
        self.draw()
        order = {}
        for name, bit in (("launch", 0), ("desc", 4), ("drogue", 5),
                          ("main", 7)):
            seen = [x for x in self.words if x & (1 << bit)]
            order[name] = self.words.index(seen[0]) if seen else None
        vals = [v for v in order.values() if v is not None]
        ok = vals == sorted(vals) and len(vals) == 4
        self.stop()
        print("TX frames = %d" % self.n_tx)
        print("RX frames = %d" % self.n_rx)
        for k, v in order.items():
            print("%s first-idx = %s" % (k, v))
        print("order launch<desc<drogue<main: %s" % ("PASS" if ok else "FAIL"))


def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="examples:\n"
               "  python tests/hil_send.py tests/traces/ascent.csv\n"
               "  python tests/hil_send.py --trace tests/traces/ascent.csv --live")
    ap.add_argument("trace", nargs="?")
    ap.add_argument("--trace", dest="trace_opt", default=None,
                    help="same as the positional trace argument")
    ap.add_argument("--port", default=None)
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--gen-ascent", default=None)
    ap.add_argument("--live", action="store_true",
                    help="stream + decode board status live (ANSI block)")
    args = ap.parse_args()
    if args.gen_ascent:
        gen_ascent(args.gen_ascent)
        return 0
    trace = args.trace_opt or args.trace
    if trace is None:
        ap.error("a trace CSV is required: "
                 "python tests/hil_send.py tests/traces/ascent.csv")
    rows = list(csv.DictReader(open(trace)))
    p0 = float(rows[0]["press_pa"])
    frames = []
    for r in rows:
        if int(r["valid"]) != 1:
            continue
        p = float(r["press_pa"])
        frames.append(synth_frame(baro_alt(p, p0), p / 100.0,
                                  float(r["ax"]), float(r["ay"]),
                                  float(r["az"]), float(r["ex"]),
                                  float(r["ey"]), float(r["ez"])))
    print("frames=%d p0=%.0f" % (len(frames), p0))
    if args.dry:
        return 0
    port = args.port or discover_port()
    if not port:
        print("no serial port")
        return 2
    # input altitudes (for INPUT ALT/VVEL display; BOARD values come back
    # over UART and are never mixed with these)
    in_alts = []
    for r in rows:
        if int(r["valid"]) != 1:
            continue
        in_alts.append(baro_alt(float(r["press_pa"]), p0))
    fd = open_port(port)
    live = LiveHil() if args.live else None
    try:
        if live:
            live.start()
            if not live.wait_ready(fd):
                print("board not in WAIT_FOR_LAUNCH; reset first")
                return 3
        os.write(fd, bytes([0xAA, 0x22, 0x8E, 0x0D, 0x0A]))  # SUT start
        time.sleep(1.6)
        t0 = time.time()
        for i, fr in enumerate(frames):
            os.write(fd, fr)
            while time.time() < t0 + 0.1 * (i + 1):
                if live:
                    live.poll(fd, i, in_alts)
                else:
                    time.sleep(0.005)
        os.write(fd, bytes([0xAA, 0x24, 0x90, 0x0D, 0x0A]))  # stop
        if live:
            live.finish(fd)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if live:
            live.stop()
        os.close(fd)
    print("streamed, stop sent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
