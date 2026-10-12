#!/usr/bin/env python3
"""Ek-7 USB-UART SIT/SUT runner (stdlib only).

Transport: ST-LINK VCOM 115200 8N1 (Ek-7 Tablo 7; RS232 seviyesi yok,
task karariyla ST-LINK VCOM kullanilir). Firmware TX-agirlikli; RX yalnizca
5 bayt Ek-7 komutlari + SUT sentetik cerceveler icin dinlenir.

Test ID'leri Ek-7 dokuman bolumlerine baglidir:
  EK7-SIT-01..09 : SİT senaryosu (Ek-7 §2.1.1, Tablo 2/3)
  EK7-SIT-10     : SİT komut+paket protokolu (Ek-7 Tablo 1/3)
  EK7-SUT-20/21  : SUT senaryosu (Ek-7 §2.1.3, Tablo 4/5/6)
El gerektiren maddeler her zaman NOT EXECUTED (asla uydurma PASS yok).

Usage:
  python3 tests/sit_runner.py [--port PATH] [--out DIR]
"""
import argparse
import glob
import os
import re
import struct
import subprocess
import sys
import termios
import time
import tty

CMD_SIT = bytes([0xAA, 0x20, 0x8C, 0x0D, 0x0A])
CMD_SUT = bytes([0xAA, 0x22, 0x8E, 0x0D, 0x0A])
CMD_STOP = bytes([0xAA, 0x24, 0x90, 0x0D, 0x0A])


def discover_port():
    cands = sorted(glob.glob("/dev/serial/by-id/*STLink*"))
    if cands:
        return cands[0]
    cands = sorted(glob.glob("/dev/ttyACM*"))
    return cands[0] if cands else None


class Serial:
    def __init__(self, path, sink=None):
        self.sink = sink
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            tty.setraw(self.fd)
            attr = termios.tcgetattr(self.fd)
            attr[4] = attr[5] = termios.B115200
            attr[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
            attr[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
            attr[0] &= ~(termios.IXON | termios.IXOFF | termios.ICRNL)
            attr[1] &= ~termios.OPOST
            attr[3] &= ~(termios.ECHO | termios.ICANON | termios.ISIG)
            termios.tcsetattr(self.fd, termios.TCSANOW, attr)
        except Exception:
            os.close(self.fd)
            raise

    def drain(self):
        try:
            while os.read(self.fd, 4096):
                pass
        except (BlockingIOError, OSError):
            pass

    def write(self, data):
        os.write(self.fd, data)

    def read_raw(self, deadline):
        out = b""
        while time.time() < deadline:
            try:
                chunk = os.read(self.fd, 4096)
                if chunk:
                    out += chunk
                    if self.sink:
                        self.sink(chunk)
                else:
                    time.sleep(0.005)
            except BlockingIOError:
                time.sleep(0.005)
            except OSError:
                break
        return out

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


class Ctx:
    def __init__(self, outdir):
        self.outdir = outdir
        os.makedirs(outdir, exist_ok=True)
        self.results = []
        self.boot_text = ""

    def log_run(self, tid, data):
        if isinstance(data, bytes):
            with open(os.path.join(self.outdir, tid + ".bin"), "wb") as f:
                f.write(data)
        else:
            with open(os.path.join(self.outdir, tid + ".log"), "w") as f:
                f.write("\n".join(data) + "\n")

    def verdict(self, tid, name, status, detail=""):
        self.results.append((tid, name, status, detail))
        if LIVE:
            LIVE.pause()
        print("%-12s %-20s %s %s" % (tid, name, status, detail),
              flush=True)
        if LIVE:
            LIVE.resume()


def cap_text(ser, seconds):
    t0 = time.time()
    lines = []
    buf = b""
    raw = ser.read_raw(t0 + seconds)
    for chunk in [raw[i:i + 1] for i in range(len(raw))]:
        buf += chunk
        if buf.endswith(b"\n"):
            lines.append("[%7.2f] %s" % (
                time.time() - t0,
                buf.decode("utf-8", "replace").rstrip("\r\n")))
            buf = b""
    return "\n".join(lines)


def has(text, pat):
    return re.search(pat, text) is not None


LIVE = None  # set from --live; single-process serial owner stays here


class LiveView:
    """Compact ANSI live monitor (text TUI + Ek-7 binary, resync-safe).

    One owner: fed from the single Serial.read_raw sink. Binary parse uses
    the same header/checksum/footer rules as the test assertions below
    (existing resync logic untouched). Ctrl-C safe via stop()."""

    FIELDS = (
        ("Basınç: (\\d+) Pa", "PRESSURE", lambda m: m.group(1) + " Pa"),
        ("İrtifa: ([+-][\\d.]+) m", "ALTITUDE", lambda m: m.group(1) + " m"),
        ("Açı: ([\\d.]+) deg", "TILT", lambda m: m.group(1) + " deg"),
        ("TILT_30_45: (ON|OFF)", "TILT_30_45",
         lambda m: m.group(1)),
        ("PHASE: (\\w+)", "PHASE", lambda m: m.group(1)),
        ("FLIGHT: ([A-Z_]+)", "STATE", lambda m: m.group(1)),
        ("MAX ALTITUDE: ([+-][\\d.]+|INVALID)", "MAX ALT",
         lambda m: m.group(1)),
        ("VERTICAL VELOCITY: ([+-][\\d.]+ m/s|INVALID)", "VVEL",
         lambda m: m.group(1)),
        ("VERT_ACC: (-?\\d+ mg|INVALID)", "VERT ACC",
         lambda m: m.group(1)),
        ("NEG_ACCEL: (ON|OFF)", "NEG", lambda m: m.group(1)),
        ("NEG_ACCEL_COUNT: (\\d+)", "NEGC", lambda m: m.group(1)),
        ("DESCENT_DETECTED: (YES|NO)", "DESCENT",
         lambda m: m.group(1)),
        ("BARO_VVEL: ([+-][\\d.]+ m/s|INVALID)", "BARO VEL",
         lambda m: m.group(1)),
        ("BARO_DROP: (ON|OFF)", "BARO DROP", lambda m: m.group(1)),
        ("BENCH_FALL: (YES|NO)", "BENCH FALL", lambda m: m.group(1)),
        ("SENSOR_HEALTH: MPU=(\\w+) BMP1=(\\w+) BMP2=(\\w+) FUSED=(\\S+)",
         "HEALTH", lambda m: "MPU %s B1 %s B2 %s F %s" % m.groups()),
        ("DEPLOY: DROGUE=([01]) MAIN=([01])", "DEPLOY",
         lambda m: "D%s M%s" % m.groups()),
    )

    def __init__(self):
        self.d = {}
        self.src = "-"
        self.n_tlm = 0
        self.n_status = 0
        self.n_text = 0
        self._tail = b""
        self._line = b""
        self._evts = []
        self._last_draw = 0.0
        self._on = False

    def start(self):
        sys.stdout.write("\033[2J\033[H\033[?25l")
        sys.stdout.flush()
        self._on = True

    def pause(self):
        if self._on:
            sys.stdout.write("\033[?25h\n")
            sys.stdout.flush()

    def resume(self):
        if self._on:
            sys.stdout.write("\033[?25l")
            sys.stdout.flush()

    def stop(self):
        if self._on:
            sys.stdout.write("\033[?25h\n")
            sys.stdout.flush()
            self._on = False

    def _rate(self):
        now = time.monotonic()
        self._evts = [t for t in self._evts if now - t < 1.0]
        return len(self._evts)

    def feed(self, chunk):
        for byte in chunk:
            b = bytes([byte])
            if b == b"\n":
                self._text_line(self._line)
                self._line = b""
            else:
                self._line += b
        data = self._tail + chunk
        i = 0
        while i + 6 <= len(data):
            if (data[i] == 0xAB and i + 36 <= len(data)
                    and data[i + 34] == 0x0D and data[i + 35] == 0x0A
                    and (sum(data[i + 1:i + 33]) & 0xFF)                     == data[i + 33]):
                v = struct.unpack(">8f", bytes(data[i + 1:i + 33]))
                self.d["ALTITUDE"] = "%.1f m" % v[0]
                self.d["PRESSURE"] = "%.0f Pa" % (v[1] * 100.0)
                self.d["ACC"] = "%.1f/%.1f/%.1f" % (v[2], v[3], v[4])
                self.d["TILT"] = "%.1f/%.1f deg" % (v[5], v[6])
                self.n_tlm += 1
                self._evts.append(time.monotonic())
                self.src = "BIN-TLM"
                i += 36
            elif (data[i] == 0xAA and i + 6 <= len(data)
                    and data[i + 4] == 0x0D and data[i + 5] == 0x0A
                    and ((data[i + 1] + data[i + 2]) & 0xFF)
                    == data[i + 3]):
                w = (data[i + 1] << 8) | data[i + 2]
                bits = []
                for bit, tag in ((0, "L"), (2, "A"), (3, "T"), (4, "D"),
                                 (5, "d"), (6, "m"), (7, "M")):
                    if w & (1 << bit):
                        bits.append(tag)
                self.d["STATUS"] = "0x%04X [%s]" % (w, "".join(bits))
                self.n_status += 1
                self._evts.append(time.monotonic())
                self.src = "BIN-STATUS"
                i += 6 if (data[i] == 0xAA) else 1
            else:
                i += 1
        self._tail = data[max(0, i - 41):]
        if time.monotonic() - self._last_draw > 0.2:
            self._last_draw = time.monotonic()
            self.draw()

    def _text_line(self, raw):
        try:
            line = raw.decode("utf-8", "replace")
        except Exception:
            return
        hit = False
        for pat, key, fmt in self.FIELDS:
            m = re.search(pat, line)
            if m:
                self.d[key] = fmt(m)
                hit = True
        if hit:
            self.n_text += 1
            self._evts.append(time.monotonic())
            self.src = "TEXT"

    def draw(self):
        if not self._on:
            return
        g = lambda k: self.d.get(k, "?")
        rows = [
            "SIT LIVE  src=%s rate=%d/s" % (self.src, self._rate()),
            "-----------------------------",
            "STATE       %s" % g("STATE"),
            "PRESSURE    %s" % g("PRESSURE"),
            "ALTITUDE    %s" % g("ALTITUDE"),
            "ACC         %s" % g("ACC"),
            "TILT        %s  [%s]" % (g("TILT"), g("TILT_30_45")),
            "VERT ACC    %s  NEG %s n=%s" % (g("VERT ACC"), g("NEG"),
                                            g("NEGC")),
            "BARO VEL    %s  DROP %s" % (g("BARO VEL"), g("BARO DROP")),
            "DESCENT     %s" % g("DESCENT"),
            "BENCH FALL  %s" % g("BENCH FALL"),
            "MAX ALT     %s   VVEL %s" % (g("MAX ALT"), g("VVEL")),
            "HEALTH      %s" % g("HEALTH"),
            "DEPLOY      %s" % g("DEPLOY"),
            "STATUS      %s" % g("STATUS"),
            "-----------------------------",
            "Frames TLM:%d ST:%d TXT:%d" % (self.n_tlm, self.n_status,
                                           self.n_text),
        ]
        sys.stdout.write("\033[H" + "\n".join(rows) + "\n")
        sys.stdout.flush()


def parse_tlm_frames(raw):
    """36B 0xAB frames with checksum; returns list of 8-float tuples."""
    frames = []
    i = 0
    while i + 36 <= len(raw):
        if raw[i] == 0xAB and raw[i + 34] == 0x0D and raw[i + 35] == 0x0A:
            if (sum(raw[i + 1:i + 33]) & 0xFF) == raw[i + 33]:
                vals = struct.unpack(">8f", bytes(raw[i + 1:i + 33]))
                if all(abs(v) < 1e6 for v in vals):
                    frames.append(vals)
                    i += 36
                    continue
        i += 1
    return frames


def parse_status_frames(raw):
    """6B 0xAA D1 D2 SUM 0D 0A status frames; returns list of words."""
    words = []
    i = 0
    while i + 6 <= len(raw):
        if (raw[i] == 0xAA and raw[i + 4] == 0x0D and raw[i + 5] == 0x0A
                and ((raw[i + 1] + raw[i + 2]) & 0xFF) == raw[i + 3]):
            words.append((raw[i + 1] << 8) | raw[i + 2])
            i += 6
        else:
            i += 1
    return words


def synth_frame(alt, press_mb, ax, ay, az, ex=0.0, ey=0.0, ez=0.0):
    body = struct.pack(">8f", round(alt, 2), round(press_mb, 2),
                       round(ax, 2), round(ay, 2), round(az, 2),
                       round(ex, 2), round(ey, 2), round(ez, 2))
    return b"\xAB" + body + bytes([(sum(body) & 0xFF), 0x0D, 0x0A])


def run(ser, ctx, port):
    ctx.verdict("EK7-SIT-01", "PORT OPEN", "PASS", port)

    # EK7-SIT-02 boot (§2.1.2: reset + banner)
    try:
        subprocess.run(["st-flash", "reset"], timeout=20,
                       capture_output=True)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        ctx.verdict("EK7-SIT-02", "RESET+BOOT", "NOT EXECUTED",
                    "st-flash missing")
        return
    time.sleep(0.2)
    ser.drain()
    text = cap_text(ser, 6)
    ctx.log_run("EK7-SIT-02", text.split("\n"))
    ok = has(text, r"SYSTEM INIT") and has(text, r"CALIBRATION START")
    det = "banner found" if ok else "banner missing"
    if not ok and has(text, r"AVIONICS WAITING FOR RESET"):
        det = "idle-waiting-NRST"
    ctx.verdict("EK7-SIT-02", "RESET+BOOT", "PASS" if ok else "FAIL", det)
    ctx.boot_text = text

    # EK7-SIT-03 health (§2.1.1: sensor identification)
    text = cap_text(ser, 22)
    ctx.log_run("EK7-SIT-03", text.split("\n"))
    blob = ctx.boot_text + "\n" + text
    mpu = has(blob, r"WHO_AM_I: HAL_OK value=0x68")
    bmp = has(blob, r"ID=0x58") and has(blob, r"BMP1 P0: \d+Pa")
    ms = has(blob, r"BMP2 ERROR") or has(blob, r"BMP2 P0: \d+Pa")
    ok = mpu and bmp and ms
    ctx.verdict("EK7-SIT-03", "SENSOR HEALTH",
                "PASS" if ok else "FAIL",
                "MPU=%s BMP1=%s BMP2-honest=%s" % (mpu, bmp, ms))

    # EK7-SIT-04 cal + P0 fusion
    cal = has(blob, r"CALIBRATION DONE")
    m = re.search(r"CAL: P0=(\d+)/(\d+)Pa FUSED=(\d+)Pa", blob)
    fok, fnote = False, "no CAL line"
    if m:
        b1, ms, fused = int(m.group(1)), int(m.group(2)), int(m.group(3))
        if ms == 0:
            fok = (fused == b1) and fused < 105000
            fnote = "B2-invalid: FUSED=%d B1=%d" % (fused, b1)
        else:
            fok = abs(fused - (b1 + ms) // 2) <= 2 and fused < 105000
            fnote = "dual: FUSED=%d B1=%d B2=%d" % (fused, b1, ms)
    ctx.verdict("EK7-SIT-04", "CAL+P0 FUSION",
                "PASS" if (cal and fok) else "FAIL",
                "cal=%s %s" % (cal, fnote))

    # EK7-SIT-05 lifecycle + config
    wait = has(blob, r"WAITING FOR LAUNCH")
    m = re.search(r"FLIGHT CONFIG: (PRODUCTION|TEST_MODE=1)", blob)
    cfg = m.group(1) if m else "?"
    bench = re.search(r"BENCH: (ARMED|OFF)", blob)
    bok = bench and ((cfg == "TEST_MODE=1") == (bench.group(1) == "ARMED"))
    nf = not has(blob, r"FLIGHT FAULT")
    ctx.verdict("EK7-SIT-05", "WAIT_FOR_LAUNCH",
                "PASS" if (wait and m and bok and nf) else "FAIL",
                "cfg=%s bench-consistent=%s nofault=%s" % (cfg, bok, nf))

    # EK7-SIT-06 quiet soak (stability, §2.1.1)
    text = cap_text(ser, 15)
    ctx.log_run("EK7-SIT-06", text.split("\n"))
    bad = [p for p in (r"DUSUS ALGILANDI", r"APOGEE CONFIRMED",
                       r"DROGUE DEPLOY EVENT", r"MAIN DEPLOY EVENT",
                       r"FLIGHT FAULT", r"LAUNCH DETECTED") if has(text, p)]
    vacc = [int(x) for x in re.findall(r"VERT_ACC: (-?\d+) mg", text)]
    vmax = max((abs(v) for v in vacc), default=None)
    vv = [float(x) for x in
          re.findall(r"BARO_VVEL: ([+-]?[0-9.]+) m/s", text)]
    vmin = min(vv, default=None)
    ok = (not bad) and (vmax is not None) and vmax < 100
    ctx.verdict("EK7-SIT-06", "QUIET SOAK",
                "PASS" if ok else "FAIL",
                "events=%s n_vacc=%d max|vacc|=%s mg min_vvel=%s"
                % (bad if bad else "none", len(vacc), vmax, vmin))

    # EK7-SIT-07/08/09 motion (Ek-7 §2.1.2: 2-3 m hareket) -> hands
    for tid, name in (("EK7-SIT-07", "UP-DOWN MOTION"),
                      ("EK7-SIT-08", "TILT-ONLY"),
                      ("EK7-SIT-09", "REPEAT NO-DUP")):
        ctx.verdict(tid, name, "NOT EXECUTED", "requires human hands")

    # EK7-SIT-10 official SIT command + binary telemetry (Tablo 1/3)
    ser.drain()
    ser.write(CMD_SIT)
    raw = ser.read_raw(time.time() + 4.5)
    ctx.log_run("EK7-SIT-10", raw)
    frames = parse_tlm_frames(raw)
    okf = False
    fnote = "frames=%d" % len(frames)
    if len(frames) >= 15:
        alts = [f[0] for f in frames]
        prs = [f[1] for f in frames]
        accs = [(f[2] ** 2 + f[3] ** 2 + f[4] ** 2) ** 0.5 for f in frames]
        okf = (all(abs(a) < 5 for a in alts)
               and all(900 < p < 1100 for p in prs)
               and all(8.0 < m < 11.5 for m in accs))
        fnote = ("frames=%d alt~%.1f press~%.0f |a|~%.1f"
                 % (len(frames), alts[-1], prs[-1], accs[-1]))
    # rate check ~10 Hz from inter-frame timing density
    ctx.verdict("EK7-SIT-10", "SIT CMD+TLM",
                "PASS" if okf else "FAIL", fnote)
    ser.write(CMD_STOP)
    time.sleep(0.6)
    ser.drain()
    text = cap_text(ser, 2)
    back = has(text, r"UP: ")
    ctx.verdict("EK7-SIT-11", "STOP+NORMAL",
                "PASS" if back else "FAIL",
                "text-TUI resumed" if back else "no TUI after STOP")

    # EK7-SUT-21 abnormal data (Ek-7: hata yonetimi) -> rejected frames
    # never feed the algorithm: no state advance, no deployment, status alive.
    # Runs FIRST so no latched deploy bits pollute the observation.
    ser.drain()
    ser.write(CMD_SUT)
    time.sleep(1.6)  # Ek-7: activation 1 s after valid command
    for _ in range(5):  # valid keepalive (also feeds watchdog)
        ser.write(synth_frame(0.0, 1000.0, 0.0, 0.0, -9.81))
        time.sleep(0.1)
    for _ in range(5):  # abnormal: rejected, watchdog NOT fed
        ser.write(synth_frame(50000.0, 99999.0, 5000.0, 0.0, 0.0))
        time.sleep(0.1)
    raw = ser.read_raw(time.time() + 0.7)
    ctx.log_run("EK7-SUT-21", raw)
    words = parse_status_frames(raw)
    nodeploy = not any((w & 0xA0) for w in words)  # drogue/main bits
    ok = len(words) >= 4 and nodeploy
    ctx.verdict("EK7-SUT-21", "ABNORMAL DATA",
                "PASS" if ok else "FAIL",
                "status-alive=%s no-deploy-from-garbage=%s"
                % (len(words) >= 4, nodeploy))

    # EK7-SUT-20 synthetic flight (Tablo 4/5/6): rest, launch spike,
    # climb 0->8 m, fast descent, free-fall accel, below-main, stop.
    ser.drain()
    ser.write(CMD_SUT)
    time.sleep(1.6)  # Ek-7: activation 1 s after valid command
    prof = []
    prof += [(0.0, (0.0, 0.0, -9.81))] * 5
    prof += [(0.0, (0.0, 0.0, -14.7))] * 4  # launch spike 1.5 g
    a = 0.0
    for _ in range(20):  # climb to 8 m
        a += 0.4
        prof.append((round(a, 2), (0.0, 0.0, -9.81)))
    for _ in range(16):  # fast descent to 0 m (-5 m/s)
        a -= 0.5
        prof.append((round(a, 2), (0.0, 0.0, -9.81)))
    prof += [(a, (0.0, 0.0, -1.0))] * 5  # free-fall accel
    prof += [(a, (0.0, 0.0, -9.81))] * 10
    t0 = time.time()
    for i, (alt, acc) in enumerate(prof):
        ser.write(synth_frame(alt, 1000.0, acc[0], acc[1], acc[2]))
        while time.time() < t0 + 0.1 * (i + 1):
            time.sleep(0.005)
    raw = ser.read_raw(time.time() + 1.0)
    ctx.log_run("EK7-SUT-20", raw)
    words = parse_status_frames(raw)
    seen_launch = any(w & 0x01 for w in words)
    seen_desc = any(w & 0x10 for w in words)
    seen_dro = any(w & 0x20 for w in words)
    seen_main = any(w & 0x80 for w in words)
    ok = (len(words) >= 10 and seen_launch and seen_desc and seen_dro
          and seen_main)
    ctx.verdict("EK7-SUT-20", "SYNTH FLIGHT",
                "PASS" if ok else "FAIL",
                "status=%d launch=%s desc=%s drogue=%s main=%s"
                % (len(words), seen_launch, seen_desc, seen_dro, seen_main))
    ser.write(CMD_STOP)
    time.sleep(0.5)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="example:\n  python tests/sit_runner.py --live")
    ap.add_argument("--port", default=None)
    ap.add_argument("--out", default="test-results")
    ap.add_argument("--live", action="store_true",
                    help="compact ANSI live monitor (assertions unchanged)")
    args = ap.parse_args()
    port = args.port or discover_port()
    if not port:
        print("no serial port found")
        sys.exit(2)
    ctx = Ctx(args.out)
    if args.live:
        LIVE = LiveView()
        LIVE.start()
    try:
        ser = Serial(port, sink=(LIVE.feed if LIVE else None))
    except OSError as e:
        print("cannot open %s: %s" % (port, e))
        sys.exit(2)
    try:
        run(ser, ctx, port)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if LIVE:
            LIVE.stop()
        ser.close()
    np_ = sum(1 for r in ctx.results if r[2] == "PASS")
    nf_ = sum(1 for r in ctx.results if r[2] == "FAIL")
    ns_ = sum(1 for r in ctx.results if r[2] == "NOT EXECUTED")
    with open(os.path.join(args.out, "summary.txt"), "w") as f:
        for tid, name, st, det in ctx.results:
            f.write("%-12s %-20s %s %s\n" % (tid, name, st, det))
        f.write("PASS=%d FAIL=%d NOT_EXECUTED=%d\n" % (np_, nf_, ns_))
    print("PASS=%d FAIL=%d NOT_EXECUTED=%d" % (np_, nf_, ns_))
    sys.exit(1 if nf_ else 0)
