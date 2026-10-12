#!/usr/bin/env python3
"""
tek_port_test.py — Arduino'yu tek port acilisiyla reset + oku.

NEDEN: CH340'da portu KAPATmak DTR'i dusurur ve Arduino'yu tekrar
reset eder. "cat ile dinle + python ile reset" yontemi iki kez reset
uretir ve teşhis ciktisi kaybolur. Burada port TEK sefer acilir:
  1) DTR pulse -> reset
  2) ayni porttan 18 sn oku
Yalnizca kapanma aninda DTR bir kez daha dusur -> kart bir kez daha
resetlenir, ama o zaman zaten veri alinmis olur.
"""
import serial
import sys
import time

PORT = "/dev/ttyUSB0"
WAIT = float(sys.argv[1]) if len(sys.argv) > 1 else 18.0


def main():
    # 1) Portu ac, tamponu BOSALT: eski baytlar kalmasin.
    s = serial.Serial(PORT, 115200, timeout=0.2)
    s.reset_input_buffer()
    s.reset_output_buffer()
    s.dtr = False
    s.rts = False
    time.sleep(0.3)
    s.reset_input_buffer()

    # 2) Auto-reset: CH340'da RTS birlikte DTR ile bootloadera sokar.
    #    DTR tek basina bazi klonlarda yalnizca 330ms'lik bir darbe uretir ve
    #    programin basinda degil, ortasinda reset yapar. Bu yuzden iki kez
    #    deneriz ve her seferinde tamponu temizleriz.
    for attempt in (1, 2):
        s.dtr = False
        s.rts = True
        time.sleep(0.02)
        s.dtr = True
        s.rts = False
        time.sleep(0.10)
        s.setDTR(False)
        time.sleep(0.40)
        s.reset_input_buffer()
        sys.stderr.write(f"[reset denemesi {attempt}]\n")
        sys.stderr.flush()

    # 3) Artik yalnizca oku. Portu ARADA KAPATMA (kapama = ikinci reset).
    t0 = time.time()
    out = bytearray()
    while time.time() - t0 < WAIT:
        try:
            chunk = s.read(4096)
        except serial.SerialException as e:
            sys.stderr.write(f"[ seri hatasi: {e} ]\n")
            break
        if chunk:
            out += chunk
        time.sleep(0.02)

    t0 = time.time()
    out = bytearray()
    while time.time() - t0 < WAIT:
        try:
            chunk = s.read(4096)
        except serial.SerialException as e:
            sys.stderr.write(f"[ seri hatasi: {e} ]\n")
            break
        if chunk:
            out += chunk
        # portu ASLA kapatma; DTR'i yuksek tut ki reset tekrar tetiklenmesin
        time.sleep(0.02)

    txt = bytes(out).decode("utf-8", "replace")
    txt = txt.replace("\x00", "")
    sys.stdout.write(txt)
    sys.stderr.write(f"\n[toplam {len(out)} bayt, {WAIT:.0f} sn]\n")
    try:
        s.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()