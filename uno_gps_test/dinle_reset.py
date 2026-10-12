#!/usr/bin/env python3
"""
dinle_reset.py — Portu AC, hazir ol, sonra butondan RESET'i bekle.

NEDEN: CH340 auto-reset bu klonda guvenilir degil. Kart, portu acildigi
anda ICINDE yazili programi (power-on reset) calistirir; ama reset
aninda veri kacirilir. Bu yuzden:
  1) port acilir ve BOSALTILIR (eski bayt yok)
  2) kullanici RESET butonuna basar
  3) port AYNEN ACIK kalir, sadece okunur -> hicbir veri kacmaz
Portu sadece en son kapatiyoruz.

Kullanim:  python3 dinle_reset.py 20
"""
import serial
import sys
import time

PORT = "/dev/ttyUSB0"
WAIT = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0


def main():
    s = serial.Serial(PORT, 115200, timeout=0.15)
    s.reset_input_buffer()
    s.reset_output_buffer()
    s.dtr = False
    s.rts = False
    time.sleep(0.4)
    s.reset_input_buffer()   # power-on reset ciktisini at

    sys.stderr.write("*** PORT HAZIR. RESET butonuna BAS ***\n")
    sys.stderr.write(f"*** {WAIT:.0f} saniye dinleniyor ***\n")
    sys.stderr.flush()

    t0 = time.time()
    out = bytearray()
    while time.time() - t0 < WAIT:
        try:
            chunk = s.read(4096)
        except serial.SerialException as e:
            sys.stderr.write(f"[hata: {e}]\n")
            break
        if chunk:
            out += chunk
            sys.stderr.write(f"[alinan: {len(out)} bayt]\n")
            sys.stderr.flush()
        time.sleep(0.02)

    txt = bytes(out).decode("utf-8", "replace").replace("\x00", "")
    sys.stdout.write(txt)
    sys.stderr.write(f"\n[toplam {len(out)} bayt]\n")
    try:
        s.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()