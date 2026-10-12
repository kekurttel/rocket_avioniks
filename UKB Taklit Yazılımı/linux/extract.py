#!/usr/bin/env python3
"""EXE'yi Linux icin acar.

PyInstaller onefile paketinden yalnizca uygulamanin kendi dosyalari cikarilir:
  app/ukb_tester_gui/**.pyc   arayuz koprusu (QML <-> Python)
  app/ukb_tester_node/**.pyc  Ek-7 protokolu, paketler, SUT test yukleyici
  app/qml, app/assets, app/sut_tests

Kaynak kod, QML ve CSV dosyalarina dokunulmaz; hepsi byte-byte ayni kopyalanir.
"""

import marshal
import struct
import sys
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "app"
COOKIE_MAGIC = b"MEI\x0c\x0b\x0a\x0b\x0e"
PY311_MAGIC = b"\xa7\r\r\n"  # importlib.util.MAGIC_NUMBER (python3.11)
# PySide6 DLL'leri, .qm cevirileri ve base_library.zip atlanir: sistem Qt'si kullanilir.
DATA_PREFIXES = ("qml", "assets", "sut_tests")


def find_exe():
    exes = sorted(
        (p for p in HERE.parent.glob("*.exe") if p.stat().st_size > 10_000_000),
        key=lambda p: p.stat().st_mtime,
    )
    if not exes:
        sys.exit("Hata: bir ust klasorde EXE bulunamadi")
    return exes[-1]


def read_carchive(blob):
    """CArchive cookie + TOC -> (entries, package_start, python_version, pylib)"""
    i = blob.rfind(COOKIE_MAGIC)
    if i < 0:
        sys.exit("Hata: PyInstaller paketi bulunamadi")
    _, pkg_len, toc, toc_len, pyver, pylib = struct.unpack("!8sIIii64s", blob[i : i + 88])
    start = len(blob) - pkg_len
    off, end, entries = start + toc, start + toc + toc_len, []
    while off < end:
        size, pos, csize, usize, compressed, tcode = struct.unpack(
            "!iIIIBc", blob[off : off + 18]
        )
        name = blob[off + 18 : off + size].split(b"\0")[0].decode()
        entries.append((name, pos, csize, usize, compressed, tcode.decode()))
        off += size
    return entries, start, pyver, pylib.split(b"\0")[0].decode()


def write_pyz(pyz, magic):
    """PYZ icindeki ukb_* modullerini sourceless .pyc olarak yazar."""
    header = magic + struct.pack("<III", 0, 0, 0)  # 16 byte pyc basligi (PEP 552)
    count = 0
    toc = marshal.loads(pyz[struct.unpack("!i", pyz[8:12])[0] :])
    for name, (is_pkg, pos, length) in toc:
        if not name.startswith("ukb"):
            continue
        rel = Path(*name.split("."))
        target = OUT / rel / "__init__.pyc" if is_pkg else OUT / rel.with_suffix(".pyc")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(header + zlib.decompress(pyz[pos : pos + length]))
        count += 1
    return count


def main():
    exe = find_exe()
    blob = exe.read_bytes()
    entries, start, pyver, pylib = read_carchive(blob)
    OUT.mkdir(parents=True, exist_ok=True)

    magic, pyc_count, data_count = None, 0, 0
    for name, pos, csize, usize, compressed, tcode in entries:
        raw = blob[start + pos : start + pos + csize]
        payload = zlib.decompress(raw) if compressed else raw
        if tcode == "z":
            magic = payload[4:8]  # PYZ: 'PYZ\0' + 4 byte pyc magic
            pyc_count = write_pyz(payload, magic)
        elif tcode == "b":
            rel = name.replace("\\", "/")
            if rel.split("/")[0] in DATA_PREFIXES:
                dst = OUT / rel
                dst.parent.mkdir(parents=True, exist_ok=True)
                dst.write_bytes(payload)
                data_count += 1

    checks = [
        (magic == PY311_MAGIC, f"bytecode surumu 3.11 ({magic!r})"),
        ((OUT / "ukb_tester_gui/backend/app_controller.pyc").exists(), "app_controller"),
        ((OUT / "ukb_tester_node/master.pyc").exists(), "protokol/master"),
        ((OUT / "qml/main.qml").exists(), "qml/main.qml"),
        (any((OUT / "sut_tests").glob("*.csv")), "sut_tests/*.csv"),
        (data_count > 20, f"veri dosyasi ({data_count})"),
    ]
    for ok, label in checks:
        print(("  OK   " if ok else "  HATA ") + label)
    print(f"\n{exe.name} -> {OUT}")
    print(f"Python {pyver} ({pylib}) | {pyc_count} modul | {data_count} dosya")
    return 0 if all(ok for ok, _ in checks) else 1


if __name__ == "__main__":
    sys.exit(main())
