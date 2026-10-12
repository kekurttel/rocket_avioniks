#!/usr/bin/env bash
# EXE'yi acar, sanal ortami kurar, gerekli paketleri yukler.
set -euo pipefail
cd "$(dirname "$0")"

[ -d app ] || python3 extract.py

if command -v uv >/dev/null 2>&1; then
	echo "==> uv ile sanal ortam (Python 3.11)"
	[ -x .venv/bin/python ] || uv venv --python 3.11 .venv
	# Addons, QML'in kullandigi QtCharts modulunu iceriyor.
	uv pip install --python .venv/bin/python PySide6-Essentials PySide6-Addons pyserial
elif command -v python3.11 >/dev/null 2>&1; then
	echo "==> python3.11 -m venv"
	python3.11 -m venv .venv
	.venv/bin/pip install -q -U pip
	.venv/bin/pip install -q PySide6-Essentials PySide6-Addons pyserial
else
	echo "Hata: uv yok ve python3.11 kurulu degil." >&2
	echo "  cozum: uv kur (https://docs.astral.sh/uv/) ya da 'sudo pacman -S python3.11'" >&2
	exit 1
fi

echo
echo "Kurulum tamam. Baslatmak icin: ./run.sh"
