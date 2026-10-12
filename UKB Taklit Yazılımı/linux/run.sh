#!/usr/bin/env bash
# O-UKB Test Cihazi Taklit Yazilimi (Linux)
set -euo pipefail
cd "$(dirname "$0")"

[ -x .venv/bin/python ] || { echo "Once ./install.sh calistir." >&2; exit 1; }
[ -d app ] || python3 extract.py

exec .venv/bin/python launcher.py "$@"
