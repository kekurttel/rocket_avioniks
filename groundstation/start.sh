#!/bin/bash
# BOZOK Ground Station başlatma: backend (:8000) + frontend (:5173).
# Kullanim: ./start.sh  (bu dosyanin bulundugu dizinden)
set -u
GS="$(cd "$(dirname "$0")" && pwd)"
LOG="$GS/logs"
mkdir -p "$LOG"

if lsof -i :8000 >/dev/null 2>&1 || lsof -i :5173 >/dev/null 2>&1; then
  echo "HATA: 8000 veya 5173 portu dolu. Once ./stop.sh calistirin."
  exit 1
fi

cd "$GS/backend" || exit 1
setsid nohup uvicorn app:app --host 127.0.0.1 --port 8000 >"$LOG/backend.log" 2>&1 < /dev/null &
echo $! > "$LOG/backend.pid"

cd "$GS/frontend" || exit 1
setsid nohup npm run dev -- --port 5173 --host 127.0.0.1 >"$LOG/frontend.log" 2>&1 < /dev/null &
echo $! > "$LOG/frontend.pid"

sleep 4
echo "--- backend ---"
curl -s -o /dev/null -w "8000: %{http_code}\n" http://127.0.0.1:8000/api/ports || echo "8000: ERISILEMIYOR"
echo "--- frontend ---"
curl -s -o /dev/null -w "5173: %{http_code}\n" http://127.0.0.1:5173/ || echo "5173: ERISILEMIYOR"
echo "Tarayici: http://127.0.0.1:5173  (loglar: $LOG/)"
