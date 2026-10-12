#!/bin/bash
# BOZOK Ground Station durdurma: start.sh ile acilan backend + frontend.
# Kullanim: ./stop.sh
set -u
GS="$(cd "$(dirname "$0")" && pwd)"
LOG="$GS/logs"

kill_by_pidfile() {
  if [ -f "$LOG/$1" ]; then
    PID="$(cat "$LOG/$1")"
    if kill -0 "$PID" 2>/dev/null; then
      kill "$PID" 2>/dev/null && echo "$2 durduruldu (pid $PID)."
    else
      echo "$2 zaten calismiyor."
    fi
    rm -f "$LOG/$1"
  else
    echo "$2 pid dosyasi yok, porta gore temizleniyor."
  fi
}

kill_by_pidfile backend.pid "Backend (:8000)"
kill_by_pidfile frontend.pid "Frontend (:5173)"
pkill -f "uvicorn app:app" 2>/dev/null
pkill -f "vite.*--port 5173" 2>/dev/null
sleep 1
if lsof -i :8000 >/dev/null 2>&1; then echo "UYARI: 8000 hala dolu."; else echo "8000 bos."; fi
if lsof -i :5173 >/dev/null 2>&1; then echo "UYARI: 5173 hala dolu."; else echo "5173 bos."; fi
