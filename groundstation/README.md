# BOZOK Ground Station

## Başlatma
```bash
cd groundstation/backend
uvicorn app:app --host 127.0.0.1 --port 8000
cd ../frontend
npm run dev -- --port 5173 --host 127.0.0.1   # tarayıcı: http://127.0.0.1:5173
```
## Durdurma
Her iki terminalde `Ctrl-C`.

## Uno bağlantısı
Ayarlar sekmesi → port seç (örn. `/dev/ttyUSB0`) → Bağlan (115200).

## Test
```bash
cd groundstation/backend && python3 - <<'EOF'
import app
app.apply_link(["[LINK] seq=1 t=100ms", "  flight=3  ek7mode=0  alt=100m  vel=500cm/s",
 "  sats=7  batt=--mV  fix=Y LED=---", "  pos=41 00.73  29 05.89"])
assert app.state["packets"] == 1 and app.state["alt_m"] == 100
print("BACKEND TEST PASS")
EOF
cd ../frontend && npx tsc --noEmit && npm run build
```

## Mimari
`USB(115200 Uno metni) → backend decoder → state → WS → React UI`. Kayıt JSONL (`backend/sessions/`), export CSV. Harita OSM (çevrimiçi gerekli), 3D prosedürel (yönelim yok → nötr poz etiketi). Tanıtım videosu `frontend/public/intro.mp4`.
