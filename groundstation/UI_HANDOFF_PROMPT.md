# BOZOK UI Revizyonu — devir promptu (kompakt)

Yeni oturuma aynen yapıştır. UI işi `opencode/mimo-v2.6-flash-free`, geri kalan `opencode/muse-spark-1.3-contributor-free`. Apple skill: `/home/kaneki/.agents/skills/apple-design/SKILL.md`.

## Ortam

Repo `/home/kaneki/rocket_avioniks/rocket_avioniks/`, iş `groundstation/`. Backend `uvicorn app:app --port 8000` (`backend/`), frontend `npm run dev -- --port 5173` (`frontend/`). Toplu: `start.sh`/`stop.sh`. Donanım: STM32→E220→Uno→USB 115200.

## Protokol (değişmez)

Hava 39B: `00 02 17` + 36B core (A5, ver 0x02, seq, t_ms, flight 0-10, ek7, alt_m i16 m, vel_cms, sats canlı, lat/lon fix-kapılı, batt hep bilinmiyor, flags bit0 FIX/bit1-3 MAVİ-SARI-KIRMIZI, tilt_ddeg @29, lean @31/@33, CRC @35). Backend `/api/state` + WS `/ws` (tilt nullable). PAKETTE YOK: basınç, ivme ekseni, gyro, sıcaklık, GPS hız. Uydurma yasak.

## İş

- MIMO (sadece `frontend/src`): bloat temizliği; LED bitleri→3 lamba kartı (şu an görünmüyor, zinciri bul-onar); rate_hz dürüst gösterimi; 2D yandan roket (irtifa+tilt+lean, ASCENT parlaması, yaw yok etiketi); Türkçe, emoji yok; `tsc`+`build` yeşil.
- SPARK-A (salt-okunur): backend alan denetimi (`led`, tilt, rate hesapları) + eksik raporu.
- SPARK-B: kabul kriterleri + ekran görüntüsü planı (1440x900/1280x720, `?nointro=1#...`).
- Komutan: entegrasyon, çakışma çözümü, test, ekran görüntüsü incelemesi, rapor.
