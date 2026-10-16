"""BOZOK ground-station serial backend.

Reads the Arduino Uno receiver on USB serial (115200), parses its
human-readable telemetry lines (CRC already validated on the Uno),
holds one authoritative state object, streams it over WebSocket,
records sessions as JSONL, and serves CSV/JSON export + replay.

Run:  uvicorn app:app --host 127.0.0.1 --port 8000
"""
import csv
import io
import json
import math
import re
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import serial
import serial.tools.list_ports
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, PlainTextResponse

SESSIONS = Path(__file__).resolve().parent / "sessions"
SESSIONS.mkdir(exist_ok=True)

SCHEMA_VERSION = 1
STALE_MS = 500
# 10 Hz telemetri için uyarlanabilir bayat eşiği: max(500 ms, 3 ölçülen
# periyot). 10 Hz'de 500 ms taban, yaklaşık beş arka arkaya kayıp paketten
# sonra bağlantıyı stale işaretler; link yokken de 500 ms'de bayatlar.
STALE_MIN_MS = 500
STALE_PERIOD_MULT = 3.0

# USB-seri VID:PID listesi (Uno'ya benzeyenler aday sayılır):
# 1a86:7523 CH340, 10c4:ea60 CP210x, 2341:0043 Uno.
BILINEN_USB_SERILER = {
    (0x1A86, 0x7523): "CH340",
    (0x10C4, 0xEA60): "CP210x",
    (0x2341, 0x0043): "Uno",
}


def port_bilgisi(p):
    """Tek seri portu zengin bilgiyle anlat (sahte veri yok, doğrudan pyserial)."""
    vid = getattr(p, "vid", None)
    pid = getattr(p, "pid", None)
    if isinstance(vid, int) and isinstance(pid, int):
        vid_pid = f"{vid:04x}:{pid:04x}"
        aday = (vid, pid) in BILINEN_USB_SERILER
    else:
        vid_pid = None
        aday = False
    return {
        "cihaz": p.device,
        "vid_pid": vid_pid,
        "aciklama": getattr(p, "description", "") or "",
        "seri_no": getattr(p, "serial_number", None),
        "uretici": getattr(p, "manufacturer", None),
        "aday": aday,
    }


def portlari_listele():
    try:
        return [port_bilgisi(p) for p in serial.tools.list_ports.comports()]
    except Exception:
        return []


def onerilen_port(bilgiler=None):
    """Bagli Uno benzeri aday varsa ilkini, yoksa ilk portu oner; port yoksa None."""
    infos = bilgiler if bilgiler is not None else portlari_listele()
    for b in infos:
        if b.get("aday"):
            return b["cihaz"]
    return infos[0]["cihaz"] if infos else None


def coz_kimlik(vid_pid, seri_no):
    """Kayitli kimlige (VID:PID + seri no) uyan guncel cihaz adi; yoksa None.
    Cekirdek adi (ttyUSBx) kaymalarinda kalici kimliktir; sahte veri yok."""
    if not vid_pid:
        return None
    yedek = None
    for b in portlari_listele():
        if b.get("vid_pid") != vid_pid:
            continue
        if seri_no and b.get("seri_no") == seri_no:
            return b["cihaz"]
        if yedek is None:
            yedek = b["cihaz"]
    return yedek

LINK_RE = re.compile(r"\[LINK\] seq=(\d+) t=(\d+)ms")
KV_RE = re.compile(r"(flight|ek7mode|sats|batt)=([^\s]+)")
ALT_RE = re.compile(r"alt=([^\s]+)")
VEL_RE = re.compile(r"vel=([^\s]+)")
FIX_RE = re.compile(r"fix=([YN])")
# Uno "LED=BY-" yazar (3 karakter: B/Y/R ya da sönük "-"); katı desen
# frontend parseLed ile aynı kümeyi kabul eder, boşluk almaz.
LED_RE = re.compile(r"LED=([BYR\-]{3})")
# Uno artik tum alani TEK satirda basar; pos SÖZLESME geregi en sondadir.
# Ancak bozuk/yari tek-satir ya da sira karisik cercevede greedy pos=(.+)
# tilt/lean'i yutup pos_raw'i kirletiyordu (tekrar uretildi: pos_raw'a
# "39.9 32.8 tilt=..." sizdi, _parse_pos 3 parca gorup None dondu, GPS sustu).
# Bu yuzden tembel eslesme + tilt/lean onunde dur: iki sira da dogru okunur.
POS_RE = re.compile(r"pos=(.+?)(?:\s+tilt=|\s+lean=|$)")
TILT_RE = re.compile(r"tilt=([^\s]+)\s+lean=([^\s]+)")

# Menzil kapilari: disi reddedilir (None sayilir), asla state'e girmez.
# |alt| > 100 km, |vel| > 1000 m/s, tilt > 180° (tel i16 0x8000 gecersiz
# dahil) fiziksel disi uydurmadir; sayac range_rej ile durustce sayilir.
ALT_MAX_M = 100000
VEL_MAX_CMS = 100000
TILT_MAX_DDEG = 1800
# Tel i16 gecersiz isareti: 0x8000 (-32768). Kaynak "olcmedim" dediginde
# bu degeri basar; gercek olcu olamaz (32 km deniz alti / -327 m/s parkuru
# asar), o yuzden menzilden bagimsiz BİLİNMEYEN sayilir.
I16_SENTINEL = -32768

state_lock = threading.Lock()
state = {
    "schema_version": SCHEMA_VERSION,
    "connected": False,
    "port": None,
    "port_kayip": False,
    "seq": None,
    "t_ms": None,
    "flight_state": None,
    "ek7_mode": None,
    "alt_m": None,
    "vel_cms": None,
    "sats": None,
    # Alan adlari geriye uyum icin lat_e5/lon_e5 kalir; olcek artik e6
    # (1e-6 derece, ~0.11 m): arayuz degeri /1e6 ile dereceye cevirir.
    "lat_e5": None,
    "lon_e5": None,
    "batt_mv": None,
    "fix": False,
    # İlk paketten önce bilinmiyor (None); gerçek sönük "---" ile karışmaz.
    # Arayüz de rx_time===null ile aynı ayrımı yapar.
    "led": None,
    "pos_raw": None,
    "tilt_ddeg": None,
    "lean_x": None,
    "lean_y": None,
    "rx_time": None,
    "stale": True,
    "stale_ms": STALE_MS,
    "packets": 0,
    "crc_ok": 0,
    "crc_bad": 0,
    "bad_blocks": 0,
    "range_rej": 0,
    "gaps": 0,
    "last_seq": None,
    "rate_hz": 0.0,
    "session_id": None,
}

recording = {"active": False, "id": None, "fh": None, "meta": {}}
_rate_times: list = []
_last_stale_mark = 0.0


def stale_threshold_ms(rate_hz=None):
    """Uyarlanabilir bayat esigi: max(500ms, 3 x olculen periyot).

    rate_hz <= 0 ya da olculmemisse taban 500 ms doner; aksi halde
    periyot = 1000/rate_hz, esik = max(500, 3*periyot). Donmus veri ile
    canli veri ayrimi tek kurala baglidir: simdi - rx_time > esik => bayat.
    """
    r = state.get("rate_hz") if rate_hz is None else rate_hz
    try:
        r = float(r or 0.0)
    except (TypeError, ValueError):
        r = 0.0
    if r <= 0.0 or r != r:  # sifir/negatif/NaN => taban
        return float(STALE_MIN_MS)
    return max(float(STALE_MIN_MS), STALE_PERIOD_MULT * (1000.0 / r))


def _record_stale_mark(now, reason):
    """Kesinti sirasinda kayit dosyasina kalp atisi yaz (export etkilenmez).

    JSONL kaydi yalnizca gecerli pakette degil, bayat donemlerde de surer;
    isaretleyici satirlar type=stale tasir, CSV disari aktarim yalnizca
    type=link satirlari okudugu icin bozulmaz. 1 sn kisitli yazilir.
    """
    global _last_stale_mark
    if not (recording["active"] and recording["fh"]):
        return
    if now - _last_stale_mark < 1.0:
        return
    _last_stale_mark = now
    try:
        recording["fh"].write(json.dumps({
            "type": "stale", "t": now, "reason": reason,
            "stale_ms": state.get("stale_ms"),
        }) + "\n")
        recording["fh"].flush()
    except Exception:
        pass


def _measured_rate_hz(timestamps):
    """Calculate observed packet rate from elapsed intervals, not window size.

    Dividing every partial 10-second window by 10 systematically reports a
    newly connected 10 Hz stream as 1 Hz after its first second. The sample
    interval estimator is correct immediately after two samples, bounded to
    a 10-second window by the caller, and reports 0 until a period is observed.
    """
    if len(timestamps) < 2:
        return 0.0
    elapsed = timestamps[-1] - timestamps[0]
    if elapsed <= 0.0 or not math.isfinite(elapsed):
        return 0.0
    return round((len(timestamps) - 1) / elapsed, 2)


def _refresh_link_age():
    now = time.time()
    alive = [t for t in _rate_times if now - t < 10.0]
    if len(alive) != len(_rate_times):
        _rate_times[:] = alive
    state["rate_hz"] = _measured_rate_hz(alive)
    state["stale_ms"] = int(round(stale_threshold_ms(state["rate_hz"])))
    # Bagli port sistemden duserse (reboot/tak-cikar sonrasi kayma ya da
    # kablo cekme): port_kayip=true + connected=false. Port adi korunur,
    # arayuz hangi portun kayboldugunu gosterebilir.
    if state.get("connected") is True and state.get("port"):
        try:
            cihazlar = {p.device for p in serial.tools.list_ports.comports()}
        except Exception:
            cihazlar = None
        if cihazlar is not None and state["port"] not in cihazlar:
            state["connected"] = False
            state["port_kayip"] = True
            w = globals().get("worker")
            if w is not None:
                try:
                    w.want = False
                    w.auto = True  # elle kapatma degil: kimlikle donus denenecek
                    if w.ser:
                        w.ser.close()
                except Exception:
                    pass
                finally:
                    w.ser = None
    if state["rx_time"] is None:
        state["stale"] = True
    elif now - state["rx_time"] > state["stale_ms"] / 1000:
        state["stale"] = True
    if state["stale"]:
        _record_stale_mark(now, "no-packet" if state["rx_time"] is None else "timeout")


def note_bad_block(n=1):
    """Atılan/bozuk blogu dürüstçe say (crc_bad ile birlikte yükselir).

    0x00 baytları, MODE-PROBE satırları, çerçevesiz ([LINK]siz) tamponlar
    buradan geçer; sayaç gerçekten artar, sürekli 0 görünmez.
    """
    with state_lock:
        state["bad_blocks"] = int(state.get("bad_blocks") or 0) + n
        state["crc_bad"] = int(state.get("crc_bad") or 0) + n


def _parse_num(tok):
    if tok in ("--", "---", None, ""):
        return None
    try:
        v = int(tok)
    except ValueError:
        try:
            v = float(tok)
        except ValueError:
            return None
    # "nan"/"inf" metni float'a donusur ama olcu degildir: state'e girerse
    # JSON NaN uretip /api/state ve /ws karelerini zehirler. BİLİNMEYEN say.
    if isinstance(v, float) and not math.isfinite(v):
        return None
    return v


def _parse_pos(s):
    """Uno pos=39.933400 32.859700 basar: ondalik derece, 6 basamak (~0.11 m).

    Donus GERCEK e6 tamsayisidir (arayuz /1e6 ile uyumlu):
    int(round(derece*1e6)). Menzil disi, sonsuz/NaN ve 2-jeton-disi
    bicimler None.
    """
    s = (s or "").strip()
    if s in ("--", ""):
        return None, None
    parts = s.split()
    if len(parts) != 2:
        # Eski 4-jetonlu "DD MM.SS" bicimi reddedilir (sozlesme: 2 jeton).
        return None, None
    try:
        lat = float(parts[0])
        lon = float(parts[1])
    except ValueError:
        return None, None
    # "nan"/"inf" metni float'a donusur ama olcu degildir; JSON'u zehirler.
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return None, None
    # Menzil kapilari: disi reddedilir (None), asla state'e girmez.
    if abs(lat) > 90 or abs(lon) > 180:
        return None, None
    return int(round(lat * 1e6)), int(round(lon * 1e6))


def apply_link(lines):
    """Fold one [LINK] block (list of stripped lines) into state.

    Yeni Uno tek satir basar (tek elemanli liste); eski 6 satirlik blok
    (cok elemanli liste) da ayni desenlerle okunur. Null kurallari aynen:
    "--" -> None, batt 0 -> None, fix'siz pos -> None. CSV/snapshot alani
    degismedi (asagidaki snap/CSV listeleri aynen)."""
    global _rate_times
    data = {"seq": None, "t_ms": None}
    for ln in lines:
        m = LINK_RE.search(ln)
        if m:
            data["seq"], data["t_ms"] = int(m.group(1)), int(m.group(2))
        for k, v in KV_RE.findall(ln):
            data[{"flight": "flight_state", "ek7mode": "ek7_mode",
                   "batt": "batt_mv"}.get(k, k)] = v
        for nk in ("flight_state", "ek7_mode", "sats", "batt_mv"):
            if nk in data and isinstance(data[nk], str):
                v = data[nk]
                if nk == "batt_mv" and v.endswith("mV"):
                    # Uno "4150mV" basar; "--" (0 mV) aynen None kalir.
                    v = v[:-2]
                data[nk] = _parse_num(v)
        m = ALT_RE.search(ln)
        if m:
            v = m.group(1)
            data["alt_m"] = None if v == "--" else _parse_num(v.rstrip("m"))
        m = VEL_RE.search(ln)
        if m:
            v = m.group(1)
            if v == "--" or v.startswith("--"):
                data["vel_cms"] = None
            elif v.startswith("-"):
                n = _parse_num(v[1:].rstrip("cm/s"))
                data["vel_cms"] = -n if n is not None else None
            else:
                data["vel_cms"] = _parse_num(v.rstrip("cm/s"))
        m = FIX_RE.search(ln)
        if m:
            data["fix"] = (m.group(1) == "Y")
        m = LED_RE.search(ln)
        if m:
            data["led"] = m.group(1)
        m = POS_RE.search(ln)
        if m:
            data["pos_raw"] = m.group(1).strip()
        m = TILT_RE.search(ln)
        if m:
            # Egiklik: "12.3" derece -> decidegree int; "--" -> yok.
            # Egilme: "0.123,-0.045" -> float cift; "--" -> yok.
            tv, lv = m.group(1), m.group(2)
            if tv == "--":
                data["tilt_ddeg"] = None
                data["lean_x"] = None
                data["lean_y"] = None
            else:
                try:
                    data["tilt_ddeg"] = int(round(float(tv) * 10.0))
                except ValueError:
                    data["tilt_ddeg"] = None
                if lv == "--":
                    data["lean_x"] = None
                    data["lean_y"] = None
                else:
                    try:
                        xs, ys = lv.split(",")
                        data["lean_x"] = float(xs)
                        data["lean_y"] = float(ys)
                    except ValueError:
                        data["lean_x"] = None
                        data["lean_y"] = None
    rej = 0
    a = data.get("alt_m")
    if a is not None and (a == I16_SENTINEL
                          or not math.isfinite(a) or abs(a) > ALT_MAX_M):
        data["alt_m"] = None
        rej += 1
    v = data.get("vel_cms")
    if v is not None and (v == I16_SENTINEL
                          or not math.isfinite(v) or abs(v) > VEL_MAX_CMS):
        data["vel_cms"] = None
        rej += 1
    t = data.get("tilt_ddeg")
    if t is not None and (not math.isfinite(t) or abs(t) > TILT_MAX_DDEG):
        # Egiklik supheliyse tum yonelim ornegi suphelidir: lean de duser.
        data["tilt_ddeg"] = None
        data["lean_x"] = None
        data["lean_y"] = None
        rej += 1
    if data["seq"] is None:
        # Çerçevesiz atık: 0x00 baytları / MODE-PROBE satırları burada elenir.
        note_bad_block()
        return
    now = time.time()
    with state_lock:
        if state["last_seq"] is not None:
            gap = (data["seq"] - state["last_seq"]) % 65536
            if gap > 1000:
                pass  # verici yeniden basladi (seq sifirlandi); kayip sayma
            elif gap > 1:
                state["gaps"] += gap - 1
        state["last_seq"] = data["seq"]
        state.update({k: data.get(k, state.get(k)) for k in (
            "seq", "t_ms", "flight_state", "ek7_mode", "alt_m",
            "vel_cms", "sats", "batt_mv", "fix", "led", "pos_raw",
            "tilt_ddeg", "lean_x", "lean_y")})
        # _parse_pos gercek e6 tamsayi doner; alan adlari geriye uyum icin
        # lat_e5/lon_e5 kaldi (olcek e6, arayuz /1e6 ile boler).
        lat, lon = _parse_pos(data.get("pos_raw"))
        # Only trust coordinates with an explicit fix flag.
        if state["fix"] and lat is not None:
            state["lat_e5"] = lat
            state["lon_e5"] = lon
        state["rx_time"] = now
        state["stale"] = False
        state["packets"] += 1
        state["crc_ok"] += 1
        state["range_rej"] = int(state.get("range_rej") or 0) + rej
        _rate_times.append(now)
        _rate_times = [t for t in _rate_times if now - t < 10.0]
        state["rate_hz"] = _measured_rate_hz(_rate_times)
        state["stale_ms"] = int(round(stale_threshold_ms(state["rate_hz"])))
    if recording["active"] and recording["fh"]:
        with state_lock:
            snap = {k: state.get(k) for k in (
                "seq", "t_ms", "flight_state", "ek7_mode", "alt_m",
                "vel_cms", "sats", "fix", "led",
                "tilt_ddeg", "lean_x", "lean_y")}
        recording["fh"].write(json.dumps({
            "type": "link", "t": now, "lines": lines, "fields": snap,
        }) + "\n")
        recording["fh"].flush()


class SerialWorker(threading.Thread):
    daemon = True

    def __init__(self):
        super().__init__()
        self.port = None
        self.ser = None
        self.want = False
        self.kimlik = None  # (vid_pid, seri_no): kayip sonrasi cozum icin
        self.auto = False  # True = kayiptan donus bekleniyor (elle kapatma degil)
        self.son_dene = 0.0

    def open(self, port, baud=115200):
        self.close()
        # DTR assert Uno'yu resetler (CH340); baglanti reseti bastirilir.
        self.ser = serial.Serial(port, baud, timeout=1, dsrdtr=False)
        try:
            self.ser.dtr = False
        except Exception:
            pass
        self.port = port
        self.want = True
        self.auto = False
        self.kimlik = None
        for b in portlari_listele():
            if b["cihaz"] == port:
                self.kimlik = (b.get("vid_pid"), b.get("seri_no"))
                break
        with state_lock:
            state["connected"] = True
            state["port"] = port
            state["port_kayip"] = False

    def close(self):
        self.want = False
        self.auto = False
        self.kimlik = None
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        with state_lock:
            state["connected"] = False
            state["port_kayip"] = False

    def run(self):
        buf = []
        while True:
            if not self.want or not self.ser:
                # Baglı değilken de hız/stale yaşlansın (sessizlikte rate 0'a düşer).
                with state_lock:
                    _refresh_link_age()
                # Kayiptan otomatik donus: kimlik cozulurse sessizce yeniden
                # baglan (3sn aralikli, DTR-resetsiz open ile). Elle kapatmada
                # auto=False oldugundan dokunulmaz.
                if self.auto and not self.want and not self.ser and self.kimlik:
                    now = time.time()
                    if now - self.son_dene > 3.0:
                        self.son_dene = now
                        dev = coz_kimlik(*self.kimlik)
                        if dev:
                            try:
                                self.open(dev)
                            except Exception:
                                pass
                time.sleep(0.2)
                continue
            try:
                raw = self.ser.readline().decode("utf-8", "replace")
            except Exception:
                with state_lock:
                    state["connected"] = False
                time.sleep(1.0)
                continue
            ln = raw.strip()
            if not ln:
                # Boş satır/timeout: sayaç donmasın, hız/stale yine yaşlansın.
                with state_lock:
                    _refresh_link_age()
                continue
            raw_log.append((time.time(), ln))
            if len(raw_log) > 60:
                del raw_log[:len(raw_log) - 60]
            if ln.startswith("[LINK]"):
                if "flight=" in ln:
                    # Tek-satir paket: bekletmeden isle (eski cok-satirli
                    # blok yarim kalmissa once onu kapat).
                    if buf:
                        apply_link(buf)
                        buf = []
                    apply_link([ln])
                    buf_t = time.time()
                else:
                    # Eski 6 satirlik blokun ilk satiri: devam satirlari
                    # birikene kadar bekle (kayit tekrar oynatma uyumu).
                    if buf:
                        apply_link(buf)
                    buf = [ln]
                    buf_t = time.time()
            # Uno devam satırları boşlukla başlar ama ln strip edildiği için
            # ln.startswith(" ") ölüdür; ham satırdaki boşluğa bakılır.
            # "GPS"/"tilt" strip sonrası G/t ile başlar, eski "fsvbpl" kümesinde
            # yoktu ve düşüyordu; küme genişletildi + anahtar öneki eklendi.
            elif buf and (raw.startswith(" ") or raw.startswith("\t")
                          or (ln and ln[0] in "fsvbpltTG")
                          or ln.startswith(("GPS", "tilt", "lean", "pos", "fix", "LED"))):
                buf.append(ln)
            else:
                if buf:
                    apply_link(buf)
                    buf = []
                else:
                    # Tampon yokken gelen tanınmaz satır (0x00 artığı,
                    # MODE-PROBE, gürültü): sessizce at ama dürüstçe say.
                    note_bad_block()
            if buf and time.time() - buf_t > 1.5:
                apply_link(buf)
                buf = []
            with state_lock:
                _refresh_link_age()


raw_log: list = []
buf_t = 0.0

worker = SerialWorker()
worker.start()

app = FastAPI(title="BOZOK Ground Station")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:5173", "http://localhost:5173"],
    allow_methods=["GET", "POST"],
    allow_headers=["Content-Type"],
)


@app.get("/api/ports")
def ports():
    bilgiler = portlari_listele()
    # "ports" geriye uyum icin korunur (cihaz dizisi); "portlar" zengin
    # bilgi, "onerilen" Uno benzeri adayi tasir.
    return {
        "ports": [b["cihaz"] for b in bilgiler],
        "portlar": bilgiler,
        "onerilen": onerilen_port(bilgiler),
    }


@app.get("/api/raw")
def raw():
    with state_lock:
        return {"lines": [ln for _, ln in raw_log[-40:]]}


@app.post("/api/connect")
def connect(body: dict):
    # Port verilmezse kod-gomulu varsayilan YOK: bagli Uno benzeri aday
    # onerilir; hic port yoksa durustce 400 donulur.
    bilgiler = portlari_listele()
    port = (body or {}).get("port") or onerilen_port(bilgiler)
    if not port:
        return JSONResponse(
            {"ok": False, "error": "port yok: cihaz takili degil"}, status_code=400)
    baud = int((body or {}).get("baud", 115200))
    try:
        worker.open(port, baud)
        return {"ok": True, "port": port}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=400)


@app.post("/api/disconnect")
def disconnect():
    worker.close()
    return {"ok": True}


@app.get("/api/state")
def get_state():
    with state_lock:
        _refresh_link_age()
        return dict(state)


@app.websocket("/ws")
async def ws_feed(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await ws.send_json(await _ws_state())
            await _sleep(0.2)
    except WebSocketDisconnect:
        pass


async def _ws_state():
    with state_lock:
        _refresh_link_age()
        return dict(state)


async def _sleep(s):
    import asyncio
    await asyncio.sleep(s)


@app.post("/api/record/start")
def record_start(body: dict):
    sid = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    path = SESSIONS / f"{sid}.jsonl"
    fh = open(path, "w")
    meta = {"type": "meta", "session_id": sid,
            "start": datetime.now(timezone.utc).isoformat(),
            "notes": body.get("notes", ""),
            "schema_version": SCHEMA_VERSION}
    fh.write(json.dumps(meta) + "\n")
    recording.update({"active": True, "id": sid, "fh": fh, "meta": meta})
    global _last_stale_mark
    _last_stale_mark = 0.0
    with state_lock:
        state["session_id"] = sid
    return {"ok": True, "session_id": sid}


@app.post("/api/record/stop")
def record_stop():
    if recording["fh"]:
        recording["fh"].close()
    recording.update({"active": False, "id": None, "fh": None})
    with state_lock:
        state["session_id"] = None
    return {"ok": True}


@app.get("/api/sessions")
def sessions():
    out = []
    for p in sorted(SESSIONS.glob("*.jsonl")):
        try:
            first = p.read_text().splitlines()[0]
            meta = json.loads(first)
        except Exception:
            meta = {}
        out.append({"id": p.stem, "size": p.stat().st_size,
                    "start": meta.get("start")})
    return {"sessions": out}


@app.get("/api/sessions/{sid}")
def session_get(sid: str):
    p = SESSIONS / f"{sid}.jsonl"
    if not p.exists():
        return JSONResponse({"error": "not found"}, status_code=404)
    return {"lines": p.read_text().splitlines()[:5000]}


@app.get("/api/sessions/{sid}/export.csv")
def session_csv(sid: str):
    p = SESSIONS / f"{sid}.jsonl"
    if not p.exists():
        return JSONResponse({"error": "not found"}, status_code=404)
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(["rx_time", "seq", "t_ms", "flight_state", "ek7_mode",
                "alt_m", "vel_cms", "sats", "fix", "led",
                "tilt_ddeg", "lean_x", "lean_y"])
    for line in p.read_text().splitlines():
        try:
            o = json.loads(line)
        except Exception:
            continue
        if o.get("type") != "link" or "fields" not in o:
            continue
        f = o["fields"]
        w.writerow([o.get("t"), f.get("seq"), f.get("t_ms"),
                    f.get("flight_state"), f.get("ek7_mode"),
                    f.get("alt_m"), f.get("vel_cms"), f.get("sats"),
                    f.get("fix"), f.get("led"),
                    f.get("tilt_ddeg"), f.get("lean_x"), f.get("lean_y")])
    return PlainTextResponse(buf.getvalue(), media_type="text/csv")
