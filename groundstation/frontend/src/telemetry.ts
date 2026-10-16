import { useEffect, useSyncExternalStore } from "react";

/** Backend JSON schema v1 (saha ekleri nullable: tilt_ddeg/lean_x/lean_y). */
export interface TelemetryState {
  /** Bağlantı kurulup ilk paket alınana kadar null kalır. */
  seq: number | null;
  t_ms: number | null;
  flight_state: number | null;
  ek7_mode: number | null;
  alt_m: number | null;
  vel_cms: number | null;
  sats: number | null;
  /** Enlem: e6 (1e-6°) tamsayı; alan adı tel/WS uyumluluğu için lat_e5. */
  lat_e5: number | null;
  /** Boylam: e6 (1e-6°) tamsayı; alan adı tel/WS uyumluluğu için lon_e5. */
  lon_e5: number | null;
  /** Paket şemasında alan var, içerik backend tarafından doldurulmuyor: hep null. */
  batt_mv: number | null;
  fix: boolean;
  led: string | null;
  /** Egiklik decidegree (0..1800); geçersiz/yok ise null. Şema v1 nullable ek. */
  tilt_ddeg: number | null;
  /** Gövde egilme yönü X/Y (birim vektör ×1); yaw yok; geçersizse null. */
  lean_x: number | null;
  lean_y: number | null;
  /** Epoch saniye; hiç paket alınmadıysa null. */
  rx_time: number | null;
  stale: boolean;
  packets: number;
  gaps: number;
  rate_hz: number;
  connected: boolean;
  port: string | null;
  /** Bağlı port sistemden düşerse true olur (connected=false ile birlikte). */
  port_kayip?: boolean | null;
  session_id: string | null;
  /** Backend tarafı tanılama sayaçları (opsiyonel). */
  crc_ok?: number | null;
  crc_bad?: number | null;
  /** Atılan çerçevesiz blok sayacı (0x00/MODE-PROBE artığı dahil). */
  bad_blocks?: number | null;
  /** Menzil dışı reddedilen ölçüm sayacı (backend kapısı: |alt|>100km vb). */
  range_rej?: number | null;
  /** Uyarlanabilir bayat eşiği, ms (max(1500, 3x periyot)). */
  stale_ms?: number | null;
  schema_version?: number | null;
}

export interface HistoryPoint {
  t: number;
  alt_m: number | null;
  vel_cms: number | null;
  sats: number | null;
  rate_hz: number;
}

export const FLIGHT_STATE_TR: Record<number, string> = {
  0: "BAŞLATMA",
  1: "KALİBRASYON",
  2: "FIRLATMA BEKLENİYOR",
  3: "YÜKSELİŞ",
  4: "APOJE ONAYLANDI",
  5: "SERBEST DÜŞÜŞ",
  6: "DROGUE AÇILDI",
  7: "ALÇALIŞ",
  8: "ANA PARAŞÜT AÇILDI",
  9: "GÖREV TAMAMLANDI",
  10: "ARIZA",
};

export const EK7_MODE_TR: Record<number, string> = {
  0: "KAPALI",
  1: "HAZIR",
  2: "AKTİF",
};

export function flightStateName(fs: number | null | undefined): string {
  if (fs === null || fs === undefined) return "VERİ YOK";
  return FLIGHT_STATE_TR[fs] ?? `BİLİNMEYEN (${fs})`;
}

export function ek7ModeName(m: number | null | undefined): string {
  if (m === null || m === undefined) return "VERİ YOK";
  return EK7_MODE_TR[m] ?? `BİLİNMEYEN (${m})`;
}

/** stale=true ise değerler gri gösterilir. */
export function isStale(s: TelemetryState | null | undefined): boolean {
  return s?.stale === true;
}

/**
 * Menzil muhafızları (backend kapısıyla aynı sınırlar — savunma derinliği).
 *
 * Backend bozuk çerçeveyi zaten reddeder, ama kayıt tekrar oynatma ve eski
 * backend'e karşı arayüz de korur: menzil dışı sonlu sayı BİLİNMEYEN (null)
 * sayılır, asla kırpılıp gerçek gibi gösterilmez. Eşik dahil geçerlidir.
 */
export const ALT_MAX_M = 100000;
export const VEL_MAX_CMS = 100000;
export const TILT_MAX_DDEG = 1800;
/** Tel i16 geçersiz işareti (0x8000): ölçüm değil, BİLİNMEYEN. */
export const I16_SENTINEL = -32768;

function menzilde(v: number | null | undefined, max: number): number | null {
  if (v === null || v === undefined || !Number.isFinite(v)) return null;
  if (v === I16_SENTINEL || Math.abs(v) > max) return null;
  return v;
}

/** İrtifa muhafızı: |alt|>100km → null. */
export function altOrNull(v: number | null | undefined): number | null {
  return menzilde(v, ALT_MAX_M);
}

/** Dikey hız muhafızı: |vel|>100000cm/s → null. */
export function velOrNull(v: number | null | undefined): number | null {
  return menzilde(v, VEL_MAX_CMS);
}

/** Eğiklik muhafızı: |tilt|>1800 decidegree → null. */
export function tiltOrNull(v: number | null | undefined): number | null {
  return menzilde(v, TILT_MAX_DDEG);
}

/** Ham değer menzil dışı mı (reddedildi rozeti için)? */
export function menzilDisi(v: number | null | undefined, max: number): boolean {
  return v !== null && v !== undefined && Number.isFinite(v) && (v === I16_SENTINEL || Math.abs(v) > max);
}

/** lat/lon yalnızca fix && non-null ise geçerlidir. */
export function isGpsValid(s: TelemetryState | null | undefined): boolean {
  if (!s) return false;
  return s.fix === true && s.lat_e5 !== null && s.lon_e5 !== null;
}

/** Renk rolleri: bilgi / uyarı / tehlike (bkz. tokens.css). */
export type Tone = "neutral" | "info" | "caution" | "warning";

/** Uçuş durumu kodunu görsel role çevirir. ARIZA = tehlike, bekleme = uyarı. */
export function flightStateTone(fs: number | null | undefined): Tone {
  if (fs === null || fs === undefined) return "neutral";
  if (fs === 10) return "warning";
  if (fs === 2) return "caution";
  if (fs >= 3 && fs <= 8) return "info";
  return "neutral";
}

/** T+ süresi: 60 sn altı "12.4 sn", üstü "1:24.0" biçiminde. */
export function formatTPlus(tMs: number | null | undefined): string {
  if (tMs === null || tMs === undefined || !Number.isFinite(tMs)) return "--";
  const s = tMs / 1000;
  const m = Math.floor(s / 60);
  const rest = s - m * 60;
  if (m <= 0) return `${rest.toFixed(1)} sn`;
  return `${m}:${rest.toFixed(1).padStart(4, "0")}`;
}

function isNumOrNull(v: unknown): v is number | null {
  return v === null || (typeof v === "number" && Number.isFinite(v));
}

function isBool(v: unknown): v is boolean {
  return typeof v === "boolean";
}

function isNumOrNullUndef(v: unknown): v is number | null | undefined {
  return v === null || v === undefined || (typeof v === "number" && Number.isFinite(v));
}

/**
 * Paket doğrulaması (şema v1).
 *
 * İlk bağlantıda backend hâlâ hiçbir paket almamış olabilir; bu durumda
 * sayısal alanlar null gelir ve arayüz "veri yok" durumunu göstermelidir.
 * Bu yüzden sayısal alanlar `null` kabul edilir, ancak anahtarların varlığı
 * ve türü zorunlu tutulur — böylece eksik/bozuk kareler uygulanmaz.
 * tilt_ddeg/lean_x/lean_y v1 nullable eklerdir: yoklukları (undefined)
 * eski backend'e karşı kabul edilir.
 */
function isValidState(v: unknown): v is TelemetryState {
  if (typeof v !== "object" || v === null) return false;
  const o = v as Record<string, unknown>;
  return (
    isNumOrNull(o.seq) &&
    isNumOrNull(o.t_ms) &&
    isNumOrNull(o.flight_state) &&
    isNumOrNull(o.ek7_mode) &&
    isNumOrNull(o.alt_m) &&
    isNumOrNull(o.vel_cms) &&
    isNumOrNull(o.sats) &&
    isBool(o.fix) &&
    isBool(o.stale) &&
    isBool(o.connected) &&
    isNumOrNullUndef(o.tilt_ddeg) &&
    isNumOrNullUndef(o.lean_x) &&
    isNumOrNullUndef(o.lean_y) &&
    typeof o.rate_hz === "number" &&
    typeof o.packets === "number" &&
    typeof o.gaps === "number"
  );
}

// ---- Single shared store (module singleton) ----

let current: TelemetryState | null = null;
let history: HistoryPoint[] = [];
let historyMax = 600;
let started = false;
let ws: WebSocket | null = null;
let wsRetryTimer: number | null = null;
let wsWanted = true;

const stateListeners = new Set<() => void>();
const historyListeners = new Set<() => void>();

function notifyState(): void {
  for (const l of stateListeners) l();
}

function notifyHistory(): void {
  for (const l of historyListeners) l();
}

function pushHistory(s: TelemetryState): void {
  // T+ yoksa grafik ekseni anlamsız olur: o kareyi geçmişe almayız.
  if (s.t_ms === null) return;
  // Menzil dışı ölçüm geçmişe zehir olarak yazılmaz: boşluk (null) olur,
  // grafik ekseni 400M gibi değerle patlamaz.
  const pt: HistoryPoint = {
    t: s.t_ms / 1000,
    alt_m: altOrNull(s.alt_m),
    vel_cms: velOrNull(s.vel_cms),
    sats: s.sats,
    rate_hz: s.rate_hz,
  };
  history.push(pt);
  if (history.length > historyMax) {
    history = history.slice(history.length - historyMax);
  }
  notifyHistory();
}

function applyState(s: TelemetryState): void {
  const prevSeq = current?.seq;
  current = s;
  notifyState();
  // Yeni paket ise geçmişe ekle; aynı seq'i tekrar ekleme.
  if (prevSeq === undefined || s.seq !== prevSeq) {
    pushHistory(s);
  }
}

async function pollOnce(): Promise<void> {
  try {
    const res = await fetch(`${apiBase()}/api/state`);
    if (!res.ok) return;
    const json: unknown = await res.json();
    if (isValidState(json)) applyState(json);
  } catch {
    // Bağlantı yoksa sessiz geç; WS veya sonraki poll toparlar.
  }
}

const _envBase: string | undefined =
  (import.meta as unknown as { env?: Record<string, string | undefined> }).env?.VITE_API_URL;

function apiBase(): string {
  return _envBase ?? "http://127.0.0.1:8000";
}

function wsUrl(): string {
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${apiBase().replace(/^https?:\/\//, "")}/ws`;
}

function connectWs(): void {
  if (!wsWanted) return;
  try {
    ws?.close();
  } catch {
    // yoksay
  }
  ws = null;
  let sock: WebSocket;
  try {
    sock = new WebSocket(wsUrl());
  } catch {
    scheduleWsRetry();
    return;
  }
  ws = sock;
  sock.onmessage = (ev: MessageEvent) => {
    try {
      const json: unknown = JSON.parse(ev.data as string);
      if (isValidState(json)) applyState(json);
    } catch {
      // bozuk kareyi yoksay
    }
  };
  sock.onclose = () => {
    if (ws === sock) ws = null;
    scheduleWsRetry();
  };
  sock.onerror = () => {
    try {
      sock.close();
    } catch {
      // yoksay
    }
  };
}

function scheduleWsRetry(): void {
  if (!wsWanted) return;
  if (wsRetryTimer !== null) return;
  wsRetryTimer = window.setTimeout(() => {
    wsRetryTimer = null;
    connectWs();
  }, 2000);
}

function ensureStarted(): void {
  if (started || typeof window === "undefined") return;
  started = true;
  wsWanted = true;
  void pollOnce();
  setInterval(() => {
    void pollOnce();
  }, 500);
  connectWs();
}

function subscribeState(cb: () => void): () => void {
  stateListeners.add(cb);
  return () => {
    stateListeners.delete(cb);
  };
}

function subscribeHistory(cb: () => void): () => void {
  historyListeners.add(cb);
  return () => {
    historyListeners.delete(cb);
  };
}

/** Paylaşılan tek telemetri durumu: 500ms poll + WS dinleyici (reconnect'li). */
export function useTelemetry(): TelemetryState | null {
  useEffect(() => {
    ensureStarted();
    return () => {
      // Bilerek kapatma yok: store uygulama ömrü boyunca tekildir.
    };
  }, []);
  return useSyncExternalStore(subscribeState, () => current, () => current);
}

/** Sınırlı geçmiş (en fazla `max`, varsayılan 600 nokta). */
export function useHistory(max = 600): HistoryPoint[] {
  useEffect(() => {
    historyMax = max;
    ensureStarted();
    if (history.length > max) {
      history = history.slice(history.length - max);
    }
    return () => {
      // store tekildir; abonelik dışındaki temizlik yok.
    };
  }, [max]);
  return useSyncExternalStore(
    subscribeHistory,
    () => history,
    () => history,
  );
}
