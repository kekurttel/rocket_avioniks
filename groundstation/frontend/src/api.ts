import { useCallback, useEffect, useState } from "react";

/**
 * Yerel seri köprüsü.
 *
 * NOT: `src/telemetry.ts` durum akışını göreli `/api/state` üzerinden alır
 * (arayüz backend ile aynı kaynaktan servis edildiğinde); bu modülün
 * uçları mevcut sözleşmeyi bozmadan sabit köprü adresini kullanır.
 */
export const API = "http://127.0.0.1:8000";

export interface PortBilgi {
  cihaz: string;
  vid_pid: string | null;
  aciklama: string;
  seri_no: string | null;
  uretici?: string | null;
  aday?: boolean;
}

export interface PortsResponse {
  /** Geriye uyum: cihaz dizisi. */
  ports: string[];
  /** Zengin port bilgisi (cihaz, VID:PID, açıklama, seri no). */
  portlar?: PortBilgi[];
  /** Uno benzeri aday varsa ilk adayın cihazı, yoksa ilk port, yoksa null. */
  onerilen?: string | null;
}

export interface SessionMeta {
  id: string;
  size: number;
  start: string | null;
}

export interface SessionsResponse {
  sessions: SessionMeta[];
}

interface ApiResult<T> {
  ok: boolean;
  status: number;
  data: T | null;
}

/**
 * Belirli aralıklarla bir uçtan JSON okur; elle tetiklenebilir `refresh`
 * döndürür (kayıt başlat/durdur sonrası liste anında tazelensin diye).
 */
export function useApi<T>(path: string, intervalMs: number) {
  const [data, setData] = useState<T | null>(null);
  const [failed, setFailed] = useState(false);
  const [tick, setTick] = useState(0);

  const refresh = useCallback(() => setTick((t) => t + 1), []);

  useEffect(() => {
    let on = true;
    const load = () => {
      fetch(API + path)
        .then((r) => (r.ok ? (r.json() as Promise<T>) : Promise.reject(new Error(String(r.status)))))
        .then((j) => {
          if (!on) return;
          setData(j);
          setFailed(false);
        })
        .catch(() => {
          if (on) setFailed(true);
        });
    };
    load();
    const id = window.setInterval(load, intervalMs);
    return () => {
      on = false;
      window.clearInterval(id);
    };
  }, [path, intervalMs, tick]);

  return { data, failed, refresh };
}

/** POST + JSON yanıtı; ağ hatasında da sessizce `ok:false` döner. */
export async function postJson<T>(path: string, body?: unknown): Promise<ApiResult<T>> {
  try {
    const res = await fetch(API + path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: body === undefined ? undefined : JSON.stringify(body),
    });
    const data = (await res.json().catch(() => null)) as T | null;
    return { ok: res.ok, status: res.status, data };
  } catch {
    return { ok: false, status: 0, data: null };
  }
}
