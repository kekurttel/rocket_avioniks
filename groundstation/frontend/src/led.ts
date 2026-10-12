/**
 * LED tetikleme bitleri — donanım mandalları.
 *
 * Alıcı (uno_lora_rx) her paket satırına `LED=BY-` yazar: 1. karakter mavi,
 * 2. sarı, 3. kırmızı; bayrak yoksa `-`. Backend (app.py LED_RE) bu metni
 * olduğu gibi `led` alanına koyar. Bu modül o üç karakteri yalnızca
 * konumsal olarak çözer; tanınmayan biçimler "bilinmiyor" sayılır.
 *
 * Tetik etiketleri firmware Src/main.c içindeki bayrak koşullarıdır:
 *   mavi    → blue_latch  (BENCH_FALL_DETECTED, tezgâh düşüşü)
 *   sarı    → drop_cond   (irtifa düşüşü / açılış koşulu)
 *   kırmızı → FLIGHT_FAULT
 */

export type LampKey = "mavi" | "sari" | "kirmizi";

export interface LedBits {
  mavi: boolean;
  sari: boolean;
  kirmizi: boolean;
}

export interface LampSpec {
  key: LampKey;
  /** Kart üzerinde görünen ad. */
  ad: string;
  /** Biti yakan firmware koşulu (dürüst kısa etiket). */
  tetik: string;
  /** Ham paket karakteri (konum sırası bu). */
  ham: "B" | "Y" | "R";
}

export const LAMPS: LampSpec[] = [
  { key: "mavi", ad: "MAVİ", tetik: "düşüş tespiti", ham: "B" },
  { key: "sari", ad: "SARI", tetik: "açılış koşulu", ham: "Y" },
  { key: "kirmizi", ad: "KIRMIZI", tetik: "arıza", ham: "R" },
];

/**
 * `led` metnini üç bite çözer.
 * Konumsal: [0]=mavi (B|−), [1]=sarı (Y|−), [2]=kırmızı (R|−).
 * Eksik/bozuk metin null döner — arayüz bunu asla tahminle doldurmaz.
 */
export function parseLed(raw: string | null | undefined): LedBits | null {
  if (raw === null || raw === undefined) return null;
  const s = raw.replace(/\s+/g, "").toUpperCase();
  if (s.length !== 3) return null;
  if (s[0] !== "B" && s[0] !== "-") return null;
  if (s[1] !== "Y" && s[1] !== "-") return null;
  if (s[2] !== "R" && s[2] !== "-") return null;
  return { mavi: s[0] === "B", sari: s[1] === "Y", kirmizi: s[2] === "R" };
}
