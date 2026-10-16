/** Sayısal biçimleme yardımcıları — arayüzde tek doğruluk kaynağı. */

export function formatClock(rx: number | null | undefined): string {
  if (rx === null || rx === undefined || !Number.isFinite(rx)) return "--:--:--";
  return new Date(rx * 1000).toLocaleTimeString("tr-TR", { hour12: false });
}

export function formatKb(bytes: number | null | undefined): string {
  if (bytes === null || bytes === undefined || !Number.isFinite(bytes)) return "--";
  if (bytes < 1024) return `${Math.round(bytes)} B`;
  return `${(bytes / 1024).toFixed(0)} KB`;
}

/** e6 (1e-6°) tamsayı, 6 basamak. */
export function formatDeg(v: number | null | undefined, digits = 6): string {
  if (v === null || v === undefined || !Number.isFinite(v)) return "--";
  return (v / 1e6).toFixed(digits);
}

export function fixed(v: number | null | undefined, digits: number): string {
  if (v === null || v === undefined || !Number.isFinite(v)) return "--";
  return v.toFixed(digits);
}

export function formatIso(iso: string | null | undefined): string {
  if (!iso) return "tarih bilinmiyor";
  const d = new Date(iso);
  if (Number.isNaN(d.getTime())) return iso;
  return d.toLocaleString("tr-TR", { hour12: false });
}
