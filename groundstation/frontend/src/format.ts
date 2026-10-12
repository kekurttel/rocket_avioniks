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

export function formatDeg(e5: number | null | undefined, digits = 5): string {
  if (e5 === null || e5 === undefined || !Number.isFinite(e5)) return "--";
  return (e5 / 1e5).toFixed(digits);
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
