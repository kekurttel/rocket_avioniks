/**
 * JS katmanı için renk aynası.
 *
 * recharts, three.js ve Leaflet SVG özellikleri CSS değişkenlerini okuyamaz
 * (presentation attribute `var()` kabul etmez); bu nedenle bu değerler burada
 * literal olarak durur. Tek kaynak `tokens.css`'tir — bu dosya yalnızca
 * oradaki paletin birebir kopyasıdır; biri değişirse diğeri de değişmelidir.
 */
export const palette = {
  info: "#2f7bff",
  infoStrong: "#5b96ff",
  infoPale: "#a9c8ff",
  text: "#e8eefb",
  textDim: "#a2b3ce",
  textMute: "#6f81a0",
  white: "#ffffff",
  caution: "#ffb020",
  cautionPale: "#ffd899",
  warning: "#ff4d63",
  grid: "rgba(160, 190, 255, 0.10)",
  line: "rgba(160, 190, 255, 0.20)",
  surface: "#0e1626",
  deep: "#04060c",
} as const;

export type Palette = typeof palette;
