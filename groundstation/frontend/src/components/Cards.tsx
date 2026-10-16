import type { ReactNode } from "react";
import type { TelemetryState } from "../telemetry";
import { ek7ModeName, tiltOrNull, velOrNull } from "../telemetry";
import { fixed, formatClock, formatDeg } from "../format";
import { LAMPS, parseLed } from "../led";

interface CardsProps {
  state: TelemetryState | null;
}

type Tone = "neutral" | "info" | "caution" | "warning";

interface CardSpec {
  key: string;
  label: string;
  value?: string;
  unit?: string;
  foot?: string;
  tone: Tone;
  unknown?: boolean;
  stale?: boolean;
  /** Değer satırının yerine geçen özel gövde (ör. LED lamba sırası). */
  body?: ReactNode;
}

function cardClass(tone: Tone, stale: boolean, unknown: boolean): string {
  const mods: string[] = [];
  if (tone !== "neutral") mods.push(`bz-card--${tone}`);
  if (stale && !unknown) mods.push("bz-card--stale");
  return mods.length === 0 ? "bz-card" : `bz-card ${mods.join(" ")}`;
}

function valueClass(tone: Tone, stale: boolean, unknown: boolean): string {
  if (unknown) return "bz-card__value bz-card__value--none";
  if (stale) return "bz-card__value bz-card__value--caution";
  return tone === "neutral" ? "bz-card__value" : `bz-card__value bz-card__value--${tone}`;
}

function Card(spec: CardSpec): ReactNode {
  return (
    <article className={cardClass(spec.tone, spec.stale === true, spec.unknown === true)}>
      <h3 className="bz-card__label">{spec.label}</h3>
      {spec.body !== undefined ? (
        spec.body
      ) : (
        <p className={valueClass(spec.tone, spec.stale === true, spec.unknown === true)}>
          <span>{spec.value ?? "—"}</span>
          {spec.unit !== undefined ? <span className="bz-card__unit">{spec.unit}</span> : null}
        </p>
      )}
      {spec.foot !== undefined ? <p className="bz-card__foot">{spec.foot}</p> : null}
    </article>
  );
}

/**
 * Üç canlı lamba (MAVİ/SARI/KIRMIZI): paketteki `LED=---` üç karakterinin
 * konumsal çözümü. İlk paketten önce ölçüm yoktur — lambalar bilinmeyen
 * durumunda kalır, "kapalı" gösterilmez.
 */
function LedLamps({ state, stale }: { state: TelemetryState | null; stale: boolean }): ReactNode {
  const bits = state === null || state.rx_time === null ? null : parseLed(state.led);
  // Ham yankı: ilk paketten önce backend varsayılanı ("---") ölçüm gibi
  // görünmemelidir — ham metin de rx_time yokken gizlenir ("--").
  const ham = state === null || state.rx_time === null ? null : state.led;
  return (
    <ul className={`bz-leds${stale ? " is-stale" : ""}`}>
      {LAMPS.map((l) => {
        const biliniyor = bits !== null;
        const acik = biliniyor && bits[l.key];
        const durum = !biliniyor ? "BİLİNMEYEN" : acik ? "AÇIK" : "KAPALI";
        return (
          <li
            key={l.key}
            className={`bz-led bz-led--${l.key}${
              !biliniyor ? " is-unknown" : acik ? " is-on" : " is-off"
            }`}
          >
            <span className="bz-led__lamp" aria-hidden="true" />
            <span className="bz-led__text">
              <span className="bz-led__name">{l.ad}</span>
              <span className="bz-led__trig">{l.tetik}</span>
            </span>
            <span className="bz-led__state">{durum}</span>
          </li>
        );
      })}
      <li className="bz-led__raw">
        ham: <span className="bz-mono">LED={ham ?? "--"}</span>
      </li>
    </ul>
  );
}

/**
 * Genel bakış kartları. Kahraman bloğunda olmayan ikincil büyüklükler burada:
 * her kart geçerliliğini (biliniyor / bilinmiyor / bayat) kendi metninde söyler.
 */
export function Cards({ state }: CardsProps): ReactNode {
  const stale = state?.stale === true;
  const yok = state === null;

  const vel = velOrNull(state?.vel_cms);
  const sats = state?.sats ?? null;
  const tilt = tiltOrNull(state?.tilt_ddeg);
  const fix = state?.fix === true;
  const lat = state?.lat_e5 ?? null;
  const lon = state?.lon_e5 ?? null;
  const konumGecerli = fix && lat !== null && lon !== null;

  const specs: CardSpec[] = [
    {
      key: "vel",
      label: "Dikey hız",
      value: vel === null ? "BİLİNMEYEN" : (vel / 100).toFixed(2),
      unit: vel === null ? undefined : "m/s",
      foot: vel === null ? "paket alanında boş" : `${vel} cm/s (ham)`,
      tone: vel === null ? "neutral" : "info",
      unknown: vel === null,
      stale,
    },
    {
      key: "sats",
      label: "Uydu sayısı",
      value: sats === null ? "BİLİNMEYEN" : String(sats),
      unit: sats === null ? undefined : "adet",
      foot: sats === null ? "alıcı sayı bildirmiyor" : "GNSS izleme",
      tone: sats === null ? "neutral" : "info",
      unknown: sats === null,
      stale,
    },
    {
      key: "konum",
      label: "Konum",
      value: yok
        ? "VERİ YOK"
        : konumGecerli
          ? `${formatDeg(lat)}, ${formatDeg(lon)}`
          : fix
            ? "GEÇERSİZ"
            : "FIX YOK",
      foot: konumGecerli
        ? `GNSS kilidi · ${sats ?? "--"} uydu`
        : fix
          ? "koordinatlar paket içinde boş"
          : "uydu kilidi bekleniyor",
      tone: konumGecerli ? "info" : "caution",
      unknown: yok,
      stale,
    },
    {
      key: "tilt",
      label: "Egiklik",
      value: tilt === null ? "BİLİNMEYEN" : (tilt / 10).toFixed(1),
      unit: tilt === null ? undefined : "°",
      foot: tilt === null ? "ivmeölçer veri yok" : "yaklaşık yönelim (ivmeölçer, yaw yok)",
      tone: tilt === null ? "neutral" : "info",
      unknown: tilt === null,
      stale,
    },
    {
      key: "ek7",
      label: "EK7 modu",
      value: state === null ? "VERİ YOK" : ek7ModeName(state.ek7_mode),
      foot: state?.ek7_mode == null ? "kod yok" : `kod ${state.ek7_mode}`,
      tone: "neutral",
      unknown: state === null || state.ek7_mode == null,
      stale,
    },
    {
      key: "rate",
      label: "Paket hızı",
      value: fixed(state?.rate_hz ?? null, 1),
      unit: "Hz",
      foot: yok ? "bağlantı yok" : `ölçülen · seq ${state.seq ?? "--"} · kayıp ${state.gaps}`,
      tone: yok || stale ? "caution" : "info",
      unknown: yok,
      stale,
    },
    {
      key: "led",
      label: "LED tetikleme",
      body: <LedLamps state={state} stale={stale} />,
      foot: yok ? "bağlantı yok" : "donanım mandalları (canlı bitler)",
      tone: "neutral",
      unknown: yok,
      stale,
    },
    {
      key: "rx",
      label: "Son paket",
      value: state?.rx_time == null ? "—" : formatClock(state.rx_time),
      foot: state?.rx_time == null ? "hiç paket alınmadı" : "yerel saat",
      tone: state?.rx_time == null ? "caution" : "neutral",
      unknown: state?.rx_time == null,
      stale,
    },
  ];

  return (
    <div className="bz-cards">
      {specs.map((s) => (
        <Card {...s} key={s.key} />
      ))}
    </div>
  );
}

export default Cards;
