import type { ReactNode } from "react";
import type { TelemetryState } from "../telemetry";
import { altOrNull, ek7ModeName, flightStateName, formatTPlus, velOrNull } from "../telemetry";
import { formatClock, formatDeg } from "../format";

interface ReadoutsProps {
  state: TelemetryState | null;
}

interface ReadoutProps {
  name: string;
  /** null → alan bilinmiyor; arayüz bunu asla tahminle doldurmaz. */
  value: string | null;
  unit?: string;
  stale?: boolean;
  /** Sözcüksel alanlarda geçerlilik/bayatlık damgası anlamsızdır. */
  text?: boolean;
}

export function Readout({ name, value, unit, stale = false, text = false }: ReadoutProps): ReactNode {
  const none = value === null;
  const valueCls = none
    ? "bz-readout__v is-none"
    : stale
      ? "bz-readout__v is-stale"
      : "bz-readout__v";
  const stateCls = text ? "" : none ? " is-none" : stale ? " is-stale" : " is-valid";
  const stateText = text ? "" : none ? "BİLİNMEYEN" : stale ? "BAYAT" : "GEÇERLİ";
  return (
    <div className="bz-readout">
      <dt className="bz-readout__k">{name}</dt>
      <dd className={valueCls}>
        <span>{none ? "—" : value}</span>
        {unit !== undefined && !none ? <span className="bz-readout__u">{unit}</span> : null}
      </dd>
      {text ? null : <dd className={`bz-readout__state${stateCls}`}>{stateText}</dd>}
    </div>
  );
}

export function ReadoutGroup({
  title,
  children,
  note,
}: {
  title: string;
  children: ReactNode;
  /** Grup altındaki dürüst tanılama satırı (dl dışına yazılır). */
  note?: string;
}): ReactNode {
  return (
    <section className="bz-panel">
      <h2 className="bz-group__title">{title}</h2>
      <dl className="bz-readouts">{children}</dl>
      {note !== undefined ? <p className="bz-hint">{note}</p> : null}
    </section>
  );
}

/** Teknik okumalar: her değer birim, geçerlilik ve bayatlık damgasıyla listelenir. */
export function Readouts({ state }: ReadoutsProps): ReactNode {
  const stale = state?.stale === true;
  const num = (v: number | null | undefined, digits: number): string | null =>
    v === null || v === undefined ? null : v.toFixed(digits);
  // Menzil muhafızları: bozuk çerçeve artığı basılmaz, BİLİNMEYEN olur.
  const alt = altOrNull(state?.alt_m);
  const vel = velOrNull(state?.vel_cms);

  return (
    <div className="bz-groups">
      <ReadoutGroup title="Uçuş" note="Göreli irtifa, kalkış noktasındaki kalibre edilmiş barometrik basınca göre hesaplanır; deniz seviyesine göre rakım değildir.">
        <Readout name="Uçuş durumu" value={state === null ? null : flightStateName(state.flight_state)} text />
        <Readout name="Uçuş durumu kodu" value={state?.flight_state == null ? null : String(state.flight_state)} />
        <Readout name="EK7 modu" value={state === null ? null : ek7ModeName(state.ek7_mode)} text />
        <Readout name="Göreli irtifa" value={num(alt, 1)} unit="m" stale={stale} />
        <Readout
          name="Dikey hız"
          value={num(vel == null ? null : vel / 100, 2)}
          unit="m/s"
          stale={stale}
        />
        <Readout name="Uçuş süresi" value={state?.t_ms == null ? null : formatTPlus(state.t_ms)} stale={stale} />
      </ReadoutGroup>

      <ReadoutGroup title="Konum">
        <Readout
          name="GNSS kilidi"
          value={state === null ? null : state.fix ? "VAR" : "YOK"}
          stale={stale}
        />
        <Readout name="Enlem" value={state?.lat_e5 == null ? null : formatDeg(state.lat_e5)} unit="°" stale={stale} />
        <Readout name="Boylam" value={state?.lon_e5 == null ? null : formatDeg(state.lon_e5)} unit="°" stale={stale} />
        <Readout name="Uydu sayısı" value={state?.sats == null ? null : String(state.sats)} unit="adet" stale={stale} />
      </ReadoutGroup>

      <ReadoutGroup title="Bağlantı">
        <Readout name="Seri port" value={state?.port ?? null} text />
        <Readout name="Akış hızı (ölçülen)" value={num(state?.rate_hz ?? null, 2)} unit="Hz" stale={stale} />
        <Readout name="Alınan paket" value={state == null ? null : String(state.packets)} />
        <Readout name="Kayıp paket" value={state == null ? null : String(state.gaps)} />
        <Readout
          name="Bozuk blok"
          value={state == null ? null : String(state.bad_blocks ?? state.crc_bad ?? 0)}
        />
        <Readout name="Son paket saati" value={state?.rx_time == null ? null : formatClock(state.rx_time)} text />
        <Readout name="Kayıt oturumu" value={state?.session_id ?? null} text />
      </ReadoutGroup>

      <ReadoutGroup
        title="Cihaz"
        note="Tanılama: batarya (batt_mv) pakette tanımlı ama hiç doldurulmuyor — okunmuyor. Basınç, ivme eksenleri, jiroskop, sıcaklık ve GPS hızı pakette yok, hiçbir yerde gösterilmez. Bozuk blok sayacı atılan çerçevesiz satırları (0x00 artığı, MODE-PROBE) sayar; CRC hatalı ile birlikte yükselir."
      >
        <Readout name="LED durumu" value={state?.led ?? null} text />
        <Readout name="Şema sürümü" value={state?.schema_version == null ? null : String(state.schema_version)} text />
        <Readout name="CRC başarılı" value={state?.crc_ok == null ? null : String(state.crc_ok)} />
        <Readout
          name="CRC hatalı / bozuk blok"
          value={
            state?.crc_bad == null && state?.bad_blocks == null
              ? null
              : `${state?.crc_bad ?? 0} / ${state?.bad_blocks ?? 0}`
          }
        />
      </ReadoutGroup>
    </div>
  );
}

export default Readouts;
