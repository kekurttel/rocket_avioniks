import type { ReactNode } from "react";
import type { TelemetryState } from "../telemetry";
import { ALT_MAX_M, altOrNull, flightStateName, flightStateTone, formatTPlus, menzilDisi } from "../telemetry";
import { fixed, formatClock } from "../format";

type BannerTone = "live" | "stale" | "down" | "idle";

interface Description {
  tone: BannerTone;
  title: string;
  detail: string;
  chip: string;
  chipTone: "info" | "caution" | "warning" | "neutral";
}

/** Bağlantı/link durumunu tek bir dürüst cümleye indirger. */
function describe(state: TelemetryState | null): Description {
  if (state === null) {
    return {
      tone: "down",
      title: "Sunucuya ulaşılamıyor",
      detail:
        "Yerel köprü (/api/state) yanıt vermiyor. Backend ayakta değilse telemetri de yoktur.",
      chip: "SUNUCU YOK",
      chipTone: "warning",
    };
  }
  if (state.connected !== true) {
    // Bağlı port sistemden düştüyse ölü-linkten önce ayrı kayıp uyarısı.
    if (state.port_kayip === true) {
      return {
        tone: "down",
        title: "Cihaz kayıp",
        detail:
          `CİHAZ KAYIP, kabloyu kontrol et: ${state.port ?? "bağlı port"} sistemde ` +
          "görünmüyor. Kabloyu takıp Ayarlar görünümünden listeden yeni portu seçerek yeniden bağlanın.",
        chip: "CİHAZ KAYIP",
        chipTone: "warning",
      };
    }
    return {
      tone: "down",
      title: "Seri bağlantı yok",
      detail: "Ayarlar görünümünden port seçip bağlanın; bağlantı kurulmadan paket akmaz.",
      chip: "BAĞLI DEĞİL",
      chipTone: "warning",
    };
  }
  if (state.stale === true) {
    const esik = state.stale_ms ?? 1500;
    return {
      tone: "stale",
      title: "Bağlantı var · veri bayat",
      detail:
        state.rx_time === null
          ? `Port açık ama henüz paket alınmadı (bayat kuralı: eşik ${(esik / 1000).toFixed(1)} sn). Alıcı çalışıyor mu?`
          : `Son paket ${formatClock(state.rx_time)} itibarıyla alındı; ${(esik / 1000).toFixed(1)} sn eşik aşıldı, değerler bayat gösteriliyor.`,
      chip: "BAYAT",
      chipTone: "caution",
    };
  }
  return {
    tone: "live",
    title: "Canlı akış",
    detail: `Paketler ölçülen ${state.rate_hz.toFixed(1)} Hz hızla akıyor; bayat eşiği ${((state.stale_ms ?? 1500) / 1000).toFixed(1)} sn (kural: en fazla 1.5 sn, yoksa 3x periyot).`,
    chip: "CANLI",
    chipTone: "info",
  };
}

export function LinkBanner({ state }: { state: TelemetryState | null }): ReactNode {
  const d = describe(state);
  const oturum = state?.session_id ?? null;
  return (
    <section className={`bz-banner bz-banner--${d.tone}`} role="status" aria-live="polite">
      <span className={`bz-banner__dot${d.tone === "live" ? " bz-dot--pulse" : ""}`} aria-hidden="true" />
      <div>
        <p className="bz-banner__title">{d.title}</p>
        <p className="bz-banner__detail">{d.detail}</p>
      </div>
      <div className="bz-banner__facts">
        <div>
          <span className="bz-banner__k">Durum</span>
          <span className="bz-banner__v">
            <span className={`bz-chip bz-chip--${d.chipTone}`}>{d.chip}</span>
          </span>
        </div>
        <div>
          <span className="bz-banner__k">Port</span>
          <span className="bz-banner__v">{state?.port ?? "atanmadı"}</span>
        </div>
        <div>
          <span className="bz-banner__k">Akış hızı (ölçülen)</span>
          <span className="bz-banner__v">{fixed(state?.rate_hz ?? null, 1)} Hz</span>
        </div>
        <div>
          <span className="bz-banner__k">Son paket</span>
          <span className="bz-banner__v">{formatClock(state?.rx_time ?? null)}</span>
        </div>
        <div>
          <span className="bz-banner__k">Paket / kayıp / bozuk</span>
          <span className="bz-banner__v">
            {state?.packets ?? 0} / {state?.gaps ?? 0} / {state?.bad_blocks ?? state?.crc_bad ?? 0}
          </span>
        </div>
        <div>
          <span className="bz-banner__k">Menzil reddi</span>
          <span className="bz-banner__v">{state?.range_rej ?? 0}</span>
        </div>
        <div>
          <span className="bz-banner__k">Bayat eşiği</span>
          <span className="bz-banner__v">{fixed((state?.stale_ms ?? 1500) / 1000, 1)} sn</span>
        </div>
        <div>
          <span className="bz-banner__k">Kayıt oturumu</span>
          <span className="bz-banner__v">{oturum ?? "kayıt yok"}</span>
        </div>
      </div>
    </section>
  );
}

/** Üst çubuktaki kompakt durum rozeti. */
export function StatusPill({ state }: { state: TelemetryState | null }): ReactNode {
  const d = describe(state);
  const toneClass = d.chipTone === "neutral" ? "" : ` bz-chip--${d.chipTone}`;
  return (
    <span className={`bz-chip${toneClass}`} title={d.detail}>
      <span
        className={`bz-chip__dot${d.tone === "live" ? " bz-dot--pulse" : ""}`}
        aria-hidden="true"
      />
      {d.chip}
    </span>
  );
}

/** Kahraman bloğu: irtifa + uçuş durumu. */
export function Hero({ state }: { state: TelemetryState | null }): ReactNode {
  const stale = state?.stale === true;
  const hamAlt = state?.alt_m ?? null;
  // Menzil dışı ham değer (400M glitch vb) asla basılmaz: BİLİNMEYEN olur.
  const alt = altOrNull(hamAlt);
  const red = menzilDisi(hamAlt, ALT_MAX_M);
  const fs = state?.flight_state ?? null;
  const tone = flightStateTone(fs);
  const toneSuffix = tone === "neutral" ? "" : ` bz-hero__name--${tone}`;

  const altClass =
    alt === null ? "bz-hero__num bz-hero__num--none" : stale ? "bz-hero__num bz-hero__num--stale" : "bz-hero__num";

  const altSub =
    state === null
      ? "sunucuya ulaşılamadı"
      : red
        ? "menzil dışı ham değer reddedildi — BİLİNMEYEN"
        : alt === null
          ? "paket alanında irtifa yok"
          : stale
            ? "bayat veri — yeni paket bekleniyor"
            : `${alt.toFixed(1)} m · taze paket`;

  return (
    <section className="bz-hero" aria-label="İrtifa ve uçuş durumu">
      <div>
        <p className="bz-hero__label">
          İrtifa{" "}
          {red && <span className="bz-chip bz-chip--warning">MENZİL DIŞI · REDDEDİLDİ</span>}
        </p>
        <p className="bz-hero__value">
          <span className={altClass}>{alt === null ? "—" : alt.toFixed(1)}</span>
          <span className="bz-hero__unit">m</span>
        </p>
        <p className="bz-hero__sub">{altSub}</p>
      </div>
      <div>
        <p className="bz-hero__label">Uçuş durumu</p>
        <p className={`bz-hero__name${toneSuffix}`}>{flightStateName(fs)}</p>
        <p className="bz-hero__sub">
          {fs === null ? "kod yok" : `kod ${fs}`}
          {" · "}
          {state === null ? "EK7 verisi yok" : `EK7 ${state.ek7_mode ?? "?"}`}
        </p>
        <div className="bz-hero__side">
          <div className="bz-hero__tile">
            <span className="bz-hero__tile-k">T+</span>
            <span className="bz-hero__tile-v">{formatTPlus(state?.t_ms ?? null)}</span>
          </div>
          <div className="bz-hero__tile">
            <span className="bz-hero__tile-k">Paket</span>
            <span className="bz-hero__tile-v">{state?.seq ?? "--"}</span>
          </div>
        </div>
      </div>
    </section>
  );
}
