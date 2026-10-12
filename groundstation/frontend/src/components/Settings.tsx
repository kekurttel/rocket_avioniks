import { useState, type ReactNode } from "react";
import { postJson, type PortBilgi } from "../api";
import type { TelemetryState } from "../telemetry";
import { formatClock } from "../format";
import { Feedback } from "./Feedback";
import { useFeedback } from "../useFeedback";
import { Readout, ReadoutGroup } from "./Readouts";

interface SettingsProps {
  state: TelemetryState | null;
  ports: string[];
  portlar: PortBilgi[];
  onerilen: string | null;
  portsFailed: boolean;
  refreshPorts: () => void;
}

const BAUD = 115200;

/** Seçenek etiketi: cihaz + açıklama + VID:PID, önerilen ve aktif işaretli. */
function portEtiketi(
  cihaz: string,
  bilgi: PortBilgi | undefined,
  onerilen: string | null,
  aktifPort: string | null,
): string {
  const parcalar = [cihaz];
  const aciklama = bilgi?.aciklama?.trim();
  if (aciklama && aciklama !== cihaz) parcalar.push(aciklama);
  if (bilgi?.vid_pid) parcalar.push(bilgi.vid_pid);
  const isaretler: string[] = [];
  if (onerilen !== null && cihaz === onerilen) isaretler.push("önerilen");
  if (aktifPort !== null && cihaz === aktifPort) isaretler.push("aktif");
  if (isaretler.length > 0) parcalar.push(`(${isaretler.join(" · ")})`);
  return parcalar.join(" — ");
}

export function Settings({
  state,
  ports,
  portlar,
  onerilen,
  portsFailed,
  refreshPorts,
}: SettingsProps): ReactNode {
  const [port, setPort] = useState("");
  const { feedback, show } = useFeedback();
  const bagli = state?.connected === true;
  const kayip = state?.port_kayip === true;
  const aktifPort = state?.port ?? null;

  const bilgiHarita = new Map(portlar.map((b) => [b.cihaz, b]));

  // Port seçimi backend ile senkron: kullanıcının seçimi, yoksa backend'in
  // açık portu, o da yoksa önerilen aday, o da yoksa listenin başı.
  const varsayilanPort = aktifPort ?? onerilen ?? ports[0] ?? "";
  const etkinPort = port !== "" ? port : varsayilanPort;
  const secenekler = etkinPort !== "" && !ports.includes(etkinPort) ? [etkinPort, ...ports] : ports;

  const baglan = async () => {
    // Kullanıcı elle seçmediyse port gönderilmez: backend önerileni kullanır.
    if (port === "" && varsayilanPort === "") {
      show(
        portsFailed ? "Sunucuya ulaşılamıyor — önce backend'i başlatın (./start.sh)" : "Önce bir port seçin",
        "caution",
      );
      return;
    }
    const govde = port === "" ? { baud: BAUD } : { port: etkinPort, baud: BAUD };
    const r = await postJson<{ ok: boolean; port: string; error?: string }>("/api/connect", govde);
    if (r.ok && r.data?.ok === true) {
      setPort("");
      show(`Bağlandı · ${r.data.port} @ ${BAUD}`);
    } else {
      show(`Bağlanılamadı${r.data?.error ? `: ${r.data.error}` : " — sunucuya ulaşılamadı"}`, "error");
    }
    refreshPorts();
  };

  const kes = async () => {
    const r = await postJson<{ ok: boolean }>("/api/disconnect");
    if (r.ok && r.data?.ok === true) show("Seri bağlantı kesildi");
    else show("Bağlantı kesilemedi — sunucuya ulaşılamadı", "error");
  };

  return (
    <>
      <section className="bz-panel">
        <header className="bz-panel__head">
          <h2 className="bz-panel__title">Seri bağlantı</h2>
          {bagli ? (
            <span className="bz-chip bz-chip--info">Bağlı</span>
          ) : (
            <span className="bz-chip bz-chip--warning">Bağlı değil</span>
          )}
        </header>

        {kayip && (
          <p className="bz-hint" role="alert">
            <span className="bz-chip bz-chip--warning">Cihaz kayıp</span> CİHAZ KAYIP, kabloyu
            kontrol et: {aktifPort ?? "bağlı port"} sistemde görünmüyor. Kabloyu takıp listeden
            yeni portu seçerek yeniden bağlanın.
          </p>
        )}

        <div className="bz-btnrow bz-btnrow--end">
          <label className="bz-field bz-field--grow">
            <span className="bz-field__label">Seri port</span>
            <select className="bz-select" value={etkinPort} onChange={(e) => setPort(e.target.value)}>
              {secenekler.length === 0 ? (
                <option value="">
                  {portsFailed
                    ? "sunucuya ulaşılamıyor — backend çalışmıyor"
                    : "port bulunamadı — cihaz takılı değil"}
                </option>
              ) : null}
              {secenekler.map((p) => (
                <option key={p} value={p}>
                  {portEtiketi(p, bilgiHarita.get(p), onerilen, bagli || kayip ? aktifPort : null)}
                </option>
              ))}
            </select>
          </label>
          <button type="button" className="bz-btn bz-btn--primary" onClick={() => void baglan()} disabled={bagli}>
            Bağlan · {BAUD}
          </button>
          <button type="button" className="bz-btn" onClick={() => void kes()} disabled={!bagli}>
            Bağlantıyı kes
          </button>
        </div>

        <p className="bz-hint">
          Baud hızı alıcı kart tarafından sabitlenmiştir ({BAUD}); port listesi bu sekme açıkken
          ~5 sn'de bir yenilenir{portsFailed ? " ve şu anda alınamıyor — sunucuya ulaşılamıyor" : ""}
          {onerilen ? `; önerilen: ${onerilen}` : ""}.
        </p>
        <Feedback value={feedback} />
      </section>

      <ReadoutGroup title="Tanılama">
        <Readout name="Sunucu (/api/state)" value={state === null ? "yanıt yok" : "yanıt veriyor"} text />
        <Readout name="Seri port" value={state?.port ?? null} text />
        <Readout
          name="Port durumu"
          value={state === null ? null : kayip ? "KAYIP — kabloyu kontrol et" : bagli ? "AÇIK" : "KAPALI"}
          text
        />
        <Readout name="Bağlantı" value={state === null ? null : bagli ? "AÇIK" : "KAPALI"} text />
        <Readout name="Baud" value={String(BAUD)} text />
        <Readout name="Son paket saati" value={state?.rx_time == null ? null : formatClock(state.rx_time)} text />
        <Readout name="Alınan paket" value={state === null ? null : String(state.packets)} />
        <Readout name="Kayıp paket" value={state === null ? null : String(state.gaps)} />
        <Readout
          name="Bozuk blok"
          value={state === null ? null : String(state.bad_blocks ?? state.crc_bad ?? 0)}
        />
        <Readout
          name="Bayat eşiği"
          value={state === null ? null : `${((state.stale_ms ?? 1500) / 1000).toFixed(1)}`}
          unit="sn"
        />
        <Readout
          name="Akış hızı (ölçülen)"
          value={state === null ? null : state.rate_hz.toFixed(2)}
          unit="Hz"
          stale={state?.stale === true}
        />
        <Readout
          name="CRC"
          value={
            state?.crc_ok == null && state?.crc_bad == null
              ? null
              : `${state?.crc_ok ?? 0} başarılı · ${state?.crc_bad ?? 0} hatalı (CRC alıcıda doğrulanır)`
          }
          text
        />
        <Readout name="Şema sürümü" value={state?.schema_version == null ? null : String(state.schema_version)} text />
        <Readout name="Kayıt oturumu" value={state?.session_id ?? null} text />
      </ReadoutGroup>
    </>
  );
}

export default Settings;
