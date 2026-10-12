import { useState, type ReactNode } from "react";
import { API, postJson, type SessionMeta } from "../api";
import type { TelemetryState } from "../telemetry";
import { altOrNull, flightStateName, velOrNull } from "../telemetry";
import { fixed, formatIso, formatKb } from "../format";
import { Feedback } from "./Feedback";
import { useFeedback } from "../useFeedback";

interface RecordingsProps {
  state: TelemetryState | null;
  sessions: SessionMeta[];
  sessionsFailed: boolean;
  refreshSessions: () => void;
}

interface LinkRec {
  seq: number | null;
  t_ms: number | null;
  flight_state: number | null;
  alt_m: number | null;
  vel_cms: number | null;
  sats: number | null;
  fix: boolean | null;
}

interface ReplayState {
  id: string;
  total: number;
  records: LinkRec[];
}

/** JSONL satırlarından yalnızca `link` kayıtlarını ayrıştırır. */
function parseLinks(lines: string[]): LinkRec[] {
  const out: LinkRec[] = [];
  for (const line of lines) {
    let obj: unknown;
    try {
      obj = JSON.parse(line);
    } catch {
      continue;
    }
    if (typeof obj !== "object" || obj === null) continue;
    const o = obj as Record<string, unknown>;
    if (o.type !== "link" || typeof o.fields !== "object" || o.fields === null) continue;
    const f = o.fields as Record<string, unknown>;
    const num = (v: unknown): number | null => (typeof v === "number" && Number.isFinite(v) ? v : null);
    // Menzil muhafızı: eski kayıttaki glitch (400M vb) önizlemeye sızmaz.
    out.push({
      seq: num(f.seq),
      t_ms: num(f.t_ms),
      flight_state: num(f.flight_state),
      alt_m: altOrNull(num(f.alt_m)),
      vel_cms: velOrNull(num(f.vel_cms)),
      sats: num(f.sats),
      fix: typeof f.fix === "boolean" ? f.fix : null,
    });
  }
  return out;
}

export function Recordings({
  state,
  sessions,
  sessionsFailed,
  refreshSessions,
}: RecordingsProps): ReactNode {
  const [notes, setNotes] = useState("");
  const [replay, setReplay] = useState<ReplayState | null>(null);
  const { feedback, show } = useFeedback();
  const active = state?.session_id != null;

  const baslat = async () => {
    const r = await postJson<{ ok: boolean; session_id: string; error?: string }>("/api/record/start", {
      notes,
    });
    if (r.ok && r.data?.ok === true) {
      show(`Kayıt başlatıldı · oturum ${r.data.session_id}`);
      setReplay(null);
    } else {
      show(`Kayıt başlatılamadı${r.data?.error ? `: ${r.data.error}` : " — sunucuya ulaşılamadı"}`, "error");
    }
    refreshSessions();
  };

  const durdur = async () => {
    const r = await postJson<{ ok: boolean }>("/api/record/stop");
    if (r.ok && r.data?.ok === true) {
      show("Kayıt durduruldu · oturum diske yazıldı");
    } else {
      show("Kayıt durdurulamadı — sunucuya ulaşılamadı", "error");
    }
    refreshSessions();
  };

  const oynat = async (id: string) => {
    try {
      const res = await fetch(`${API}/api/sessions/${id}`);
      if (!res.ok) throw new Error(String(res.status));
      const j = (await res.json()) as { lines?: string[] };
      const lines = j.lines ?? [];
      setReplay({ id, total: lines.length, records: parseLinks(lines) });
      show(`Oturum ${id} yüklendi · salt-okunur önizleme`);
    } catch {
      show(`Oturum ${id} yüklenemedi`, "error");
    }
  };

  return (
    <>
      <section className="bz-panel">
        <header className="bz-panel__head">
          <h2 className="bz-panel__title">Kayıt denetimi</h2>
          {active ? (
            <span className="bz-chip bz-chip--info">
              <span className="bz-chip__dot bz-dot--pulse" aria-hidden="true" />
              Kayıt aktif · {state?.session_id}
            </span>
          ) : (
            <span className="bz-chip">Kayıt yok</span>
          )}
        </header>

        <div className="bz-btnrow">
          <label className="bz-field bz-field--grow">
            <span className="bz-field__label">Uçuş notu</span>
            <input
              className="bz-input"
              value={notes}
              onChange={(e) => setNotes(e.target.value)}
              placeholder="ör. motor testi — 3. deneme"
            />
          </label>
          <button type="button" className="bz-btn bz-btn--primary" onClick={baslat} disabled={active}>
            Kaydı başlat
          </button>
          <button type="button" className="bz-btn bz-btn--danger" onClick={durdur} disabled={!active}>
            Kaydı durdur
          </button>
        </div>
        <Feedback value={feedback} />
      </section>

      <section className="bz-panel">
        <header className="bz-panel__head">
          <h2 className="bz-panel__title">Oturumlar</h2>
          <span className="bz-panel__meta">{sessions.length} kayıt</span>
        </header>

        {sessionsFailed && (
          <p className="bz-empty">Kayıt listesi alınamadı — sunucuya ulaşılamıyor.</p>
        )}
        {!sessionsFailed && sessions.length === 0 && (
          <p className="bz-empty">Henüz kayıt yok. Üstten bir kayıt başlattığınızda burada listelenir.</p>
        )}

        <ul className="bz-sessions">
          {sessions.map((s) => (
            <li key={s.id} className="bz-session">
              <span className="bz-session__id">{s.id}</span>
              <span className="bz-session__meta">
                {formatIso(s.start)} · {formatKb(s.size)}
              </span>
              <span className="bz-session__actions">
                <button type="button" className="bz-btn bz-btn--sm" onClick={() => void oynat(s.id)}>
                  Yeniden oynat
                </button>
                <a
                  className="bz-btn bz-btn--sm"
                  href={`${API}/api/sessions/${s.id}/export.csv`}
                  download={`${s.id}.csv`}
                >
                  CSV indir
                </a>
              </span>
            </li>
          ))}
        </ul>

        {replay !== null && (
          <div className="bz-replay">
            <header className="bz-panel__head">
              <h3 className="bz-panel__title">Önizleme · {replay.id}</h3>
              <span className="bz-chip">Salt okunur</span>
              <span className="bz-panel__meta">
                {replay.total} satır · {replay.records.length} link kaydı · ilk {Math.min(12, replay.records.length)}
              </span>
              <button
                type="button"
                className="bz-btn bz-btn--sm"
                onClick={() => setReplay(null)}
              >
                Kapat
              </button>
            </header>
            {replay.records.length === 0 ? (
              <p className="bz-empty">Bu oturumda ayrıştırılabilir link kaydı yok.</p>
            ) : (
              <div className="bz-replay__wrap">
                <table className="bz-replay__table">
                  <thead>
                    <tr>
                      <th scope="col">seq</th>
                      <th scope="col">T+ (ms)</th>
                      <th scope="col">Uçuş durumu</th>
                      <th scope="col">Göreli irtifa (m)</th>
                      <th scope="col">Hız (m/s)</th>
                      <th scope="col">Uydu</th>
                      <th scope="col">Fix</th>
                    </tr>
                  </thead>
                  <tbody>
                    {replay.records.slice(0, 12).map((r, i) => (
                      <tr key={`${r.seq ?? "x"}-${i}`}>
                        <td>{r.seq ?? "—"}</td>
                        <td>{r.t_ms ?? "—"}</td>
                        <td>{flightStateName(r.flight_state)}</td>
                        <td>{fixed(r.alt_m, 1)}</td>
                        <td>{r.vel_cms === null ? "—" : (r.vel_cms / 100).toFixed(2)}</td>
                        <td>{r.sats ?? "—"}</td>
                        <td>{r.fix === null ? "—" : r.fix ? "VAR" : "YOK"}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            )}
          </div>
        )}
      </section>
    </>
  );
}

export default Recordings;
