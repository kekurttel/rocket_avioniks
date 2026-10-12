import {
  useCallback,
  useEffect,
  useRef,
  useState,
  type KeyboardEvent as ReactKeyboardEvent,
  type ReactNode,
} from "react";
import { useApi, type PortsResponse, type SessionsResponse } from "./api";
import { useTelemetry, useHistory } from "./telemetry";
import { formatDeg } from "./format";
import { Cards } from "./components/Cards";
import { Charts } from "./components/Charts";
import { Readout, ReadoutGroup, Readouts } from "./components/Readouts";
import { Hero, LinkBanner, StatusPill } from "./components/Status";
import { Recordings } from "./components/Recordings";
import { Settings } from "./components/Settings";
import { Intro } from "./components/Intro";
import MapView from "./components/MapView";
import Rocket2D from "./components/Rocket2D";
import Rocket3D from "./components/Rocket3D";
import { PanelGuard } from "./components/PanelGuard";

/** Sekmeler ve kalıcı bağlantıları (location.hash ile derin bağlantı). */
const VIEWS = [
  { id: "genel-bakis", label: "Genel Bakış" },
  { id: "telemetri", label: "Telemetri" },
  { id: "harita", label: "GPS Harita" },
  { id: "grafikler", label: "Grafikler" },
  { id: "kayitlar", label: "Kayıtlar" },
  { id: "tanitim", label: "Tanıtım" },
  { id: "ayarlar", label: "Ayarlar" },
] as const;

type ViewId = (typeof VIEWS)[number]["id"];

function viewFromHash(): ViewId {
  const raw = window.location.hash.replace(/^#/, "");
  return VIEWS.some((v) => v.id === raw) ? (raw as ViewId) : "genel-bakis";
}

function introWanted(): boolean {
  try {
    if (window.matchMedia("(prefers-reduced-motion: reduce)").matches) return false;
    try {
      if (new URLSearchParams(window.location.search).has("nointro")) return false;
    } catch {}
    return sessionStorage.getItem("bozok-intro") !== "done";
  } catch {
    return true;
  }
}

function BrandMark(): ReactNode {
  return (
    <span className="bz-brand__mark" aria-hidden="true">
      <svg viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M12 2.5 20 21l-8-4.5L4 21 12 2.5Z" fill="currentColor" />
      </svg>
    </span>
  );
}

export default function App() {
  const live = useTelemetry();
  const chartHistory = useHistory(600);

  const [view, setView] = useState<ViewId>(() => viewFromHash());
  const [intro, setIntro] = useState<boolean>(() => introWanted());
  const tabRefs = useRef<Partial<Record<ViewId, HTMLButtonElement | null>>>({});

  const portsApi = useApi<PortsResponse>("/api/ports", 5000);
  const sessionsApi = useApi<SessionsResponse>("/api/sessions", 5000);

  // Sekme değişince adres çubuğu da güncellenir (derin bağlantı korunur).
  useEffect(() => {
    window.history.replaceState(null, "", `#${view}`);
  }, [view]);

  // Ayarlar sekmesi her açılışta port listesini tazeler (5 sn anket sürer).
  useEffect(() => {
    if (view === "ayarlar") portsApi.refresh();
  }, [view]);
  useEffect(() => {
    // Elle düzenlenen / geri-ileri ile değiştirilen hash'i uygula.
    const onHash = () => setView(viewFromHash());
    window.addEventListener("hashchange", onHash);
    return () => window.removeEventListener("hashchange", onHash);
  }, []);

  const closeIntro = useCallback(() => {
    try {
      sessionStorage.setItem("bozok-intro", "done");
    } catch {
      // oturum deposu kapalıysa yoksay
    }
    setIntro(false);
  }, []);

  const onTabKeyDown = (e: ReactKeyboardEvent) => {
    const idx = VIEWS.findIndex((v) => v.id === view);
    let next = idx;
    if (e.key === "ArrowRight") next = (idx + 1) % VIEWS.length;
    else if (e.key === "ArrowLeft") next = (idx - 1 + VIEWS.length) % VIEWS.length;
    else if (e.key === "Home") next = 0;
    else if (e.key === "End") next = VIEWS.length - 1;
    else return;
    e.preventDefault();
    const id = VIEWS[next].id;
    setView(id);
    tabRefs.current[id]?.focus();
  };

  return (
    <div className="bz-app">
      {intro && <Intro onClose={closeIntro} />}

      <header className="bz-topbar">
        <div className="bz-brand">
          <BrandMark />
          <span>
            <span className="bz-brand__name">BOZOK</span>
            <span className="bz-brand__sub">Görev Kontrol</span>
          </span>
        </div>

        <StatusPill state={live} />

        <nav className="bz-tabs" role="tablist" aria-label="Görünümler" onKeyDown={onTabKeyDown}>
          {VIEWS.map((v) => (
            <button
              key={v.id}
              ref={(el) => {
                tabRefs.current[v.id] = el;
              }}
              id={`tab-${v.id}`}
              className="bz-tabs__btn"
              role="tab"
              type="button"
              aria-selected={view === v.id}
              aria-controls={`panel-${v.id}`}
              tabIndex={view === v.id ? 0 : -1}
              onClick={() => setView(v.id)}
            >
              {v.label}
            </button>
          ))}
        </nav>
      </header>

      <main className="bz-stage">
        <div
          key={view}
          id={`panel-${view}`}
          className="bz-view"
          role="tabpanel"
          aria-labelledby={`tab-${view}`}
          tabIndex={-1}
        >
          {view === "genel-bakis" && (
            <>
              <LinkBanner state={live} />
              <Hero state={live} />
              <div className="bz-grid-2">
                <PanelGuard name="Harita">
                  <MapView state={live} />
                </PanelGuard>
                <div className="bz-stack">
                  <PanelGuard name="Roket 2B">
                    <Rocket2D
                      alt_m={live?.alt_m ?? null}
                      flight_state={live?.flight_state ?? null}
                      tilt_ddeg={live?.tilt_ddeg ?? null}
                      lean_x={live?.lean_x ?? null}
                      lean_y={live?.lean_y ?? null}
                    />
                  </PanelGuard>
                  <PanelGuard name="Roket 3B">
                    <Rocket3D
                      alt_m={live?.alt_m ?? null}
                      flight_state={live?.flight_state ?? null}
                      tilt_ddeg={live?.tilt_ddeg ?? null}
                      lean_x={live?.lean_x ?? null}
                      lean_y={live?.lean_y ?? null}
                    />
                  </PanelGuard>
                </div>
              </div>
              <Cards state={live} />
            </>
          )}

          {view === "telemetri" && (
            <>
              <div className="bz-viewhead">
                <h1 className="bz-viewhead__title">Telemetri okumaları</h1>
                <p className="bz-viewhead__note">
                  Yalnızca pakette alan alan değerler; eksik alanlar tahmin edilmez.
                </p>
              </div>
              <Readouts state={live} />
            </>
          )}

          {view === "harita" && (
            <>
              <div className="bz-viewhead">
                <h1 className="bz-viewhead__title">GPS haritası</h1>
                <p className="bz-viewhead__note">
                  Konum yalnızca GNSS kilidi varken ve koordinat geçerliyken çizilir.
                </p>
              </div>
              <PanelGuard name="Harita">
                <MapView state={live} />
              </PanelGuard>
              <ReadoutGroup title="Konum">
                <Readout
                  name="GNSS kilidi"
                  value={live === null ? null : live.fix ? "VAR" : "YOK"}
                  stale={live?.stale === true}
                />
                <Readout
                  name="Enlem"
                  value={live?.lat_e5 == null ? null : formatDeg(live.lat_e5)}
                  unit="°"
                  stale={live?.stale === true}
                />
                <Readout
                  name="Boylam"
                  value={live?.lon_e5 == null ? null : formatDeg(live.lon_e5)}
                  unit="°"
                  stale={live?.stale === true}
                />
                <Readout
                  name="Uydu sayısı"
                  value={live?.sats == null ? null : String(live.sats)}
                  unit="adet"
                  stale={live?.stale === true}
                />
              </ReadoutGroup>
            </>
          )}

          {view === "grafikler" && (
            <>
              <div className="bz-viewhead">
                <h1 className="bz-viewhead__title">Grafikler</h1>
                <p className="bz-viewhead__note">
                  Son {chartHistory.length} örnek · eksenler birim etiketli, animasyon kapalı.
                </p>
              </div>
              <PanelGuard name="Grafikler">
                <Charts history={chartHistory} />
              </PanelGuard>
            </>
          )}

          {view === "kayitlar" && (
            <>
              <div className="bz-viewhead">
                <h1 className="bz-viewhead__title">Kayıtlar</h1>
                <p className="bz-viewhead__note">
                  Kayıtlar sunucuda JSONL olarak tutulur; CSV dışa aktarımı hazır.
                </p>
              </div>
              <Recordings
                state={live}
                sessions={sessionsApi.data?.sessions ?? []}
                sessionsFailed={sessionsApi.failed}
                refreshSessions={sessionsApi.refresh}
              />
            </>
          )}

          {view === "tanitim" && (
            <section className="bz-panel">
              <header className="bz-panel__head">
                <h2 className="bz-panel__title">BOZOK tanıtımı</h2>
                <span className="bz-chip">10 sn · sessiz · atlanabilir</span>
              </header>
              <video
                className="bz-intro__video"
                src="/intro.mp4"
                controls
                autoPlay
                muted
                playsInline
              />
              <p className="bz-footnote">
                Bu video ilk açılışta da gösterilir; hareket azaltma tercihi açıksa otomatik
                gösterim atlanır.
              </p>
            </section>
          )}

          {view === "ayarlar" && (
            <>
              <div className="bz-viewhead">
                <h1 className="bz-viewhead__title">Ayarlar</h1>
                <p className="bz-viewhead__note">Port seçimi ve bağlantı durumu backend ile senkron.</p>
              </div>
              <Settings
                state={live}
                ports={portsApi.data?.ports ?? []}
                portlar={portsApi.data?.portlar ?? []}
                onerilen={portsApi.data?.onerilen ?? null}
                portsFailed={portsApi.failed}
                refreshPorts={portsApi.refresh}
              />
            </>
          )}
        </div>
      </main>
    </div>
  );
}
