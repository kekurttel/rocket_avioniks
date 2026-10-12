import { useEffect, useRef, useState } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import type { TelemetryState } from "../telemetry";
import { useTelemetry } from "../telemetry";
import { palette } from "../theme";

// Paylaşılan tipe uyumlu, bağımsız derlenebilen yedek arayüz (tüm alanlar opsiyonel).
interface YedekTelemetri {
  lat_e5?: number | null;
  lon_e5?: number | null;
  fix?: boolean;
  stale?: boolean;
  alt_m?: number | null;
  flight_state?: number | null;
}

type GirisDurumu = Partial<TelemetryState> & YedekTelemetri;

interface MapViewProps {
  state?: GirisDurumu | null;
}

function gecerliNokta(s: GirisDurumu | null | undefined): [number, number] | null {
  if (!s) return null;
  if (s.fix !== true) return null;
  if (s.lat_e5 == null || s.lon_e5 == null) return null;
  const lat = s.lat_e5 / 1e5;
  const lon = s.lon_e5 / 1e5;
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) return null;
  if (lat < -90 || lat > 90 || lon < -180 || lon > 180) return null;
  return [lat, lon];
}

const VARSAYILAN_MERKEZ: [number, number] = [39.9334, 32.8597];

function azHareket(): boolean {
  try {
    return window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  } catch {
    return false;
  }
}

export default function MapView({ state }: MapViewProps) {
  const canli = useTelemetry();
  const s: GirisDurumu | null = state !== undefined ? state : canli;

  const divRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<L.Map | null>(null);
  const markerRef = useRef<L.CircleMarker | null>(null);
  const lineRef = useRef<L.Polyline | null>(null);
  const trackRef = useRef<Array<[number, number]>>([]);
  const followRef = useRef<boolean>(true);

  const [takip, setTakip] = useState<boolean>(true);
  const [karoHata, setKaroHata] = useState<boolean>(false);
  const [noktaSayisi, setNoktaSayisi] = useState<number>(0);
  const [sonKonum, setSonKonum] = useState<[number, number] | null>(null);

  followRef.current = takip;

  const fix = s?.fix === true;
  const bayat = s?.stale === true;
  const gecerli = gecerliNokta(s) !== null;

  // Haritayı bir kez kur (react-leaflet yok).
  useEffect(() => {
    if (!divRef.current || mapRef.current) return;
    const sessiz = azHareket();
    const map = L.map(divRef.current, {
      center: VARSAYILAN_MERKEZ,
      zoom: 6,
      zoomControl: true,
      // Hareket azaltma seçiliyse yaprak animasyonları kapatılır.
      zoomAnimation: !sessiz,
      fadeAnimation: !sessiz,
      markerZoomAnimation: !sessiz,
    });
    mapRef.current = map;

    const tiles = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution: "© OpenStreetMap katkıda bulunanlar",
      maxZoom: 19,
    });
    tiles.on("tileerror", () => setKaroHata(true));
    tiles.addTo(map);

    lineRef.current = L.polyline([], { color: palette.info, weight: 3, opacity: 0.9 }).addTo(map);

    const kullaniciTasidi = () => {
      if (followRef.current) setTakip(false);
    };
    map.on("dragstart", kullaniciTasidi);
    map.on("zoomstart", kullaniciTasidi);

    const boyut = () => map.invalidateSize();
    window.addEventListener("resize", boyut);
    const gozlemci = new ResizeObserver(boyut);
    gozlemci.observe(divRef.current);

    return () => {
      window.removeEventListener("resize", boyut);
      gozlemci.disconnect();
      map.off("dragstart", kullaniciTasidi);
      map.off("zoomstart", kullaniciTasidi);
      map.remove();
      mapRef.current = null;
      markerRef.current = null;
      lineRef.current = null;
    };
  }, []);

  // Telemetri geldikçe işaretçi + izi güncelle. Geçersiz noktalar asla ize eklenmez.
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;
    const p = gecerliNokta(s);
    if (!p) return; // fix kaybı: son nokta korunur, marker kaldırılmaz
    const onceki = trackRef.current[trackRef.current.length - 1];
    if (!onceki || onceki[0] !== p[0] || onceki[1] !== p[1]) {
      trackRef.current.push(p);
      setNoktaSayisi(trackRef.current.length);
    }
    setSonKonum(p);
    lineRef.current?.setLatLngs(trackRef.current);
    if (!markerRef.current) {
      markerRef.current = L.circleMarker(p, {
        radius: 9,
        color: palette.white,
        weight: 2,
        fillColor: palette.info,
        fillOpacity: 1,
      }).addTo(map);
    } else {
      markerRef.current.setLatLng(p);
    }
    if (followRef.current) {
      map.setView(p, Math.max(map.getZoom(), 14));
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [s?.fix, s?.lat_e5, s?.lon_e5]);

  const yenidenOrtala = () => {
    const map = mapRef.current;
    if (!map) return;
    setTakip(true);
    followRef.current = true;
    const hedef = gecerliNokta(s) ?? sonKonum;
    if (hedef) map.setView(hedef, Math.max(map.getZoom(), 14));
    else map.setView(VARSAYILAN_MERKEZ, 6);
  };

  return (
    <section className="bz-panel bz-map" aria-label="GPS haritası">
      <header className="bz-panel__head">
        <h2 className="bz-panel__title">GPS haritası</h2>
        {bayat && <span className="bz-chip bz-chip--caution">Bayat · son konum korunuyor</span>}
        {!gecerli && (
          <span className="bz-chip bz-chip--caution">
            {fix ? "Koordinat geçersiz" : "Fix bekleniyor"}
          </span>
        )}
        <span className="bz-panel__meta">İz: {noktaSayisi} nokta</span>
      </header>

      <div className="bz-map__canvas" ref={divRef} />

      <div className="bz-map__foot">
        <label className="bz-check">
          <input
            type="checkbox"
            checked={takip}
            onChange={(e) => {
              const v = e.target.checked;
              setTakip(v);
              if (v) yenidenOrtala();
            }}
          />
          Otomatik takip
        </label>
        <button type="button" className="bz-btn bz-btn--sm" onClick={yenidenOrtala}>
          Yeniden ortala
        </button>
        {sonKonum !== null ? (
          <span className="bz-map__coords">
            {sonKonum[0].toFixed(5)}, {sonKonum[1].toFixed(5)}
          </span>
        ) : (
          <span className="bz-map__coords">konum yok</span>
        )}
      </div>

      {karoHata && (
        <p className="bz-footnote">
          Harita kutucukları yüklenemedi — çevrimdışı olabilirsiniz. Son bilinen konum
          korunmaya devam eder.
        </p>
      )}
    </section>
  );
}
