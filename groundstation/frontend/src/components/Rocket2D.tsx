import { useEffect, useRef } from "react";
import { ALT_MAX_M, altOrNull, flightStateName, flightStateTone, menzilDisi, tiltOrNull } from "../telemetry";

/**
 * Roket 2B — yan görünüm (saf SVG + CSS geçişleri, ek bağımlılık yok).
 *
 * Sözleşmeye sadık: yalnızca paketteki alanlar çizilir.
 *   alt_m       → 0–3000 m ölçeğinde tırmanış (ölçek üstü ayrı etiketle söylenir)
 *   menzil dışı (|alt|>100km, örn 400M glitch) → BİLİNMEYEN: roket rampada,
 *               "Menzil dışı · reddedildi" rozeti; apoje/geçmiş zehirlenmez
 *   tilt_ddeg   → egiklik açısı; lean_x/lean_y → egilme yönü (gövde ekseni)
 *   flight_state==3 (YÜKSELİŞ) → yalnız o zaman motor parlaması
 *   flight_state>=4            → apoje işareti (gözlenen en yüksek irtifa)
 *   tilt/lean yok              → nötr poz + dürüst etiket
 *
 * Hareket: kritik sönümlü CSS eğrisi (aşım yok), prefers-reduced-motion
 * altında transform geçişi tamamen kapanır (bkz. index.css).
 */
export interface Rocket2DProps {
  alt_m: number | null;
  flight_state: number | null;
  tilt_ddeg: number | null;
  lean_x: number | null;
  lean_y: number | null;
}

const OLCU_MAX = 3000;
const ZEMIN_Y = 312;
const TEPE_Y = 26;
const ROKET_BOY = 76;
/** İrtifa ekseni payı: taban 0 m'de zeminde, 3000 m'de tepenin hemen altında. */
const ATIM = ZEMIN_Y - (TEPE_Y + ROKET_BOY);
const MERKEZ_X = 342;
const TICKS = [0, 1000, 2000, 3000];

/** İrtifa → roket tabanının SVG y koordinatı (ölçek dışı değerler kırpılır). */
function yBase(alt: number | null): number {
  if (alt === null || !Number.isFinite(alt)) return ZEMIN_Y;
  const f = Math.min(Math.max(alt, 0), OLCU_MAX) / OLCU_MAX;
  return ZEMIN_Y - f * ATIM;
}

function num(v: number | null): boolean {
  return typeof v === "number" && Number.isFinite(v);
}

export default function Rocket2D(props: Rocket2DProps) {
  const hamAlt = num(props.alt_m) ? props.alt_m : null;
  // Menzil muhafızı: 400M glitch ölçeğe değil BİLİNMEYEN'e düşer; roket
  // rampada kalır, apoje işareti zehirlenmez. Ölçek kelepçesi korunur.
  const alt = altOrNull(hamAlt);
  const red = menzilDisi(hamAlt, ALT_MAX_M);
  const durum = num(props.flight_state) ? props.flight_state : null;
  const tilt = tiltOrNull(num(props.tilt_ddeg) ? props.tilt_ddeg : null);
  const lx = num(props.lean_x) ? props.lean_x : null;
  const ly = num(props.lean_y) ? props.lean_y : null;

  const yonVar = tilt !== null && lx !== null && ly !== null;

  // Görünüm düzlemi (ekran X = gövde X): eğilmenin derinlikteki bileşeni
  // boy kısalması olarak çizilir — düz bir yan görünümde başka türlü
  // gösterilemez; hiçbir bileşen uydurulmaz. Ölü bant olduğu gibi kalır;
  // küçük açılarda gövde neredeyse dikey görünür — canlılık bunun yerine
  // aşağıdaki yön okundan ve derece yazısından gelir (her zaman çizilir).
  let aci = 0;
  let kisa = 1;
  let yonX: number | null = null;
  let yonY: number | null = null;
  if (yonVar) {
    const mag = Math.hypot(lx as number, ly as number);
    if (mag > 1e-6) {
      const nx = (lx as number) / mag;
      const ny = (ly as number) / mag;
      yonX = nx;
      yonY = ny;
      const th = (((tilt as number) / 10) * Math.PI) / 180;
      const s = Math.sin(th);
      const c = Math.cos(th);
      aci = (Math.atan2(s * nx, c) * 180) / Math.PI;
      kisa = Math.sqrt(Math.max(0, 1 - (s * ny) * (s * ny)));
    }
  }
  const egikDeg = tilt !== null ? (tilt as number) / 10 : null;

  // Gözlenen en yüksek irtifa: apoje işareti bunu gösterir (uydurma yok).
  const apojeRef = useRef<number | null>(null);
  useEffect(() => {
    if (alt === null) return;
    apojeRef.current = apojeRef.current === null ? alt : Math.max(apojeRef.current, alt);
  }, [alt]);

  const apojeAktif = durum !== null && durum >= 4;
  const apoje = apojeRef.current;
  const atesta = durum === 3;
  const olcekUstu = alt !== null && alt > OLCU_MAX;
  const tone = flightStateTone(durum);
  const toneSuffix = tone === "neutral" ? "" : ` bz-footnote__state--${tone}`;

  const aria = `Roket yan görünümü: göreli irtifa ${
    alt !== null ? `${alt.toFixed(0)} metre` : "bilinmiyor"
  }, uçuş durumu ${flightStateName(durum)}, yönelim ${
    yonVar ? "yaklaşık (ivmeölçer)" : "veri yok, nötr poz"
  }${atesta ? ", motor parlaması açık" : ""}`;

  return (
    <section className="bz-panel bz-r2d" aria-label="Roket iki boyutlu yan görünüm">
      <header className="bz-panel__head">
        <h2 className="bz-panel__title">Roket 2B · yan görünüm</h2>
        {yonVar ? (
          <span className="bz-chip bz-chip--info">yaklaşık yönelim (ivmeölçer, yaw yok)</span>
        ) : (
          <span className="bz-chip bz-chip--caution">Yön verisi yok · nötr poz</span>
        )}
        {apojeAktif && <span className="bz-chip bz-chip--info">Apoje</span>}
        {alt === null && !red && <span className="bz-chip bz-chip--caution">İrtifa yok</span>}
        {red && <span className="bz-chip bz-chip--warning">Menzil dışı · reddedildi</span>}
        {olcekUstu && <span className="bz-chip bz-chip--caution">Ölçek üstü</span>}
        <span className="bz-panel__meta">
          Göreli irtifa: {alt !== null ? `${alt.toFixed(1)} m` : "—"}
          {yonVar && egikDeg !== null ? ` · eğik ${egikDeg.toFixed(1)}°` : ""}
        </span>
      </header>

      <figure className="bz-r2d__canvas">
        <svg
          className="bz-r2d__svg"
          viewBox="0 0 640 340"
          preserveAspectRatio="xMidYMid meet"
          role="img"
          aria-label={aria}
        >
          {/* İrtifa ızgarası + etiketler (roket tabanıyla aynı ölçek) */}
          {TICKS.map((t) => {
            const y = yBase(t);
            return (
              <g key={t}>
                <line className="bz-r2d__grid" x1="54" x2="630" y1={y} y2={y} />
                <text className="bz-r2d__tick" x="46" y={y} dy="0.32em" textAnchor="end">
                  {t}
                </text>
              </g>
            );
          })}
          <text className="bz-r2d__tick" x="46" y={yBase(3000) - 14} textAnchor="end">
            m
          </text>
          <line className="bz-r2d__axis" x1="54" y1={TEPE_Y} x2="54" y2={ZEMIN_Y} />

          {/* Apoje işareti: yalnız flight_state >= 4 */}
          {apojeAktif && apoje !== null && (
            <g className="bz-r2d__apogee">
              <line x1="54" x2="630" y1={yBase(apoje)} y2={yBase(apoje)} />
              <text x="630" y={yBase(apoje) - 8} textAnchor="end">
                APOJE (gözlenen) ≈ {apoje.toFixed(0)} m
              </text>
            </g>
          )}

          {/* Zemin + rampa */}
          <rect className="bz-r2d__soil" x="54" y={ZEMIN_Y} width="576" height="28" />
          <line className="bz-r2d__ground" x1="54" x2="630" y1={ZEMIN_Y} y2={ZEMIN_Y} />
          <rect className="bz-r2d__pad" x="324" y={ZEMIN_Y} width="36" height="6" />

          {/* Tırmanma izi */}
          <line
            className="bz-r2d__trail"
            x1={MERKEZ_X}
            y1={ZEMIN_Y}
            x2={MERKEZ_X}
            y2={yBase(alt)}
          />

          {/* Yön oku (üstten bakış): tilt varken her zaman çizilir. Ok boyu
              sabit — yalnızca lean_x/lean_y yönünü gösterir; sayı gerçektir
              (tilt/10). Yön tanımsızsa (mag≈0) nokta + derece kalır. */}
          {yonVar && (
            <g
              transform="translate(572, 248)"
              role="img"
              aria-label={
                egikDeg !== null
                  ? `Eğilme yönü oku: ${egikDeg.toFixed(1)} derece, yaw yok`
                  : "Eğilme yönü oku"
              }
            >
              <circle className="bz-r2d__lean-ring" r="24" />
              <line className="bz-r2d__lean-cross" x1="-24" x2="24" y1="0" y2="0" />
              <line className="bz-r2d__lean-cross" x1="0" x2="0" y1="-24" y2="24" />
              <line
                className="bz-r2d__lean-arrow"
                x1="0"
                y1="0"
                x2={(yonX ?? 0) * 17}
                y2={-(yonY ?? 0) * 17}
              />
              <circle
                className="bz-r2d__lean-tip"
                cx={(yonX ?? 0) * 17}
                cy={-(yonY ?? 0) * 17}
                r="3"
              />
              <text className="bz-r2d__lean-deg" y="42" textAnchor="middle">
                {egikDeg !== null ? `${egikDeg.toFixed(1)}°` : "--"}
              </text>
              <text className="bz-r2d__lean-cap" y="54" textAnchor="middle">
                yaw yok
              </text>
            </g>
          )}

          {/* Roket: konum/eğim yalnızca transform ile (kritik sönümlü geçiş) */}
          <g
            className="bz-r2d__rocket"
            style={{
              transform: `translate(${MERKEZ_X}px, ${yBase(alt)}px) rotate(${aci}deg) scale(1, ${kisa})`,
              transformOrigin: "0px 0px",
              transformBox: "view-box",
            }}
          >
            <ellipse className={`bz-r2d__glow${atesta ? " is-on" : ""}`} cx="0" cy="14" rx="20" ry="26" />
            <polygon className={`bz-r2d__flame${atesta ? " is-on" : ""}`} points="0,30 -7,0 7,0" />
            <path className="bz-r2d__nozzle" d="M -5,-9 L 5,-9 L 7,0 L -7,0 Z" />
            <rect className="bz-r2d__body" x="-9" y="-52" width="18" height="43" />
            <rect className="bz-r2d__band" x="-9" y="-36" width="18" height="6" />
            <polygon className="bz-r2d__fin" points="-9,-14 -22,0 -9,0" />
            <polygon className="bz-r2d__fin" points="9,-14 22,0 9,0" />
            <polygon className="bz-r2d__nose" points="0,-76 -9,-52 9,-52" />
          </g>
        </svg>
      </figure>

      <p className="bz-footnote">
        <span className={`bz-footnote__state${toneSuffix}`}>{flightStateName(durum)}</span>
        {" · "}İrtifa ekseni roket tabanına göredir (0–3000 m).{" "}
        {alt === null
          ? "Paket alanında irtifa yok — roket rampada duruyor. "
          : ""}
        {yonVar
          ? "Eğilme yaklaşık yönelimdir (ivmeölçer, yaw yok); derinlikteki bileşen boy kısalması olarak çizilir. Sağ alttaki ok eğilme yönünü üstten bakışla gösterir (ok boyu sabit, derece gerçektir); gövde küçük açılarda neredeyse dikey kalır."
          : "Göreli irtifa kalkış noktasının kalibre edilmiş barometrik basıncına göredir (deniz seviyesi rakımı değildir). Paket canlı yönelim taşımıyorsa model nötr pozda durur; yalnızca irtifaya göre yükselir."}
      </p>
    </section>
  );
}
