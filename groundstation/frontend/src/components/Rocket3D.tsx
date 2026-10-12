import { useEffect, useRef } from "react";
import * as THREE from "three";
import { ALT_MAX_M, altOrNull, flightStateName, flightStateTone, menzilDisi, tiltOrNull } from "../telemetry";
import { palette } from "../theme";

// Paylaşılan tipe uyumlu yedek (opsiyonel alanlar — bağımsız derleme için).
interface YedekDurum {
  alt_m?: number | null;
  flight_state?: number | null;
  tilt_ddeg?: number | null;
  lean_x?: number | null;
  lean_y?: number | null;
}

export interface Rocket3DProps {
  alt_m: number | null;
  flight_state: number | null;
  tilt_ddeg: number | null;
  lean_x: number | null;
  lean_y: number | null;
}

function cozAlt(p: Rocket3DProps | YedekDurum): number | null {
  const v = (p as YedekDurum).alt_m;
  // Menzil muhafızı: 400M glitch sahneye/başlığa sızmaz, BİLİNMEYEN olur.
  return altOrNull(typeof v === "number" && Number.isFinite(v) ? v : null);
}

function cozDurum(p: Rocket3DProps | YedekDurum): number | null {
  const v = (p as YedekDurum).flight_state;
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

function cozLean(p: Rocket3DProps | YedekDurum): { x: number; y: number } | null {
  const v = p as YedekDurum;
  if (
    typeof v.lean_x === "number" &&
    typeof v.lean_y === "number" &&
    Number.isFinite(v.lean_x) &&
    Number.isFinite(v.lean_y)
  ) {
    return { x: v.lean_x, y: v.lean_y };
  }
  return null;
}

function cozTilt(p: Rocket3DProps | YedekDurum): number | null {
  const v = (p as YedekDurum).tilt_ddeg;
  return tiltOrNull(typeof v === "number" && Number.isFinite(v) ? v : null);
}

function azHareket(): boolean {
  try {
    return window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  } catch {
    return false;
  }
}

// En az sürükle-döndür + tekerlek-yakınlaştırma (three/examples yok).
function orbitKur(k: THREE.PerspectiveCamera, el: HTMLElement, h: THREE.Vector3): () => void {
  let t = 0.6, p = 1.0, r = 22, s = false, x = 0, y = 0;
  const uyg = () => { k.position.set(h.x + r * Math.sin(p) * Math.sin(t), h.y + r * Math.cos(p), h.z + r * Math.sin(p) * Math.cos(t)); k.lookAt(h); };
  const as = (e: PointerEvent) => { s = true; x = e.clientX; y = e.clientY; try { el.setPointerCapture(e.pointerId); } catch { /* yoksay */ } };
  const hr = (e: PointerEvent) => { if (!s) return; t -= (e.clientX - x) * 0.005; p = Math.min(1.5, Math.max(0.15, p - (e.clientY - y) * 0.005)); x = e.clientX; y = e.clientY; uyg(); };
  const bt = () => { s = false; };
  const wz = (e: WheelEvent) => { e.preventDefault(); r = Math.min(60, Math.max(6, r * (1 + Math.sign(e.deltaY) * 0.1))); uyg(); };
  el.addEventListener("pointerdown", as); window.addEventListener("pointermove", hr); window.addEventListener("pointerup", bt);
  el.addEventListener("wheel", wz, { passive: false }); uyg();
  return () => { el.removeEventListener("pointerdown", as); window.removeEventListener("pointermove", hr); window.removeEventListener("pointerup", bt); el.removeEventListener("wheel", wz); };
}

export default function Rocket3D(props: Rocket3DProps) {
  const kutuRef = useRef<HTMLDivElement | null>(null);
  const altRef = useRef<number | null>(cozAlt(props));
  const durumRef = useRef<number | null>(cozDurum(props));
  const tiltRef = useRef<number | null>(cozTilt(props));
  const leanRef = useRef<{ x: number; y: number } | null>(cozLean(props));

  const alt = cozAlt(props);
  const durum = cozDurum(props);
  const tilt = cozTilt(props);
  const lean = cozLean(props);
  const red = menzilDisi(
    typeof (props as YedekDurum).alt_m === "number" ? (props as YedekDurum).alt_m : null,
    ALT_MAX_M,
  );
  const apoje = durum !== null && durum >= 4;
  // Yön rozeti de gövde gibi tilt + lean birlikte varken yaklaşıktır;
  // yalnız tilt yetmez (yön tanımsız kalır).
  const yonYaklasik = tilt !== null && lean !== null;

  // Render döngüsü prop'ları doğrudan okumaz; en güncel değerler ref'e yazılır.
  useEffect(() => {
    altRef.current = alt;
    durumRef.current = durum;
    tiltRef.current = tilt;
    leanRef.current = lean;
  });

  useEffect(() => {
    const kutu = kutuRef.current;
    if (!kutu) return;
    const sessiz = azHareket();

    const sahne = new THREE.Scene();
    sahne.background = new THREE.Color(palette.deep);
    sahne.fog = new THREE.Fog(palette.deep, 40, 90);

    const kamera = new THREE.PerspectiveCamera(50, 1, 0.1, 200);
    let isleyici: THREE.WebGLRenderer;
    try {
      isleyici = new THREE.WebGLRenderer({ antialias: true });
    } catch (e) {
      const uyari = document.createElement("div");
      uyari.textContent = "3B görünüm kullanılamıyor (WebGL yok): " + String((e as Error)?.message ?? e);
      uyari.style.cssText = "padding:12px;color:#ffa3b0;border:1px solid rgba(255,77,99,.4);border-radius:8px;font-size:12px";
      kutu.appendChild(uyari);
      return;
    }
    isleyici.setClearColor(palette.deep, 1);
    kutu.appendChild(isleyici.domElement);
    isleyici.domElement.style.display = "block";
    isleyici.domElement.style.touchAction = "none";

    sahne.add(new THREE.AmbientLight(0xffffff, 0.7));
    const gunes = new THREE.DirectionalLight(0xffffff, 1.4);
    gunes.position.set(6, 12, 8);
    sahne.add(gunes);

    // Fırlatma rampası
    const rampa = new THREE.Mesh(
      new THREE.CircleGeometry(6, 48),
      new THREE.MeshStandardMaterial({ color: 0x101a2e, roughness: 0.95 }),
    );
    rampa.rotation.x = -Math.PI / 2;
    sahne.add(rampa);
    const izgara = new THREE.GridHelper(24, 24, 0x2a3c5c, 0x1b2942);
    izgara.position.y = 0.01;
    sahne.add(izgara);

    // Roket: burun + gövde + kanatçıklar (daima dikey / nötr poz)
    const roket = new THREE.Group();
    const govdeMat = new THREE.MeshStandardMaterial({ color: 0x2b3446, roughness: 0.45, metalness: 0.7 });
    const maviMat = new THREE.MeshStandardMaterial({ color: palette.info, roughness: 0.35, metalness: 0.4 });
    const govde = new THREE.Mesh(new THREE.CylinderGeometry(0.5, 0.5, 3, 24), govdeMat);
    govde.position.y = 2.2;
    roket.add(govde);
    const burun = new THREE.Mesh(new THREE.ConeGeometry(0.5, 1.2, 24), maviMat);
    burun.position.y = 4.3;
    roket.add(burun);
    const serit = new THREE.Mesh(new THREE.CylinderGeometry(0.52, 0.52, 0.18, 24), maviMat);
    serit.position.y = 3.1;
    roket.add(serit);
    const lule = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.42, 0.35, 20), govdeMat);
    lule.position.y = 0.55;
    roket.add(lule);
    // Motor parlaması: yalnızca gerçek ASCENT bayrağında (flight_state==3) görünür.
    const atesIsigi = new THREE.PointLight(0xffa030, 0, 14, 1.6);
    atesIsigi.position.y = 0.2;
    roket.add(atesIsigi);
    const alev = new THREE.Mesh(
      new THREE.ConeGeometry(0.26, 0.9, 16),
      new THREE.MeshBasicMaterial({ color: 0xffb020, transparent: true, opacity: 0.85 }),
    );
    alev.rotation.x = Math.PI;
    alev.position.y = -0.1;
    alev.visible = false;
    roket.add(alev);
    const kanatGeo = new THREE.BoxGeometry(0.08, 1.0, 0.6);
    for (let i = 0; i < 4; i++) {
      const k = new THREE.Mesh(kanatGeo, i % 2 === 0 ? govdeMat : maviMat);
      const a = (i / 4) * Math.PI * 2;
      k.position.set(Math.sin(a) * 0.55, 1.1, Math.cos(a) * 0.55);
      k.rotation.y = a;
      roket.add(k);
    }
    sahne.add(roket);

    // Rakım çizgisi + işaretçi
    const cizgiGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 1, 0)]);
    const cizgi = new THREE.Line(cizgiGeo, new THREE.LineBasicMaterial({ color: palette.infoStrong }));
    sahne.add(cizgi);
    const isaret = new THREE.Mesh(
      new THREE.SphereGeometry(0.12, 12, 12),
      new THREE.MeshBasicMaterial({ color: palette.infoStrong }),
    );
    sahne.add(isaret);

    // Apoje halkası
    const halka = new THREE.Mesh(
      new THREE.TorusGeometry(1.4, 0.06, 10, 48),
      new THREE.MeshBasicMaterial({ color: palette.infoStrong, transparent: true, opacity: 0.9 }),
    );
    halka.rotation.x = Math.PI / 2;
    sahne.add(halka);

    // Yön oku (yer düzlemi): tilt + lean varken her zaman çizilir. Boy
    // sabittir — yalnızca lean yönünü gösterir; gövde küçük açılarda
    // neredeyse dikey kalsa bile ok ve derece yazısı kıpırdar. Yaw yok.
    const yonOku = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0.06, 0),
      2.4,
      palette.caution,
      0.55,
      0.32,
    );
    yonOku.visible = false;
    sahne.add(yonOku);

    const hedef = new THREE.Vector3(0, 3, 0);
    const orbitBirak = orbitKur(kamera, isleyici.domElement, hedef);

    const boyutlandir = () => {
      const w = kutu.clientWidth || 320;
      const h = kutu.clientHeight || 340;
      isleyici.setSize(w, h);
      kamera.aspect = w / h;
      kamera.updateProjectionMatrix();
    };
    boyutlandir();
    const gozlemci = new ResizeObserver(boyutlandir);
    gozlemci.observe(kutu);
    window.addEventListener("resize", boyutlandir);

    let raf = 0;
    let canli = true;
    const saat = new THREE.Clock();
    const dongu = () => {
      if (!canli) return;
      const t = saat.getElapsedTime();
      const a = altRef.current;
      const yuk = a != null ? (Math.min(Math.max(a, 0), 3000) / 3000) * 18 : 0;
      roket.position.y = yuk; // yalnızca dikey öteleme + yaklaşık eğilme
      // Yaklaşık yönelim (ivmeölçer, yaw yok): tilt açısı lean yönüne yatırır.
      // SADECE rotation.x/z yazılır, rotation.y (yaw) asla değişmez.
      {
        const td = tiltRef.current;
        const ln = leanRef.current;
        if (td != null && ln != null) {
          const mag = Math.hypot(ln.x, ln.y);
          if (mag > 1e-6) {
            const rad = ((td / 10) * Math.PI) / 180;
            const nx = ln.x / mag;
            const ny = ln.y / mag;
            roket.rotation.set(rad * ny, 0, -rad * nx);
            yonOku.visible = true;
            yonOku.setDirection(new THREE.Vector3(nx, 0, ny).normalize());
          } else {
            roket.rotation.set(0, 0, 0);
            // Yön tanımsız: ok gizlenir, derece başlıktaki yazıdan okunur.
            yonOku.visible = false;
          }
        } else {
          roket.rotation.set(0, 0, 0);
          yonOku.visible = false;
        }
      }
      const roketY = yuk + 2.2;
      const pts = [new THREE.Vector3(0.9, 0, 0), new THREE.Vector3(0.9, roketY, 0)];
      cizgiGeo.setFromPoints(pts);
      isaret.position.set(0.9, roketY, 0);
      hedef.set(0, 2 + yuk * 0.5, 0);
      const apojeAktif = durumRef.current !== null && durumRef.current >= 4;
      halka.visible = apojeAktif;
      // Motor parlaması yalnızca gerçek ASCENT bayrağında; yönelim uydurulmaz.
      const atesta = durumRef.current === 3;
      alev.visible = atesta;
      atesIsigi.intensity = atesta ? 2.2 + (sessiz ? 0 : 0.5 * Math.sin(t * 9)) : 0;
      if (apojeAktif) {
        halka.position.y = roketY + 0.6;
        // Hareket azaltma seçiliyse nabız kapatılır, halka sabit kalır.
        if (sessiz) {
          halka.scale.set(1, 1, 1);
          (halka.material as THREE.MeshBasicMaterial).opacity = 0.85;
        } else {
          const nabiz = 1 + 0.08 * Math.sin(t * Math.PI);
          halka.scale.set(nabiz, nabiz, 1);
          (halka.material as THREE.MeshBasicMaterial).opacity = 0.6 + 0.25 * Math.sin(t * Math.PI);
        }
      }
      isleyici.render(sahne, kamera);
      raf = requestAnimationFrame(dongu);
    };
    dongu();

    return () => {
      canli = false;
      cancelAnimationFrame(raf);
      orbitBirak();
      gozlemci.disconnect();
      window.removeEventListener("resize", boyutlandir);
      sahne.traverse((o) => {
        const m = o as THREE.Mesh;
        if (m.geometry) m.geometry.dispose();
        const mat = (m as THREE.Mesh).material as THREE.Material | THREE.Material[] | undefined;
        if (Array.isArray(mat)) mat.forEach((x) => x.dispose());
        else if (mat) mat.dispose();
      });
      cizgiGeo.dispose();
      isleyici.dispose();
      kutu.removeChild(isleyici.domElement);
    };
  }, []);

  const tone = flightStateTone(durum);
  const toneSuffix = tone === "neutral" ? "" : ` bz-footnote__state--${tone}`;

  return (
    <section className="bz-panel bz-rocket" aria-label="Roket üç boyutlu görünüm">
      <header className="bz-panel__head">
        <h2 className="bz-panel__title">Roket 3B</h2>
        {yonYaklasik ? (
          <span className="bz-chip bz-chip--info">yaklaşık yönelim (ivmeölçer)</span>
        ) : (
          <span className="bz-chip bz-chip--caution">Yön verisi yok · nötr poz</span>
        )}
        {apoje && <span className="bz-chip bz-chip--info">Apoje</span>}
        {red && <span className="bz-chip bz-chip--warning">Menzil dışı · reddedildi</span>}
        <span className="bz-panel__meta">
          Rakım: {alt != null ? `${alt.toFixed(1)} m` : "—"}
          {yonYaklasik && tilt !== null ? ` · eğik ${(tilt / 10).toFixed(1)}°` : ""}
        </span>
      </header>

      <div className="bz-rocket__canvas" ref={kutuRef} />

      <p className="bz-footnote">
        <span className={`bz-footnote__state${toneSuffix}`}>
          {flightStateName(durum)}
        </span>
        {" · "}Sürükle: döndür · Tekerlek: yakınlaştır.{" "}
        {yonYaklasik
          ? "Eğilme yaklaşık yönelimdir (ivmeölçer, yaw yok); yaw asla uydurulmaz. Yerdeki kehribar ok eğilme yönünü gösterir (boy sabit, derece gerçektir)."
          : "Pakette canlı yönelim (attitude) alanı bulunmadığı için model nötr pozda durur; yalnızca irtifaya göre yükselir."}
      </p>
    </section>
  );
}
