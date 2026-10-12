import { useEffect, useRef, type ReactNode } from "react";

interface IntroProps {
  onClose: () => void;
}

/** İlk açılış tanıtım katmanı: tıkla veya Esc ile geç; hareket azaltma seçiliyse hiç açılmaz. */
export function Intro({ onClose }: IntroProps): ReactNode {
  const skipRef = useRef<HTMLButtonElement | null>(null);

  useEffect(() => {
    skipRef.current?.focus();
    const h = (e: KeyboardEvent) => {
      if (e.key === "Escape" || e.key === " ") {
        e.preventDefault();
        onClose();
      }
    };
    window.addEventListener("keydown", h);
    return () => window.removeEventListener("keydown", h);
  }, [onClose]);

  return (
    <div
      className="bz-intro"
      role="dialog"
      aria-modal="true"
      aria-label="BOZOK tanıtım videosu"
      onClick={onClose}
    >
      <video className="bz-intro__video" src="/intro.mp4" autoPlay muted playsInline />
      <p className="bz-intro__hint">Atlamak için ekrana tıklayın veya Esc tuşuna basın</p>
      <button ref={skipRef} type="button" className="bz-btn bz-intro__skip" onClick={onClose}>
        Geç (Esc)
      </button>
    </div>
  );
}

export default Intro;
