import type { ReactNode } from "react";
import type { FeedbackValue } from "../useFeedback";

/** Geri bildirim balonu. Alan hep DOM'da kalır (aria-live), balon göründüğünde söner. */
export function Feedback({ value }: { value: FeedbackValue | null }): ReactNode {
  return (
    <div className="bz-feedback" aria-live="polite">
      {value === null ? null : (
        <p
          key={value.id}
          className={value.tone === "info" ? "bz-toast" : `bz-toast bz-toast--${value.tone}`}
        >
          {value.text}
        </p>
      )}
    </div>
  );
}

export default Feedback;
