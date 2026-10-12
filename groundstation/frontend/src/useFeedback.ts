import { useCallback, useEffect, useRef, useState } from "react";

export type FeedbackTone = "info" | "error" | "caution";

export interface FeedbackValue {
  text: string;
  tone: FeedbackTone;
  id: number;
}

/**
 * İşlem geri bildirimi (durum / tamamlama / uyarı / hata).
 * Geri bildirim alanı DOM'da kalıcıdır (aria-live için), metin ise zaman
 * aşımında söner.
 */
export function useFeedback(timeoutMs = 3600) {
  const [feedback, setFeedback] = useState<FeedbackValue | null>(null);
  const timer = useRef<number | null>(null);

  useEffect(
    () => () => {
      if (timer.current !== null) window.clearTimeout(timer.current);
    },
    [],
  );

  const show = useCallback(
    (text: string, tone: FeedbackTone = "info") => {
      setFeedback({ text, tone, id: Date.now() });
      if (timer.current !== null) window.clearTimeout(timer.current);
      timer.current = window.setTimeout(() => setFeedback(null), timeoutMs);
    },
    [timeoutMs],
  );

  return { feedback, show };
}
