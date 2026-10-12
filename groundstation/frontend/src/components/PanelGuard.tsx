import { Component, type ReactNode } from "react";

/**
 * Panelleri sarmalayan hata sınırı: bir görünüm çökerse uygulamanın tamamı
 * değil yalnızca o panel kaybolur.
 */
export class PanelGuard extends Component<{ name: string; children: ReactNode }, { err: string | null }> {
  state = { err: null as string | null };
  static getDerivedStateFromError(e: unknown) {
    return { err: String((e as Error)?.message ?? e) };
  }
  render() {
    if (this.state.err) {
      return (
        <div className="bz-panel bz-banner--down" role="alert">
          <p className="bz-banner__title">{this.props.name} açılamadı</p>
          <p className="bz-banner__detail bz-mono">{this.state.err}</p>
        </div>
      );
    }
    return this.props.children;
  }
}
