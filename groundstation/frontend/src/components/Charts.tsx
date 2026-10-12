import type { ReactNode } from "react";
import {
  CartesianGrid,
  Line,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from "recharts";
import type { HistoryPoint } from "../telemetry";
import { ALT_MAX_M, altOrNull, VEL_MAX_CMS, velOrNull } from "../telemetry";
import { palette } from "../theme";

interface ChartsProps {
  history: HistoryPoint[];
}

type Row = Record<string, number | null>;

const MAX_POINTS = 600;

function bounded(history: HistoryPoint[]): HistoryPoint[] {
  if (history.length <= MAX_POINTS) return history;
  return history.slice(history.length - MAX_POINTS);
}

const axisTick = {
  fill: palette.textMute,
  fontSize: 11,
  fontFamily: "ui-monospace, Consolas, monospace",
};

const axisLine = { stroke: palette.grid };

const tooltipStyle = {
  background: palette.surface,
  border: `1px solid ${palette.line}`,
  borderRadius: 8,
  fontFamily: "ui-monospace, Consolas, monospace",
  fontSize: 12,
  color: palette.text,
};

function EmptyNote({ note }: { note: string }): ReactNode {
  return <div className="bz-empty">{note}</div>;
}

/** Y ekseni kelepçe öğesi (recharts AxisDomainItem ile aynı küme). */
type AlanOge = string | number | ((d: number) => string | number);

interface ChartSpec {
  title: string;
  /** Grafik başlığındaki birim. */
  unit: string;
  /** Y eksenindeki birim etiketi. */
  yUnit: string;
  data: Row[];
  dataKey: string;
  name: string;
  stroke: string;
  hasData: boolean;
  emptyNote: string;
  format: (v: unknown) => string;
  minZero?: boolean;
  /** Y ekseni kelepçesi: menzil dışı glitch ekseni patlatamaz. */
  domain?: readonly [AlanOge, AlanOge];
}

function Chart(spec: ChartSpec): ReactNode {
  return (
    <section className="bz-panel bz-chart">
      <header className="bz-panel__head">
        <h2 className="bz-panel__title">{spec.title}</h2>
        <span className="bz-chip">{spec.unit}</span>
      </header>
      <div className="bz-chart__canvas">
        {!spec.hasData ? (
          <EmptyNote note={spec.emptyNote} />
        ) : (
          <ResponsiveContainer width="100%" height={220}>
            <LineChart data={spec.data} margin={{ top: 8, right: 12, bottom: 22, left: 0 }}>
              <CartesianGrid stroke={palette.grid} strokeDasharray="3 3" />
              <XAxis
                dataKey="t"
                tick={axisTick}
                axisLine={axisLine}
                tickLine={axisLine}
                tickFormatter={(v: number) => v.toFixed(0)}
                label={{ value: "T+ (sn)", position: "bottom", offset: 0, fill: palette.textMute, fontSize: 10 }}
              />
              <YAxis
                width={48}
                tick={axisTick}
                axisLine={axisLine}
                tickLine={axisLine}
                domain={
                  spec.domain ??
                  (spec.minZero === true ? [0, "auto"] : ["auto", "auto"])
                }
                label={{ value: spec.yUnit, angle: -90, position: "insideLeft", offset: 10, fill: palette.textMute, fontSize: 10 }}
              />
              <Tooltip
                contentStyle={tooltipStyle}
                labelFormatter={(v: unknown) => `T+ ${Number(v).toFixed(1)} sn`}
                formatter={(v: unknown) => [v === undefined || v === null ? "--" : spec.format(v), spec.name]}
              />
              <Line
                type="monotone"
                dataKey={spec.dataKey}
                name={spec.name}
                stroke={spec.stroke}
                dot={false}
                strokeWidth={2}
                connectNulls
                isAnimationActive={false}
              />
            </LineChart>
          </ResponsiveContainer>
        )}
      </div>
    </section>
  );
}

/** Canlı grafikler: yalnızca şemadaki alanlar, eksenler birim etiketli. */
export function Charts({ history }: ChartsProps): ReactNode {
  const data = bounded(history);
  // Menzil muhafızı (çift katman: pushHistory zaten temizler, kayıt tekrar
  // oynatma ve eski backend'e karşı burada da): dışı null → boşluk çizilir.
  const rows: Row[] = data.map((p) => ({ t: p.t, alt_m: altOrNull(p.alt_m), vel_cms: velOrNull(p.vel_cms), sats: p.sats, rate_hz: p.rate_hz }));
  const velRows: Row[] = data.map((p) => {
    const v = velOrNull(p.vel_cms);
    return { t: p.t, vel_ms: v === null ? null : v / 100 };
  });
  const hasAlt = rows.some((r) => r.alt_m !== null);
  const hasVel = velRows.some((r) => r.vel_ms !== null);
  const hasSats = rows.some((r) => r.sats !== null);
  // Eksen kelepçeleri: veri temizliği aşılsa bile eksen menzilde kalır.
  const altDomain: readonly [AlanOge, AlanOge] = [
    (d: number) => Math.max(d, -1000),
    (d: number) => Math.min(d, ALT_MAX_M),
  ];
  const velDomain: readonly [AlanOge, AlanOge] = [
    (d: number) => Math.max(d, -VEL_MAX_CMS / 100),
    (d: number) => Math.min(d, VEL_MAX_CMS / 100),
  ];

  return (
    <div className="bz-charts">
      <Chart
        title="Göreli irtifa — zaman"
        unit="metre"
        yUnit="m"
        data={rows}
        dataKey="alt_m"
        name="Göreli irtifa"
        stroke={palette.info}
        hasData={hasAlt}
        emptyNote="Göreli irtifa verisi yok — telemetri paketleri bekleniyor"
        format={(v) => `${Number(v).toFixed(1)} m`}
        domain={altDomain}
      />
      <Chart
        title="Dikey hız — zaman"
        unit="metre/saniye"
        yUnit="m/s"
        data={velRows}
        dataKey="vel_ms"
        name="Dikey hız"
        stroke={palette.white}
        hasData={hasVel}
        emptyNote="Dikey hız verisi yok — telemetri paketleri bekleniyor"
        format={(v) => `${Number(v).toFixed(2)} m/s`}
        domain={velDomain}
      />
      <Chart
        title="Paket hızı — zaman"
        unit="ölçülen hertz"
        yUnit="Hz"
        data={rows}
        dataKey="rate_hz"
        name="Paket hızı"
        stroke={palette.caution}
        hasData={rows.length > 0}
        emptyNote="Paket akışı yok — bağlantı kurulmadı"
        format={(v) => `${Number(v).toFixed(1)} Hz`}
        minZero
      />
      <Chart
        title="Uydu sayısı — zaman"
        unit="adet"
        yUnit="adet"
        data={rows}
        dataKey="sats"
        name="Uydu"
        stroke={palette.infoPale}
        hasData={hasSats}
        emptyNote="Uydu sayısı bilinmiyor — telemetri paketleri bekleniyor"
        format={(v) => `${Number(v).toFixed(0)} adet`}
        minZero
      />
    </div>
  );
}

export default Charts;
