import React from 'react';
import {
	AbsoluteFill,
	Img,
	Sequence,
	staticFile,
	useCurrentFrame,
	useVideoConfig,
	interpolate,
	spring,
	Easing,
} from 'remotion';

/* ==========================================================================
   BOZOK — 10s title sequence, 1920x1080 @ 30fps (300 frames).

   Beat sheet
     0.0-1.5s  ATMOSPHERE  dark field, technical rules draw in, ignition glow
     1.5-3.5s  REVEAL      two authored crops of the liftoff plate (markings,
                           nozzle/flame) + the indoor second angle strip
     3.5-6.0s  MOMENTUM    opposing letterbox pans, kinetic typography, card
     6.0-8.0s  TITLE       restrained BOZOK lockup
     8.0-10.0s SLATE       clean hand-off to mission control

   Every value is authored: no Math.random anywhere, so frames are
   bit-identical on every render.
   ========================================================================== */

/* ---------- tokens ---------- */
const INK = '#05070d';
const PAPER = '#f2f4f7';
const RED = '#e63329';
const SLATE = '#0d1420'; // ground-station background — the slate matches it
const MUTED = '#8ea0b8';
const HAIR = 'rgba(242,244,247,0.14)';
const HAIR_STRONG = 'rgba(242,244,247,0.30)';
const MONO = 'ui-monospace, SFMono-Regular, Menlo, Consolas, monospace';
const SANS = "system-ui, -apple-system, 'Segoe UI', Roboto, Arial, sans-serif";

/* ---------- easing / motion ----------
   ease-out expo-ish for reveals and settles (fast arrival, long tail),
   ease-in-out for camera moves so nothing starts or stops abruptly. */
const EO = Easing.bezier(0.16, 1, 0.3, 1);
const EIO = Easing.bezier(0.65, 0, 0.35, 1);
/* damping ratio 1.0 -> critically damped, no overshoot (Apple default) */
const CRIT = {damping: 20, stiffness: 100, mass: 1};
/* damping ratio ~0.9 -> a little carry, only where something was thrown */
const CARRY = {damping: 17, stiffness: 90, mass: 1};

const ip = (
	f: number,
	range: number[],
	out: number[],
	easing?: (t: number) => number
): number =>
	interpolate(f, range, out, {
		extrapolateLeft: 'clamp',
		extrapolateRight: 'clamp',
		easing,
	});

/* ---------- source plates ---------- */
const HERO = {w: 1600, h: 1065}; // 1.jpeg — liftoff
const HANGAR = {w: 439, h: 224}; // 3.png — indoor second angle

type Rect = {x: number; y: number; w: number; h: number};

/* Authored crop window inside a plate, with a parallax offset so the frame
   can pan across the still (a camera move, not a zoomed photo). */
const Crop: React.FC<{
	src: string;
	srcW: number;
	srcH: number;
	crop: Rect;
	w: number;
	h: number;
	ox?: number;
	oy?: number;
	boost?: number;
}> = ({src, srcW, srcH, crop, w, h, ox = 0, oy = 0, boost = 1}) => {
	const s = Math.max(w / crop.w, h / crop.h) * boost;
	return (
		<Img
			src={staticFile(src)}
			style={{
				position: 'absolute',
				maxWidth: 'none',
				width: srcW * s,
				height: srcH * s,
				left: w / 2 - (crop.x + crop.w / 2) * s + ox,
				top: h / 2 - (crop.y + crop.h / 2) * s + oy,
			}}
		/>
	);
};

/* Framed panel: inner hairline + soft lift, clips its crop. */
const Plate: React.FC<{
	x: number;
	y: number;
	w: number;
	h: number;
	clip?: string;
	opacity?: number;
	shadow?: string;
	children: React.ReactNode;
}> = ({x, y, w, h, clip, opacity = 1, shadow, children}) => (
	<div
		style={{
			position: 'absolute',
			left: x,
			top: y,
			width: w,
			height: h,
			overflow: 'hidden',
			opacity,
			clipPath: clip,
			backgroundColor: '#04060b',
			boxShadow: shadow
				? `inset 0 0 0 1px ${HAIR_STRONG}, ${shadow}`
				: `inset 0 0 0 1px ${HAIR_STRONG}, 0 40px 90px rgba(0,0,0,0.6)`,
		}}
	>
		{children}
	</div>
);

/* Small technical label: red tick + mono caps. */
const Tag: React.FC<{
	x: number;
	y: number;
	text: string;
	opacity?: number;
	dx?: number;
	color?: string;
	tracking?: number;
	size?: number;
}> = ({x, y, text, opacity = 1, dx = 0, color = PAPER, tracking = 6, size = 17}) => (
	<div
		style={{
			position: 'absolute',
			left: x,
			top: y,
			display: 'flex',
			alignItems: 'center',
			gap: 12,
			opacity,
			transform: `translateX(${dx}px)`,
		}}
	>
		<div style={{width: 18, height: 2, backgroundColor: color === RED ? PAPER : RED}} />
		<span
			style={{
				fontFamily: MONO,
				fontSize: size,
				letterSpacing: tracking,
				color,
				whiteSpace: 'nowrap',
			}}
		>
			{text}
		</span>
	</div>
);

/* ==========================================================================
   SCENE 1 — ATMOSPHERE (0 - 1.5s)
   ========================================================================== */
const RAILS = [408, 960, 1512];

const SceneAtmosphere: React.FC = () => {
	const l = useCurrentFrame();
	const o = ip(l, [0, 6], [0, 1]);

	const glow = ip(l, [10, 44], [0, 1], EIO);
	const horizon = ip(l, [6, 30], [0, 1], EO);
	const corners = ip(l, [8, 26], [0, 1], EO);
	const label = ip(l, [16, 34], [0, 1], EO);
	const labelTrack = ip(l, [16, 44], [13, 7], EO);
	const cross = ip(l, [24, 40], [0, 1], EO);

	return (
		<AbsoluteFill style={{backgroundColor: INK, opacity: o}}>
			{/* ignition light waiting below the frame */}
			<AbsoluteFill
				style={{
					background: `radial-gradient(130% 74% at 50% 122%, rgba(230,51,41,${(
						0.22 * glow
					).toFixed(3)}) 0%, rgba(230,51,41,0.05) 34%, transparent 64%)`,
				}}
			/>

			{/* vertical technical rails drawing upward */}
			{RAILS.map((x, i) => {
				const p = ip(l, [2 + i * 5, 28 + i * 5], [0, 1], EO);
				return (
					<div
						key={x}
						style={{
							position: 'absolute',
							left: x,
							top: 150,
							width: 1,
							height: 760,
							backgroundColor: HAIR,
							transform: `scaleY(${p})`,
							transformOrigin: 'bottom',
						}}
					/>
				);
			})}

			{/* horizon rule + measure ticks */}
			<div
				style={{
					position: 'absolute',
					left: 960 - (1160 * horizon) / 2,
					top: 628,
					width: 1160 * horizon,
					height: 1,
					backgroundColor: RED,
					opacity: 0.95,
					boxShadow: '0 0 22px rgba(230,51,41,0.55)',
				}}
			/>
			<div
				style={{
					position: 'absolute',
					left: 960 - (720 * horizon) / 2,
					top: 636,
					width: 720 * horizon,
					height: 1,
					backgroundColor: PAPER,
					opacity: 0.22,
				}}
			/>
			{Array.from({length: 13}, (_, i) => {
				const t = ip(l, [18 + i * 1.4, 34 + i * 1.4], [0, 1], EO);
				const x = 384 + i * 96;
				return (
					<div
						key={i}
						style={{
							position: 'absolute',
							left: x,
							top: 630,
							width: 1,
							height: 10,
							backgroundColor: PAPER,
							opacity: 0.28 * t,
						}}
					/>
				);
			})}

			{/* centre crosshair */}
			<div style={{position: 'absolute', left: 934, top: 628, width: 52, height: 1, backgroundColor: PAPER, opacity: 0.5 * cross}} />
			<div style={{position: 'absolute', left: 960, top: 602, width: 1, height: 52, backgroundColor: PAPER, opacity: 0.5 * cross}} />

			{/* corner brackets */}
			{[
				{left: 76, top: 76, b: 'borderLeft,borderTop'},
				{right: 76, top: 76, b: 'borderRight,borderTop'},
				{left: 76, bottom: 76, b: 'borderLeft,borderBottom'},
				{right: 76, bottom: 76, b: 'borderRight,borderBottom'},
			].map((c, i) => {
				const sides = c.b.split(',');
				return (
					<div
						key={i}
						style={{
							position: 'absolute',
							left: c.left,
							right: c.right,
							top: c.top,
							bottom: c.bottom,
							width: 28,
							height: 28,
							opacity: 0.55 * corners,
							borderLeft: sides.includes('borderLeft') ? `1px solid ${HAIR_STRONG}` : undefined,
							borderRight: sides.includes('borderRight') ? `1px solid ${HAIR_STRONG}` : undefined,
							borderTop: sides.includes('borderTop') ? `1px solid ${HAIR_STRONG}` : undefined,
							borderBottom: sides.includes('borderBottom') ? `1px solid ${HAIR_STRONG}` : undefined,
						}}
					/>
				);
			})}

			{/* only word on screen: mission control */}
			<div
				style={{
					position: 'absolute',
					left: 96,
					top: 918,
					display: 'flex',
					alignItems: 'center',
					gap: 14,
					opacity: label,
				}}
			>
				<div style={{width: 20, height: 2, backgroundColor: RED}} />
				<span
					style={{
						fontFamily: MONO,
						fontSize: 19,
						letterSpacing: labelTrack,
						color: PAPER,
					}}
				>
					MISSION CONTROL
				</span>
			</div>
			<div
				style={{
					position: 'absolute',
					left: 96,
					top: 890,
					width: 240 * label,
					height: 1,
					backgroundColor: HAIR,
				}}
			/>
		</AbsoluteFill>
	);
};

/* ==========================================================================
   SCENE 2 — REVEAL (1.5 - 3.5s)
   Authored crops of the liftoff plate + the indoor second-angle strip.
   ========================================================================== */
const PA = {x: 120, y: 176, w: 440, h: 648};
const CROP_A: Rect = {x: 640, y: 96, w: 330, h: 430}; // ÇETİN 03 + BOZOK markings

const PB = {x: 760, y: 190, w: 820, h: 520};
const CROP_B: Rect = {x: 540, y: 400, w: 540, h: 500}; // nozzle, flame, smoke

const PC = {x: 1180, y: 790, w: 520, h: 200};
const CROP_C: Rect = {x: 0, y: 44, w: 439, h: 168}; // 3.png, posters cropped off

const SceneReveal: React.FC = () => {
	const l = useCurrentFrame();
	const {fps} = useVideoConfig();

	/* panel A — revealed bottom-up like the rocket leaving the rail */
	const wA = ip(l, [4, 22], [0, 1], EO);
	const driftA = ip(l, [0, 60], [120, 84], EIO);
	const parA = ip(l, [0, 60], [26, -20], EIO);
	const parAx = ip(l, [0, 60], [0, -14], EIO);
	const tagA = ip(l, [20, 34], [0, 1], EO);

	/* panel B — wipes in from the left, camera settles down the flame */
	const wB = ip(l, [20, 38], [0, 1], EO);
	const parB = ip(l, [18, 60], [26, -18], EIO);
	const tagB = ip(l, [36, 50], [0, 1], EO);

	/* panel C — thrown into place, carries a touch of momentum */
	const sC = spring({frame: l - 30, fps, config: CARRY});
	const yC = interpolate(sC, [0, 1], [70, 0]);
	const tagC = ip(l, [42, 54], [0, 1], EO);

	const baseline = ip(l, [0, 24], [0, 1], EO);

	return (
		<AbsoluteFill style={{backgroundColor: INK, opacity: ip(l, [0, 6], [0, 1])}}>
			{/* technical baseline */}
			<div
				style={{
					position: 'absolute',
					left: 120,
					top: 1034,
					width: 1580 * baseline,
					height: 1,
					backgroundColor: HAIR,
				}}
			/>

			{/* A — body markings */}
			<Tag x={driftA} y={140} text="ROKET TAKIMI" opacity={tagA} dx={(1 - tagA) * -14} />
			<Plate
				x={driftA}
				y={PA.y}
				w={PA.w}
				h={PA.h}
				clip={`inset(${(1 - wA) * 100}% 0 0 0)`}
			>
				<Crop
					src="1.jpeg"
					srcW={HERO.w}
					srcH={HERO.h}
					crop={CROP_A}
					w={PA.w}
					h={PA.h}
					ox={parAx}
					oy={parA}
				/>
			</Plate>

			{/* B — nozzle + flame */}
			<Tag x={PB.x} y={154} text="ATEŞLEME" opacity={tagB} dx={(1 - tagB) * 16} />
			<Plate
				x={PB.x}
				y={PB.y}
				w={PB.w}
				h={PB.h}
				clip={`inset(0 ${(1 - wB) * 100}% 0 0)`}
			>
				<Crop
					src="1.jpeg"
					srcW={HERO.w}
					srcH={HERO.h}
					crop={CROP_B}
					w={PB.w}
					h={PB.h}
					oy={parB}
				/>
			</Plate>

			{/* C — second angle, indoor */}
			<Tag x={PC.x} y={754} text="BOZOK" opacity={tagC} dx={(1 - tagC) * 12} />
			<Plate x={PC.x} y={PC.y + yC} w={PC.w} h={PC.h} opacity={sC > 0 ? 1 : 0}>
				<Crop
					src="3.png"
					srcW={HANGAR.w}
					srcH={HANGAR.h}
					crop={CROP_C}
					w={PC.w}
					h={PC.h}
					oy={ip(l, [34, 60], [8, -6], EIO)}
				/>
			</Plate>
		</AbsoluteFill>
	);
};

/* ==========================================================================
   SCENE 3 — MOMENTUM (3.5 - 6s)
   Two letterbox pans across the liftoff plate travelling in opposite
   directions, kinetic type between them, second-angle card riding along.
   ========================================================================== */
/* Letterbox windows. The crop is the plate's full width but shorter than the
   band's aspect, so the plate over-hangs horizontally and the inner pan has
   real slack — the band can never show its own edge. Band travel is kept
   inside [1920-BAND_W, 0] so the window always covers the frame. */
const BAND_W = 2096;
const BAND_H = 260;
const BAND_TRAVEL_A = [-20, -170];
const BAND_TRAVEL_B = [-170, -20];
const BAND_PAN = 110; // < (plate width - band width) / 2
const CROP_TOP: Rect = {x: 0, y: 132, w: 1600, h: 176}; // sky + body markings
const CROP_BOT: Rect = {x: 0, y: 806, w: 1600, h: 176}; // smoke + ground

const SceneMomentum: React.FC = () => {
	const l = useCurrentFrame();
	const {fps} = useVideoConfig();

	/* pans */
	const topX = ip(l, [0, 75], BAND_TRAVEL_A, EIO);
	const topInner = ip(l, [0, 75], [-BAND_PAN, BAND_PAN], EIO);
	const botX = ip(l, [0, 75], BAND_TRAVEL_B, EIO);
	const botInner = ip(l, [0, 75], [BAND_PAN, -BAND_PAN], EIO);

	/* kinetic typography — throws in, decelerates, never quite stops */
	const t1 = ip(l, [4, 34], [0, 1], EO);
	const bozokX = interpolate(t1, [0, 1], [-620, 0]);
	const track1 = ip(l, [4, 40], [46, -4], EO);

	const t2 = ip(l, [16, 46], [0, 1], EO);
	const teamX = interpolate(t2, [0, 1], [440, 0]);
	const track2 = ip(l, [16, 46], [22, 10], EO);

	const tagIn = ip(l, [8, 24], [0, 1], EO);

	/* second-angle card, thrown left across the lower band */
	const sCard = spring({frame: l - 26, fps, config: CARRY});
	const cardX = interpolate(sCard, [0, 1], [1780, 1268]);
	const cardInner = ip(l, [26, 75], [40, -20], EIO);

	return (
		<AbsoluteFill style={{backgroundColor: INK, opacity: ip(l, [0, 6], [0, 1])}}>
			{/* upper band */}
			<div
				style={{
					position: 'absolute',
					left: topX,
					top: 104,
					width: BAND_W,
					height: BAND_H,
					overflow: 'hidden',
					boxShadow: `inset 0 0 0 1px ${HAIR}`,
				}}
			>
				<Crop
					src="1.jpeg"
					srcW={HERO.w}
					srcH={HERO.h}
					crop={CROP_TOP}
					w={BAND_W}
					h={BAND_H}
					ox={topInner}
				/>
			</div>

			{/* lower band */}
			<div
				style={{
					position: 'absolute',
					left: botX,
					top: 716,
					width: BAND_W,
					height: BAND_H,
					overflow: 'hidden',
					boxShadow: `inset 0 0 0 1px ${HAIR}`,
				}}
			>
				<Crop
					src="1.jpeg"
					srcW={HERO.w}
					srcH={HERO.h}
					crop={CROP_BOT}
					w={BAND_W}
					h={BAND_H}
					ox={botInner}
				/>
			</div>

			{/* kinetic type */}
			<div
				style={{
					position: 'absolute',
					left: 0,
					right: 0,
					top: 388,
					textAlign: 'center',
					fontFamily: SANS,
					fontWeight: 900,
					fontSize: 196,
					lineHeight: 1,
					color: PAPER,
					letterSpacing: track1,
					opacity: ip(l, [4, 16], [0, 1], EO),
					transform: `translateX(${bozokX}px)`,
					textShadow: '0 18px 70px rgba(0,0,0,0.85)',
				}}
			>
				BOZOK
			</div>
			<div
				style={{
					position: 'absolute',
					left: 0,
					right: 0,
					top: 620,
					textAlign: 'center',
					fontFamily: SANS,
					fontSize: 40,
					fontWeight: 600,
					color: 'rgba(242,244,247,0.88)',
					letterSpacing: track2,
					opacity: t2,
					transform: `translateX(${teamX}px)`,
				}}
			>
				ROKET TAKIMI
			</div>

			{/* tag riding the upper band */}
			<Tag
				x={140 + topX * 0.35}
				y={58}
				text="ATEŞLEME"
				opacity={tagIn}
				color={RED}
				size={19}
				tracking={8}
			/>

			{/* second-angle card — straddles the band edge so it reads as a
			    layer, not as a sticker on the smoke */}
			<Plate
				x={cardX}
				y={656}
				w={470}
				h={188}
				opacity={sCard > 0.001 ? 1 : 0}
				shadow="0 50px 120px rgba(0,0,0,0.85)"
			>
				<Crop
					src="3.png"
					srcW={HANGAR.w}
					srcH={HANGAR.h}
					crop={CROP_C}
					w={470}
					h={188}
					ox={cardInner}
				/>
			</Plate>
		</AbsoluteFill>
	);
};

/* ==========================================================================
   SCENE 4 — TITLE (6 - 8s)
   ========================================================================== */
const TITLE = 'BOZOK'.split('');

const SceneTitle: React.FC = () => {
	const l = useCurrentFrame();

	const frameIn = ip(l, [2, 24], [0, 1], EO);
	const rule = ip(l, [24, 42], [0, 1], EO);
	const team = ip(l, [30, 46], [0, 1], EO);
	const teamTrack = ip(l, [30, 52], [20, 13], EO);
	const mc = ip(l, [36, 50], [0, 1], EO);
	const glow = ip(l, [4, 40], [0, 1], EIO);

	return (
		<AbsoluteFill style={{backgroundColor: INK, opacity: ip(l, [0, 6], [0, 1])}}>
			{/* distant exhaust light, very low */}
			<AbsoluteFill
				style={{
					background: `radial-gradient(90% 55% at 50% 108%, rgba(230,51,41,${(
						0.16 * glow
					).toFixed(3)}) 0%, transparent 62%)`,
				}}
			/>
			{/* quiet technical frame */}
			<div style={{position: 'absolute', left: 64, top: 64, right: 64, height: 1, backgroundColor: HAIR, transform: `scaleX(${frameIn})`, transformOrigin: 'left'}} />
			<div style={{position: 'absolute', left: 64, bottom: 64, right: 64, height: 1, backgroundColor: HAIR, transform: `scaleX(${frameIn})`, transformOrigin: 'right'}} />

			<AbsoluteFill style={{justifyContent: 'center', alignItems: 'center'}}>
				<div style={{transform: 'translateY(-34px)', textAlign: 'center'}}>
					<div
						style={{
							fontFamily: SANS,
							fontWeight: 900,
							fontSize: 248,
							lineHeight: 1,
							letterSpacing: -8,
							color: PAPER,
						}}
					>
						{TITLE.map((ch, i) => {
							const p = ip(l, [6 + i * 3, 24 + i * 3], [0, 1], EO);
							return (
								<span
									key={i}
									style={{
										display: 'inline-block',
										opacity: p,
										transform: `translateY(${(1 - p) * 34}px)`,
									}}
								>
									{ch}
								</span>
							);
						})}
					</div>
					<div
						style={{
							width: 420 * rule,
							height: 3,
							margin: '34px auto 0 auto',
							backgroundColor: RED,
						}}
					/>
					<div
						style={{
							marginTop: 28,
							fontFamily: SANS,
							fontSize: 40,
							fontWeight: 600,
							letterSpacing: teamTrack,
							color: PAPER,
							opacity: team,
							transform: `translateY(${(1 - team) * 14}px)`,
						}}
					>
						ROKET TAKIMI
					</div>
					<div
						style={{
							marginTop: 18,
							fontFamily: MONO,
							fontSize: 17,
							letterSpacing: 9,
							color: MUTED,
							opacity: mc,
						}}
					>
						MISSION CONTROL
					</div>
				</div>
			</AbsoluteFill>
		</AbsoluteFill>
	);
};

/* ==========================================================================
   SCENE 5 — SLATE (8 - 10s) — lands on the dashboard colour
   ========================================================================== */
const SceneSlate: React.FC = () => {
	const l = useCurrentFrame();

	const bg = ip(l, [0, 14], [0, 1], EIO);
	const mark = ip(l, [8, 24], [0, 1], EO);
	const rule = ip(l, [14, 32], [0, 1], EO);
	const team = ip(l, [20, 36], [0, 1], EO);

	return (
		<AbsoluteFill style={{backgroundColor: INK}}>
			<AbsoluteFill style={{backgroundColor: SLATE, opacity: bg}} />
			<AbsoluteFill style={{justifyContent: 'center', alignItems: 'center'}}>
				<div style={{textAlign: 'center', transform: 'translateY(-10px)'}}>
					<div
						style={{
							display: 'flex',
							alignItems: 'center',
							justifyContent: 'center',
							gap: 16,
							opacity: mark,
							transform: `translateY(${(1 - mark) * 12}px)`,
						}}
					>
						<div style={{width: 10, height: 10, backgroundColor: RED}} />
						<span
							style={{
								fontFamily: MONO,
								fontSize: 26,
								letterSpacing: 12,
								color: PAPER,
							}}
						>
							MISSION CONTROL
						</span>
					</div>
					<div
						style={{
							width: 360 * rule,
							height: 1,
							margin: '26px auto 0 auto',
							backgroundColor: HAIR_STRONG,
						}}
					/>
					<div
						style={{
							marginTop: 24,
							fontFamily: SANS,
							fontSize: 34,
							fontWeight: 500,
							letterSpacing: 16,
							color: MUTED,
							opacity: team,
							transform: `translateY(${(1 - team) * 10}px)`,
						}}
					>
						ROKET TAKIMI
					</div>
				</div>
			</AbsoluteFill>

			{/* quiet identity mark */}
			<div
				style={{
					position: 'absolute',
					left: 76,
					top: 68,
					fontFamily: MONO,
					fontSize: 15,
					letterSpacing: 7,
					color: PAPER,
					opacity: 0.45 * mark,
				}}
			>
				BOZOK
			</div>
		</AbsoluteFill>
	);
};

/* ==========================================================================
   ASSEMBLY — scenes overlap by 6 frames and dissolve into each other
   ========================================================================== */
export const Intro: React.FC = () => (
	<AbsoluteFill style={{backgroundColor: INK}}>
		<Sequence from={0} durationInFrames={51} name="Atmosphere">
			<SceneAtmosphere />
		</Sequence>
		<Sequence from={45} durationInFrames={66} name="Reveal">
			<SceneReveal />
		</Sequence>
		<Sequence from={105} durationInFrames={81} name="Momentum">
			<SceneMomentum />
		</Sequence>
		<Sequence from={180} durationInFrames={66} name="Title">
			<SceneTitle />
		</Sequence>
		<Sequence from={240} durationInFrames={60} name="Slate">
			<SceneSlate />
		</Sequence>
	</AbsoluteFill>
);
