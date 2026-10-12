import React from 'react';
import {Player} from '@remotion/player';
import {Intro} from './Intro';

/**
 * Non-blocking intro overlay with click-to-skip.
 *
 * Usage (in dashboard app):
 *   import {IntroPlayer} from './PlayerSnippet';
 *   const [showIntro, setShowIntro] = React.useState(true);
 *   {showIntro && <IntroPlayer onSkip={() => setShowIntro(false)} autoplay />}
 *
 * Props:
 *  - onSkip: called when user clicks overlay / presses Esc / video ends.
 *  - autoplay: start playing immediately (default true).
 *  - clickToSkip: show "GEÇ / SKIP" hint, whole overlay clickable (default true).
 */
export const IntroPlayer: React.FC<{
	onSkip: () => void;
	autoplay?: boolean;
	clickToSkip?: boolean;
}> = ({onSkip, autoplay = true, clickToSkip = true}) => {
	const wrapRef = React.useRef<HTMLDivElement>(null);

	React.useEffect(() => {
		const onKey = (e: KeyboardEvent) => {
			if (e.key === 'Escape' || e.key === ' ') onSkip();
		};
		window.addEventListener('keydown', onKey);
		return () => window.removeEventListener('keydown', onKey);
	}, [onSkip]);

	return (
		<div
			ref={wrapRef}
			onClick={() => {
				if (clickToSkip) onSkip();
			}}
			style={{
				position: 'fixed',
				inset: 0,
				zIndex: 9999,
				background: '#05070d',
				cursor: clickToSkip ? 'pointer' : 'default',
			}}
			title="Geçmek için tıklayın / Click to skip"
		>
			<Player
				component={Intro}
				durationInFrames={300}
				fps={30}
				compositionWidth={1920}
				compositionHeight={1080}
				autoPlay={autoplay}
				loop={false}
				style={{width: '100%', height: '100%'}}
				acknowledgeRemotionLicense
				onEnded={onSkip}
			/>
			{clickToSkip && (
				<button
					onClick={(e) => {
						e.stopPropagation();
						onSkip();
					}}
					style={{
						position: 'absolute',
						right: 24,
						bottom: 24,
						padding: '10px 22px',
						fontSize: 16,
						letterSpacing: 3,
						background: 'rgba(5,7,13,0.7)',
						color: '#f2f4f7',
						border: '1px solid rgba(242,244,247,0.5)',
						cursor: 'pointer',
					}}
				>
					GEÇ ▸ SKIP
				</button>
			)}
		</div>
	);
};
