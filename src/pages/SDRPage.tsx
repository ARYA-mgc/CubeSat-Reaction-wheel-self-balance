/**
 * SDRPage.tsx — v3
 * GNU Radio-Style SDR Live View
 * Real RTL-SDR mode: connects to sdr-bridge/server.cjs via WebSocket
 * Simulated mode: generates demo signal data as fallback
 */

import { useEffect, useRef, useCallback, useState } from 'react';
import { useNavigate } from 'react-router-dom';
import {
    ArrowLeft, Radio, Play, Pause, RotateCcw, Activity, Waves,
    Signal, Terminal, Settings, Wifi, WifiOff, Zap,
    ChevronDown, ChevronUp, AlertTriangle, CheckCircle, Server,
    Cpu, Hash, Clock
} from 'lucide-react';
import { useSDRStore } from '../store/sdrStore';
import ADCSControl from '../components/ADCSControl';
import SensorDashboard from '../components/SensorDashboard';
import {
    computePSD, generateSimulatedSignal, generateSimulatedTelemetry,
    parseTelemetry, generateIQConstellation, dbToColor, smoothPSD,
} from '../simulation/sdrEngine';
import type { IQPoint } from '../simulation/sdrEngine';
import { cn } from '../utils/cn';

// ═══════════════════════════════════════════════════════════════
// CONSTANTS
// ═══════════════════════════════════════════════════════════════
const FFT_SIZE = 1024;
const WATERFALL_ROWS = 200;
const FREQ_PRESETS = [
    { label: '2.400 GHz', freq: 2400.000 },
    { label: '2.401 GHz', freq: 2401.500 },
    { label: '2.402 GHz', freq: 2402.400 },
    { label: '2.440 GHz', freq: 2440.000 },
    { label: '2.480 GHz', freq: 2480.000 },
];

// ═══════════════════════════════════════════════════════════════
// SPECTRUM ANALYZER
// ═══════════════════════════════════════════════════════════════
function SpectrumAnalyzer() {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const smoothedRef = useRef<Float32Array | null>(null);
    const peakHoldRef = useRef<Float32Array | null>(null);
    const lastPsdRef = useRef<Float32Array | null>(null);
    const animRef = useRef<number>(0);
    const { isReceiving, centerFreqMHz, latestPSD, sampleRateKHz, mode } = useSDRStore();

    const draw = useCallback(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        const dpr = window.devicePixelRatio || 1;
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width * dpr;
        canvas.height = rect.height * dpr;
        ctx.scale(dpr, dpr);
        const W = rect.width, H = rect.height;

        // Background
        ctx.fillStyle = '#080c12';
        ctx.fillRect(0, 0, W, H);

        const mL = 50, mR = 15, mT = 12, mB = 30;
        const pW = W - mL - mR, pH = H - mT - mB;
        const minDb = -120, maxDb = -20, dbR = maxDb - minDb;

        // Grid
        ctx.strokeStyle = 'rgba(0, 200, 255, 0.06)';
        ctx.lineWidth = 1;
        ctx.font = '10px JetBrains Mono, monospace';
        for (let db = minDb; db <= maxDb; db += 10) {
            const y = mT + pH * (1 - (db - minDb) / dbR);
            ctx.beginPath(); ctx.moveTo(mL, y); ctx.lineTo(mL + pW, y); ctx.stroke();
            ctx.fillStyle = '#4a5568'; ctx.textAlign = 'right';
            ctx.fillText(`${db}`, mL - 6, y + 3);
        }

        // Frequency axis
        const spanMHz = sampleRateKHz / 1000;
        const fStart = centerFreqMHz - spanMHz / 2;
        const fEnd = centerFreqMHz + spanMHz / 2;
        ctx.textAlign = 'center';
        const fStep = spanMHz > 2 ? 0.5 : spanMHz > 1 ? 0.2 : spanMHz > 0.5 ? 0.1 : 0.05;
        for (let f = Math.ceil(fStart / fStep) * fStep; f <= fEnd; f += fStep) {
            const x = mL + ((f - fStart) / (fEnd - fStart)) * pW;
            ctx.strokeStyle = 'rgba(0, 200, 255, 0.05)';
            ctx.beginPath(); ctx.moveTo(x, mT); ctx.lineTo(x, mT + pH); ctx.stroke();
            ctx.fillStyle = '#4a5568'; ctx.fillText(f.toFixed(f >= 100 ? 1 : 2), x, H - 8);
        }
        ctx.fillStyle = '#64748b'; ctx.font = '10px Inter, sans-serif';
        ctx.textAlign = 'center'; ctx.fillText('Frequency (MHz)', W / 2, H - 1);
        ctx.save(); ctx.translate(12, H / 2); ctx.rotate(-Math.PI / 2);
        ctx.fillText('Power (dB)', 0, 0); ctx.restore();

        if (!latestPSD || latestPSD.length === 0) {
            ctx.fillStyle = '#1a2332'; ctx.fillRect(mL, mT, pW, pH);
            ctx.fillStyle = '#4a5568'; ctx.font = '14px Inter, sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText(isReceiving ? 'Acquiring signal...' : 'Start receiving', W / 2, H / 2);
            return;
        }

        // Smooth PSD
        const psd = smoothPSD(latestPSD, smoothedRef.current, 0.3);
        smoothedRef.current = psd;

        // Update peak hold
        if (!peakHoldRef.current || peakHoldRef.current.length !== psd.length) {
            peakHoldRef.current = Float32Array.from(psd);
        } else {
            for (let i = 0; i < psd.length; i++) {
                peakHoldRef.current[i] = Math.max(peakHoldRef.current[i] * 0.997, psd[i]);
            }
        }

        // Noise floor
        const noiseY = mT + pH * (1 - (-95 - minDb) / dbR);
        ctx.strokeStyle = 'rgba(255, 170, 0, 0.25)'; ctx.setLineDash([4, 4]);
        ctx.beginPath(); ctx.moveTo(mL, noiseY); ctx.lineTo(mL + pW, noiseY); ctx.stroke();
        ctx.setLineDash([]); ctx.fillStyle = 'rgba(255, 170, 0, 0.4)';
        ctx.font = '9px JetBrains Mono, monospace'; ctx.textAlign = 'left';
        ctx.fillText('Noise Floor', mL + 4, noiseY - 4);

        // Peak hold trace (faded)
        ctx.beginPath();
        for (let i = 0; i < peakHoldRef.current.length; i++) {
            const x = mL + (i / peakHoldRef.current.length) * pW;
            const db = Math.max(minDb, Math.min(maxDb, peakHoldRef.current[i]));
            const y = mT + pH * (1 - (db - minDb) / dbR);
            i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
        }
        ctx.strokeStyle = 'rgba(255, 100, 100, 0.25)'; ctx.lineWidth = 1; ctx.stroke();

        // Gradient fill
        const grad = ctx.createLinearGradient(0, mT, 0, mT + pH);
        grad.addColorStop(0, 'rgba(0, 212, 255, 0.35)');
        grad.addColorStop(0.5, 'rgba(0, 255, 136, 0.12)');
        grad.addColorStop(1, 'rgba(0, 212, 255, 0.01)');

        ctx.beginPath();
        let peakDb = -200, peakX = 0;
        for (let i = 0; i < psd.length; i++) {
            const x = mL + (i / psd.length) * pW;
            const db = Math.max(minDb, Math.min(maxDb, psd[i]));
            const y = mT + pH * (1 - (db - minDb) / dbR);
            i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
            if (psd[i] > peakDb) { peakDb = psd[i]; peakX = x; }
        }
        ctx.lineTo(mL + pW, mT + pH); ctx.lineTo(mL, mT + pH); ctx.closePath();
        ctx.fillStyle = grad; ctx.fill();

        // Trace line
        ctx.beginPath();
        for (let i = 0; i < psd.length; i++) {
            const x = mL + (i / psd.length) * pW;
            const db = Math.max(minDb, Math.min(maxDb, psd[i]));
            const y = mT + pH * (1 - (db - minDb) / dbR);
            i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
        }
        ctx.strokeStyle = '#00d4ff'; ctx.lineWidth = 1.5;
        ctx.shadowColor = 'rgba(0, 212, 255, 0.4)'; ctx.shadowBlur = 4;
        ctx.stroke(); ctx.shadowBlur = 0;

        // Peak marker
        if (peakDb > -85) {
            const pY = mT + pH * (1 - (Math.max(minDb, Math.min(maxDb, peakDb)) - minDb) / dbR);
            ctx.beginPath(); ctx.arc(peakX, pY, 4, 0, 2 * Math.PI);
            ctx.fillStyle = '#ff4444'; ctx.fill();
            ctx.fillStyle = '#ff6666'; ctx.font = '9px JetBrains Mono, monospace';
            ctx.textAlign = 'left'; ctx.fillText(`${peakDb.toFixed(1)} dB`, peakX + 8, pY - 6);
        }

        // Center freq marker
        const cX = mL + pW / 2;
        ctx.strokeStyle = 'rgba(0, 255, 136, 0.35)'; ctx.setLineDash([2, 3]);
        ctx.beginPath(); ctx.moveTo(cX, mT); ctx.lineTo(cX, mT + pH); ctx.stroke();
        ctx.setLineDash([]); ctx.fillStyle = 'rgba(0, 255, 136, 0.6)';
        ctx.font = '9px JetBrains Mono, monospace'; ctx.textAlign = 'center';
        ctx.fillText(`${centerFreqMHz.toFixed(3)} MHz`, cX, mT + 14);
    }, [isReceiving, centerFreqMHz, latestPSD, sampleRateKHz, mode]);

    useEffect(() => {
        let running = true;
        let last = 0;
        const loop = (ts: number) => {
            if (!running) return;
            if (ts - last > 50) {
                const psd = useSDRStore.getState().latestPSD;
                if (psd !== lastPsdRef.current) { lastPsdRef.current = psd; draw(); last = ts; }
            }
            animRef.current = requestAnimationFrame(loop);
        };
        draw();
        animRef.current = requestAnimationFrame(loop);
        return () => { running = false; cancelAnimationFrame(animRef.current); };
    }, [draw]);

    return <canvas ref={canvasRef} className="w-full h-full" />;
}

// ═══════════════════════════════════════════════════════════════
// WATERFALL — scrolling canvas approach
// ═══════════════════════════════════════════════════════════════
function WaterfallDisplay() {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const offRef = useRef<HTMLCanvasElement | null>(null);
    const lastLen = useRef(0);
    const animRef = useRef<number>(0);
    const { waterfallHistory, isReceiving } = useSDRStore();

    const draw = useCallback(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        const rect = canvas.getBoundingClientRect();
        const W = Math.floor(rect.width);
        const H = Math.floor(rect.height);
        canvas.width = W; canvas.height = H;

        ctx.fillStyle = '#050810';
        ctx.fillRect(0, 0, W, H);

        if (waterfallHistory.length === 0) {
            ctx.fillStyle = '#4a5568'; ctx.font = '12px Inter, sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText(isReceiving ? 'Building waterfall...' : 'No waterfall data', W / 2, H / 2);
            return;
        }

        // Create offscreen canvas for row rendering
        if (!offRef.current) offRef.current = document.createElement('canvas');
        const off = offRef.current;
        off.width = W; off.height = 1;
        const offCtx = off.getContext('2d')!;

        const rows = waterfallHistory.length;
        const rowH = Math.max(1, Math.ceil(H / WATERFALL_ROWS));
        const minDb = -120, maxDb = -20;

        for (let r = 0; r < rows; r++) {
            const psd = waterfallHistory[r];
            const y = H - (rows - r) * rowH;
            if (y < -rowH || y > H) continue;

            // Draw 1-pixel-high row on offscreen, then scale to rowH
            const imgData = offCtx.createImageData(W, 1);
            const px = imgData.data;
            for (let x = 0; x < W; x++) {
                const bin = Math.floor((x / W) * psd.length);
                const [cr, cg, cb] = dbToColor(psd[bin], minDb, maxDb);
                const idx = x * 4;
                px[idx] = cr; px[idx + 1] = cg; px[idx + 2] = cb; px[idx + 3] = 255;
            }
            offCtx.putImageData(imgData, 0, 0);
            ctx.drawImage(off, 0, 0, W, 1, 0, y, W, rowH);
        }

        // Time labels
        ctx.fillStyle = 'rgba(255,255,255,0.3)';
        ctx.font = '9px JetBrains Mono, monospace'; ctx.textAlign = 'right';
        ctx.fillText('now', W - 4, H - 4);
    }, [waterfallHistory, isReceiving]);

    useEffect(() => {
        let running = true;
        let last = 0;
        const loop = (ts: number) => {
            if (!running) return;
            if (ts - last > 80) {
                const wh = useSDRStore.getState().waterfallHistory;
                if (wh.length !== lastLen.current) { lastLen.current = wh.length; draw(); last = ts; }
            }
            animRef.current = requestAnimationFrame(loop);
        };
        draw();
        animRef.current = requestAnimationFrame(loop);
        return () => { running = false; cancelAnimationFrame(animRef.current); };
    }, [draw]);

    return <canvas ref={canvasRef} className="w-full h-full" />;
}

// ═══════════════════════════════════════════════════════════════
// CONSTELLATION
// ═══════════════════════════════════════════════════════════════
function ConstellationDiagram() {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const animRef = useRef<number>(0);
    const { isReceiving, iqPoints, mode } = useSDRStore();

    const draw = useCallback(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        const dpr = window.devicePixelRatio || 1;
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width * dpr; canvas.height = rect.height * dpr;
        ctx.scale(dpr, dpr);
        const W = rect.width, H = rect.height;
        const S = Math.min(W, H), cx = W / 2, cy = H / 2, r = S * 0.42;

        ctx.fillStyle = '#080c12'; ctx.fillRect(0, 0, W, H);

        // Grid
        ctx.strokeStyle = 'rgba(0, 212, 255, 0.08)'; ctx.lineWidth = 1;
        for (let d = 0.25; d <= 1; d += 0.25) {
            ctx.beginPath(); ctx.arc(cx, cy, r * d, 0, 2 * Math.PI); ctx.stroke();
        }
        ctx.strokeStyle = 'rgba(0, 212, 255, 0.12)';
        ctx.beginPath(); ctx.moveTo(cx - r, cy); ctx.lineTo(cx + r, cy);
        ctx.moveTo(cx, cy - r); ctx.lineTo(cx, cy + r); ctx.stroke();
        ctx.fillStyle = '#4a5568'; ctx.font = '9px JetBrains Mono, monospace';
        ctx.textAlign = 'center'; ctx.fillText('I', cx + r + 12, cy + 4); ctx.fillText('Q', cx, cy - r - 6);

        let points: IQPoint[];
        if (mode === 'usrp' && iqPoints.length > 0) {
            points = iqPoints;
        } else if (isReceiving) {
            points = generateIQConstellation(true, 80);
        } else {
            ctx.fillStyle = '#4a5568'; ctx.font = '12px Inter, sans-serif';
            ctx.textAlign = 'center'; ctx.fillText('No signal', cx, cy); return;
        }

        for (const pt of points) {
            const x = cx + pt.i * r, y = cy - pt.q * r;
            const g = ctx.createRadialGradient(x, y, 0, x, y, 5);
            g.addColorStop(0, 'rgba(0, 255, 136, 0.5)'); g.addColorStop(1, 'rgba(0, 255, 136, 0)');
            ctx.fillStyle = g; ctx.fillRect(x - 5, y - 5, 10, 10);
            ctx.beginPath(); ctx.arc(x, y, 1.5, 0, 2 * Math.PI);
            ctx.fillStyle = '#00ff88'; ctx.fill();
        }
        ctx.fillStyle = 'rgba(0, 212, 255, 0.4)'; ctx.font = '9px JetBrains Mono, monospace';
        ctx.textAlign = 'center';
        ctx.fillText(mode === 'usrp' ? `REAL IQ • ${points.length} pts` : 'SIM IQ', cx, cy + r + 16);
    }, [isReceiving, iqPoints, mode]);

    useEffect(() => {
        let running = true, last = 0;
        const loop = (ts: number) => {
            if (!running) return;
            if (ts - last > 120) { draw(); last = ts; }
            animRef.current = requestAnimationFrame(loop);
        };
        animRef.current = requestAnimationFrame(loop);
        return () => { running = false; cancelAnimationFrame(animRef.current); };
    }, [draw]);

    return <canvas ref={canvasRef} className="w-full h-full" />;
}

// ═══════════════════════════════════════════════════════════════
// DECODED DATA STREAM
// ═══════════════════════════════════════════════════════════════
function DecodedDataStream() {
    const scrollRef = useRef<HTMLDivElement>(null);
    const { packets, mode } = useSDRStore();
    const [autoScroll, setAutoScroll] = useState(true);

    useEffect(() => {
        if (autoScroll && scrollRef.current) scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
    }, [packets, autoScroll]);

    const fmt = (ts: number) => {
        const d = new Date(ts);
        return `${d.getHours().toString().padStart(2, '0')}:${d.getMinutes().toString().padStart(2, '0')}:${d.getSeconds().toString().padStart(2, '0')}.${d.getMilliseconds().toString().padStart(3, '0')}`;
    };

    return (
        <div className="flex flex-col h-full">
            <div className="flex items-center justify-between px-3 py-1.5 border-b border-white/5 shrink-0">
                <div className="flex items-center gap-2">
                    <Terminal className="w-3.5 h-3.5 text-emerald-400" />
                    <span className="text-[10px] uppercase tracking-widest text-gray-500">
                        {mode === 'usrp' ? 'Decoded RF Packets' : 'Decoded Packets'}
                    </span>
                </div>
                <div className="flex items-center gap-2">
                    <span className="text-[10px] font-mono text-gray-600">{packets.length}</span>
                    <button
                        onClick={() => setAutoScroll(!autoScroll)}
                        className={cn("text-[9px] px-1.5 py-0.5 rounded",
                            autoScroll ? "bg-emerald-500/20 text-emerald-400" : "bg-white/5 text-gray-500")}
                    >
                        {autoScroll ? '⬇ AUTO' : '⏸'}
                    </button>
                </div>
            </div>
            <div ref={scrollRef}
                className="flex-1 overflow-y-auto p-2 font-mono text-[11px] space-y-0.5"
                onScroll={(e) => {
                    const el = e.currentTarget;
                    setAutoScroll(el.scrollHeight - el.scrollTop - el.clientHeight < 30);
                }}
            >
                {packets.length === 0 ? (
                    <div className="text-gray-600 text-center py-8 space-y-2">
                        <Radio className="w-6 h-6 mx-auto opacity-30" />
                        <div>{mode === 'usrp'
                            ? 'Listening for RF packets…'
                            : 'Waiting for packets…'
                        }</div>
                        {mode === 'usrp' && (
                            <div className="text-[9px] text-gray-700">
                                Ensure FS1000A transmitter is active
                            </div>
                        )}
                    </div>
                ) : (
                    packets.map((pkt, i) => (
                        <div key={i} className={cn(
                            "flex gap-2 py-0.5 px-1 rounded",
                            pkt.valid ? "hover:bg-white/5" : "bg-red-500/10"
                        )}>
                            <span className="text-gray-600 shrink-0">{fmt(pkt.timestamp)}</span>
                            <span className={cn("shrink-0", pkt.valid ? "text-emerald-400" : "text-red-400")}>
                                {pkt.valid ? '✓' : '✗'}
                            </span>
                            {pkt.valid ? (
                                <span className="text-gray-300">
                                    <span className="text-cyan-400">X:</span>{(pkt.x ?? 0).toString().padStart(5)}
                                    <span className="text-cyan-400 ml-1">Y:</span>{(pkt.y ?? 0).toString().padStart(5)}
                                    <span className="text-cyan-400 ml-1">Z:</span>{(pkt.z ?? 0).toString().padStart(5)}
                                    <span className="text-amber-400 ml-2">T:</span>{(pkt.temp ?? 0).toFixed(1)}°C
                                    <span className="text-amber-400 ml-1">H:</span>{(pkt.humidity ?? 0).toFixed(1)}%
                                    <span className="text-gray-600 ml-2">[{(pkt.rssi ?? -99).toFixed(0)} dBm]</span>
                                </span>
                            ) : (
                                <span className="text-red-400/70">
                                    {pkt.raw ? `RAW: ${pkt.raw}` : 'CRC_ERROR'}
                                </span>
                            )}
                        </div>
                    ))
                )}
            </div>
        </div>
    );
}

// ═══════════════════════════════════════════════════════════════
// HARDWARE STATUS
// ═══════════════════════════════════════════════════════════════
function HardwareStatus() {
    const { hwInfo, serverConnected, rtlAvailable } = useSDRStore();

    if (!serverConnected || !hwInfo) return null;

    return (
        <div className="mt-4 p-3 bg-cyan-500/5 border border-cyan-500/10 rounded-xl space-y-2">
            <div className="flex items-center gap-2 mb-1">
                <Cpu className="w-3.5 h-3.5 text-cyan-400" />
                <span className="text-[10px] font-bold uppercase tracking-wider text-cyan-400">USRP Hardware</span>
            </div>

            <div className="grid grid-cols-2 gap-2">
                <div className="space-y-0.5">
                    <div className="text-[9px] text-gray-500 flex items-center gap-1"><Server className="w-2.5 h-2.5" />Model</div>
                    <div className="text-[10px] text-gray-300 font-medium truncate">{hwInfo.model}</div>
                </div>
                <div className="space-y-0.5">
                    <div className="text-[9px] text-gray-500 flex items-center gap-1"><Hash className="w-2.5 h-2.5" />Serial</div>
                    <div className="text-[10px] text-gray-300 font-medium font-mono truncate">{hwInfo.serial}</div>
                </div>
                <div className="space-y-0.5">
                    <div className="text-[9px] text-gray-500 flex items-center gap-1"><Clock className="w-2.5 h-2.5" />Master Clock</div>
                    <div className="text-[10px] text-gray-300 font-medium">{hwInfo.master_clock_rate > 0 ? `${(hwInfo.master_clock_rate / 1e6).toFixed(1)} MHz` : 'Dynamic'}</div>
                </div>
                <div className="space-y-0.5">
                    <div className="text-[9px] text-gray-500 flex items-center gap-1"><Activity className="w-2.5 h-2.5" />Subdev</div>
                    <div className="text-[10px] text-gray-300 font-medium truncate">{hwInfo.mboard_name}</div>
                </div>
            </div>
        </div>
    );
}

// ═══════════════════════════════════════════════════════════════
// SIGNAL METRICS
// ═══════════════════════════════════════════════════════════════
function SignalMetrics() {
    const { snr, rssi, packetCount, errorCount, bitrate, centerFreqMHz, sampleRateKHz,
        mode, serverConnected, bitsDecoded, peakPower } = useSDRStore();
    const err = packetCount > 0 ? ((errorCount / packetCount) * 100).toFixed(1) : '0.0';

    const rows = [
        { l: 'Center Freq', v: (centerFreqMHz ?? 433.840).toFixed(3), u: 'MHz', c: 'text-cyan-400', icon: Radio },
        { l: 'SNR', v: (snr ?? 0).toFixed(1), u: 'dB', c: (snr ?? 0) > 10 ? 'text-emerald-400' : (snr ?? 0) > 5 ? 'text-amber-400' : 'text-red-400', icon: Signal },
        { l: 'RSSI', v: (rssi ?? -99).toFixed(0), u: 'dBm', c: (rssi ?? -99) > -70 ? 'text-emerald-400' : (rssi ?? -99) > -90 ? 'text-amber-400' : 'text-red-400', icon: Activity },
        { l: 'Sample Rate', v: (sampleRateKHz ?? 250).toString(), u: 'kSPS', c: 'text-gray-300', icon: Zap },
        { l: 'Bitrate', v: (bitrate ?? 2000).toString(), u: 'bps', c: 'text-gray-300', icon: Activity },
        { l: 'Peak Power', v: (peakPower ?? 0).toFixed(0), u: '', c: 'text-orange-400', icon: Waves },
        { l: 'Bits/Frame', v: (bitsDecoded ?? 0).toString(), u: '', c: 'text-cyan-400', icon: Terminal },
        { l: 'Packets', v: (packetCount ?? 0).toString(), u: 'total', c: 'text-emerald-400', icon: Terminal },
        { l: 'Error Rate', v: err, u: '%', c: parseFloat(err) < 5 ? 'text-emerald-400' : 'text-red-400', icon: AlertTriangle },
    ];

    return (
        <div className="space-y-1 p-3">
            <div className="flex items-center gap-2 mb-3">
                <Signal className="w-3.5 h-3.5 text-cyan-400" />
                <span className="text-[10px] uppercase tracking-widest text-gray-500">Signal Metrics</span>
                {mode === 'usrp' && (
                    <span className={cn("text-[9px] px-1.5 py-0.5 rounded ml-auto",
                        serverConnected ? "bg-emerald-500/20 text-emerald-400" : "bg-red-500/20 text-red-400")}>
                        {serverConnected ? 'LIVE' : 'OFFLINE'}
                    </span>
                )}
            </div>
            {rows.map(m => (
                <div key={m.l} className="flex items-center justify-between py-1 px-2 rounded-lg hover:bg-white/5 transition-colors">
                    <div className="flex items-center gap-2">
                        <m.icon className="w-3 h-3 text-gray-600" />
                        <span className="text-[11px] text-gray-400">{m.l}</span>
                    </div>
                    <div className="flex items-baseline gap-1">
                        <span className={cn("text-sm font-mono font-medium tabular-nums", m.c)}>{m.v}</span>
                        {m.u && <span className="text-[9px] text-gray-600">{m.u}</span>}
                    </div>
                </div>
            ))}
        </div>
    );
}

// ═══════════════════════════════════════════════════════════════
// CONTROL PANEL — with freq presets, squelch, sample rate
// ═══════════════════════════════════════════════════════════════
function ControlPanel() {
    const {
        mode, isReceiving, centerFreqMHz, gain, sampleRateKHz, squelch,
        serverConnected, serverMessage, rtlAvailable, ws,
        setMode, toggleReceiving, setCenterFreq, setGain, setSampleRate, setSquelch, reset,
        connectServer, disconnectServer,
    } = useSDRStore();
    const [showAdvanced, setShowAdvanced] = useState(false);

    const sendTestPacket = () => {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ action: 'testPacket' }));
        }
    };

    const handleModeChange = (m: 'simulated' | 'usrp') => {
        setMode(m);
        m === 'usrp' ? connectServer() : disconnectServer();
    };

    return (
        <div className="p-3 space-y-3">
            <div className="flex items-center gap-2 mb-1">
                <Settings className="w-3.5 h-3.5 text-cyan-400" />
                <span className="text-[10px] uppercase tracking-widest text-gray-500">SDR Controls</span>
            </div>

            {/* Mode */}
            <div className="flex gap-1">
                <button onClick={() => handleModeChange('simulated')}
                    className={cn("flex-1 py-2 text-[11px] font-medium rounded-lg border transition-all",
                        mode === 'simulated' ? "bg-cyan-500/15 text-cyan-400 border-cyan-500/30"
                            : "bg-white/5 text-gray-500 border-white/5 hover:bg-white/10")}>
                    Simulated
                </button>
                <button onClick={() => handleModeChange('usrp')}
                    className={cn("flex-1 py-2 text-[11px] font-medium rounded-lg border transition-all",
                        mode === 'usrp' ? "bg-violet-500/15 text-violet-400 border-violet-500/30"
                            : "bg-white/5 text-gray-500 border-white/5 hover:bg-white/10")}>
                    Hardware (USRP/RTL)
                </button>
            </div>

            {/* Server status */}
            {mode === 'usrp' && (
                <div className={cn("p-2 rounded-lg border text-[10px]",
                    serverConnected ? "bg-emerald-500/10 border-emerald-500/20 text-emerald-400"
                        : "bg-amber-500/10 border-amber-500/20 text-amber-400")}>
                    <div className="flex items-center gap-2 mb-1">
                        {serverConnected ? <><CheckCircle className="w-3 h-3" /><span>SDR Server Connected</span></>
                            : <><Server className="w-3 h-3" /><span>SDR Server Required</span></>}
                    </div>
                    {!serverConnected && <div className="text-[9px] text-gray-400 mt-1">Run: <span className="text-amber-300 font-mono">node sdr-bridge/server.cjs</span></div>}
                    {serverMessage && <div className="text-[9px] text-gray-400 mt-1 truncate">{serverMessage}</div>}
                    {serverConnected && (
                        <div className="flex items-center gap-1 mt-1 text-[9px] text-emerald-400">
                            <CheckCircle className="w-3 h-3" /><span>USRP Engine Online</span>
                        </div>
                    )}
                    {!serverConnected && (
                        <button onClick={connectServer}
                            className="mt-2 w-full py-1.5 bg-white/5 hover:bg-white/10 rounded text-[10px] text-gray-300 transition-all">
                            Retry Connection
                        </button>
                    )}
                </div>
            )}

            {/* Start/Stop */}
            <button onClick={toggleReceiving}
                disabled={mode === 'usrp' && !serverConnected}
                className={cn("w-full flex items-center justify-center gap-2 py-2.5 rounded-xl font-medium text-sm border transition-all",
                    isReceiving ? "bg-red-500/15 text-red-400 border-red-500/30 hover:bg-red-500/25"
                        : "bg-emerald-500/15 text-emerald-400 border-emerald-500/30 hover:bg-emerald-500/25",
                    mode === 'usrp' && !serverConnected && "opacity-50 cursor-not-allowed")}>
                {isReceiving ? <Pause className="w-4 h-4" /> : <Play className="w-4 h-4" />}
                {isReceiving ? 'Stop Receiving' : 'Start Receiving'}
            </button>

            {/* Frequency */}
            <div>
                <label className="text-[10px] text-gray-500 uppercase tracking-wider">Frequency (MHz)</label>
                <input type="number" step="0.001" value={centerFreqMHz}
                    onChange={e => setCenterFreq(parseFloat(e.target.value) || 2400.0)}
                    className="w-full mt-1 px-3 py-2 bg-black/40 border border-white/10 rounded-lg text-sm font-mono text-white focus:border-cyan-500/50 focus:outline-none" />
            </div>

            <HardwareStatus />

            {/* Frequency presets */}
            <div>
                <label className="text-[10px] text-gray-500 uppercase tracking-wider mb-1 block">Presets</label>
                <div className="flex flex-wrap gap-1">
                    {FREQ_PRESETS.map(p => (
                        <button key={p.freq} onClick={() => setCenterFreq(p.freq)}
                            className={cn("px-2 py-1 rounded text-[10px] font-mono border transition-all",
                                Math.abs(centerFreqMHz - p.freq) < 0.001
                                    ? "bg-cyan-500/20 text-cyan-400 border-cyan-500/30"
                                    : "bg-white/5 text-gray-500 border-white/5 hover:bg-white/10")}>
                            {p.label}
                        </button>
                    ))}
                </div>
            </div>

            {/* Gain slider */}
            <div>
                <div className="flex justify-between">
                    <label className="text-[10px] text-gray-500 uppercase tracking-wider">Gain</label>
                    <span className="text-[10px] font-mono text-cyan-400">{gain} dB</span>
                </div>
                <input type="range" min="0" max="50" value={gain}
                    onChange={e => setGain(parseInt(e.target.value))}
                    className="w-full mt-1 h-1.5 bg-white/10 rounded-full appearance-none cursor-pointer accent-cyan-500" />
            </div>

            {/* Advanced */}
            <button onClick={() => setShowAdvanced(!showAdvanced)}
                className="flex items-center gap-1 text-[10px] text-gray-500 hover:text-gray-400 transition-colors">
                {showAdvanced ? <ChevronUp className="w-3 h-3" /> : <ChevronDown className="w-3 h-3" />}
                Advanced Settings
            </button>

            {showAdvanced && (
                <div className="space-y-6">
                    <SensorDashboard />
                    <ADCSControl />
                    <DecodedDataStream />
                    {/* Sample Rate */}
                    <div>
                        <label className="text-[10px] text-gray-500 uppercase tracking-wider">Sample Rate</label>
                        <div className="flex gap-1 mt-1">
                            {[250, 1000, 2000, 2400].map(r => (
                                <button key={r} onClick={() => setSampleRate(r)}
                                    className={cn("flex-1 py-1.5 text-[10px] font-mono rounded border transition-all",
                                        sampleRateKHz === r
                                            ? "bg-cyan-500/15 text-cyan-400 border-cyan-500/30"
                                            : "bg-white/5 text-gray-500 border-white/5 hover:bg-white/10")}>
                                    {r >= 1000 ? `${r / 1000}M` : `${r}k`}
                                </button>
                            ))}
                        </div>
                    </div>

                    {/* Squelch */}
                    <div>
                        <div className="flex justify-between">
                            <label className="text-[10px] text-gray-500 uppercase tracking-wider">Squelch</label>
                            <span className="text-[10px] font-mono text-cyan-400">{squelch} dB</span>
                        </div>
                        <input type="range" min="-120" max="-20" value={squelch}
                            onChange={e => setSquelch(parseInt(e.target.value))}
                            className="w-full mt-1 h-1.5 bg-white/10 rounded-full appearance-none cursor-pointer accent-orange-500" />
                    </div>
                </div>
            )}

            {/* Test Packet (RTL-SDR only) */}
            {mode === 'usrp' && serverConnected && (
                <button onClick={sendTestPacket}
                    className="w-full flex items-center justify-center gap-2 py-2 rounded-lg bg-violet-500/10 hover:bg-violet-500/20 text-violet-400 border border-violet-500/20 text-[11px] transition-all">
                    <Terminal className="w-3 h-3" /> Send Test Packet
                </button>
            )}

            {/* Reset */}
            <button onClick={reset}
                className="w-full flex items-center justify-center gap-2 py-2 rounded-lg bg-white/5 hover:bg-white/10 text-gray-400 border border-white/5 text-[11px] transition-all">
                <RotateCcw className="w-3 h-3" /> Reset SDR
            </button>
        </div>
    );
}

// ═══════════════════════════════════════════════════════════════
// SIMULATION LOOP (fallback)
// ═══════════════════════════════════════════════════════════════
function useSimulationLoop() {
    const {
        isReceiving, mode, centerFreqMHz, gain,
        addPacket, pushWaterfallRow, setLatestPSD, updateMetrics, incrementError,
    } = useSDRStore();
    const frameRef = useRef<number>(0);
    const pktRef = useRef<number>(0);
    const prevRef = useRef<Float32Array | null>(null);

    useEffect(() => {
        if (!isReceiving || mode !== 'simulated') return;
        let running = true;

        const process = () => {
            if (!running) return;
            const str = gain / 50;
            const samples = generateSimulatedSignal(FFT_SIZE, centerFreqMHz, true, str, -100);
            const psd = computePSD(samples, FFT_SIZE);
            const adj = new Float32Array(psd.length);
            for (let i = 0; i < psd.length; i++) adj[i] = psd[i] + gain * 0.5 - 10;
            const sm = smoothPSD(adj, prevRef.current, 0.3);
            prevRef.current = sm;
            setLatestPSD(sm); pushWaterfallRow(sm);

            let pk = -200, avg = 0;
            for (let i = 0; i < sm.length; i++) { if (sm[i] > pk) pk = sm[i]; avg += sm[i]; }
            avg /= sm.length;
            updateMetrics({ snr: pk - avg, rssi: pk - 30, bitrate: 2000, bits: 0, peak: pk });

            frameRef.current = window.setTimeout(process, 60);
        };

        const decode = () => {
            if (!running) return;
            const raw = generateSimulatedTelemetry();
            const pkt = parseTelemetry(raw, -60);
            if (Math.random() > 0.15) addPacket(pkt);
            else { addPacket({ ...pkt, valid: false, raw: 'CRC_ERROR' }); incrementError(); }
            pktRef.current = window.setTimeout(decode, 1200 + Math.random() * 800);
        };

        process(); decode();
        return () => { running = false; clearTimeout(frameRef.current); clearTimeout(pktRef.current); };
    }, [isReceiving, mode, centerFreqMHz, gain, addPacket, pushWaterfallRow, setLatestPSD, updateMetrics, incrementError]);
}

// ═══════════════════════════════════════════════════════════════
// MAIN PAGE
// ═══════════════════════════════════════════════════════════════
export default function SDRPage() {
    const navigate = useNavigate();
    const { isReceiving, mode, centerFreqMHz, packetCount, serverConnected, sampleRateKHz } = useSDRStore();
    useSimulationLoop();

    return (
        <div className="min-h-screen bg-[#060a10] text-white overflow-hidden flex flex-col">
            {/* Top Bar */}
            <div className="shrink-0 flex items-center justify-between px-4 py-2.5 bg-black/40 border-b border-white/5 backdrop-blur-sm">
                <div className="flex items-center gap-4">
                    <button onClick={() => navigate('/')}
                        className="flex items-center gap-2 px-4 py-2 bg-white/5 hover:bg-white/10 text-white border border-white/10 rounded-xl transition-all">
                        <ArrowLeft className="w-4 h-4" /> <span className="text-sm">Back</span>
                    </button>
                    <div className="flex items-center gap-3">
                        <div className="relative">
                            <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-orange-500/20 to-red-500/20 border border-orange-500/30 flex items-center justify-center">
                                <Radio className={cn("w-5 h-5 text-orange-400", isReceiving && "animate-pulse")} />
                            </div>
                            {isReceiving && <div className="absolute -top-0.5 -right-0.5 w-2.5 h-2.5 bg-emerald-500 rounded-full pulse-indicator" />}
                        </div>
                        <div>
                            <h1 className="text-lg font-bold bg-gradient-to-r from-orange-400 to-red-400 bg-clip-text text-transparent">SDR Live View</h1>
                            <p className="text-[10px] text-gray-500 uppercase tracking-widest">
                                {mode === 'usrp'
                                    ? 'HARDWARE SDR (USRP/RTL)' : 'Simulated'} • {centerFreqMHz.toFixed(3)} MHz • {sampleRateKHz >= 1000 ? `${sampleRateKHz / 1000}M` : `${sampleRateKHz}k`}SPS
                            </p>
                        </div>
                    </div>
                </div>

                <div className="flex items-center gap-3">
                    <div className="flex items-center gap-2 px-3 py-2 bg-black/30 rounded-lg border border-white/5">
                        {isReceiving ? <Wifi className="w-3.5 h-3.5 text-emerald-400" /> : <WifiOff className="w-3.5 h-3.5 text-gray-500" />}
                        <span className="text-[10px] text-gray-500 uppercase">RX</span>
                        <div className={cn('w-1.5 h-1.5 rounded-full', isReceiving ? 'bg-emerald-500 pulse-indicator' : 'bg-gray-600')} />
                        <span className={cn("text-[10px] font-mono uppercase tracking-wider",
                            isReceiving ? "text-emerald-400" : "text-gray-500")}>
                            {isReceiving ? 'RECEIVING' : 'IDLE'}
                        </span>
                    </div>

                    <div className={cn("flex items-center gap-2 px-3 py-2 bg-black/30 rounded-lg border",
                        mode === 'usrp' ? 'border-violet-500/30' : 'border-white/5')}>
                        <Zap className={cn("w-3.5 h-3.5", mode === 'usrp' ? 'text-violet-400' : 'text-cyan-400')} />
                        <span className={cn("text-[10px] font-mono uppercase", mode === 'usrp' ? 'text-violet-400' : 'text-cyan-400')}>
                            {mode === 'usrp' ? 'REAL SDR' : 'SIM'}
                        </span>
                    </div>

                    {mode === 'usrp' && (
                        <div className={cn("flex items-center gap-2 px-3 py-2 bg-black/30 rounded-lg border",
                            serverConnected ? "border-emerald-500/30" : "border-red-500/30")}>
                            <Server className={cn("w-3.5 h-3.5", serverConnected ? "text-emerald-400" : "text-red-400")} />
                            <span className={cn("text-[10px] font-mono uppercase",
                                serverConnected ? "text-emerald-400" : "text-red-400")}>
                                {serverConnected ? 'SERVER ✓' : 'SERVER ✗'}
                            </span>
                        </div>
                    )}

                    <div className="flex items-center gap-2 px-3 py-2 bg-black/30 rounded-lg border border-white/5">
                        <Waves className="w-3.5 h-3.5 text-orange-400" />
                        <span className="text-[10px] font-mono text-orange-400">{centerFreqMHz.toFixed(3)} MHz</span>
                    </div>

                    <div className="flex items-center gap-2 px-3 py-2 bg-black/30 rounded-lg border border-white/5">
                        <Terminal className="w-3.5 h-3.5 text-emerald-400" />
                        <span className="text-[10px] font-mono text-emerald-400">{packetCount} pkts</span>
                    </div>
                </div>
            </div>

            {/* Main Grid */}
            <div className="flex-1 grid grid-cols-[240px_1fr_280px] gap-2 p-2 min-h-0">
                {/* Left */}
                <div className="flex flex-col gap-2 overflow-y-auto">
                    <div className="glass-panel rounded-xl overflow-hidden"><ControlPanel /></div>
                    <div className="glass-panel rounded-xl overflow-hidden flex-1"><SignalMetrics /></div>
                </div>

                {/* Center */}
                <div className="flex flex-col gap-2 min-h-0">
                    <div className="glass-panel rounded-xl overflow-hidden flex-[3] min-h-0 relative">
                        <div className="absolute top-2 left-3 flex items-center gap-2 z-10">
                            <Activity className="w-3.5 h-3.5 text-cyan-400" />
                            <span className="text-[10px] uppercase tracking-widest text-gray-400 font-medium">Spectrum</span>
                            {isReceiving && (
                                <div className="flex items-center gap-1 ml-2 px-1.5 py-0.5 bg-emerald-500/15 rounded text-[9px] text-emerald-400">
                                    <div className="w-1.5 h-1.5 bg-emerald-500 rounded-full animate-pulse" />
                                    {mode === 'usrp' ? 'REAL' : 'SIM'}
                                </div>
                            )}
                        </div>
                        <SpectrumAnalyzer />
                    </div>

                    <div className="flex gap-2 flex-[2] min-h-0">
                        <div className="glass-panel rounded-xl overflow-hidden flex-[2] relative">
                            <div className="absolute top-2 left-3 flex items-center gap-2 z-10">
                                <Waves className="w-3.5 h-3.5 text-orange-400" />
                                <span className="text-[10px] uppercase tracking-widest text-gray-400 font-medium">Waterfall</span>
                            </div>
                            <WaterfallDisplay />
                        </div>
                        <div className="glass-panel rounded-xl overflow-hidden flex-1 relative">
                            <div className="absolute top-2 left-3 flex items-center gap-2 z-10">
                                <Signal className="w-3.5 h-3.5 text-emerald-400" />
                                <span className="text-[10px] uppercase tracking-widest text-gray-400 font-medium">Constellation</span>
                            </div>
                            <ConstellationDiagram />
                        </div>
                    </div>
                </div>

                {/* Right */}
                <div className="glass-panel rounded-xl overflow-hidden flex flex-col gap-2 bg-transparent border-none w-80">
                    <div className="glass-panel rounded-xl overflow-hidden">
                        <SensorDashboard />
                    </div>
                    <div className="glass-panel rounded-xl overflow-hidden flex-1">
                        <ADCSControl />
                    </div>
                    <div className="glass-panel rounded-xl overflow-hidden flex-[2] flex flex-col">
                        <DecodedDataStream />
                    </div>
                </div>
            </div>

            {/* Bottom Bar */}
            <div className="shrink-0 flex items-center justify-between px-4 py-2 bg-black/40 border-t border-white/5">
                <span className="text-[10px] text-gray-600 font-mono">
                    S-BAND {centerFreqMHz >= 1000 ? (centerFreqMHz / 1000).toFixed(3) : centerFreqMHz.toFixed(3)} GHz • RH_ASK OOK • {mode === 'usrp' ? 'Real USRP-2900' : 'Simulated'}
                </span>
                <span className="text-[10px] text-gray-600 font-mono">
                    FFT: {FFT_SIZE}-pt • {sampleRateKHz >= 1000 ? `${sampleRateKHz / 1000}M` : `${sampleRateKHz}k`}SPS • CubeDynamics SDR v3
                </span>
            </div>

            {/* Corner decorations */}
            <div className="absolute top-0 left-0 w-20 h-20 border-l-2 border-t-2 border-orange-500/20 rounded-tl-3xl pointer-events-none" />
            <div className="absolute top-0 right-0 w-20 h-20 border-r-2 border-t-2 border-orange-500/20 rounded-tr-3xl pointer-events-none" />
            <div className="absolute bottom-0 left-0 w-20 h-20 border-l-2 border-b-2 border-orange-500/20 rounded-bl-3xl pointer-events-none" />
            <div className="absolute bottom-0 right-0 w-20 h-20 border-r-2 border-b-2 border-orange-500/20 rounded-br-3xl pointer-events-none" />
        </div>
    );
}
