/**
 * ADCSControl.tsx
 * Real-time remote control for the physical CubeSat over UDP
 */

import { useState } from 'react';
import { RefreshCw, Radio, Zap, ShieldAlert } from 'lucide-react';
import { useIMUStore } from '../store/imuStore';
import { cn } from '../utils/cn';

export default function ADCSControl() {
    const { isConnected, setTargetAngles, zeroIMU } = useIMUStore();

    const [pitch, setPitch] = useState(0);
    const [roll, setRoll] = useState(0);
    const [yaw, setYaw] = useState(0);
    const [isSending, setIsSending] = useState(false);

    const handleApply = () => {
        setIsSending(true);
        setTargetAngles(pitch, roll, yaw);
        setTimeout(() => setIsSending(false), 500);
    };

    const handleReset = () => {
        setPitch(0);
        setRoll(0);
        setYaw(0);
        setTargetAngles(0, 0, 0);
    };

    return (
        <div className="w-full h-full glass-panel rounded-xl overflow-hidden flex flex-col">
            {/* Header */}
            <div className="px-4 py-3 border-b border-white/5 bg-black/40 flex items-center justify-between">
                <div className="flex items-center gap-2">
                    <Radio className={cn("w-4 h-4", isConnected ? "text-emerald-400 animate-pulse" : "text-gray-500")} />
                    <h2 className="text-sm font-semibold text-white uppercase tracking-wider">ADCS Command Uplink</h2>
                </div>
                <div className={cn(
                    "px-2 py-0.5 rounded-full text-[10px] font-bold uppercase",
                    isConnected ? "bg-emerald-500/10 text-emerald-400 border border-emerald-500/20" : "bg-red-500/10 text-red-400 border border-red-500/20"
                )}>
                    {isConnected ? "Direct Link" : "Offline"}
                </div>
            </div>

            {/* Content */}
            <div className="flex-1 p-4 space-y-5 overflow-y-auto">
                <div className="grid grid-cols-1 gap-4">
                    {/* Pitch Control */}
                    <div className="space-y-2">
                        <div className="flex justify-between items-center text-[10px] font-mono text-gray-500 uppercase">
                            <span>Pitch Axis (θ)</span>
                            <span className="text-cyan-400">{pitch.toFixed(1)}°</span>
                        </div>
                        <input
                            type="range" min="-45" max="45" step="1"
                            value={pitch} onChange={(e) => setPitch(parseFloat(e.target.value))}
                            className="w-full h-1.5 bg-white/5 rounded-full appearance-none cursor-pointer accent-cyan-500"
                        />
                    </div>

                    {/* Roll Control */}
                    <div className="space-y-2">
                        <div className="flex justify-between items-center text-[10px] font-mono text-gray-500 uppercase">
                            <span>Roll Axis (φ)</span>
                            <span className="text-emerald-400">{roll.toFixed(1)}°</span>
                        </div>
                        <input
                            type="range" min="-45" max="45" step="1"
                            value={roll} onChange={(e) => setRoll(parseFloat(e.target.value))}
                            className="w-full h-1.5 bg-white/5 rounded-full appearance-none cursor-pointer accent-emerald-500"
                        />
                    </div>

                    {/* Yaw Control */}
                    <div className="space-y-2">
                        <div className="flex justify-between items-center text-[10px] font-mono text-gray-500 uppercase">
                            <span>Yaw Axis (ψ)</span>
                            <span className="text-amber-400">{yaw.toFixed(1)}°</span>
                        </div>
                        <input
                            type="range" min="-180" max="180" step="1"
                            value={yaw} onChange={(e) => setYaw(parseFloat(e.target.value))}
                            className="w-full h-1.5 bg-white/5 rounded-full appearance-none cursor-pointer accent-amber-500"
                        />
                    </div>
                </div>

                {/* Safety Warning */}
                <div className="p-3 rounded-lg bg-amber-500/5 border border-amber-500/10 flex gap-3">
                    <ShieldAlert className="w-4 h-4 text-amber-500 shrink-0 mt-0.5" />
                    <p className="text-[10px] text-amber-500/80 leading-relaxed italic">
                        Commands are sent via private UDP link. High gain movements may destabilize the rope suspension. Monitor reaction wheel RPMs.
                    </p>
                </div>
            </div>

            {/* Actions */}
            <div className="p-4 pt-0 space-y-2">
                <button
                    onClick={handleApply}
                    disabled={!isConnected}
                    className={cn(
                        "w-full py-2.5 rounded-lg text-xs font-bold uppercase tracking-widest transition-all flex items-center justify-center gap-2",
                        isConnected
                            ? "bg-cyan-500 text-black hover:bg-cyan-400 active:scale-95"
                            : "bg-gray-800 text-gray-500 cursor-not-allowed"
                    )}
                >
                    {isSending ? <RefreshCw className="w-3.5 h-3.5 animate-spin" /> : <Zap className="w-3.5 h-3.5" />}
                    Transmit Uplink
                </button>

                <button
                    onClick={zeroIMU}
                    disabled={!isConnected}
                    className={cn(
                        "w-full py-2.5 rounded-lg text-xs font-bold uppercase tracking-widest transition-all border",
                        isConnected
                            ? "bg-violet-500/10 text-violet-400 border-violet-500/30 hover:bg-violet-500/20 active:scale-95"
                            : "bg-gray-800 text-gray-500 border-white/5 cursor-not-allowed"
                    )}
                >
                    Zero Orientation
                </button>

                <button
                    onClick={handleReset}
                    disabled={!isConnected}
                    className="w-full py-2 rounded-lg text-[10px] font-mono uppercase text-gray-500 hover:text-white hover:bg-white/5 transition-all"
                >
                    Re-Zero Center Point
                </button>
            </div>
        </div>
    );
}
