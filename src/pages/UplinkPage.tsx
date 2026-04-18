/**
 * UplinkPage.tsx
 * Dedicated command transmission interface for CubeSat ADCS
 */

import { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import {
    ArrowLeft,
    Send,
    RotateCw,
    RotateCcw,
    Activity,
    ShieldCheck,
    Zap,
    Crosshair
} from 'lucide-react';
import { useIMUStore } from '../store/imuStore';
import { cn } from '../utils/cn';

export default function UplinkPage() {
    const navigate = useNavigate();
    const {
        isConnected,
        rotateRelative,
        calibrate,
        setTargetAngles,
        motorsEnabled,
        toggleMotors,
        imuQuaternion,
        dataRate,
        connect
    } = useIMUStore();

    // Auto-connect on mount for this dedicated terminal
    useEffect(() => {
        if (!isConnected) {
            console.log('[Uplink] Initiating proactive bridge connection...');
            connect();
        }
    }, [isConnected, connect]);

    const [commandValue, setCommandValue] = useState(30);
    const [isCalibrating, setIsCalibrating] = useState<string | null>(null);

    const handleRotate = (axis: 'pitch' | 'roll' | 'yaw', dir: 'left' | 'right') => {
        const degrees = dir === 'left' ? -commandValue : commandValue;
        rotateRelative(axis, degrees);
    };

    const handleCalibrateStart = (mode: 'left' | 'right' | 'center') => {
        setIsCalibrating(mode);
        calibrate(mode);
    };

    const handleCalibrateStop = () => {
        setIsCalibrating(null);
        calibrate('stop');
    };

    return (
        <div className="min-h-screen bg-[#0a0a0c] text-slate-200 font-sans selection:bg-violet-500/30">
            {/* Header */}
            <div className="border-b border-white/5 bg-black/40 backdrop-blur-xl sticky top-0 z-50">
                <div className="max-w-7xl mx-auto px-4 h-16 flex items-center justify-between">
                    <div className="flex items-center gap-4">
                        <button
                            onClick={() => navigate('/')}
                            className="p-2 hover:bg-white/5 rounded-lg transition-colors text-slate-400 hover:text-white"
                        >
                            <ArrowLeft className="w-5 h-5" />
                        </button>
                        <div className="flex items-center gap-2">
                            <div className="w-8 h-8 rounded-lg bg-violet-600/20 flex items-center justify-center border border-violet-500/30">
                                <Send className="w-4 h-4 text-violet-400" />
                            </div>
                            <div>
                                <h1 className="text-sm font-bold tracking-tight text-white uppercase">Uplink Terminal</h1>
                                <p className="text-[10px] text-slate-500 font-mono">SECURE COMMAND CHANNEL</p>
                            </div>
                        </div>
                    </div>

                    <div className="flex items-center gap-6">
                        {/* Master Motor Toggle */}
                        <div className="flex items-center gap-3 px-4 py-1.5 bg-black/40 border border-white/5 rounded-full">
                            <span className={`text-[10px] font-bold font-mono tracking-tighter ${motorsEnabled ? 'text-green-500' : 'text-amber-500'}`}>
                                {motorsEnabled ? 'MOTORS LIVE' : 'MOTORS STANDBY'}
                            </span>
                            <button
                                onClick={toggleMotors}
                                className={`w-10 h-5 rounded-full transition-all relative ${motorsEnabled ? 'bg-green-500' : 'bg-slate-700'}`}
                            >
                                <div className={`absolute top-1 w-3 h-3 bg-white rounded-full transition-all ${motorsEnabled ? 'left-6' : 'left-1'}`} />
                            </button>
                        </div>

                        <div className="flex items-center gap-2 px-3 py-1.5 bg-black/40 border border-white/5 rounded-full">
                            <div className={`w-1.5 h-1.5 rounded-full ${isConnected ? 'bg-emerald-500 animate-pulse' : 'bg-red-500'}`} />
                            <span className="text-[11px] font-medium text-slate-300">
                                {isConnected ? 'LINK ESTABLISHED' : 'NO UPLINK SIGNAL'}
                            </span>
                        </div>
                    </div>
                </div>
            </div>

            {/* Live Telemetry Bar - Mini Readout */}
            {isConnected && (
                <div className="bg-violet-600/5 border-b border-violet-500/10 py-2">
                    <div className="max-w-7xl mx-auto px-4 flex items-center justify-between">
                        <div className="flex items-center gap-6 overflow-x-auto no-scrollbar">
                            <div className="flex items-center gap-2">
                                <span className="text-[9px] font-bold text-violet-500/50 uppercase">Readout:</span>
                                <span className="text-xs font-mono text-violet-300">
                                    I:{imuQuaternion?.i.toFixed(3) || '0.000'}
                                </span>
                                <span className="text-xs font-mono text-violet-300">
                                    J:{imuQuaternion?.j.toFixed(3) || '0.000'}
                                </span>
                                <span className="text-xs font-mono text-violet-300">
                                    K:{imuQuaternion?.k.toFixed(3) || '0.000'}
                                </span>
                                <span className="text-xs font-mono text-violet-300">
                                    R:{imuQuaternion?.real.toFixed(3) || '0.000'}
                                </span>
                            </div>
                        </div>
                        <div className="flex items-center gap-2">
                            <Activity className="w-3 h-3 text-emerald-500" />
                            <span className="text-[10px] font-mono text-emerald-500/70">{dataRate} Hz</span>
                        </div>
                    </div>
                </div>
            )}

            <main className="max-w-7xl mx-auto px-4 py-8">
                <div className="grid grid-cols-1 lg:grid-cols-12 gap-6">
                    {/* Left Column - Manual Control */}
                    <div className="lg:col-span-8 space-y-6">
                        <div className="glass-panel p-8 border-violet-500/30">
                            <div className="flex items-center justify-between mb-8">
                                <div className="flex items-center gap-3">
                                    <div className="w-10 h-10 rounded-xl bg-violet-500/10 flex items-center justify-center border border-violet-500/30">
                                        <Zap className="w-6 h-6 text-violet-400" />
                                    </div>
                                    <div>
                                        <h3 className="font-bold text-lg">Reaction Wheel Uplink</h3>
                                        <p className="text-[10px] text-gray-500 font-mono">ADCS_CMD_VECTOR_DISPATCH</p>
                                    </div>
                                </div>
                                <div className="flex items-center gap-2">
                                    {[15, 30, 45, 90].map((val) => (
                                        <button
                                            key={val}
                                            onClick={() => setCommandValue(val)}
                                            className={cn(
                                                "px-3 py-1.5 text-[10px] font-mono border rounded-lg transition-all",
                                                commandValue === val
                                                    ? "bg-violet-500/20 border-violet-500/40 text-violet-400"
                                                    : "bg-white/5 border-white/10 text-slate-500 hover:bg-white/10"
                                            )}
                                        >
                                            {val}°
                                        </button>
                                    ))}
                                </div>
                            </div>

                            <div className="grid grid-cols-3 gap-8">
                                {/* Roll Control */}
                                <div className="space-y-4">
                                    <div className="text-center">
                                        <span className="text-xs font-bold text-gray-400 uppercase tracking-widest">Axis Roll</span>
                                    </div>
                                    <div className="flex flex-col gap-3">
                                        <button
                                            disabled={!isConnected || !motorsEnabled}
                                            onClick={() => handleRotate('roll', 'left')}
                                            className="group flex flex-col items-center gap-2 p-6 bg-white/5 border border-white/10 rounded-2xl hover:bg-violet-500/10 hover:border-violet-500/30 transition-all disabled:opacity-20"
                                        >
                                            <RotateCcw className="w-8 h-8 text-violet-400 group-hover:scale-110 transition-transform" />
                                            <span className="text-[10px] font-bold text-gray-500 group-hover:text-violet-400">LEFT</span>
                                        </button>
                                        <button
                                            disabled={!isConnected || !motorsEnabled}
                                            onClick={() => handleRotate('roll', 'right')}
                                            className="group flex flex-col items-center gap-2 p-6 bg-white/5 border border-white/10 rounded-2xl hover:bg-violet-500/10 hover:border-violet-500/30 transition-all disabled:opacity-20"
                                        >
                                            <RotateCw className="w-8 h-8 text-violet-400 group-hover:scale-110 transition-transform" />
                                            <span className="text-[10px] font-bold text-gray-500 group-hover:text-violet-400">RIGHT</span>
                                        </button>
                                    </div>
                                </div>

                                {/* Pitch Control */}
                                <div className="space-y-4">
                                    <div className="text-center">
                                        <span className="text-xs font-bold text-gray-400 uppercase tracking-widest">Axis Pitch</span>
                                    </div>
                                    <div className="flex flex-col gap-3">
                                        <button
                                            disabled={!isConnected || !motorsEnabled}
                                            onClick={() => handleRotate('pitch', 'left')}
                                            className="group flex flex-col items-center gap-2 p-6 bg-white/5 border border-white/10 rounded-2xl hover:bg-emerald-500/10 hover:border-emerald-500/30 transition-all disabled:opacity-20"
                                        >
                                            <RotateCcw className="w-8 h-8 text-emerald-400 -rotate-90 group-hover:-translate-y-1 transition-transform" />
                                            <span className="text-[10px] font-bold text-gray-500 group-hover:text-emerald-400">DECREASE</span>
                                        </button>
                                        <button
                                            disabled={!isConnected || !motorsEnabled}
                                            onClick={() => handleRotate('pitch', 'right')}
                                            className="group flex flex-col items-center gap-2 p-6 bg-white/5 border border-white/10 rounded-2xl hover:bg-emerald-500/10 hover:border-emerald-500/30 transition-all disabled:opacity-20"
                                        >
                                            <RotateCw className="w-8 h-8 text-emerald-400 90 group-hover:translate-y-1 transition-transform" />
                                            <span className="text-[10px] font-bold text-gray-500 group-hover:text-emerald-400">INCREASE</span>
                                        </button>
                                    </div>
                                </div>

                                {/* Yaw Control */}
                                <div className="space-y-4">
                                    <div className="text-center">
                                        <span className="text-xs font-bold text-gray-400 uppercase tracking-widest">Axis Yaw</span>
                                    </div>
                                    <div className="flex flex-col gap-3">
                                        <button
                                            disabled={!isConnected || !motorsEnabled}
                                            onClick={() => handleRotate('yaw', 'left')}
                                            className="group flex flex-col items-center gap-2 p-6 bg-white/5 border border-white/10 rounded-2xl hover:bg-cyan-500/10 hover:border-cyan-500/30 transition-all disabled:opacity-20"
                                        >
                                            <RotateCcw className="w-8 h-8 text-cyan-400 group-hover:-rotate-45 transition-transform" />
                                            <span className="text-[10px] font-bold text-gray-500 group-hover:text-cyan-400">CCW</span>
                                        </button>
                                        <button
                                            disabled={!isConnected || !motorsEnabled}
                                            onClick={() => handleRotate('yaw', 'right')}
                                            className="group flex flex-col items-center gap-2 p-6 bg-white/5 border border-white/10 rounded-2xl hover:bg-cyan-500/10 hover:border-cyan-500/30 transition-all disabled:opacity-20"
                                        >
                                            <RotateCw className="w-8 h-8 text-cyan-400 group-hover:rotate-45 transition-transform" />
                                            <span className="text-[10px] font-bold text-gray-500 group-hover:text-cyan-400">CW</span>
                                        </button>
                                    </div>
                                </div>
                            </div>

                            <div className="mt-8 flex items-center gap-4 p-4 bg-black/40 border border-white/5 rounded-xl">
                                <div className="w-2 h-2 rounded-full bg-violet-500 animate-pulse" />
                                <span className="text-xs text-gray-400 flex-1">Relaying via UDP Bridge Port: <span className="text-white font-mono">4210</span></span>
                                <div className="flex gap-2">
                                    <button
                                        onClick={() => setTargetAngles(0, 0, 0)}
                                        className="px-4 py-1.5 bg-red-500/10 hover:bg-red-500/20 text-red-400 border border-red-500/30 rounded-lg text-[10px] font-bold uppercase"
                                    >
                                        Emergency Neutral
                                    </button>
                                </div>
                            </div>
                        </div>
                    </div>

                    {/* Right Column - Calibration */}
                    <div className="lg:col-span-4 space-y-6">
                        <div className="glass-panel border-white/5 rounded-2xl overflow-hidden bg-black/40">
                            <div className="px-5 py-4 border-b border-white/5 bg-white/[0.02] flex items-center justify-between">
                                <div className="flex items-center gap-3">
                                    <div className="p-2 bg-amber-500/10 rounded-lg">
                                        <Crosshair className="w-4 h-4 text-amber-500" />
                                    </div>
                                    <div>
                                        <h3 className="text-sm font-bold text-white">Calibration Station</h3>
                                        <p className="text-[10px] text-slate-500">Interactive Axis Alignment</p>
                                    </div>
                                </div>
                            </div>

                            <div className="p-5 space-y-6">
                                <div className="space-y-4">
                                    <div className="p-4 bg-amber-500/5 border border-amber-500/10 rounded-xl">
                                        <p className="text-[11px] text-amber-200/70 leading-relaxed mb-3 font-medium">
                                            Select a target limit to begin "Crawling". The CubeSat will rotate slowly. Press **STOP** when it reaches precisely 90° or 180°.
                                        </p>

                                        <div className="grid grid-cols-3 gap-2">
                                            <button
                                                disabled={isCalibrating !== null}
                                                onClick={() => handleCalibrateStart('left')}
                                                className={cn(
                                                    "px-3 py-2 rounded-lg border text-[10px] font-bold transition-all",
                                                    isCalibrating === 'left' ? "bg-amber-500 text-black border-amber-500" : "bg-amber-500/10 text-amber-400 border-amber-500/20 hover:bg-amber-500/20"
                                                )}
                                            >
                                                CRAWL L (90°)
                                            </button>
                                            <button
                                                disabled={isCalibrating !== null}
                                                onClick={() => handleCalibrateStart('right')}
                                                className={cn(
                                                    "px-3 py-2 rounded-lg border text-[10px] font-bold transition-all",
                                                    isCalibrating === 'right' ? "bg-amber-500 text-black border-amber-500" : "bg-amber-500/10 text-amber-400 border-amber-500/20 hover:bg-amber-500/20"
                                                )}
                                            >
                                                CRAWL R (180°)
                                            </button>
                                            <button
                                                disabled={isCalibrating !== null}
                                                onClick={() => handleCalibrateStart('center')}
                                                className="px-3 py-2 bg-slate-800/50 hover:bg-slate-800 text-slate-400 rounded-lg border border-white/5 text-[10px] font-bold transition-all"
                                            >
                                                HOME
                                            </button>
                                        </div>
                                    </div>

                                    {isCalibrating && (
                                        <button
                                            onClick={handleCalibrateStop}
                                            className="w-full py-4 bg-red-600 hover:bg-red-500 text-white rounded-xl border border-red-500/50 shadow-lg shadow-red-600/20 font-bold text-sm flex items-center justify-center gap-3 animate-pulse"
                                        >
                                            <div className="w-2 h-2 rounded-full bg-white animate-ping" />
                                            STOP & SAVE LIMIT
                                        </button>
                                    )}
                                </div>

                                <div className="pt-4 border-t border-white/5">
                                    <div className="flex items-center gap-2 mb-4">
                                        <ShieldCheck className="w-4 h-4 text-emerald-500" />
                                        <span className="text-xs font-bold text-white uppercase tracking-wider">Safety Status</span>
                                    </div>
                                    <div className="grid grid-cols-2 gap-3 text-center">
                                        <div className="p-3 bg-black/40 rounded-xl border border-white/5">
                                            <div className="text-[10px] text-slate-500 uppercase mb-1">IMU Drift</div>
                                            <div className="text-xs font-mono text-emerald-400">0.02°/h</div>
                                        </div>
                                        <div className="p-3 bg-black/40 rounded-xl border border-white/5">
                                            <div className="text-[10px] text-slate-500 uppercase mb-1">Signal</div>
                                            <div className="text-xs font-mono text-cyan-400">-42 dBm</div>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </main>
        </div>
    );
}
