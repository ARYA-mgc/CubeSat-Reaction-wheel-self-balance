/**
 * Toolbar.tsx
 * Top toolbar with simulation controls - Enhanced UI
 */

import { Play, Pause, RotateCcw, Zap, Satellite, Radio, Clock, Cpu, Compass, Wifi, WifiOff, Globe, ArrowRight, Send } from 'lucide-react';
import { useNavigate } from 'react-router-dom';
import { useSimulationStore } from '../store/simulationStore';
import { useIMUStore } from '../store/imuStore';
import { cn } from '../utils/cn';
import ThemeToggle from './ThemeToggle';

export default function Toolbar() {
  const navigate = useNavigate();
  const { isRunning, linkStatus, currentData, start, stop, reset, injectFault } = useSimulationStore();
  const { useIMUMode, isConnected: imuConnected, dataRate, toggleMode } = useIMUStore();

  const getLinkStatusColor = () => {
    switch (linkStatus) {
      case 'connected':
        return 'bg-emerald-500';
      case 'degraded':
        return 'bg-amber-500';
      case 'disconnected':
        return 'bg-red-500';
      default:
        return 'bg-gray-500';
    }
  };

  const getLinkStatusText = () => {
    switch (linkStatus) {
      case 'connected':
        return 'NOMINAL';
      case 'degraded':
        return 'DEGRADED';
      case 'disconnected':
        return 'OFFLINE';
      default:
        return 'UNKNOWN';
    }
  };

  const simTime = currentData?.physicsState.time ?? 0;

  return (
    <div className="w-full glass-panel border-b border-white/10 px-4 py-2.5 flex items-center justify-between sticky top-0 z-50">
      {/* Logo & Branding */}
      <div className="flex items-center gap-4">
        <div className="flex items-center gap-3">
          <div className="relative">
            <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-cyan-500/20 to-emerald-500/20 border border-cyan-500/30 flex items-center justify-center">
              <Satellite className={cn("w-5 h-5 text-cyan-400", isRunning && "spin-slow")} />
            </div>
            {isRunning && (
              <div className="absolute -top-0.5 -right-0.5 w-2.5 h-2.5 bg-emerald-500 rounded-full pulse-indicator" />
            )}
          </div>
          <div>
            <h1 className="text-lg font-bold gradient-text tracking-tight">CubeDynamics</h1>
            <p className="text-[10px] text-gray-500 uppercase tracking-widest">HIL Simulation Platform</p>
          </div>
        </div>

        {/* Simulation Time */}
        <div className="hidden md:flex items-center gap-2 px-3 py-1.5 bg-black/30 rounded-lg border border-white/5">
          <Clock className="w-3.5 h-3.5 text-gray-500" />
          <span className="text-xs font-mono text-gray-400">T+</span>
          <span className="text-sm font-mono text-white tabular-nums">{simTime.toFixed(2)}s</span>
        </div>
      </div>

      {/* Controls */}
      <div className="flex items-center gap-2">
        {!isRunning ? (
          <button
            onClick={start}
            className="flex items-center gap-2 px-5 py-2.5 bg-emerald-500/10 hover:bg-emerald-500/20 text-emerald-400 border border-emerald-500/30 rounded-xl transition-all font-medium text-sm group"
          >
            <Play className="w-4 h-4 group-hover:scale-110 transition-transform" fill="currentColor" />
            <span>Start</span>
          </button>
        ) : (
          <button
            onClick={stop}
            className="flex items-center gap-2 px-5 py-2.5 bg-amber-500/10 hover:bg-amber-500/20 text-amber-400 border border-amber-500/30 rounded-xl transition-all font-medium text-sm"
          >
            <Pause className="w-4 h-4" fill="currentColor" />
            <span>Pause</span>
          </button>
        )}

        <button
          onClick={reset}
          className="flex items-center gap-2 px-4 py-2.5 bg-white/5 hover:bg-white/10 text-gray-300 border border-white/10 rounded-xl transition-all font-medium text-sm"
        >
          <RotateCcw className="w-4 h-4" />
          <span className="hidden sm:inline">Reset</span>
        </button>

        <div className="w-px h-8 bg-white/10 mx-1" />

        <button
          onClick={injectFault}
          disabled={!isRunning}
          className="flex items-center gap-2 px-4 py-2.5 bg-red-500/10 hover:bg-red-500/20 text-red-400 border border-red-500/30 rounded-xl transition-all font-medium text-sm disabled:opacity-30 disabled:cursor-not-allowed disabled:hover:bg-red-500/10"
        >
          <Zap className="w-4 h-4" />
          <span className="hidden sm:inline">Inject Fault</span>
        </button>

        <div className="w-px h-8 bg-white/10 mx-1" />

        {/* IMU Mode Toggle */}
        <button
          onClick={toggleMode}
          className={cn(
            "flex items-center gap-2 px-4 py-2.5 border rounded-xl transition-all font-medium text-sm",
            useIMUMode
              ? "bg-violet-500/20 hover:bg-violet-500/30 text-violet-400 border-violet-500/30"
              : "bg-white/5 hover:bg-white/10 text-gray-400 border-white/10"
          )}
        >
          <Compass className={cn("w-4 h-4", useIMUMode && "animate-pulse")} />
          <span className="hidden sm:inline">IMU Mode</span>
        </button>

        <div className="w-px h-8 bg-white/10 mx-1" />

        {/* Uplink Terminal - Dedicated Command Page */}
        <button
          onClick={() => navigate('/uplink')}
          className="flex items-center gap-2 px-4 py-2.5 bg-violet-500/10 hover:bg-violet-500/20 text-violet-400 border border-violet-500/30 rounded-xl transition-all font-medium text-sm group"
        >
          <Send className="w-4 h-4 group-hover:-translate-y-1 group-hover:translate-x-1 transition-transform" />
          <span className="hidden sm:inline">Uplink Terminal</span>
          <ArrowRight className="w-3 h-3 opacity-0 group-hover:opacity-100 transition-opacity" />
        </button>

        {/* View Track - Navigation to LEO Tracking Page */}
        <button
          onClick={() => navigate('/tracking')}
          className="flex items-center gap-2 px-4 py-2.5 bg-gradient-to-r from-cyan-500/10 to-emerald-500/10 hover:from-cyan-500/20 hover:to-emerald-500/20 text-cyan-400 border border-cyan-500/30 rounded-xl transition-all font-medium text-sm group"
        >
          <Globe className="w-4 h-4 group-hover:animate-spin" style={{ animationDuration: '3s' }} />
          <span className="hidden sm:inline">View Track</span>
          <ArrowRight className="w-3 h-3 opacity-0 group-hover:opacity-100 transition-opacity" />
        </button>

        {/* SDR View - Navigation to SDR Live View Page */}
        <button
          onClick={() => navigate('/sdr')}
          className="flex items-center gap-2 px-4 py-2.5 bg-gradient-to-r from-orange-500/10 to-red-500/10 hover:from-orange-500/20 hover:to-red-500/20 text-orange-400 border border-orange-500/30 rounded-xl transition-all font-medium text-sm group"
        >
          <Radio className="w-4 h-4 group-hover:animate-pulse" />
          <span className="hidden sm:inline">SDR View</span>
          <ArrowRight className="w-3 h-3 opacity-0 group-hover:opacity-100 transition-opacity" />
        </button>
      </div>

      {/* Status Indicators */}
      <div className="flex items-center gap-3">
        {/* Theme Toggle */}
        <ThemeToggle />

        <div className="w-px h-8 bg-white/10" />

        {/* OBC Status */}
        <div className="flex items-center gap-2 px-3 py-2 bg-black/30 rounded-lg border border-white/5">
          <Cpu className="w-3.5 h-3.5 text-cyan-400" />
          <span className="text-[10px] text-gray-500 uppercase">OBC</span>
          <div className={cn(
            "w-1.5 h-1.5 rounded-full",
            isRunning ? "bg-emerald-500 pulse-indicator" : "bg-gray-600"
          )} />
        </div>

        {/* Link Status */}
        <div className="flex items-center gap-2 px-3 py-2 bg-black/30 rounded-lg border border-white/5">
          <Radio className={cn("w-3.5 h-3.5", linkStatus === 'connected' ? "text-emerald-400" : "text-gray-500")} />
          <span className="text-[10px] text-gray-500 uppercase">LoRa</span>
          <div className={cn(
            'w-1.5 h-1.5 rounded-full',
            getLinkStatusColor(),
            linkStatus === 'connected' && isRunning && 'pulse-indicator'
          )} />
          <span className={cn(
            "text-[10px] font-mono uppercase tracking-wider",
            linkStatus === 'connected' ? "text-emerald-400" :
              linkStatus === 'degraded' ? "text-amber-400" : "text-gray-500"
          )}>
            {getLinkStatusText()}
          </span>
        </div>

        {/* IMU Status */}
        <div className={cn(
          "flex items-center gap-2 px-3 py-2 bg-black/30 rounded-lg border",
          useIMUMode ? "border-violet-500/30" : "border-white/5"
        )}>
          {imuConnected ? (
            <Wifi className="w-3.5 h-3.5 text-violet-400" />
          ) : (
            <WifiOff className="w-3.5 h-3.5 text-gray-500" />
          )}
          <span className="text-[10px] text-gray-500 uppercase">IMU</span>
          <div className={cn(
            'w-1.5 h-1.5 rounded-full',
            imuConnected ? 'bg-violet-500 pulse-indicator' : 'bg-gray-600'
          )} />
          {imuConnected && (
            <span className="text-[10px] font-mono text-violet-400">
              {dataRate}Hz
            </span>
          )}
        </div>
      </div>
    </div>
  );
}
