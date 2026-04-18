/**
 * StatusPanel.tsx
 * Displays real-time telemetry values in a modern card layout - Enhanced UI
 */

import { Database, RotateCcw, Gauge, Zap, Clock, ArrowRight } from 'lucide-react';
import { useSimulationStore } from '../store/simulationStore';
import { cn } from '../utils/cn';

interface DataCardProps {
  label: string;
  value: number;
  unit: string;
  color?: 'red' | 'green' | 'cyan' | 'amber' | 'white';
  precision?: number;
  showBar?: boolean;
  barMax?: number;
}

function DataCard({ label, value, unit, color = 'white', precision = 3, showBar = false, barMax = 180 }: DataCardProps) {
  const colorClasses = {
    red: 'text-red-400',
    green: 'text-emerald-400',
    cyan: 'text-cyan-400',
    amber: 'text-amber-400',
    white: 'text-white',
  };

  const bgClasses = {
    red: 'bg-red-500',
    green: 'bg-emerald-500',
    cyan: 'bg-cyan-500',
    amber: 'bg-amber-500',
    white: 'bg-white',
  };

  const barWidth = showBar ? Math.min(Math.abs(value) / barMax * 100, 100) : 0;

  return (
    <div className="relative bg-black/30 rounded-lg p-2.5 border border-white/5 overflow-hidden group hover:border-white/10 transition-colors">
      {showBar && (
        <div 
          className={cn("absolute bottom-0 left-0 h-0.5 transition-all", bgClasses[color])}
          style={{ width: `${barWidth}%`, opacity: 0.5 }}
        />
      )}
      <div className="flex items-center justify-between">
        <span className="text-[10px] font-mono text-gray-500 uppercase tracking-wider">{label}</span>
        <span className="text-[9px] font-mono text-gray-600">{unit}</span>
      </div>
      <div className={cn("text-lg font-mono font-semibold tabular-nums mt-0.5", colorClasses[color])}>
        {value >= 0 ? ' ' : ''}{value.toFixed(precision)}
      </div>
    </div>
  );
}

interface SectionProps {
  title: string;
  icon: React.ReactNode;
  iconColor: string;
  children: React.ReactNode;
}

function Section({ title, icon, iconColor, children }: SectionProps) {
  return (
    <div className="space-y-2">
      <div className="flex items-center gap-2 px-1">
        <div className={cn("p-1 rounded", iconColor)}>
          {icon}
        </div>
        <span className="text-[10px] font-semibold text-gray-400 uppercase tracking-wider">{title}</span>
      </div>
      {children}
    </div>
  );
}

export default function StatusPanel() {
  const currentData = useSimulationStore((state) => state.currentData);
  const setpoint = useSimulationStore((state) => state.setpoint);
  
  if (!currentData) {
    return (
      <div className="w-full h-full glass-panel rounded-xl flex items-center justify-center">
        <div className="text-center">
          <Database className="w-8 h-8 text-gray-600 mx-auto mb-2" />
          <span className="text-gray-500 font-mono text-xs">Awaiting telemetry...</span>
        </div>
      </div>
    );
  }
  
  const euler = currentData.eulerAngles;
  const angVel = currentData.physicsState.angularVelocity;
  const torque = currentData.controlOutput.torque;
  const angVelMag = Math.sqrt(angVel.x ** 2 + angVel.y ** 2 + angVel.z ** 2);
  
  // Calculate attitude error
  const rollDeg = euler[0] * 180 / Math.PI;
  const pitchDeg = euler[1] * 180 / Math.PI;
  const yawDeg = euler[2] * 180 / Math.PI;
  
  const rollError = Math.abs(rollDeg - setpoint.roll);
  const pitchError = Math.abs(pitchDeg - setpoint.pitch);
  const yawError = Math.abs(yawDeg - setpoint.yaw);
  const totalError = Math.sqrt(rollError ** 2 + pitchError ** 2 + yawError ** 2);
  
  const isOnTarget = totalError < 5; // Within 5 degrees
  
  return (
    <div className="w-full h-full glass-panel rounded-xl overflow-hidden flex flex-col">
      {/* Header */}
      <div className="px-4 py-3 border-b border-white/5 bg-black/20">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-2">
            <Database className="w-4 h-4 text-cyan-400" />
            <h2 className="text-sm font-semibold text-white">Telemetry</h2>
          </div>
          <div className={cn(
            "px-2 py-0.5 rounded-full text-[9px] font-mono uppercase tracking-wider",
            isOnTarget 
              ? "bg-emerald-500/20 text-emerald-400 border border-emerald-500/30"
              : "bg-amber-500/20 text-amber-400 border border-amber-500/30"
          )}>
            {isOnTarget ? 'On Target' : 'Slewing'}
          </div>
        </div>
      </div>
      
      {/* Content */}
      <div className="flex-1 overflow-y-auto p-3 space-y-4">
        {/* Attitude */}
        <Section 
          title="Attitude" 
          icon={<RotateCcw className="w-3 h-3 text-cyan-400" />}
          iconColor="bg-cyan-500/10"
        >
          <div className="grid grid-cols-3 gap-2">
            <DataCard label="Roll" value={rollDeg} unit="°" color="red" precision={2} showBar barMax={180} />
            <DataCard label="Pitch" value={pitchDeg} unit="°" color="green" precision={2} showBar barMax={180} />
            <DataCard label="Yaw" value={yawDeg} unit="°" color="cyan" precision={2} showBar barMax={180} />
          </div>
          
          {/* Target Comparison */}
          <div className="mt-2 px-2 py-1.5 bg-black/20 rounded-lg border border-white/5">
            <div className="flex items-center justify-between text-[9px] font-mono">
              <span className="text-gray-500">Target</span>
              <div className="flex items-center gap-1 text-gray-400">
                <span className="text-red-400">{setpoint.roll.toFixed(0)}°</span>
                <ArrowRight className="w-2 h-2" />
                <span className="text-emerald-400">{setpoint.pitch.toFixed(0)}°</span>
                <ArrowRight className="w-2 h-2" />
                <span className="text-cyan-400">{setpoint.yaw.toFixed(0)}°</span>
              </div>
            </div>
            <div className="flex items-center justify-between text-[9px] font-mono mt-1">
              <span className="text-gray-500">Error</span>
              <span className={cn(
                totalError < 1 ? "text-emerald-400" :
                totalError < 5 ? "text-amber-400" : "text-red-400"
              )}>
                {totalError.toFixed(2)}°
              </span>
            </div>
          </div>
        </Section>
        
        {/* Angular Velocity */}
        <Section 
          title="Angular Velocity" 
          icon={<Gauge className="w-3 h-3 text-emerald-400" />}
          iconColor="bg-emerald-500/10"
        >
          <div className="grid grid-cols-2 gap-2">
            <DataCard label="ωx" value={angVel.x} unit="rad/s" precision={4} />
            <DataCard label="ωy" value={angVel.y} unit="rad/s" precision={4} />
            <DataCard label="ωz" value={angVel.z} unit="rad/s" precision={4} />
            <DataCard label="|ω|" value={angVelMag} unit="rad/s" color="amber" precision={4} />
          </div>
        </Section>
        
        {/* Control Torque */}
        <Section 
          title="Control Torque" 
          icon={<Zap className="w-3 h-3 text-amber-400" />}
          iconColor="bg-amber-500/10"
        >
          <div className="grid grid-cols-3 gap-2">
            <DataCard label="τx" value={torque.x * 1000} unit="mN·m" precision={4} />
            <DataCard label="τy" value={torque.y * 1000} unit="mN·m" precision={4} />
            <DataCard label="τz" value={torque.z * 1000} unit="mN·m" precision={4} />
          </div>
        </Section>
        
        {/* Simulation Time */}
        <Section 
          title="Mission Clock" 
          icon={<Clock className="w-3 h-3 text-white" />}
          iconColor="bg-white/10"
        >
          <div className="bg-black/30 rounded-lg p-3 border border-white/5">
            <div className="text-center">
              <span className="text-2xl font-mono font-bold text-white tabular-nums tracking-wider">
                {formatTime(currentData.physicsState.time)}
              </span>
              <p className="text-[9px] font-mono text-gray-500 mt-1">
                MET (Mission Elapsed Time)
              </p>
            </div>
          </div>
        </Section>
      </div>
    </div>
  );
}

function formatTime(seconds: number): string {
  const mins = Math.floor(seconds / 60);
  const secs = Math.floor(seconds % 60);
  const ms = Math.floor((seconds % 1) * 100);
  return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}.${ms.toString().padStart(2, '0')}`;
}
