/**
 * ControlPanel.tsx
 * Left panel for PID tuning and system configuration - Enhanced UI
 */

import { useState } from 'react';
import { Settings, Target, Sliders, Gauge, RefreshCw, ChevronRight, Waves } from 'lucide-react';
import { useSimulationStore } from '../store/simulationStore';
import { cn } from '../utils/cn';

interface SectionProps {
  title: string;
  icon: React.ReactNode;
  color: string;
  children: React.ReactNode;
  defaultOpen?: boolean;
}

function Section({ title, icon, color, children, defaultOpen = true }: SectionProps) {
  const [isOpen, setIsOpen] = useState(defaultOpen);
  
  return (
    <div className="border border-white/5 rounded-xl overflow-hidden bg-black/20">
      <button 
        onClick={() => setIsOpen(!isOpen)}
        className={cn(
          "w-full flex items-center gap-2 px-3 py-2.5 transition-colors",
          "hover:bg-white/5"
        )}
      >
        <div className={cn("p-1.5 rounded-lg", color)}>
          {icon}
        </div>
        <span className="text-sm font-medium text-white flex-1 text-left">{title}</span>
        <ChevronRight className={cn(
          "w-4 h-4 text-gray-500 transition-transform",
          isOpen && "rotate-90"
        )} />
      </button>
      {isOpen && (
        <div className="px-3 pb-3 space-y-2">
          {children}
        </div>
      )}
    </div>
  );
}

interface InputFieldProps {
  label: string;
  value: number;
  onChange: (value: number) => void;
  step?: string;
  unit?: string;
  color?: string;
}

function InputField({ label, value, onChange, step = "0.001", unit, color = "cyan" }: InputFieldProps) {
  const colorClasses: Record<string, string> = {
    cyan: "focus:border-cyan-500/50 focus:ring-cyan-500/20",
    green: "focus:border-emerald-500/50 focus:ring-emerald-500/20",
    amber: "focus:border-amber-500/50 focus:ring-amber-500/20",
    white: "focus:border-white/30 focus:ring-white/10",
  };
  
  return (
    <label className="block">
      <div className="flex items-center justify-between mb-1">
        <span className="text-[11px] text-gray-500 font-mono uppercase tracking-wider">{label}</span>
        {unit && <span className="text-[10px] text-gray-600 font-mono">{unit}</span>}
      </div>
      <input
        type="number"
        step={step}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value) || 0)}
        className={cn(
          "w-full px-3 py-2 bg-black/40 border border-white/10 rounded-lg",
          "text-white font-mono text-sm tabular-nums",
          "focus:outline-none focus:ring-2 transition-all",
          colorClasses[color]
        )}
      />
    </label>
  );
}

export default function ControlPanel() {
  const { pidGains, setpoint, setPIDGains, setSetpoint, setSensorNoise, setInitialConditions } = useSimulationStore();
  
  const [localPID, setLocalPID] = useState(pidGains);
  const [localSetpoint, setLocalSetpoint] = useState(setpoint);
  const [noise, setNoise] = useState(0.01);
  const [initialOmega, setInitialOmega] = useState({ wx: 0.1, wy: 0, wz: 0 });
  
  const handlePIDUpdate = () => {
    setPIDGains(localPID.Kp, localPID.Ki, localPID.Kd);
  };
  
  const handleSetpointUpdate = () => {
    setSetpoint(localSetpoint.roll, localSetpoint.pitch, localSetpoint.yaw);
  };
  
  const handleNoiseUpdate = () => {
    setSensorNoise(noise);
  };
  
  const handleInitialConditionsUpdate = () => {
    setInitialConditions(initialOmega.wx, initialOmega.wy, initialOmega.wz);
  };

  const applyPreset = (preset: 'stable' | 'aggressive' | 'smooth') => {
    const presets = {
      stable: { Kp: 0.01, Ki: 0.001, Kd: 0.005 },
      aggressive: { Kp: 0.025, Ki: 0.003, Kd: 0.008 },
      smooth: { Kp: 0.005, Ki: 0.0005, Kd: 0.01 },
    };
    setLocalPID(presets[preset]);
    setPIDGains(presets[preset].Kp, presets[preset].Ki, presets[preset].Kd);
  };
  
  return (
    <div className="w-full h-full glass-panel rounded-xl overflow-hidden flex flex-col">
      {/* Header */}
      <div className="px-4 py-3 border-b border-white/5 bg-black/20">
        <div className="flex items-center gap-2">
          <Settings className="w-4 h-4 text-cyan-400" />
          <h2 className="text-sm font-semibold text-white">Control Panel</h2>
        </div>
        <p className="text-[10px] text-gray-500 mt-0.5">Simulation Parameters</p>
      </div>
      
      {/* Content */}
      <div className="flex-1 overflow-y-auto p-3 space-y-3">
        {/* PID Controller */}
        <Section 
          title="PID Controller" 
          icon={<Sliders className="w-3.5 h-3.5 text-cyan-400" />}
          color="bg-cyan-500/10"
        >
          {/* Presets */}
          <div className="flex gap-1.5 mb-3">
            {['stable', 'aggressive', 'smooth'].map((preset) => (
              <button
                key={preset}
                onClick={() => applyPreset(preset as 'stable' | 'aggressive' | 'smooth')}
                className="flex-1 px-2 py-1.5 text-[10px] font-mono uppercase tracking-wider bg-white/5 hover:bg-cyan-500/10 border border-white/10 hover:border-cyan-500/30 text-gray-400 hover:text-cyan-400 rounded-lg transition-all"
              >
                {preset}
              </button>
            ))}
          </div>
          
          <InputField 
            label="Kp (Proportional)" 
            value={localPID.Kp} 
            onChange={(v) => setLocalPID({ ...localPID, Kp: v })}
            step="0.001"
            color="cyan"
          />
          <InputField 
            label="Ki (Integral)" 
            value={localPID.Ki} 
            onChange={(v) => setLocalPID({ ...localPID, Ki: v })}
            step="0.0001"
            color="cyan"
          />
          <InputField 
            label="Kd (Derivative)" 
            value={localPID.Kd} 
            onChange={(v) => setLocalPID({ ...localPID, Kd: v })}
            step="0.001"
            color="cyan"
          />
          
          <button
            onClick={handlePIDUpdate}
            className="w-full mt-2 px-3 py-2 bg-cyan-500/10 hover:bg-cyan-500/20 text-cyan-400 border border-cyan-500/30 rounded-lg text-xs font-medium transition-all flex items-center justify-center gap-2"
          >
            <RefreshCw className="w-3.5 h-3.5" />
            Apply PID Gains
          </button>
        </Section>
        
        {/* Target Attitude */}
        <Section 
          title="Target Attitude" 
          icon={<Target className="w-3.5 h-3.5 text-emerald-400" />}
          color="bg-emerald-500/10"
        >
          <InputField 
            label="Roll" 
            value={localSetpoint.roll} 
            onChange={(v) => setLocalSetpoint({ ...localSetpoint, roll: v })}
            step="1"
            unit="deg"
            color="green"
          />
          <InputField 
            label="Pitch" 
            value={localSetpoint.pitch} 
            onChange={(v) => setLocalSetpoint({ ...localSetpoint, pitch: v })}
            step="1"
            unit="deg"
            color="green"
          />
          <InputField 
            label="Yaw" 
            value={localSetpoint.yaw} 
            onChange={(v) => setLocalSetpoint({ ...localSetpoint, yaw: v })}
            step="1"
            unit="deg"
            color="green"
          />
          
          <button
            onClick={handleSetpointUpdate}
            className="w-full mt-2 px-3 py-2 bg-emerald-500/10 hover:bg-emerald-500/20 text-emerald-400 border border-emerald-500/30 rounded-lg text-xs font-medium transition-all flex items-center justify-center gap-2"
          >
            <Target className="w-3.5 h-3.5" />
            Set Target
          </button>
        </Section>
        
        {/* Sensor Noise */}
        <Section 
          title="Sensor Noise" 
          icon={<Waves className="w-3.5 h-3.5 text-amber-400" />}
          color="bg-amber-500/10"
          defaultOpen={false}
        >
          <div className="space-y-2">
            <div className="flex items-center justify-between">
              <span className="text-[11px] text-gray-500 font-mono uppercase tracking-wider">Noise Level</span>
              <span className="text-xs font-mono text-amber-400">{(noise * 100).toFixed(1)}%</span>
            </div>
            <input
              type="range"
              min="0"
              max="0.1"
              step="0.001"
              value={noise}
              onChange={(e) => setNoise(parseFloat(e.target.value))}
              className="w-full h-1.5 bg-white/10 rounded-full appearance-none cursor-pointer [&::-webkit-slider-thumb]:appearance-none [&::-webkit-slider-thumb]:w-3 [&::-webkit-slider-thumb]:h-3 [&::-webkit-slider-thumb]:rounded-full [&::-webkit-slider-thumb]:bg-amber-400"
            />
            <button
              onClick={handleNoiseUpdate}
              className="w-full mt-2 px-3 py-2 bg-amber-500/10 hover:bg-amber-500/20 text-amber-400 border border-amber-500/30 rounded-lg text-xs font-medium transition-all"
            >
              Apply Noise
            </button>
          </div>
        </Section>
        
        {/* Initial Angular Velocity */}
        <Section 
          title="Initial Conditions" 
          icon={<Gauge className="w-3.5 h-3.5 text-white" />}
          color="bg-white/10"
          defaultOpen={false}
        >
          <InputField 
            label="ωx" 
            value={initialOmega.wx} 
            onChange={(v) => setInitialOmega({ ...initialOmega, wx: v })}
            step="0.01"
            unit="rad/s"
            color="white"
          />
          <InputField 
            label="ωy" 
            value={initialOmega.wy} 
            onChange={(v) => setInitialOmega({ ...initialOmega, wy: v })}
            step="0.01"
            unit="rad/s"
            color="white"
          />
          <InputField 
            label="ωz" 
            value={initialOmega.wz} 
            onChange={(v) => setInitialOmega({ ...initialOmega, wz: v })}
            step="0.01"
            unit="rad/s"
            color="white"
          />
          
          <button
            onClick={handleInitialConditionsUpdate}
            className="w-full mt-2 px-3 py-2 bg-white/5 hover:bg-white/10 text-white border border-white/20 rounded-lg text-xs font-medium transition-all"
          >
            Set Initial ω
          </button>
        </Section>
      </div>
      
      {/* Footer */}
      <div className="px-3 py-2 border-t border-white/5 bg-black/20">
        <p className="text-[9px] text-gray-600 text-center font-mono">
          Reset simulation after changing initial conditions
        </p>
      </div>
    </div>
  );
}
