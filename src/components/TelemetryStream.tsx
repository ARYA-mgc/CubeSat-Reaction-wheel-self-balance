/**
 * TelemetryStream.tsx
 * Real-time scrolling telemetry charts (oscilloscope style) - Enhanced UI
 */

import { useEffect, useState } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, ReferenceLine } from 'recharts';
import { Activity, Gauge, RotateCcw } from 'lucide-react';
import { useSimulationStore } from '../store/simulationStore';

interface DataPoint {
  time: number;
  roll: number;
  pitch: number;
  yaw: number;
  torqueX: number;
  torqueY: number;
  torqueZ: number;
  angVelMag: number;
}

interface ChartLegendItemProps {
  color: string;
  label: string;
  value?: number;
  unit?: string;
}

function ChartLegendItem({ color, label, value, unit = '°' }: ChartLegendItemProps) {
  return (
    <div className="flex items-center gap-2">
      <div className="w-2 h-2 rounded-full" style={{ backgroundColor: color }} />
      <span className="text-[10px] font-mono text-gray-400">{label}</span>
      {value !== undefined && (
        <span className="text-[11px] font-mono tabular-nums" style={{ color }}>
          {value.toFixed(1)}{unit}
        </span>
      )}
    </div>
  );
}

const CustomTooltip = ({ active, payload, label }: any) => {
  if (active && payload && payload.length) {
    return (
      <div className="bg-black/90 backdrop-blur-sm border border-white/10 rounded-lg px-3 py-2 shadow-xl">
        <p className="text-[10px] font-mono text-cyan-400 mb-1">
          T+ {label?.toFixed(2)}s
        </p>
        {payload.map((entry: any, index: number) => (
          <div key={index} className="flex items-center gap-2">
            <div 
              className="w-1.5 h-1.5 rounded-full" 
              style={{ backgroundColor: entry.color }}
            />
            <span className="text-[10px] font-mono text-gray-400">{entry.name}:</span>
            <span 
              className="text-[11px] font-mono tabular-nums"
              style={{ color: entry.color }}
            >
              {entry.value?.toFixed(2)}
            </span>
          </div>
        ))}
      </div>
    );
  }
  return null;
};

export default function TelemetryStream() {
  const [dataBuffer, setDataBuffer] = useState<DataPoint[]>([]);
  const currentData = useSimulationStore((state) => state.currentData);
  const maxPoints = 300; // 5 seconds at 60fps
  
  useEffect(() => {
    if (currentData && !currentData.packetDropped) {
      const euler = currentData.eulerAngles;
      const torque = currentData.controlOutput.torque;
      const angVel = currentData.physicsState.angularVelocity;
      const angVelMag = Math.sqrt(angVel.x ** 2 + angVel.y ** 2 + angVel.z ** 2);
      
      const newPoint: DataPoint = {
        time: currentData.physicsState.time,
        roll: euler[0] * 180 / Math.PI,
        pitch: euler[1] * 180 / Math.PI,
        yaw: euler[2] * 180 / Math.PI,
        torqueX: torque.x * 1000, // Convert to mN·m
        torqueY: torque.y * 1000,
        torqueZ: torque.z * 1000,
        angVelMag: angVelMag,
      };
      
      setDataBuffer((prev) => {
        const updated = [...prev, newPoint];
        return updated.slice(-maxPoints);
      });
    }
  }, [currentData]);

  const latestData = dataBuffer[dataBuffer.length - 1];
  
  return (
    <div className="w-full h-full glass-panel rounded-xl overflow-hidden flex flex-col">
      {/* Attitude Chart */}
      <div className="flex-1 min-h-0 border-b border-white/5 flex flex-col">
        {/* Header */}
        <div className="px-4 py-2 flex items-center justify-between border-b border-white/5 bg-black/20">
          <div className="flex items-center gap-2">
            <div className="p-1.5 rounded-lg bg-cyan-500/10">
              <RotateCcw className="w-3.5 h-3.5 text-cyan-400" />
            </div>
            <span className="text-xs font-medium text-white">Attitude</span>
            <span className="text-[10px] text-gray-500 font-mono">Euler Angles</span>
          </div>
          <div className="flex items-center gap-4">
            <ChartLegendItem color="#f87171" label="Roll" value={latestData?.roll} />
            <ChartLegendItem color="#4ade80" label="Pitch" value={latestData?.pitch} />
            <ChartLegendItem color="#22d3ee" label="Yaw" value={latestData?.yaw} />
          </div>
        </div>
        
        {/* Chart */}
        <div className="flex-1 p-2">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={dataBuffer} margin={{ top: 10, right: 10, left: -20, bottom: 5 }}>
              <defs>
                <linearGradient id="rollGradient" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="5%" stopColor="#f87171" stopOpacity={0.3}/>
                  <stop offset="95%" stopColor="#f87171" stopOpacity={0}/>
                </linearGradient>
              </defs>
              <CartesianGrid strokeDasharray="2 4" stroke="rgba(255,255,255,0.05)" vertical={false} />
              <XAxis
                dataKey="time"
                stroke="rgba(255,255,255,0.1)"
                tick={{ fill: '#4a4a4a', fontSize: 9 }}
                tickFormatter={(value) => value.toFixed(0)}
                axisLine={false}
              />
              <YAxis
                stroke="rgba(255,255,255,0.1)"
                tick={{ fill: '#4a4a4a', fontSize: 9 }}
                axisLine={false}
                tickLine={false}
              />
              <ReferenceLine y={0} stroke="rgba(255,255,255,0.1)" strokeDasharray="3 3" />
              <Tooltip content={<CustomTooltip />} />
              <Line
                type="monotone"
                dataKey="roll"
                stroke="#f87171"
                strokeWidth={1.5}
                dot={false}
                isAnimationActive={false}
                name="Roll"
              />
              <Line
                type="monotone"
                dataKey="pitch"
                stroke="#4ade80"
                strokeWidth={1.5}
                dot={false}
                isAnimationActive={false}
                name="Pitch"
              />
              <Line
                type="monotone"
                dataKey="yaw"
                stroke="#22d3ee"
                strokeWidth={1.5}
                dot={false}
                isAnimationActive={false}
                name="Yaw"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>
      
      {/* Torque Chart */}
      <div className="flex-1 min-h-0 flex flex-col">
        {/* Header */}
        <div className="px-4 py-2 flex items-center justify-between border-b border-white/5 bg-black/20">
          <div className="flex items-center gap-2">
            <div className="p-1.5 rounded-lg bg-emerald-500/10">
              <Gauge className="w-3.5 h-3.5 text-emerald-400" />
            </div>
            <span className="text-xs font-medium text-white">Control Torque</span>
            <span className="text-[10px] text-gray-500 font-mono">Reaction Wheels</span>
          </div>
          <div className="flex items-center gap-4">
            <ChartLegendItem color="#f87171" label="τx" value={latestData?.torqueX} unit=" mN·m" />
            <ChartLegendItem color="#4ade80" label="τy" value={latestData?.torqueY} unit=" mN·m" />
            <ChartLegendItem color="#22d3ee" label="τz" value={latestData?.torqueZ} unit=" mN·m" />
          </div>
        </div>
        
        {/* Chart */}
        <div className="flex-1 p-2">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={dataBuffer} margin={{ top: 10, right: 10, left: -20, bottom: 5 }}>
              <CartesianGrid strokeDasharray="2 4" stroke="rgba(255,255,255,0.05)" vertical={false} />
              <XAxis
                dataKey="time"
                stroke="rgba(255,255,255,0.1)"
                tick={{ fill: '#4a4a4a', fontSize: 9 }}
                tickFormatter={(value) => value.toFixed(0)}
                axisLine={false}
              />
              <YAxis
                stroke="rgba(255,255,255,0.1)"
                tick={{ fill: '#4a4a4a', fontSize: 9 }}
                axisLine={false}
                tickLine={false}
              />
              <ReferenceLine y={0} stroke="rgba(255,255,255,0.1)" strokeDasharray="3 3" />
              <Tooltip content={<CustomTooltip />} />
              <Line
                type="monotone"
                dataKey="torqueX"
                stroke="#f87171"
                strokeWidth={1.5}
                dot={false}
                isAnimationActive={false}
                name="τx"
              />
              <Line
                type="monotone"
                dataKey="torqueY"
                stroke="#4ade80"
                strokeWidth={1.5}
                dot={false}
                isAnimationActive={false}
                name="τy"
              />
              <Line
                type="monotone"
                dataKey="torqueZ"
                stroke="#22d3ee"
                strokeWidth={1.5}
                dot={false}
                isAnimationActive={false}
                name="τz"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>
      
      {/* Footer Status */}
      <div className="px-4 py-1.5 border-t border-white/5 bg-black/20 flex items-center justify-between">
        <div className="flex items-center gap-2">
          <Activity className="w-3 h-3 text-cyan-400" />
          <span className="text-[9px] font-mono text-gray-500">
            {dataBuffer.length} samples • {(dataBuffer.length / 60).toFixed(1)}s buffer
          </span>
        </div>
        <div className="text-[9px] font-mono text-gray-600">
          T+ {latestData?.time.toFixed(2) || '0.00'}s
        </div>
      </div>
    </div>
  );
}
