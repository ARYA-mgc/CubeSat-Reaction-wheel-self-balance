/**
 * SatelliteInfo.tsx
 * CubeSat information and status panel
 */

import { Satellite, Battery, Thermometer, Radio, Cpu, Zap, Activity } from 'lucide-react';
import { useIMUStore } from '../../store/imuStore';
import { cn } from '../../utils/cn';

export default function SatelliteInfo() {
    const isIMUConnected = useIMUStore((state) => state.isConnected);
    const useIMUMode = useIMUStore((state) => state.useIMUMode);
    const dataRate = useIMUStore((state) => state.dataRate);

    // Mock satellite data (would come from real telemetry in production)
    const satData = {
        name: 'CUBESAT-1',
        noradId: '99999',
        intlDesignator: '2024-001A',
        type: '1U CubeSat',
        mass: '1.33 kg',
        size: '10×10×10 cm',
        power: '2.8',
        battery: 87,
        temperature: 22,
        mode: useIMUMode ? 'IMU CONTROL' : 'ATTITUDE HOLD',
        status: isIMUConnected ? 'NOMINAL' : 'STANDBY',
    };

    return (
        <div className="glass-panel rounded-xl p-4 border border-violet-500/20">
            {/* Header with satellite icon */}
            <div className="flex items-center gap-3 mb-4">
                <div className="w-12 h-12 rounded-xl bg-gradient-to-br from-violet-500/20 to-cyan-500/20 border border-violet-500/30 flex items-center justify-center">
                    <Satellite className="w-6 h-6 text-violet-400" />
                </div>
                <div className="flex-1">
                    <h3 className="text-lg font-bold text-white">{satData.name}</h3>
                    <p className="text-[10px] text-gray-500 font-mono">NORAD: {satData.noradId}</p>
                </div>
                <div className={cn(
                    "px-2 py-1 rounded-lg text-[10px] font-mono uppercase",
                    satData.status === 'NOMINAL'
                        ? "bg-emerald-500/20 text-emerald-400 border border-emerald-500/30"
                        : "bg-amber-500/20 text-amber-400 border border-amber-500/30"
                )}>
                    {satData.status}
                </div>
            </div>

            {/* Quick stats */}
            <div className="grid grid-cols-3 gap-2 mb-4">
                <div className="bg-black/30 rounded-lg p-2 text-center">
                    <div className="text-[10px] text-gray-500 mb-0.5">Type</div>
                    <div className="text-xs font-mono text-white">{satData.type}</div>
                </div>
                <div className="bg-black/30 rounded-lg p-2 text-center">
                    <div className="text-[10px] text-gray-500 mb-0.5">Mass</div>
                    <div className="text-xs font-mono text-white">{satData.mass}</div>
                </div>
                <div className="bg-black/30 rounded-lg p-2 text-center">
                    <div className="text-[10px] text-gray-500 mb-0.5">Size</div>
                    <div className="text-xs font-mono text-white">{satData.size}</div>
                </div>
            </div>

            {/* Status indicators */}
            <div className="space-y-2 mb-4">
                {/* Battery */}
                <StatusBar
                    icon={<Battery className="w-3.5 h-3.5" />}
                    label="Battery"
                    value={satData.battery}
                    max={100}
                    unit="%"
                    color={satData.battery > 50 ? 'emerald' : satData.battery > 20 ? 'amber' : 'red'}
                />

                {/* Power */}
                <StatusBar
                    icon={<Zap className="w-3.5 h-3.5" />}
                    label="Power"
                    value={parseFloat(satData.power)}
                    max={5}
                    unit="W"
                    color="cyan"
                />

                {/* Temperature */}
                <StatusBar
                    icon={<Thermometer className="w-3.5 h-3.5" />}
                    label="Temp"
                    value={satData.temperature}
                    max={60}
                    unit="°C"
                    color={satData.temperature < 40 ? 'cyan' : 'amber'}
                />
            </div>

            {/* Subsystem status */}
            <div className="border-t border-white/10 pt-4">
                <div className="text-[10px] text-gray-500 uppercase mb-2">Subsystems</div>
                <div className="grid grid-cols-2 gap-2">
                    <SubsystemItem
                        icon={<Cpu className="w-3 h-3" />}
                        name="OBC"
                        status="ONLINE"
                        active
                    />
                    <SubsystemItem
                        icon={<Radio className="w-3 h-3" />}
                        name="Comms"
                        status="ACTIVE"
                        active
                    />
                    <SubsystemItem
                        icon={<Activity className="w-3 h-3" />}
                        name="ADCS"
                        status={useIMUMode ? 'IMU MODE' : 'AUTO'}
                        active={useIMUMode}
                        highlight={useIMUMode}
                    />
                    <SubsystemItem
                        icon={<Satellite className="w-3 h-3" />}
                        name="Payload"
                        status="READY"
                        active
                    />
                </div>
            </div>

            {/* IMU Data Rate (when connected) */}
            {isIMUConnected && (
                <div className="mt-4 p-2 bg-violet-500/10 rounded-lg border border-violet-500/20">
                    <div className="flex items-center justify-between">
                        <span className="text-[10px] text-violet-400 uppercase">IMU Data Rate</span>
                        <span className="text-sm font-mono text-violet-400">{dataRate} Hz</span>
                    </div>
                </div>
            )}
        </div>
    );
}

function StatusBar({
    icon,
    label,
    value,
    max,
    unit,
    color
}: {
    icon: React.ReactNode;
    label: string;
    value: number;
    max: number;
    unit: string;
    color: 'emerald' | 'amber' | 'red' | 'cyan';
}) {
    const percentage = (value / max) * 100;

    const colorClasses = {
        emerald: 'from-emerald-500 to-emerald-400',
        amber: 'from-amber-500 to-amber-400',
        red: 'from-red-500 to-red-400',
        cyan: 'from-cyan-500 to-cyan-400',
    };

    return (
        <div className="flex items-center gap-2">
            <span className="text-gray-500">{icon}</span>
            <span className="text-[10px] text-gray-500 w-12">{label}</span>
            <div className="flex-1 h-1.5 bg-black/40 rounded-full overflow-hidden">
                <div
                    className={cn("h-full bg-gradient-to-r transition-all duration-500", colorClasses[color])}
                    style={{ width: `${Math.min(100, percentage)}%` }}
                />
            </div>
            <span className="text-xs font-mono text-white w-12 text-right">
                {value}{unit}
            </span>
        </div>
    );
}

function SubsystemItem({
    icon,
    name,
    status,
    active,
    highlight = false,
}: {
    icon: React.ReactNode;
    name: string;
    status: string;
    active: boolean;
    highlight?: boolean;
}) {
    return (
        <div className={cn(
            "flex items-center gap-2 p-2 rounded-lg",
            highlight ? "bg-violet-500/10 border border-violet-500/20" : "bg-black/30"
        )}>
            <span className={active ? "text-emerald-400" : "text-gray-500"}>{icon}</span>
            <div className="flex-1">
                <div className="text-[10px] text-gray-400">{name}</div>
                <div className={cn(
                    "text-[10px] font-mono",
                    active ? (highlight ? "text-violet-400" : "text-emerald-400") : "text-gray-500"
                )}>
                    {status}
                </div>
            </div>
            <div className={cn(
                "w-1.5 h-1.5 rounded-full",
                active ? (highlight ? "bg-violet-500" : "bg-emerald-500") : "bg-gray-600"
            )} />
        </div>
    );
}
