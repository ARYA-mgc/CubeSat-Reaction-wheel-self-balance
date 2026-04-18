import { useIMUStore } from '../store/imuStore';
import { Thermometer, Droplets, Fan, Activity } from 'lucide-react';
import { cn } from '../utils/cn';
import { useThemeStore } from '../store/themeStore';

export default function SensorDashboard() {
    const { environment, motorValues, dataRate, isConnected } = useIMUStore();
    const theme = useThemeStore((state) => state.theme);
    const isDark = theme === 'dark';

    const temp = environment?.temp ?? 0;
    const hum = environment?.hum ?? 0;
    const [m1, m2, m3] = motorValues.length === 3 ? motorValues : [0, 0, 0];

    return (
        <div className={cn(
            "p-4 rounded-xl border backdrop-blur-md",
            isDark ? "bg-black/40 border-white/10" : "bg-white/60 border-black/10"
        )}>
            <div className="flex items-center justify-between mb-4">
                <h3 className={cn("text-sm font-bold uppercase tracking-wider", isDark ? "text-cyan-400" : "text-cyan-700")}>
                    Start Tracker Telemetry
                </h3>
                <div className="flex items-center gap-2">
                    <Activity className={cn("w-4 h-4", isConnected ? "text-green-500 animate-pulse" : "text-red-500")} />
                    <span className="text-xs font-mono">{dataRate} Hz</span>
                </div>
            </div>

            <div className="grid grid-cols-2 gap-4">
                {/* Environment */}
                <div className={cn("p-3 rounded-lg border", isDark ? "bg-white/5 border-white/5" : "bg-white/50 border-black/5")}>
                    <div className="flex items-center gap-2 mb-2">
                        <Thermometer className="w-4 h-4 text-orange-500" />
                        <span className="text-xs opacity-70">Temperature</span>
                    </div>
                    <div className="text-2xl font-mono font-bold">
                        {temp.toFixed(1)}°C
                    </div>
                </div>

                <div className={cn("p-3 rounded-lg border", isDark ? "bg-white/5 border-white/5" : "bg-white/50 border-black/5")}>
                    <div className="flex items-center gap-2 mb-2">
                        <Droplets className="w-4 h-4 text-blue-500" />
                        <span className="text-xs opacity-70">Humidity</span>
                    </div>
                    <div className="text-2xl font-mono font-bold">
                        {hum.toFixed(1)}%
                    </div>
                </div>

                <div className={cn("p-3 rounded-lg border col-span-2", isDark ? "bg-white/5 border-white/5" : "bg-white/50 border-black/5")}>
                    <div className="flex items-center gap-2 mb-2">
                        {/* Replace with Mountain icon if available, or just text */}
                        <span className="text-xs opacity-70">Altitude (Relative)</span>
                    </div>
                    <div className="text-2xl font-mono font-bold">
                        {((environment?.alt ?? 0)).toFixed(2)} m
                    </div>
                </div>
            </div>

            {/* Motors */}
            <div className="mt-4">
                <div className="flex items-center gap-2 mb-3">
                    <Fan className="w-4 h-4 text-purple-500" />
                    <span className="text-xs opacity-70">Reaction Wheels (PWM)</span>
                </div>

                <div className="space-y-3">
                    <MotorBar label="M1 (X)" value={m1} color="bg-red-500" isDark={isDark} />
                    <MotorBar label="M2 (Y)" value={m2} color="bg-green-500" isDark={isDark} />
                    <MotorBar label="M3 (Z)" value={m3} color="bg-blue-500" isDark={isDark} />
                </div>
            </div>
        </div>
    );
}

function MotorBar({ label, value, color, isDark }: { label: string, value: number, color: string, isDark: boolean }) {
    return (
        <div className="flex items-center gap-3">
            <span className="text-xs font-mono w-10 opacity-70">{label}</span>
            <div className={cn("flex-1 h-2 rounded-full overflow-hidden", isDark ? "bg-white/10" : "bg-black/10")}>
                <div
                    className={cn("h-full transition-all duration-100", color)}
                    style={{ width: `${(value / 255) * 100}%` }}
                />
            </div>
            <span className="text-xs font-mono w-8 text-right tabular-nums opacity-70">{value}</span>
        </div>
    );
}
