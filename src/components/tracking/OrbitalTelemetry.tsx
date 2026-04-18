/**
 * OrbitalTelemetry.tsx
 * Orbital parameters and telemetry display panel
 */

import { useMemo } from 'react';
import { Globe, Gauge, Clock, TrendingUp, MapPin, Orbit } from 'lucide-react';
import { useTrackingStore } from '../../store/trackingStore';
import { cn } from '../../utils/cn';

// Earth constants
const EARTH_RADIUS_KM = 6371;

export default function OrbitalTelemetry() {
    const orbitalParams = useTrackingStore((state) => state.orbitalParams);
    const orbitalAngle = useTrackingStore((state) => state.orbitalAngle);
    const missionElapsedTime = useTrackingStore((state) => state.missionElapsedTime);
    const isOrbiting = useTrackingStore((state) => state.isOrbiting);
    const timeWarp = useTrackingStore((state) => state.timeWarp);

    // Calculate derived values
    const calculations = useMemo(() => {
        const { altitude, inclination } = orbitalParams;

        // Orbital velocity (simplified circular orbit)
        const semiMajorAxis = EARTH_RADIUS_KM + altitude;
        const velocity = Math.sqrt(398600 / semiMajorAxis); // km/s (using Earth's GM)

        // Ground track position (simplified)
        const longitude = ((orbitalAngle * 180 / Math.PI) - (missionElapsedTime / 240) * 360) % 360;
        const latitude = Math.sin(orbitalAngle) * inclination;

        // Orbit completion percentage
        const orbitProgress = (orbitalAngle / (2 * Math.PI)) * 100;

        return {
            velocity,
            longitude: longitude.toFixed(2),
            latitude: latitude.toFixed(2),
            orbitProgress,
        };
    }, [orbitalParams, orbitalAngle, missionElapsedTime]);

    // Format mission elapsed time
    const formatMET = (seconds: number) => {
        const hrs = Math.floor(seconds / 3600);
        const mins = Math.floor((seconds % 3600) / 60);
        const secs = Math.floor(seconds % 60);
        return `${hrs.toString().padStart(2, '0')}:${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
    };

    return (
        <div className="glass-panel rounded-xl p-4 border border-emerald-500/20">
            {/* Header */}
            <div className="flex items-center justify-between mb-4">
                <div className="flex items-center gap-2">
                    <Orbit className="w-4 h-4 text-emerald-400" />
                    <span className="text-xs font-mono text-gray-400 uppercase tracking-wider">
                        Orbital Telemetry
                    </span>
                </div>
                <div className={cn(
                    "px-2 py-0.5 rounded-full text-[10px] font-mono",
                    isOrbiting
                        ? "bg-emerald-500/20 text-emerald-400"
                        : "bg-gray-500/20 text-gray-400"
                )}>
                    {isOrbiting ? 'TRACKING' : 'PAUSED'}
                </div>
            </div>

            {/* Mission Elapsed Time */}
            <div className="bg-black/40 rounded-lg p-3 mb-4">
                <div className="flex items-center gap-2 mb-1">
                    <Clock className="w-3 h-3 text-gray-500" />
                    <span className="text-[10px] text-gray-500 uppercase">Mission Elapsed Time</span>
                    {timeWarp > 1 && (
                        <span className="text-[10px] text-amber-400 ml-auto">{timeWarp}x</span>
                    )}
                </div>
                <div className="text-2xl font-mono text-white tabular-nums tracking-wider">
                    {formatMET(missionElapsedTime)}
                </div>
            </div>

            {/* Orbital Parameters Grid */}
            <div className="grid grid-cols-2 gap-2 mb-4">
                <TelemetryItem
                    icon={<TrendingUp className="w-3 h-3" />}
                    label="Altitude"
                    value={orbitalParams.altitude.toFixed(1)}
                    unit="km"
                    color="cyan"
                />
                <TelemetryItem
                    icon={<Gauge className="w-3 h-3" />}
                    label="Velocity"
                    value={calculations.velocity.toFixed(2)}
                    unit="km/s"
                    color="amber"
                />
                <TelemetryItem
                    icon={<Globe className="w-3 h-3" />}
                    label="Inclination"
                    value={orbitalParams.inclination.toFixed(1)}
                    unit="°"
                    color="violet"
                />
                <TelemetryItem
                    icon={<Clock className="w-3 h-3" />}
                    label="Period"
                    value={orbitalParams.period.toFixed(1)}
                    unit="min"
                    color="emerald"
                />
            </div>

            {/* Apogee / Perigee */}
            <div className="flex gap-2 mb-4">
                <div className="flex-1 bg-black/30 rounded-lg p-2 text-center">
                    <div className="text-[10px] text-gray-500 uppercase mb-1">Apogee</div>
                    <div className="text-sm font-mono text-red-400">
                        {orbitalParams.apogee} <span className="text-[10px] text-gray-500">km</span>
                    </div>
                </div>
                <div className="flex-1 bg-black/30 rounded-lg p-2 text-center">
                    <div className="text-[10px] text-gray-500 uppercase mb-1">Perigee</div>
                    <div className="text-sm font-mono text-cyan-400">
                        {orbitalParams.perigee} <span className="text-[10px] text-gray-500">km</span>
                    </div>
                </div>
            </div>

            {/* Ground Track */}
            <div className="bg-black/30 rounded-lg p-3">
                <div className="flex items-center gap-2 mb-2">
                    <MapPin className="w-3 h-3 text-gray-500" />
                    <span className="text-[10px] text-gray-500 uppercase">Ground Track</span>
                </div>
                <div className="flex justify-between">
                    <div>
                        <span className="text-[10px] text-gray-500">LAT</span>
                        <span className="text-sm font-mono text-white ml-2">{calculations.latitude}°</span>
                    </div>
                    <div>
                        <span className="text-[10px] text-gray-500">LON</span>
                        <span className="text-sm font-mono text-white ml-2">{calculations.longitude}°</span>
                    </div>
                </div>
            </div>

            {/* Orbit Progress */}
            <div className="mt-4">
                <div className="flex justify-between text-[10px] text-gray-500 mb-1">
                    <span>Orbit Progress</span>
                    <span>{calculations.orbitProgress.toFixed(1)}%</span>
                </div>
                <div className="h-1.5 bg-black/40 rounded-full overflow-hidden">
                    <div
                        className="h-full bg-gradient-to-r from-emerald-500 to-cyan-500 transition-all duration-300"
                        style={{ width: `${calculations.orbitProgress}%` }}
                    />
                </div>
            </div>
        </div>
    );
}

function TelemetryItem({
    icon,
    label,
    value,
    unit,
    color
}: {
    icon: React.ReactNode;
    label: string;
    value: string;
    unit: string;
    color: 'cyan' | 'amber' | 'violet' | 'emerald';
}) {
    const colorClasses = {
        cyan: 'text-cyan-400',
        amber: 'text-amber-400',
        violet: 'text-violet-400',
        emerald: 'text-emerald-400',
    };

    return (
        <div className="bg-black/30 rounded-lg p-2">
            <div className="flex items-center gap-1 mb-1">
                <span className="text-gray-500">{icon}</span>
                <span className="text-[10px] text-gray-500 uppercase">{label}</span>
            </div>
            <div className={cn("text-lg font-mono tabular-nums", colorClasses[color])}>
                {value}
                <span className="text-[10px] text-gray-500 ml-1">{unit}</span>
            </div>
        </div>
    );
}
