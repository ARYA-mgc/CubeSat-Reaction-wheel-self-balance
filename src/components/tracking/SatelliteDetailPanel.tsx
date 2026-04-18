/**
 * SatelliteDetailPanel.tsx
 * Detailed info panel for selected satellite with real orbital data
 */

import { X, Satellite, Globe2, Clock, TrendingUp, Navigation, Radio } from 'lucide-react';
import { useTrackingStore } from '../../store/trackingStore';
import { calculateSatellitePosition, type SatelliteData } from '../../services/satelliteService';
import { cn } from '../../utils/cn';

// Category display info
const CATEGORY_INFO: Record<SatelliteData['category'], { name: string; color: string; description: string }> = {
    starlink: { name: 'Starlink', color: '#ffffff', description: 'SpaceX internet constellation' },
    oneweb: { name: 'OneWeb', color: '#ffd93d', description: 'OneWeb internet constellation' },
    iridium: { name: 'Iridium NEXT', color: '#6bcb77', description: 'Satellite phone network' },
    iss: { name: 'Space Station', color: '#ff6b6b', description: 'International Space Station' },
    weather: { name: 'Weather', color: '#4da8ff', description: 'Meteorological satellite' },
    science: { name: 'Science', color: '#c084fc', description: 'Scientific research satellite' },
    other: { name: 'Other', color: '#94a3b8', description: 'Miscellaneous LEO satellite' },
};

export default function SatelliteDetailPanel() {
    const selectedSatellite = useTrackingStore((state) => state.selectedSatellite);
    const missionElapsedTime = useTrackingStore((state) => state.missionElapsedTime);
    const selectSatellite = useTrackingStore((state) => state.selectSatellite);

    if (!selectedSatellite) return null;

    const catInfo = CATEGORY_INFO[selectedSatellite.category];
    const position = calculateSatellitePosition(selectedSatellite, missionElapsedTime);

    return (
        <div className="glass-panel rounded-xl border border-white/20 overflow-hidden animate-in slide-in-from-right-5 duration-200">
            {/* Header */}
            <div
                className="px-4 py-3 flex items-center justify-between"
                style={{ borderBottom: `2px solid ${catInfo.color}40` }}
            >
                <div className="flex items-center gap-3">
                    <div
                        className="w-10 h-10 rounded-lg flex items-center justify-center"
                        style={{ backgroundColor: `${catInfo.color}20` }}
                    >
                        <Satellite className="w-5 h-5" style={{ color: catInfo.color }} />
                    </div>
                    <div>
                        <h3 className="text-sm font-bold text-white">{selectedSatellite.name}</h3>
                        <p className="text-[10px] text-gray-500">{catInfo.description}</p>
                    </div>
                </div>
                <button
                    onClick={() => selectSatellite(null)}
                    className="p-1.5 hover:bg-white/10 rounded-lg transition-colors"
                >
                    <X className="w-4 h-4 text-gray-400" />
                </button>
            </div>

            {/* Content */}
            <div className="p-4 space-y-4">
                {/* Identity */}
                <div className="grid grid-cols-2 gap-2">
                    <InfoItem label="NORAD ID" value={selectedSatellite.noradId.toString()} />
                    <InfoItem label="Intl. Designator" value={selectedSatellite.intlDesignator} />
                    <InfoItem label="Category" value={catInfo.name} color={catInfo.color} />
                    <InfoItem label="Epoch Year" value={selectedSatellite.epochYear.toString()} />
                </div>

                {/* Orbital elements */}
                <div className="border-t border-white/10 pt-4">
                    <div className="flex items-center gap-2 mb-3">
                        <Globe2 className="w-3.5 h-3.5 text-cyan-400" />
                        <span className="text-[10px] text-gray-400 uppercase tracking-wider">Orbital Elements</span>
                    </div>
                    <div className="grid grid-cols-2 gap-2">
                        <InfoItem
                            label="Altitude"
                            value={`${selectedSatellite.altitude.toFixed(0)} km`}
                            icon={<TrendingUp className="w-3 h-3" />}
                        />
                        <InfoItem
                            label="Inclination"
                            value={`${selectedSatellite.inclination.toFixed(1)}°`}
                            icon={<Navigation className="w-3 h-3" />}
                        />
                        <InfoItem
                            label="Period"
                            value={`${selectedSatellite.period.toFixed(1)} min`}
                            icon={<Clock className="w-3 h-3" />}
                        />
                        <InfoItem
                            label="Mean Motion"
                            value={`${selectedSatellite.meanMotion.toFixed(2)} rev/day`}
                            icon={<Radio className="w-3 h-3" />}
                        />
                    </div>
                </div>

                {/* Current position */}
                <div className="border-t border-white/10 pt-4">
                    <div className="flex items-center gap-2 mb-3">
                        <Navigation className="w-3.5 h-3.5 text-emerald-400" />
                        <span className="text-[10px] text-gray-400 uppercase tracking-wider">Current Position</span>
                    </div>
                    <div className="grid grid-cols-2 gap-2">
                        <InfoItem label="Latitude" value={`${position.lat.toFixed(2)}°`} />
                        <InfoItem label="Longitude" value={`${position.lon.toFixed(2)}°`} />
                    </div>
                </div>

                {/* TLE Data */}
                <div className="border-t border-white/10 pt-4">
                    <div className="flex items-center gap-2 mb-2">
                        <span className="text-[10px] text-gray-400 uppercase tracking-wider">TLE Parameters</span>
                    </div>
                    <div className="bg-black/30 rounded-lg p-2 font-mono text-[10px] text-gray-400 space-y-1">
                        <div>RAAN: {selectedSatellite.raan.toFixed(4)}°</div>
                        <div>Arg. of Perigee: {selectedSatellite.argOfPerigee.toFixed(4)}°</div>
                        <div>Mean Anomaly: {selectedSatellite.meanAnomaly.toFixed(4)}°</div>
                        <div>Eccentricity: {selectedSatellite.eccentricity.toFixed(7)}</div>
                    </div>
                </div>
            </div>

            {/* Footer */}
            <div className="px-4 py-2 bg-black/30 border-t border-white/5">
                <div className="flex items-center justify-between text-[10px] text-gray-500">
                    <span>Source: CelesTrak TLE</span>
                    <span className="text-cyan-500/70">LIVE</span>
                </div>
            </div>
        </div>
    );
}

function InfoItem({
    label,
    value,
    icon,
    color
}: {
    label: string;
    value: string;
    icon?: React.ReactNode;
    color?: string;
}) {
    return (
        <div className="bg-black/30 rounded-lg p-2">
            <div className="flex items-center gap-1 mb-0.5">
                {icon && <span className="text-gray-500">{icon}</span>}
                <span className="text-[10px] text-gray-500 uppercase">{label}</span>
            </div>
            <div
                className="text-sm font-mono tabular-nums"
                style={{ color: color || '#ffffff' }}
            >
                {value}
            </div>
        </div>
    );
}
