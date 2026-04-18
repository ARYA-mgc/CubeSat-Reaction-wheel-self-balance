/**
 * SatelliteLegend.tsx
 * Interactive legend with category toggles and real-time counts
 */

import { Satellite, Globe2, Radio, Cloud, Microscope, CircleDot, Eye, EyeOff } from 'lucide-react';
import { useTrackingStore } from '../../store/trackingStore';
import { type SatelliteData } from '../../services/satelliteService';
import { cn } from '../../utils/cn';

// Category display config
const CATEGORIES: {
    key: SatelliteData['category'];
    name: string;
    color: string;
    icon: React.ComponentType<{ className?: string }>;
}[] = [
        { key: 'starlink', name: 'Starlink', color: '#ffffff', icon: Satellite },
        { key: 'oneweb', name: 'OneWeb', color: '#ffd93d', icon: Globe2 },
        { key: 'iridium', name: 'Iridium', color: '#6bcb77', icon: Radio },
        { key: 'iss', name: 'ISS', color: '#ff6b6b', icon: CircleDot },
        { key: 'weather', name: 'Weather', color: '#4da8ff', icon: Cloud },
        { key: 'science', name: 'Science', color: '#c084fc', icon: Microscope },
    ];

export default function SatelliteLegend() {
    const loadedSatellites = useTrackingStore((state) => state.loadedSatellites);
    const visibleCategories = useTrackingStore((state) => state.visibleCategories);
    const toggleCategory = useTrackingStore((state) => state.toggleCategory);

    // Count by category
    const counts = CATEGORIES.reduce((acc, cat) => {
        acc[cat.key] = loadedSatellites.filter(s => s.category === cat.key).length;
        return acc;
    }, {} as Record<string, number>);

    const totalVisible = loadedSatellites.filter(s => visibleCategories.has(s.category)).length;
    const totalCount = loadedSatellites.length;

    return (
        <div className="glass-panel rounded-xl p-4 border border-white/10">
            {/* Header */}
            <div className="flex items-center justify-between mb-3">
                <div className="flex items-center gap-2">
                    <Satellite className="w-4 h-4 text-cyan-400" />
                    <span className="text-xs font-mono text-gray-400 uppercase tracking-wider">
                        LEO Satellites
                    </span>
                </div>
                <div className={cn(
                    "px-2 py-0.5 rounded-full text-xs font-mono",
                    totalCount > 0 ? "bg-emerald-500/20 text-emerald-400" : "bg-amber-500/20 text-amber-400"
                )}>
                    {totalVisible}/{totalCount}
                </div>
            </div>

            {/* Category toggles */}
            <div className="space-y-1">
                {CATEGORIES.map(({ key, name, color, icon: Icon }) => {
                    const count = counts[key] || 0;
                    const isVisible = visibleCategories.has(key);

                    return (
                        <button
                            key={key}
                            onClick={() => toggleCategory(key)}
                            disabled={count === 0}
                            className={cn(
                                "w-full flex items-center gap-2 px-2 py-1.5 rounded-lg transition-all",
                                count === 0
                                    ? "opacity-30 cursor-not-allowed"
                                    : isVisible
                                        ? "bg-white/10 hover:bg-white/15"
                                        : "bg-white/5 hover:bg-white/10 opacity-60"
                            )}
                        >
                            <div
                                className={cn(
                                    "w-2.5 h-2.5 rounded-full transition-all",
                                    !isVisible && "opacity-30"
                                )}
                                style={{ backgroundColor: color }}
                            />
                            <Icon className="w-3 h-3 text-gray-500" />
                            <span className={cn(
                                "text-xs flex-1 text-left",
                                isVisible ? "text-gray-300" : "text-gray-500"
                            )}>
                                {name}
                            </span>
                            <span className={cn(
                                "text-xs font-mono tabular-nums",
                                isVisible ? "text-white" : "text-gray-600"
                            )}>
                                {count}
                            </span>
                            {count > 0 && (
                                isVisible
                                    ? <Eye className="w-3 h-3 text-gray-500" />
                                    : <EyeOff className="w-3 h-3 text-gray-600" />
                            )}
                        </button>
                    );
                })}
            </div>

            {/* Quick actions */}
            <div className="mt-3 pt-3 border-t border-white/5 flex gap-2">
                <button
                    onClick={() => CATEGORIES.forEach(c => {
                        if (!visibleCategories.has(c.key) && counts[c.key] > 0) toggleCategory(c.key);
                    })}
                    className="flex-1 text-[10px] text-cyan-400 hover:text-cyan-300 transition-colors"
                >
                    Show All
                </button>
                <span className="text-gray-600">|</span>
                <button
                    onClick={() => CATEGORIES.forEach(c => {
                        if (visibleCategories.has(c.key)) toggleCategory(c.key);
                    })}
                    className="flex-1 text-[10px] text-gray-500 hover:text-gray-400 transition-colors"
                >
                    Hide All
                </button>
            </div>

            {/* Data source */}
            <div className="mt-3 pt-3 border-t border-white/5">
                <div className="flex items-center justify-between text-[10px] text-gray-500">
                    <span>Source: CelesTrak</span>
                    <span className={cn(
                        "px-1.5 py-0.5 rounded",
                        totalCount > 0 ? "bg-emerald-500/20 text-emerald-400" : "bg-amber-500/20 text-amber-400"
                    )}>
                        {totalCount > 0 ? 'LIVE' : 'LOADING'}
                    </span>
                </div>
            </div>
        </div>
    );
}
