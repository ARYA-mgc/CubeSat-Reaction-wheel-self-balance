/**
 * AttitudeIndicator.tsx
 * Aircraft-style artificial horizon / attitude indicator HUD
 */

import * as THREE from 'three';
import { useIMUStore } from '../../store/imuStore';
import { cn } from '../../utils/cn';

export default function AttitudeIndicator() {
    const imuQuaternion = useIMUStore((state) => state.imuQuaternion);
    const imuOffset = useIMUStore((state) => state.imuOffset);
    const isConnected = useIMUStore((state) => state.isConnected);
    const useIMUMode = useIMUStore((state) => state.useIMUMode);

    // Convert quaternion to Euler angles (degrees)
    const getEulerAngles = () => {
        if (!imuQuaternion) return { roll: 0, pitch: 0, yaw: 0 };

        let q = new THREE.Quaternion(
            imuQuaternion.i,
            imuQuaternion.j,
            imuQuaternion.k,
            imuQuaternion.real
        );

        if (imuOffset) {
            const offsetQ = new THREE.Quaternion(
                imuOffset.i,
                imuOffset.j,
                imuOffset.k,
                imuOffset.real
            );
            q.premultiply(offsetQ);
        }

        const euler = new THREE.Euler().setFromQuaternion(q, 'YXZ');

        return {
            roll: euler.x * (180 / Math.PI),
            pitch: euler.y * (180 / Math.PI),
            yaw: euler.z * (180 / Math.PI)
        };
    };

    const { roll, pitch, yaw } = getEulerAngles();
    const normalizedYaw = ((yaw % 360) + 360) % 360;

    return (
        <div className="glass-panel rounded-xl p-4 border border-cyan-500/20">
            <div className="flex items-center gap-2 mb-4">
                <div className={cn(
                    "w-2 h-2 rounded-full",
                    useIMUMode && isConnected ? "bg-emerald-500 animate-pulse" : "bg-gray-600"
                )} />
                <span className="text-xs font-mono text-gray-400 uppercase tracking-wider">
                    Attitude Indicator
                </span>
            </div>

            {/* Artificial Horizon */}
            <div className="relative w-48 h-48 mx-auto mb-4 rounded-full overflow-hidden border-2 border-gray-700">
                {/* Sky and ground */}
                <div
                    className="absolute inset-0 transition-transform duration-100"
                    style={{
                        transform: `rotate(${-roll}deg) translateY(${pitch * 2}px)`,
                    }}
                >
                    {/* Sky */}
                    <div className="absolute inset-x-0 top-0 h-1/2 bg-gradient-to-b from-cyan-600 to-cyan-400" />
                    {/* Ground */}
                    <div className="absolute inset-x-0 bottom-0 h-1/2 bg-gradient-to-b from-amber-700 to-amber-900" />
                    {/* Horizon line */}
                    <div className="absolute left-0 right-0 top-1/2 h-0.5 bg-white transform -translate-y-1/2" />

                    {/* Pitch ladder */}
                    {[-30, -20, -10, 10, 20, 30].map((angle) => (
                        <div
                            key={angle}
                            className="absolute left-1/4 right-1/4 h-px bg-white/50"
                            style={{ top: `${50 - angle * 2}%` }}
                        />
                    ))}
                </div>

                {/* Fixed aircraft reference */}
                <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
                    {/* Wings */}
                    <div className="absolute w-16 h-1 bg-yellow-500 left-4 top-1/2 transform -translate-y-1/2" />
                    <div className="absolute w-16 h-1 bg-yellow-500 right-4 top-1/2 transform -translate-y-1/2" />
                    {/* Center dot */}
                    <div className="w-3 h-3 bg-yellow-500 rounded-full border-2 border-yellow-600" />
                </div>

                {/* Outer ring markings */}
                {[...Array(36)].map((_, i) => (
                    <div
                        key={i}
                        className="absolute w-0.5 bg-white/30"
                        style={{
                            height: i % 3 === 0 ? '8px' : '4px',
                            left: '50%',
                            top: 0,
                            transform: `rotate(${i * 10}deg) translateX(-50%)`,
                            transformOrigin: '50% 96px',
                        }}
                    />
                ))}
            </div>

            {/* Compass / Heading */}
            <div className="relative h-8 mb-4 overflow-hidden rounded bg-black/40">
                <div
                    className="absolute h-full flex items-center transition-transform duration-100"
                    style={{
                        transform: `translateX(${-normalizedYaw * 2 + 96}px)`,
                    }}
                >
                    {[...Array(72)].map((_, i) => {
                        const heading = (i * 10) % 360;
                        const label = heading === 0 ? 'N' : heading === 90 ? 'E' : heading === 180 ? 'S' : heading === 270 ? 'W' : heading.toString();
                        const isCardinal = ['N', 'E', 'S', 'W'].includes(label);

                        return (
                            <div key={i} className="w-5 flex-shrink-0 flex flex-col items-center">
                                <div className={cn(
                                    "h-2 w-px",
                                    isCardinal ? "bg-cyan-400" : "bg-gray-500"
                                )} />
                                <span className={cn(
                                    "text-[8px] font-mono",
                                    isCardinal ? "text-cyan-400" : "text-gray-500"
                                )}>
                                    {i % 2 === 0 ? label : ''}
                                </span>
                            </div>
                        );
                    })}
                </div>

                {/* Center indicator */}
                <div className="absolute left-1/2 top-0 w-0.5 h-full bg-yellow-500 transform -translate-x-1/2" />
                <div className="absolute left-1/2 -top-1 transform -translate-x-1/2">
                    <div className="w-0 h-0 border-l-4 border-r-4 border-t-4 border-l-transparent border-r-transparent border-t-yellow-500" />
                </div>
            </div>

            {/* Numerical readouts */}
            <div className="grid grid-cols-3 gap-2 text-center">
                <div className="bg-black/30 rounded-lg p-2">
                    <div className="text-[10px] text-gray-500 uppercase">Roll</div>
                    <div className={cn(
                        "text-lg font-mono tabular-nums",
                        Math.abs(roll) > 30 ? "text-amber-400" : "text-cyan-400"
                    )}>
                        {roll.toFixed(1)}°
                    </div>
                </div>
                <div className="bg-black/30 rounded-lg p-2">
                    <div className="text-[10px] text-gray-500 uppercase">Pitch</div>
                    <div className={cn(
                        "text-lg font-mono tabular-nums",
                        Math.abs(pitch) > 30 ? "text-amber-400" : "text-emerald-400"
                    )}>
                        {pitch.toFixed(1)}°
                    </div>
                </div>
                <div className="bg-black/30 rounded-lg p-2">
                    <div className="text-[10px] text-gray-500 uppercase">Yaw</div>
                    <div className="text-lg font-mono tabular-nums text-violet-400">
                        {normalizedYaw.toFixed(1)}°
                    </div>
                </div>
            </div>
        </div>
    );
}
