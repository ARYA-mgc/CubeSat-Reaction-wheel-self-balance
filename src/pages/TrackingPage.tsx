/**
 * TrackingPage.tsx
 * Main LEO Orbital Tracking Visualization Page - Enhanced
 */

import { Suspense, useRef } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls, Environment, Float } from '@react-three/drei';
import { useNavigate } from 'react-router-dom';
import * as THREE from 'three';
import {
    ArrowLeft,
    Satellite,
    Play,
    Pause,
    RotateCcw,
    FastForward,
    Compass,
    Globe,
    Eye,
    RefreshCw,
} from 'lucide-react';

// Components
import Earth from '../components/tracking/Earth';
import OrbitalPath from '../components/tracking/OrbitalPath';
import OrbitingCubeSat from '../components/tracking/OrbitingCubeSat';
import RealSatellites from '../components/tracking/RealSatellites';
import Starfield from '../components/tracking/Starfield';
import AttitudeIndicator from '../components/tracking/AttitudeIndicator';
import OrbitalTelemetry from '../components/tracking/OrbitalTelemetry';
import SatelliteInfo from '../components/tracking/SatelliteInfo';
import SatelliteLegend from '../components/tracking/SatelliteLegend';
import SatelliteDetailPanel from '../components/tracking/SatelliteDetailPanel';
import SensorDashboard from '../components/SensorDashboard';

// Stores
import { useTrackingStore } from '../store/trackingStore';
import { useIMUStore } from '../store/imuStore';
import { cn } from '../utils/cn';

// Camera controller for smooth following - FIXED
function CameraController() {
    const { camera } = useThree();
    const cameraMode = useTrackingStore((state) => state.cameraMode);
    const cubeSatPosition = useTrackingStore((state) => state.cubeSatPosition);

    const targetPosition = useRef(new THREE.Vector3(3, 2, 3));

    useFrame(() => {
        if (cameraMode === 'follow') {
            // Follow the CubeSat using its actual position from store
            const { x, y, z } = cubeSatPosition;

            // Position camera behind and above the satellite
            const distance = 0.4;
            const angle = Math.atan2(z, x);

            targetPosition.current.set(
                x + Math.cos(angle) * distance,
                y + 0.2,
                z + Math.sin(angle) * distance
            );

            // Smoothly interpolate camera position
            camera.position.lerp(targetPosition.current, 0.05);

            // Look at the satellite
            camera.lookAt(x, y, z);
        } else if (cameraMode === 'earth') {
            targetPosition.current.set(0, 0.5, 3);
            camera.position.lerp(targetPosition.current, 0.03);
            camera.lookAt(0, 0, 0);
        }
        // 'free' mode - OrbitControls handles it
    });

    return null;
}

// Loader component
function Loader() {
    return (
        <Float speed={2} rotationIntensity={0.5} floatIntensity={0.5}>
            <mesh>
                <sphereGeometry args={[0.1, 16, 16]} />
                <meshStandardMaterial color="#00d4ff" wireframe />
            </mesh>
        </Float>
    );
}

// Orbit radius (Earth radius + altitude scaled)
const ORBIT_RADIUS = 1.15;

export default function TrackingPage() {
    const navigate = useNavigate();
    const controlsRef = useRef<any>(null);

    // Stores
    const {
        isOrbiting,
        timeWarp,
        toggleOrbiting,
        setTimeWarp,
        setCameraMode,
        cameraMode,
        reset,
        selectedSatellite,
        loadedSatellites,
    } = useTrackingStore();
    const { useIMUMode, isConnected: imuConnected, toggleMode } = useIMUStore();

    // Time warp options
    const timeWarpOptions = [1, 10, 100, 1000];

    return (
        <div className="min-h-screen bg-[#050510] text-white overflow-hidden relative">
            {/* Full-screen 3D Canvas */}
            <div className="absolute inset-0">
                <Canvas
                    camera={{ position: [3, 1.5, 3], fov: 45 }}
                    shadows
                    gl={{ antialias: true, alpha: false }}
                    dpr={[1, 2]}
                >
                    <Suspense fallback={<Loader />}>
                        {/* Deep space background */}
                        <color attach="background" args={['#050510']} />
                        <Starfield count={8000} radius={80} />

                        {/* Lighting */}
                        <ambientLight intensity={0.15} />
                        <directionalLight
                            position={[10, 5, 5]}
                            intensity={2.5}
                            castShadow
                            color="#ffffff"
                        />
                        <pointLight position={[-10, -5, -5]} intensity={0.3} color="#4da8ff" />
                        <pointLight position={[5, 5, -10]} intensity={0.2} color="#ffd93d" />

                        {/* Earth */}
                        <Earth />

                        {/* Orbital path for our CubeSat */}
                        <OrbitalPath radius={ORBIT_RADIUS} inclination={51.6} />

                        {/* Our CubeSat */}
                        <OrbitingCubeSat orbitRadius={ORBIT_RADIUS} inclination={51.6} />

                        {/* Real LEO satellites from CelesTrak */}
                        <RealSatellites />

                        {/* Camera controller */}
                        <CameraController />

                        {/* Controls - disabled when following */}
                        <OrbitControls
                            ref={controlsRef}
                            enablePan={cameraMode === 'free'}
                            enableZoom={true}
                            enableRotate={cameraMode === 'free'}
                            minDistance={1.5}
                            maxDistance={10}
                            autoRotate={!isOrbiting && cameraMode === 'free'}
                            autoRotateSpeed={0.3}
                        />

                        {/* Environment for reflections */}
                        <Environment preset="night" />
                    </Suspense>
                </Canvas>
            </div>

            {/* UI Overlays */}
            <div className="absolute inset-0 pointer-events-none">
                {/* Top bar */}
                <div className="absolute top-0 left-0 right-0 p-4 flex items-center justify-between pointer-events-auto">
                    {/* Back button & Title */}
                    <div className="flex items-center gap-4">
                        <button
                            onClick={() => navigate('/')}
                            className="flex items-center gap-2 px-4 py-2.5 bg-white/5 hover:bg-white/10 backdrop-blur-sm text-white border border-white/10 rounded-xl transition-all"
                        >
                            <ArrowLeft className="w-4 h-4" />
                            <span className="text-sm">Back</span>
                        </button>

                        <div className="flex items-center gap-3">
                            <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-cyan-500/20 to-emerald-500/20 border border-cyan-500/30 flex items-center justify-center">
                                <Globe className="w-5 h-5 text-cyan-400" />
                            </div>
                            <div>
                                <h1 className="text-lg font-bold gradient-text">LEO Tracking</h1>
                                <p className="text-[10px] text-gray-500 uppercase tracking-widest">Real Satellite Data</p>
                            </div>
                        </div>
                    </div>

                    {/* Controls */}
                    <div className="flex items-center gap-2">
                        {/* Play/Pause */}
                        <button
                            onClick={toggleOrbiting}
                            className={cn(
                                "flex items-center gap-2 px-4 py-2.5 rounded-xl transition-all font-medium text-sm border",
                                isOrbiting
                                    ? "bg-amber-500/10 hover:bg-amber-500/20 text-amber-400 border-amber-500/30"
                                    : "bg-emerald-500/10 hover:bg-emerald-500/20 text-emerald-400 border-emerald-500/30"
                            )}
                        >
                            {isOrbiting ? <Pause className="w-4 h-4" /> : <Play className="w-4 h-4" />}
                            <span>{isOrbiting ? 'Pause' : 'Play'}</span>
                        </button>

                        {/* Reset */}
                        <button
                            onClick={reset}
                            className="flex items-center gap-2 px-4 py-2.5 bg-white/5 hover:bg-white/10 text-gray-300 border border-white/10 rounded-xl transition-all"
                        >
                            <RotateCcw className="w-4 h-4" />
                        </button>

                        {/* Time warp */}
                        <div className="flex items-center gap-1 px-3 py-2 bg-black/50 backdrop-blur-sm rounded-xl border border-white/10">
                            <FastForward className="w-4 h-4 text-gray-400" />
                            {timeWarpOptions.map((warp) => (
                                <button
                                    key={warp}
                                    onClick={() => setTimeWarp(warp)}
                                    className={cn(
                                        "px-2 py-1 text-xs font-mono rounded transition-all",
                                        timeWarp === warp
                                            ? "bg-cyan-500/20 text-cyan-400"
                                            : "text-gray-500 hover:text-gray-300"
                                    )}
                                >
                                    {warp}x
                                </button>
                            ))}
                        </div>

                        <div className="w-px h-8 bg-white/10" />

                        {/* Camera modes */}
                        <div className="flex items-center gap-1 px-3 py-2 bg-black/50 backdrop-blur-sm rounded-xl border border-white/10">
                            <Eye className="w-4 h-4 text-gray-400 mr-1" />
                            {(['free', 'follow', 'earth'] as const).map((mode) => (
                                <button
                                    key={mode}
                                    onClick={() => setCameraMode(mode)}
                                    className={cn(
                                        "px-2 py-1 text-xs font-mono rounded capitalize transition-all",
                                        cameraMode === mode
                                            ? "bg-violet-500/20 text-violet-400"
                                            : "text-gray-500 hover:text-gray-300"
                                    )}
                                >
                                    {mode}
                                </button>
                            ))}
                        </div>

                        <div className="w-px h-8 bg-white/10" />

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
                            <span>IMU Mode</span>
                            {imuConnected && (
                                <div className="w-2 h-2 bg-violet-500 rounded-full animate-pulse" />
                            )}
                        </button>
                    </div>
                </div>

                {/* Left panel - Attitude Indicator & Legend */}
                <div className="absolute left-4 top-24 w-64 pointer-events-auto space-y-4 max-h-[calc(100vh-8rem)] overflow-y-auto scrollbar-thin">
                    <AttitudeIndicator />
                    {imuConnected && <SensorDashboard />}
                    <SatelliteLegend />
                </div>

                {/* Right panel - Telemetry & Selected Satellite */}
                <div className="absolute right-4 top-24 w-72 pointer-events-auto space-y-4 max-h-[calc(100vh-8rem)] overflow-y-auto scrollbar-thin">
                    <OrbitalTelemetry />
                    {selectedSatellite ? (
                        <SatelliteDetailPanel />
                    ) : (
                        <SatelliteInfo />
                    )}
                </div>

                {/* Bottom info bar */}
                <div className="absolute bottom-4 left-1/2 -translate-x-1/2 flex items-center gap-4 px-6 py-3 bg-black/60 backdrop-blur-sm rounded-xl border border-white/10 pointer-events-auto">
                    <div className="flex items-center gap-2">
                        <Satellite className="w-4 h-4 text-cyan-400" />
                        <span className="text-xs text-gray-400">Active Satellites:</span>
                        <span className="text-sm font-mono text-white">{loadedSatellites.length}</span>
                    </div>
                    <div className="w-px h-4 bg-white/20" />
                    <div className="flex items-center gap-2">
                        <div className="w-2 h-2 rounded-full bg-emerald-500 animate-pulse" />
                        <span className="text-xs text-gray-400">Your CubeSat:</span>
                        <span className="text-sm font-mono text-emerald-400">TRACKING</span>
                    </div>
                    <div className="w-px h-4 bg-white/20" />
                    <div className="text-xs text-gray-500">
                        Click satellites for details • Toggle categories in legend
                    </div>
                </div>

                {/* Corner decorations */}
                <div className="absolute top-0 left-0 w-24 h-24 border-l-2 border-t-2 border-cyan-500/20 rounded-tl-3xl" />
                <div className="absolute top-0 right-0 w-24 h-24 border-r-2 border-t-2 border-cyan-500/20 rounded-tr-3xl" />
                <div className="absolute bottom-0 left-0 w-24 h-24 border-l-2 border-b-2 border-cyan-500/20 rounded-bl-3xl" />
                <div className="absolute bottom-0 right-0 w-24 h-24 border-r-2 border-b-2 border-cyan-500/20 rounded-br-3xl" />
            </div>
        </div>
    );
}
