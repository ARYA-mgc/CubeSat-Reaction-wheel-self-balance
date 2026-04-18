/**
 * RealSatellites.tsx
 * Enhanced satellite visualization with click selection and real CelesTrak data
 */

import { useRef, useMemo, useState, useEffect } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { Html } from '@react-three/drei';
import * as THREE from 'three';
import {
    fetchAllLEOSatellites,
    getSampleSatellites,
    calculateSatellitePosition,
    type SatelliteData
} from '../../services/satelliteService';
import { useTrackingStore } from '../../store/trackingStore';

// Color map for satellite categories
const CATEGORY_COLORS: Record<SatelliteData['category'], string> = {
    starlink: '#ffffff',
    oneweb: '#ffd93d',
    iridium: '#6bcb77',
    iss: '#ff6b6b',
    weather: '#4da8ff',
    science: '#c084fc',
    other: '#94a3b8',
};

export default function RealSatellites() {
    const { raycaster, camera, pointer } = useThree();
    const groupRef = useRef<THREE.Group>(null);
    const meshRefs = useRef<Map<string, THREE.Mesh>>(new Map());

    const [satellites, setSatellites] = useState<SatelliteData[]>([]);
    const [loading, setLoading] = useState(true);

    const missionElapsedTime = useTrackingStore((state) => state.missionElapsedTime);
    const visibleCategories = useTrackingStore((state) => state.visibleCategories);
    const selectedSatellite = useTrackingStore((state) => state.selectedSatellite);
    const hoveredSatellite = useTrackingStore((state) => state.hoveredSatellite);
    const selectSatellite = useTrackingStore((state) => state.selectSatellite);
    const setHoveredSatellite = useTrackingStore((state) => state.setHoveredSatellite);
    const setLoadedSatellites = useTrackingStore((state) => state.setLoadedSatellites);

    // Fetch real satellite data on mount
    useEffect(() => {
        let mounted = true;

        async function loadSatellites() {
            setLoading(true);

            try {
                const data = await fetchAllLEOSatellites();

                if (mounted) {
                    if (data.length > 0) {
                        setSatellites(data);
                        setLoadedSatellites(data);
                    } else {
                        const fallback = getSampleSatellites();
                        setSatellites(fallback);
                        setLoadedSatellites(fallback);
                    }
                }
            } catch (e) {
                if (mounted) {
                    const fallback = getSampleSatellites();
                    setSatellites(fallback);
                    setLoadedSatellites(fallback);
                }
            } finally {
                if (mounted) {
                    setLoading(false);
                }
            }
        }

        loadSatellites();

        // Refresh data every 5 minutes
        const interval = setInterval(loadSatellites, 5 * 60 * 1000);

        return () => {
            mounted = false;
            clearInterval(interval);
        };
    }, [setLoadedSatellites]);

    // Filter satellites by visible categories
    const visibleSatellites = useMemo(() => {
        return satellites.filter(sat => visibleCategories.has(sat.category));
    }, [satellites, visibleCategories]);

    // Handle click detection
    useFrame(() => {
        if (!groupRef.current) return;

        // Update satellite positions
        visibleSatellites.forEach((sat) => {
            const mesh = meshRefs.current.get(sat.id);
            if (mesh) {
                const pos = calculateSatellitePosition(sat, missionElapsedTime);
                mesh.position.set(pos.x, pos.y, pos.z);
            }
        });
    });

    // Handle clicks
    const handleClick = (sat: SatelliteData) => {
        selectSatellite(selectedSatellite?.id === sat.id ? null : sat);
    };

    // Handle hover
    const handlePointerEnter = (sat: SatelliteData) => {
        setHoveredSatellite(sat);
        document.body.style.cursor = 'pointer';
    };

    const handlePointerLeave = () => {
        setHoveredSatellite(null);
        document.body.style.cursor = 'default';
    };

    if (loading) {
        return (
            <mesh>
                <sphereGeometry args={[0.02, 8, 8]} />
                <meshBasicMaterial color="#00d4ff" wireframe />
            </mesh>
        );
    }

    return (
        <group ref={groupRef}>
            {visibleSatellites.map((sat) => {
                const pos = calculateSatellitePosition(sat, missionElapsedTime);
                const isSelected = selectedSatellite?.id === sat.id;
                const isHovered = hoveredSatellite?.id === sat.id;
                const isISS = sat.category === 'iss';

                // Size based on category and state
                let scale = 0.008;
                if (isISS) scale = 0.02;
                else if (sat.category === 'iridium') scale = 0.01;
                if (isSelected || isHovered) scale *= 1.5;

                return (
                    <group key={sat.id}>
                        {/* Satellite sphere */}
                        <mesh
                            ref={(mesh) => {
                                if (mesh) meshRefs.current.set(sat.id, mesh);
                            }}
                            position={[pos.x, pos.y, pos.z]}
                            onClick={() => handleClick(sat)}
                            onPointerEnter={() => handlePointerEnter(sat)}
                            onPointerLeave={handlePointerLeave}
                        >
                            <sphereGeometry args={[scale, isISS ? 16 : 8, isISS ? 16 : 8]} />
                            <meshBasicMaterial
                                color={CATEGORY_COLORS[sat.category]}
                                transparent={!isSelected && !isHovered}
                                opacity={isSelected || isHovered ? 1 : 0.8}
                            />
                        </mesh>

                        {/* Selection ring */}
                        {(isSelected || isHovered) && (
                            <mesh position={[pos.x, pos.y, pos.z]} rotation={[Math.PI / 2, 0, 0]}>
                                <ringGeometry args={[scale * 1.5, scale * 2, 32]} />
                                <meshBasicMaterial
                                    color={CATEGORY_COLORS[sat.category]}
                                    transparent
                                    opacity={0.5}
                                    side={THREE.DoubleSide}
                                />
                            </mesh>
                        )}

                        {/* Hover label */}
                        {isHovered && !isSelected && (
                            <Html position={[pos.x, pos.y + 0.05, pos.z]} center>
                                <div className="px-2 py-1 bg-black/80 backdrop-blur-sm rounded text-xs text-white whitespace-nowrap border border-white/20">
                                    {sat.name}
                                </div>
                            </Html>
                        )}
                    </group>
                );
            })}

            {/* Orbital rings for visualization */}
            {[1.08, 1.12, 1.18].map((radius, i) => (
                <mesh
                    key={i}
                    rotation={[Math.PI / 2 + (i * 0.3 - 0.3), i * 0.4, 0]}
                >
                    <torusGeometry args={[radius, 0.0008, 4, 128]} />
                    <meshBasicMaterial
                        color="#ffffff"
                        transparent
                        opacity={0.05}
                    />
                </mesh>
            ))}
        </group>
    );
}
