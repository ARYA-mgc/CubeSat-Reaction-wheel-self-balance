/**
 * OrbitalPath.tsx
 * Visual orbital trajectory ring around Earth - Fixed inclination
 */

import { useMemo } from 'react';
import * as THREE from 'three';
import { Line } from '@react-three/drei';

interface OrbitalPathProps {
    radius: number;
    inclination?: number;
    color?: string;
}

export default function OrbitalPath({
    radius,
    inclination = 51.6,
    color = '#00d4ff',
}: OrbitalPathProps) {
    const inclinationRad = (inclination * Math.PI) / 180;

    // Create orbital ring points with proper inclination matching CubeSat
    const ringPoints = useMemo(() => {
        const points: [number, number, number][] = [];
        const segments = 128;

        for (let i = 0; i <= segments; i++) {
            const angle = (i / segments) * Math.PI * 2;
            const x = Math.cos(angle) * radius;
            const z = Math.sin(angle) * radius;

            // Apply same inclination transform as CubeSat
            const y = z * Math.sin(inclinationRad);
            const adjustedZ = z * Math.cos(inclinationRad);

            points.push([x, y, adjustedZ]);
        }

        return points;
    }, [radius, inclinationRad]);

    return (
        <group>
            {/* Main orbital ring using drei Line */}
            <Line
                points={ringPoints}
                color={color}
                lineWidth={1.5}
                transparent
                opacity={0.6}
            />

            {/* Glowing torus following same path */}
            <mesh>
                <tubeGeometry args={[
                    new THREE.CatmullRomCurve3(
                        Array.from({ length: 129 }, (_, i) => {
                            const angle = (i / 128) * Math.PI * 2;
                            const x = Math.cos(angle) * radius;
                            const z = Math.sin(angle) * radius;
                            const y = z * Math.sin(inclinationRad);
                            const adjustedZ = z * Math.cos(inclinationRad);
                            return new THREE.Vector3(x, y, adjustedZ);
                        }),
                        true
                    ),
                    128,
                    0.003,
                    8,
                    true
                ]} />
                <meshBasicMaterial
                    color={color}
                    transparent
                    opacity={0.25}
                />
            </mesh>

            {/* Orbital nodes (ascending/descending) */}
            {[0, Math.PI].map((angle, i) => {
                const x = Math.cos(angle) * radius;
                const z = Math.sin(angle) * radius;
                const y = z * Math.sin(inclinationRad);
                const adjustedZ = z * Math.cos(inclinationRad);

                return (
                    <mesh key={i} position={[x, y, adjustedZ]}>
                        <sphereGeometry args={[0.015, 12, 12]} />
                        <meshBasicMaterial color={i === 0 ? '#00ff88' : '#ff6b6b'} />
                    </mesh>
                );
            })}
        </group>
    );
}

// Orbital trail behind satellite
export function OrbitalTrail({
    radius,
    currentAngle,
    trailLength = Math.PI / 4,
    inclination = 51.6,
    color = '#00ff88'
}: {
    radius: number;
    currentAngle: number;
    trailLength?: number;
    inclination?: number;
    color?: string;
}) {
    const inclinationRad = (inclination * Math.PI) / 180;

    const trailPoints = useMemo(() => {
        const points: [number, number, number][] = [];
        const segments = 64;
        const startAngle = currentAngle - trailLength;

        for (let i = 0; i <= segments; i++) {
            const angle = startAngle + (i / segments) * trailLength;
            const x = Math.cos(angle) * radius;
            const z = Math.sin(angle) * radius;

            // Apply inclination
            const y = z * Math.sin(inclinationRad);
            const adjustedZ = z * Math.cos(inclinationRad);

            points.push([x, y, adjustedZ]);
        }

        return points;
    }, [radius, currentAngle, trailLength, inclinationRad]);

    // Trail tube curve
    const trailCurve = useMemo(() => {
        return new THREE.CatmullRomCurve3(
            Array.from({ length: 32 }, (_, i) => {
                const angle = currentAngle - trailLength + (i / 31) * trailLength;
                const x = Math.cos(angle) * radius;
                const z = Math.sin(angle) * radius;
                const y = z * Math.sin(inclinationRad);
                const adjustedZ = z * Math.cos(inclinationRad);
                return new THREE.Vector3(x, y, adjustedZ);
            })
        );
    }, [radius, currentAngle, trailLength, inclinationRad]);

    return (
        <group>
            {/* Trail line using drei Line */}
            <Line
                points={trailPoints}
                color={color}
                lineWidth={2}
                transparent
                opacity={0.7}
            />

            {/* Glowing trail tube */}
            <mesh>
                <tubeGeometry args={[trailCurve, 32, 0.006, 8, false]} />
                <meshBasicMaterial
                    color={color}
                    transparent
                    opacity={0.35}
                />
            </mesh>
        </group>
    );
}
