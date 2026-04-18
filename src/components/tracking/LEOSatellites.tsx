/**
 * LEOSatellites.tsx
 * Other satellites in Low Earth Orbit for visual effect
 */

import { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

interface Satellite {
    id: string;
    name: string;
    orbitRadius: number;
    inclination: number;
    initialAngle: number;
    speed: number;
    size: number;
    color: string;
}

// Generate random LEO satellites
function generateSatellites(count: number): Satellite[] {
    const satellites: Satellite[] = [];

    const names = [
        'STARLINK', 'ONEWEB', 'IRIDIUM', 'GLOBALSTAR', 'ORBCOMM',
        'SPIRE', 'PLANET', 'SWARM', 'LEMUR', 'DOVE',
        'FLOCK', 'SKYSAT', 'ICEYE', 'CAPELLA', 'HAWK'
    ];

    for (let i = 0; i < count; i++) {
        satellites.push({
            id: `sat-${i}`,
            name: `${names[i % names.length]}-${Math.floor(i / names.length) + 1}`,
            orbitRadius: 1.05 + Math.random() * 0.15, // 1.05 to 1.20 Earth radii
            inclination: Math.random() * 90, // 0 to 90 degrees
            initialAngle: Math.random() * Math.PI * 2,
            speed: 0.1 + Math.random() * 0.2, // Variable speeds
            size: 0.003 + Math.random() * 0.005,
            color: ['#ffffff', '#00d4ff', '#ff6b6b', '#ffd93d', '#6bcb77'][Math.floor(Math.random() * 5)],
        });
    }

    return satellites;
}

export default function LEOSatellites({ count = 40 }: { count?: number }) {
    const groupRef = useRef<THREE.Group>(null);
    const satellitesRef = useRef<THREE.InstancedMesh>(null);

    const satellites = useMemo(() => generateSatellites(count), [count]);

    // Create instanced mesh for performance
    const dummy = useMemo(() => new THREE.Object3D(), []);
    const colors = useMemo(() => {
        const colorArray = new Float32Array(count * 3);
        satellites.forEach((sat, i) => {
            const color = new THREE.Color(sat.color);
            colorArray[i * 3] = color.r;
            colorArray[i * 3 + 1] = color.g;
            colorArray[i * 3 + 2] = color.b;
        });
        return colorArray;
    }, [satellites, count]);

    useFrame((state) => {
        if (!satellitesRef.current) return;

        const time = state.clock.elapsedTime;

        satellites.forEach((sat, i) => {
            const angle = sat.initialAngle + time * sat.speed * 0.1;
            const incRad = (sat.inclination * Math.PI) / 180;

            const x = Math.cos(angle) * sat.orbitRadius;
            const z = Math.sin(angle) * sat.orbitRadius;
            const y = z * Math.sin(incRad);
            const adjustedZ = z * Math.cos(incRad);

            dummy.position.set(x, y, adjustedZ);
            dummy.scale.setScalar(sat.size * 10);
            dummy.updateMatrix();

            satellitesRef.current!.setMatrixAt(i, dummy.matrix);
        });

        satellitesRef.current.instanceMatrix.needsUpdate = true;
    });

    return (
        <group ref={groupRef}>
            <instancedMesh ref={satellitesRef} args={[undefined, undefined, count]}>
                <sphereGeometry args={[1, 8, 8]} />
                <meshBasicMaterial color="#ffffff" />
            </instancedMesh>

            {/* Orbital rings for some satellites (visual effect) */}
            {[1.08, 1.12, 1.18].map((radius, i) => (
                <mesh key={i} rotation={[Math.PI / 2, 0, i * 0.5]}>
                    <torusGeometry args={[radius, 0.0005, 4, 128]} />
                    <meshBasicMaterial
                        color="#ffffff"
                        transparent
                        opacity={0.08}
                    />
                </mesh>
            ))}
        </group>
    );
}

// Individual satellite with trail (for special satellites)
export function TrackedSatellite({
    orbitRadius,
    inclination,
    color = '#ff6b6b',
    speed = 0.15,
    initialAngle = 0,
}: {
    orbitRadius: number;
    inclination: number;
    color?: string;
    speed?: number;
    initialAngle?: number;
}) {
    const meshRef = useRef<THREE.Mesh>(null);
    const trailRef = useRef<THREE.Line>(null);

    const trailPositions = useRef<THREE.Vector3[]>([]);
    const maxTrailLength = 50;

    const incRad = (inclination * Math.PI) / 180;

    useFrame((state) => {
        if (!meshRef.current) return;

        const time = state.clock.elapsedTime;
        const angle = initialAngle + time * speed;

        const x = Math.cos(angle) * orbitRadius;
        const z = Math.sin(angle) * orbitRadius;
        const y = z * Math.sin(incRad);
        const adjustedZ = z * Math.cos(incRad);

        meshRef.current.position.set(x, y, adjustedZ);

        // Update trail
        trailPositions.current.push(new THREE.Vector3(x, y, adjustedZ));
        if (trailPositions.current.length > maxTrailLength) {
            trailPositions.current.shift();
        }

        if (trailRef.current && trailPositions.current.length > 1) {
            const geometry = new THREE.BufferGeometry().setFromPoints(trailPositions.current);
            trailRef.current.geometry.dispose();
            trailRef.current.geometry = geometry;
        }
    });

    return (
        <group>
            <mesh ref={meshRef}>
                <sphereGeometry args={[0.015, 16, 16]} />
                <meshBasicMaterial color={color} />
            </mesh>

            <line ref={trailRef}>
                <bufferGeometry />
                <lineBasicMaterial color={color} transparent opacity={0.5} />
            </line>
        </group>
    );
}
