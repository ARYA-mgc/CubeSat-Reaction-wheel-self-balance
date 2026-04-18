/**
 * OrbitingCubeSat.tsx
 * CubeSat that follows orbital path while attitude is controlled by IMU
 */

import { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { useTrackingStore } from '../../store/trackingStore';
import { useIMUStore } from '../../store/imuStore';
import { OrbitalTrail } from './OrbitalPath';

interface OrbitingCubeSatProps {
    orbitRadius: number;
    inclination?: number;
}

export default function OrbitingCubeSat({
    orbitRadius,
    inclination = 51.6
}: OrbitingCubeSatProps) {
    const groupRef = useRef<THREE.Group>(null);
    const satBodyRef = useRef<THREE.Group>(null);
    const glowRef = useRef<THREE.Mesh>(null);

    const orbitalAngle = useTrackingStore((state) => state.orbitalAngle);
    const tick = useTrackingStore((state) => state.tick);

    // IMU integration for attitude control
    const useIMUMode = useIMUStore((state) => state.useIMUMode);
    const imuQuaternion = useIMUStore((state) => state.imuQuaternion);
    const environment = useIMUStore((state) => state.environment);

    const targetQuaternion = useRef(new THREE.Quaternion());

    const inclinationRad = (inclination * Math.PI) / 180;

    useFrame((state, delta) => {
        // Update orbital position
        tick(delta);

        if (groupRef.current) {
            // Calculate dynamic orbit radius based on altitude
            const altOffset = useIMUMode && environment?.alt ? (environment.alt * 0.01) : 0;
            const currentRadius = orbitRadius + altOffset;

            // Calculate position on orbit
            const x = Math.cos(orbitalAngle) * currentRadius;
            const z = Math.sin(orbitalAngle) * currentRadius;

            // Apply inclination to position
            const inclinedY = z * Math.sin(inclinationRad);
            const inclinedZ = z * Math.cos(inclinationRad);

            groupRef.current.position.set(x, inclinedY, inclinedZ);
        }

        if (satBodyRef.current) {
            if (useIMUMode && imuQuaternion) {
                // 1. Convert BNO08x to Three.js
                const rawQ = new THREE.Quaternion(
                    imuQuaternion.i,
                    imuQuaternion.j,
                    imuQuaternion.k,
                    imuQuaternion.real
                );

                // 2. Apply Offset
                if (useIMUStore.getState().imuOffset) {
                    const offset = useIMUStore.getState().imuOffset!;
                    const offsetQ = new THREE.Quaternion(offset.i, offset.j, offset.k, offset.real);
                    targetQuaternion.current.multiplyQuaternions(offsetQ, rawQ);
                } else {
                    targetQuaternion.current.copy(rawQ);
                }

                // 3. Correction for "Up" direction (same as SatVisualizer)
                const correction = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
                targetQuaternion.current.premultiply(correction);

                // 4. Slerp for smooth motion
                satBodyRef.current.quaternion.slerp(targetQuaternion.current, Math.min(1, delta * 15));
            } else {
                // Default orientation - point "down" toward Earth
                const earthDirection = new THREE.Vector3(
                    -Math.cos(orbitalAngle),
                    0,
                    -Math.sin(orbitalAngle)
                ).normalize();

                satBodyRef.current.lookAt(
                    satBodyRef.current.position.clone().add(earthDirection)
                );
            }
        }

        // Pulsing glow effect
        if (glowRef.current) {
            const pulse = Math.sin(state.clock.elapsedTime * 2) * 0.3 + 0.7;
            glowRef.current.scale.setScalar(pulse);
        }
    });

    return (
        <>
            {/* Orbital trail */}
            <OrbitalTrail
                radius={orbitRadius}
                currentAngle={orbitalAngle}
                trailLength={Math.PI / 3}
                inclination={inclination}
                color="#00ff88"
            />

            <group ref={groupRef}>
                {/* Subtle selection glow */}
                <mesh ref={glowRef}>
                    <sphereGeometry args={[0.06, 16, 16]} />
                    <meshBasicMaterial
                        color="#00ff88"
                        transparent
                        opacity={0.1}
                    />
                </mesh>

                {/* CubeSat body with IMU-controlled attitude */}
                <group ref={satBodyRef} scale={[0.4, 0.4, 0.4]}>
                    {/* Main body - 1U CubeSat */}
                    <mesh castShadow receiveShadow>
                        <boxGeometry args={[0.1, 0.1, 0.1]} />
                        <meshStandardMaterial
                            color="#1a1a2e"
                            metalness={0.9}
                            roughness={0.15}
                            envMapIntensity={0.8}
                        />
                    </mesh>

                    {/* Gold foil panels */}
                    {[
                        [0, 0.0505, 0],
                        [0, -0.0505, 0],
                        [0.0505, 0, 0],
                        [-0.0505, 0, 0],
                    ].map((pos, i) => (
                        <mesh key={i} position={pos as [number, number, number]} castShadow>
                            <boxGeometry args={i < 2 ? [0.095, 0.001, 0.095] : [0.001, 0.095, 0.095]} />
                            <meshStandardMaterial color="#c4a000" metalness={0.95} roughness={0.1} />
                        </mesh>
                    ))}

                    {/* Solar panels - Left */}
                    <group position={[-0.12, 0, 0]}>
                        <mesh castShadow>
                            <boxGeometry args={[0.08, 0.002, 0.12]} />
                            <meshStandardMaterial color="#1a365d" metalness={0.7} roughness={0.2} />
                        </mesh>
                        {[-0.025, 0, 0.025].map((x, i) => (
                            [-0.04, -0.01, 0.02, 0.05].map((z, j) => (
                                <mesh key={`l${i}${j}`} position={[x, 0.002, z]}>
                                    <boxGeometry args={[0.02, 0.001, 0.025]} />
                                    <meshStandardMaterial color="#0f172a" metalness={0.9} roughness={0.05} />
                                </mesh>
                            ))
                        ))}
                    </group>

                    {/* Solar panels - Right */}
                    <group position={[0.12, 0, 0]}>
                        <mesh castShadow>
                            <boxGeometry args={[0.08, 0.002, 0.12]} />
                            <meshStandardMaterial color="#1a365d" metalness={0.7} roughness={0.2} />
                        </mesh>
                        {[-0.025, 0, 0.025].map((x, i) => (
                            [-0.04, -0.01, 0.02, 0.05].map((z, j) => (
                                <mesh key={`r${i}${j}`} position={[x, 0.002, z]}>
                                    <boxGeometry args={[0.02, 0.001, 0.025]} />
                                    <meshStandardMaterial color="#0f172a" metalness={0.9} roughness={0.05} />
                                </mesh>
                            ))
                        ))}
                    </group>

                    {/* Antenna */}
                    <mesh position={[0, 0.075, 0]}>
                        <cylinderGeometry args={[0.002, 0.002, 0.05]} />
                        <meshStandardMaterial
                            color="#ffcc00"
                            metalness={0.95}
                            roughness={0.05}
                            emissive="#ffcc00"
                            emissiveIntensity={0.3}
                        />
                    </mesh>

                    {/* Antenna tip */}
                    <mesh position={[0, 0.105, 0]}>
                        <sphereGeometry args={[0.004]} />
                        <meshStandardMaterial
                            color="#ffcc00"
                            metalness={0.95}
                            roughness={0.05}
                            emissive="#ffcc00"
                            emissiveIntensity={0.5}
                        />
                    </mesh>

                    {/* Status LED */}
                    <mesh position={[0, 0, 0.052]}>
                        <sphereGeometry args={[0.005]} />
                        <meshBasicMaterial color="#00ff00" />
                    </mesh>
                </group>
            </group>
        </>
    );
}
