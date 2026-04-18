/**
 * Starfield.tsx
 * Immersive space background with stars
 */

import { useMemo, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

interface StarfieldProps {
    count?: number;
    radius?: number;
}

export default function Starfield({ count = 5000, radius = 50 }: StarfieldProps) {
    const pointsRef = useRef<THREE.Points>(null);

    const { positions, colors, sizes } = useMemo(() => {
        const positions = new Float32Array(count * 3);
        const colors = new Float32Array(count * 3);
        const sizes = new Float32Array(count);

        for (let i = 0; i < count; i++) {
            // Distribute stars on a sphere
            const theta = Math.random() * Math.PI * 2;
            const phi = Math.acos(2 * Math.random() - 1);
            const r = radius + (Math.random() - 0.5) * 10;

            positions[i * 3] = r * Math.sin(phi) * Math.cos(theta);
            positions[i * 3 + 1] = r * Math.sin(phi) * Math.sin(theta);
            positions[i * 3 + 2] = r * Math.cos(phi);

            // Star colors (white to blue to yellow)
            const colorChoice = Math.random();
            if (colorChoice < 0.7) {
                // White stars
                colors[i * 3] = 1;
                colors[i * 3 + 1] = 1;
                colors[i * 3 + 2] = 1;
            } else if (colorChoice < 0.85) {
                // Blue stars
                colors[i * 3] = 0.7;
                colors[i * 3 + 1] = 0.8;
                colors[i * 3 + 2] = 1;
            } else {
                // Yellow/orange stars
                colors[i * 3] = 1;
                colors[i * 3 + 1] = 0.9;
                colors[i * 3 + 2] = 0.7;
            }

            // Varying star sizes
            sizes[i] = Math.random() * 2 + 0.5;
        }

        return { positions, colors, sizes };
    }, [count, radius]);

    // Star twinkle effect
    useFrame((state) => {
        if (!pointsRef.current) return;

        const time = state.clock.elapsedTime;
        const geometry = pointsRef.current.geometry;
        const sizesAttr = geometry.attributes.size as THREE.BufferAttribute;

        for (let i = 0; i < count; i++) {
            const twinkle = Math.sin(time * (2 + i * 0.01) + i) * 0.3 + 0.7;
            sizesAttr.array[i] = sizes[i] * twinkle;
        }

        sizesAttr.needsUpdate = true;
    });

    const starMaterial = useMemo(() => {
        return new THREE.PointsMaterial({
            size: 0.5,
            sizeAttenuation: true,
            vertexColors: true,
            transparent: true,
            opacity: 0.9,
            blending: THREE.AdditiveBlending,
        });
    }, []);

    return (
        <>
            <points ref={pointsRef}>
                <bufferGeometry>
                    <bufferAttribute
                        attach="attributes-position"
                        count={count}
                        array={positions}
                        itemSize={3}
                    />
                    <bufferAttribute
                        attach="attributes-color"
                        count={count}
                        array={colors}
                        itemSize={3}
                    />
                    <bufferAttribute
                        attach="attributes-size"
                        count={count}
                        array={sizes}
                        itemSize={1}
                    />
                </bufferGeometry>
                <primitive object={starMaterial} attach="material" />
            </points>

            {/* Milky Way band - subtle glow */}
            <mesh rotation={[Math.PI / 4, 0, 0]}>
                <torusGeometry args={[40, 5, 8, 64]} />
                <meshBasicMaterial
                    color="#4a5568"
                    transparent
                    opacity={0.03}
                    side={THREE.DoubleSide}
                />
            </mesh>
        </>
    );
}
