/**
 * Earth.tsx
 * Photorealistic Earth with NASA textures and atmospheric effects
 */

import { useRef, useMemo } from 'react';
import { useFrame, useLoader } from '@react-three/fiber';
import { TextureLoader } from 'three';
import * as THREE from 'three';

// Earth radius in our scene units
const EARTH_RADIUS = 1;

// NASA texture URLs (public domain)
const EARTH_TEXTURE_URL = 'https://raw.githubusercontent.com/mrdoob/three.js/dev/examples/textures/planets/earth_atmos_2048.jpg';
const EARTH_BUMP_URL = 'https://raw.githubusercontent.com/mrdoob/three.js/dev/examples/textures/planets/earth_normal_2048.jpg';
const EARTH_SPECULAR_URL = 'https://raw.githubusercontent.com/mrdoob/three.js/dev/examples/textures/planets/earth_specular_2048.jpg';
const CLOUD_TEXTURE_URL = 'https://raw.githubusercontent.com/mrdoob/three.js/dev/examples/textures/planets/earth_clouds_1024.png';

// Atmospheric glow shader
const atmosphereVertexShader = `
  varying vec3 vNormal;
  varying vec3 vPosition;
  void main() {
    vNormal = normalize(normalMatrix * normal);
    vPosition = (modelViewMatrix * vec4(position, 1.0)).xyz;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }
`;

const atmosphereFragmentShader = `
  varying vec3 vNormal;
  varying vec3 vPosition;
  uniform vec3 glowColor;
  uniform float intensity;
  uniform float power;
  
  void main() {
    vec3 viewDirection = normalize(-vPosition);
    float fresnel = pow(1.0 - abs(dot(vNormal, viewDirection)), power);
    gl_FragColor = vec4(glowColor, fresnel * intensity);
  }
`;

// Inner glow for atmosphere edge
const innerGlowFragmentShader = `
  varying vec3 vNormal;
  varying vec3 vPosition;
  
  void main() {
    vec3 viewDirection = normalize(-vPosition);
    float intensity = pow(0.65 - dot(vNormal, viewDirection), 2.0);
    vec3 atmosphereColor = vec3(0.3, 0.6, 1.0);
    gl_FragColor = vec4(atmosphereColor, intensity * 0.5);
  }
`;

export default function Earth() {
    const earthRef = useRef<THREE.Mesh>(null);
    const cloudsRef = useRef<THREE.Mesh>(null);
    const atmosphereRef = useRef<THREE.Mesh>(null);

    // Load NASA textures
    const [earthTexture, bumpMap, specularMap, cloudTexture] = useLoader(TextureLoader, [
        EARTH_TEXTURE_URL,
        EARTH_BUMP_URL,
        EARTH_SPECULAR_URL,
        CLOUD_TEXTURE_URL,
    ]);

    // Configure textures
    useMemo(() => {
        [earthTexture, bumpMap, specularMap, cloudTexture].forEach(texture => {
            if (texture) {
                texture.colorSpace = THREE.SRGBColorSpace;
                texture.anisotropy = 16;
            }
        });
    }, [earthTexture, bumpMap, specularMap, cloudTexture]);

    // Atmosphere shader materials
    const outerAtmosphereMaterial = useMemo(() => {
        return new THREE.ShaderMaterial({
            vertexShader: atmosphereVertexShader,
            fragmentShader: atmosphereFragmentShader,
            uniforms: {
                glowColor: { value: new THREE.Color(0x93cfef) },
                intensity: { value: 0.7 },
                power: { value: 4.0 },
            },
            transparent: true,
            blending: THREE.AdditiveBlending,
            side: THREE.BackSide,
            depthWrite: false,
        });
    }, []);

    const innerAtmosphereMaterial = useMemo(() => {
        return new THREE.ShaderMaterial({
            vertexShader: atmosphereVertexShader,
            fragmentShader: innerGlowFragmentShader,
            transparent: true,
            blending: THREE.AdditiveBlending,
            side: THREE.FrontSide,
            depthWrite: false,
        });
    }, []);

    useFrame((_, delta) => {
        // Rotate Earth (one rotation per day at 1000x time warp would be ~0.07 rad/s)
        if (earthRef.current) {
            earthRef.current.rotation.y += delta * 0.05;
        }
        // Clouds rotate slightly faster
        if (cloudsRef.current) {
            cloudsRef.current.rotation.y += delta * 0.06;
        }
    });

    return (
        <group>
            {/* Main Earth sphere with NASA textures */}
            <mesh ref={earthRef} castShadow receiveShadow>
                <sphereGeometry args={[EARTH_RADIUS, 64, 64]} />
                <meshPhongMaterial
                    map={earthTexture}
                    bumpMap={bumpMap}
                    bumpScale={0.05}
                    specularMap={specularMap}
                    specular={new THREE.Color(0x333333)}
                    shininess={15}
                />
            </mesh>

            {/* Cloud layer */}
            <mesh ref={cloudsRef}>
                <sphereGeometry args={[EARTH_RADIUS * 1.01, 48, 48]} />
                <meshPhongMaterial
                    map={cloudTexture}
                    transparent
                    opacity={0.4}
                    depthWrite={false}
                    side={THREE.DoubleSide}
                />
            </mesh>

            {/* Inner atmosphere (rim lighting) */}
            <mesh scale={[1.01, 1.01, 1.01]}>
                <sphereGeometry args={[EARTH_RADIUS, 48, 48]} />
                <primitive object={innerAtmosphereMaterial} attach="material" />
            </mesh>

            {/* Outer atmospheric glow */}
            <mesh ref={atmosphereRef} scale={[1.15, 1.15, 1.15]}>
                <sphereGeometry args={[EARTH_RADIUS, 48, 48]} />
                <primitive object={outerAtmosphereMaterial} attach="material" />
            </mesh>

            {/* Secondary outer glow for depth */}
            <mesh scale={[1.25, 1.25, 1.25]}>
                <sphereGeometry args={[EARTH_RADIUS, 32, 32]} />
                <meshBasicMaterial
                    color="#4da8ff"
                    transparent
                    opacity={0.05}
                    side={THREE.BackSide}
                    depthWrite={false}
                />
            </mesh>
        </group>
    );
}
