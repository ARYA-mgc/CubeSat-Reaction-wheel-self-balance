/**
 * SatVisualizer.tsx
 * 3D visualization of CubeSat using React Three Fiber - Enhanced
 */

import { useRef, Suspense } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Stars, Environment, Float } from '@react-three/drei';
import * as THREE from 'three';
import { useSimulationStore } from '../store/simulationStore';
import { useThemeStore } from '../store/themeStore';
import { useIMUStore } from '../store/imuStore';
import { Maximize2, Box, RotateCw } from 'lucide-react';
import { cn } from '../utils/cn';

// CubeSat 3D Model (1U cube) - Enhanced
function CubeSat() {
  const groupRef = useRef<THREE.Group>(null);
  const targetQuaternion = useRef(new THREE.Quaternion());
  const currentData = useSimulationStore((state) => state.currentData);
  const useIMUMode = useIMUStore((state) => state.useIMUMode);
  const imuQuaternion = useIMUStore((state) => state.imuQuaternion);
  const imuOffset = useIMUStore((state) => state.imuOffset);

  useFrame((_, delta) => {
    if (groupRef.current) {
      if (useIMUMode && imuQuaternion) {
        // 1. Convert BNO08x output to Three.js quaternion
        // Note: BNO08x Game Rotation Vector is already a valid unit quaternion
        const rawQ = new THREE.Quaternion(
          imuQuaternion.i,
          imuQuaternion.j,
          imuQuaternion.k,
          imuQuaternion.real
        );

        // 2. Apply Offset (Zeroing) if available
        if (imuOffset) {
          const offsetQ = new THREE.Quaternion(
            imuOffset.i,
            imuOffset.j,
            imuOffset.k,
            imuOffset.real
          );
          targetQuaternion.current.multiplyQuaternions(offsetQ, rawQ);
        } else {
          targetQuaternion.current.copy(rawQ);
        }

        // 3. Coordinate System Mapping (Standardizing for Sat View)
        // Usually needs a -90deg X rotation to make "Up" match the IMU's orientation
        // in a Y-up world.
        const correction = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
        targetQuaternion.current.premultiply(correction);

        // 4. Smooth interpolation (Slerp) for 60fps feel
        groupRef.current.quaternion.slerp(targetQuaternion.current, Math.min(1, delta * 15));

      } else if (currentData) {
        // Use simulation physics data
        const q = currentData.physicsState.quaternion;
        targetQuaternion.current.set(q.x, q.y, q.z, q.w);
        groupRef.current.quaternion.slerp(targetQuaternion.current, Math.min(1, delta * 10));
      }
    }
  });

  return (
    <group ref={groupRef}>
      {/* Main body - 1U CubeSat (10x10x10 cm) */}
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
        {/* Solar cell grid */}
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
        {/* Solar cell grid */}
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
        <meshStandardMaterial color="#ffcc00" metalness={0.95} roughness={0.05} emissive="#ffcc00" emissiveIntensity={0.2} />
      </mesh>

      {/* Antenna tip */}
      <mesh position={[0, 0.105, 0]}>
        <sphereGeometry args={[0.004]} />
        <meshStandardMaterial color="#ffcc00" metalness={0.95} roughness={0.05} emissive="#ffcc00" emissiveIntensity={0.3} />
      </mesh>

      {/* Body axes */}
      <axesHelper args={[0.2]} />
    </group>
  );
}

// Angular velocity vector visualization
function AngularVelocityVector() {
  const lineRef = useRef<THREE.Line>(null);
  const currentData = useSimulationStore((state) => state.currentData);

  useFrame(() => {
    if (lineRef.current && currentData) {
      const ω = currentData.physicsState.angularVelocity;
      const magnitude = Math.sqrt(ω.x ** 2 + ω.y ** 2 + ω.z ** 2);

      if (magnitude > 0.001) {
        const scale = magnitude * 5;
        const positions = lineRef.current.geometry.attributes.position;
        positions.setXYZ(1, ω.x * scale / magnitude, ω.y * scale / magnitude, ω.z * scale / magnitude);
        positions.needsUpdate = true;
        lineRef.current.visible = true;
      } else {
        lineRef.current.visible = false;
      }
    }
  });

  const points = [new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0.2, 0)];
  const geometry = new THREE.BufferGeometry().setFromPoints(points);

  return (
    <primitive object={new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: '#ffaa00', linewidth: 2 }))} ref={lineRef} />
  );
}

// Loader component
function Loader() {
  return (
    <Float speed={2} rotationIntensity={0.5} floatIntensity={0.5}>
      <mesh>
        <boxGeometry args={[0.05, 0.05, 0.05]} />
        <meshStandardMaterial color="#00d4ff" wireframe />
      </mesh>
    </Float>
  );
}

// Scene environment based on theme
function SceneEnvironment({ isDark }: { isDark: boolean }) {
  return (
    <>
      {isDark ? (
        <>
          <Stars radius={100} depth={50} count={3000} factor={3} fade speed={0.3} />
          <Environment preset="night" />
          <color attach="background" args={['#0a0a15']} />
        </>
      ) : (
        <>
          <Environment preset="city" />
          <color attach="background" args={['#e8f4fc']} />
          <fog attach="fog" args={['#e8f4fc', 0.5, 3]} />
        </>
      )}
    </>
  );
}

// Main visualizer component
export default function SatVisualizer() {
  const isRunning = useSimulationStore((state) => state.isRunning);
  const currentData = useSimulationStore((state) => state.currentData);
  const theme = useThemeStore((state) => state.theme);
  const isDark = theme === 'dark';

  const angVelMag = currentData
    ? Math.sqrt(
      currentData.physicsState.angularVelocity.x ** 2 +
      currentData.physicsState.angularVelocity.y ** 2 +
      currentData.physicsState.angularVelocity.z ** 2
    )
    : 0;

  return (
    <div className={cn(
      "w-full h-full rounded-xl overflow-hidden border relative transition-colors duration-500",
      isDark
        ? "bg-gradient-to-b from-[#0a0a15] to-[#0f0f1a] border-white/5"
        : "bg-gradient-to-b from-[#e8f4fc] to-[#f0f9ff] border-black/10"
    )}>
      {/* Canvas */}
      <Canvas
        camera={{ position: [0.35, 0.25, 0.35], fov: 45 }}
        shadows
        gl={{ antialias: true, alpha: true }}
        dpr={[1, 2]}
      >
        <Suspense fallback={<Loader />}>
          {/* Theme-aware environment */}
          <SceneEnvironment isDark={isDark} />

          {/* Lighting */}
          <ambientLight intensity={isDark ? 0.2 : 0.5} />
          <directionalLight
            position={[5, 5, 5]}
            intensity={isDark ? 1.5 : 2}
            castShadow
            shadow-mapSize={[2048, 2048]}
            color="#ffffff"
          />
          <pointLight position={[-3, -3, 2]} intensity={isDark ? 0.3 : 0.1} color={isDark ? "#00d4ff" : "#0891b2"} />
          <pointLight position={[3, -2, -3]} intensity={isDark ? 0.2 : 0.1} color={isDark ? "#ff6600" : "#d97706"} />

          {/* Scene */}
          <CubeSat />
          <AngularVelocityVector />

          {/* Ground plane hint */}
          <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.15, 0]} receiveShadow>
            <planeGeometry args={[2, 2]} />
            <meshStandardMaterial color={isDark ? "#0a0a0a" : "#d1e8f5"} transparent opacity={isDark ? 0.5 : 0.8} />
          </mesh>

          {/* Grid helper */}
          <gridHelper args={[1, 20, isDark ? '#1a1a2e' : '#94c4df', isDark ? '#1a1a2e' : '#b8daf0']} position={[0, -0.15, 0]} />

          {/* Controls */}
          <OrbitControls
            enablePan={true}
            enableZoom={true}
            enableRotate={true}
            minDistance={0.2}
            maxDistance={1.5}
            autoRotate={!isRunning && !currentData}
            autoRotateSpeed={0.5}
          />
        </Suspense>
      </Canvas>

      {/* Overlay UI */}
      <div className="absolute inset-0 pointer-events-none">
        {/* Top left - Title */}
        <div className="absolute top-3 left-3 flex items-center gap-2">
          <div className={cn(
            "flex items-center gap-2 px-3 py-1.5 backdrop-blur-sm rounded-lg border",
            isDark ? "bg-black/60 border-cyan-500/20" : "bg-white/80 border-cyan-600/20 shadow-sm"
          )}>
            <Box className={cn("w-3.5 h-3.5", isDark ? "text-cyan-400" : "text-cyan-600")} />
            <span className={cn("text-xs font-mono", isDark ? "text-cyan-400" : "text-cyan-700")}>3D Attitude View</span>
          </div>
        </div>

        {/* Top right - Controls hint */}
        <div className="absolute top-3 right-3">
          <div className={cn(
            "px-2.5 py-1.5 backdrop-blur-sm rounded-lg border",
            isDark ? "bg-black/60 border-white/10" : "bg-white/80 border-black/10 shadow-sm"
          )}>
            <span className={cn("text-[10px]", isDark ? "text-gray-500" : "text-gray-500")}>Drag to rotate • Scroll to zoom</span>
          </div>
        </div>

        {/* Bottom left - Angular velocity indicator */}
        <div className="absolute bottom-3 left-3">
          <div className={cn(
            "flex items-center gap-2 px-3 py-2 backdrop-blur-sm rounded-lg border",
            isDark ? "bg-black/60 border-white/10" : "bg-white/80 border-black/10 shadow-sm"
          )}>
            <RotateCw className={cn(
              "w-3.5 h-3.5",
              isDark ? "text-amber-400" : "text-amber-600",
              angVelMag > 0.01 ? 'animate-spin' : ''
            )}
              style={{ animationDuration: `${Math.max(0.5, 2 / (angVelMag + 0.1))}s` }}
            />
            <div className="text-xs font-mono">
              <span className={isDark ? "text-gray-500" : "text-gray-500"}>|ω| = </span>
              <span className={cn("tabular-nums", isDark ? "text-amber-400" : "text-amber-600")}>{angVelMag.toFixed(4)}</span>
              <span className={isDark ? "text-gray-600" : "text-gray-400"} > rad/s</span>
            </div>
          </div>
        </div>

        {/* Bottom right - Maximize button */}
        <button className={cn(
          "absolute bottom-3 right-3 p-2 backdrop-blur-sm rounded-lg border pointer-events-auto transition-colors",
          isDark
            ? "bg-black/60 border-white/10 hover:border-white/20"
            : "bg-white/80 border-black/10 hover:border-black/20 shadow-sm"
        )}>
          <Maximize2 className={cn("w-4 h-4", isDark ? "text-gray-400" : "text-gray-500")} />
        </button>
      </div>

      {/* Corner decorations */}
      <div className={cn(
        "absolute top-0 left-0 w-8 h-8 border-l-2 border-t-2 rounded-tl-xl",
        isDark ? "border-cyan-500/30" : "border-cyan-600/30"
      )} />
      <div className={cn(
        "absolute top-0 right-0 w-8 h-8 border-r-2 border-t-2 rounded-tr-xl",
        isDark ? "border-cyan-500/30" : "border-cyan-600/30"
      )} />
      <div className={cn(
        "absolute bottom-0 left-0 w-8 h-8 border-l-2 border-b-2 rounded-bl-xl",
        isDark ? "border-cyan-500/30" : "border-cyan-600/30"
      )} />
      <div className={cn(
        "absolute bottom-0 right-0 w-8 h-8 border-r-2 border-b-2 rounded-br-xl",
        isDark ? "border-cyan-500/30" : "border-cyan-600/30"
      )} />
    </div>
  );
}
