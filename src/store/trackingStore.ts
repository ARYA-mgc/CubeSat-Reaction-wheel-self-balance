/**
 * trackingStore.ts
 * Enhanced Zustand store for orbital tracking with satellite selection and filters
 */

import { create } from 'zustand';
import type { SatelliteData } from '../services/satelliteService';

interface OrbitalParameters {
  altitude: number;
  inclination: number;
  period: number;
  eccentricity: number;
  apogee: number;
  perigee: number;
}

interface TrackingStore {
  // Orbital state
  orbitalAngle: number;
  orbitalSpeed: number;
  isOrbiting: boolean;
  timeWarp: number;

  // Orbital parameters
  orbitalParams: OrbitalParameters;

  // Camera
  cameraMode: 'free' | 'follow' | 'earth';

  // Satellite selection
  selectedSatellite: SatelliteData | null;
  hoveredSatellite: SatelliteData | null;
  loadedSatellites: SatelliteData[];

  // Category filters
  visibleCategories: Set<SatelliteData['category']>;

  // Mission time
  missionElapsedTime: number;

  // CubeSat position (for camera follow)
  cubeSatPosition: { x: number; y: number; z: number };

  // Actions
  setOrbitalAngle: (angle: number) => void;
  toggleOrbiting: () => void;
  setTimeWarp: (warp: number) => void;
  setCameraMode: (mode: 'free' | 'follow' | 'earth') => void;
  selectSatellite: (satellite: SatelliteData | null) => void;
  setHoveredSatellite: (satellite: SatelliteData | null) => void;
  setLoadedSatellites: (satellites: SatelliteData[]) => void;
  toggleCategory: (category: SatelliteData['category']) => void;
  setCubeSatPosition: (pos: { x: number; y: number; z: number }) => void;
  tick: (deltaTime: number) => void;
  reset: () => void;
}

const DEFAULT_ORBITAL_PARAMS: OrbitalParameters = {
  altitude: 408,
  inclination: 51.6,
  period: 92.68,
  eccentricity: 0.0001,
  apogee: 410,
  perigee: 406,
};

const ALL_CATEGORIES: SatelliteData['category'][] = ['starlink', 'oneweb', 'iridium', 'iss', 'weather', 'science', 'other'];

export const useTrackingStore = create<TrackingStore>()((set, get) => ({
  orbitalAngle: 0,
  orbitalSpeed: (2 * Math.PI) / (DEFAULT_ORBITAL_PARAMS.period * 60),
  isOrbiting: true,
  timeWarp: 100,

  orbitalParams: DEFAULT_ORBITAL_PARAMS,

  cameraMode: 'free',

  selectedSatellite: null,
  hoveredSatellite: null,
  loadedSatellites: [],

  visibleCategories: new Set(ALL_CATEGORIES),

  missionElapsedTime: 0,

  cubeSatPosition: { x: 1.15, y: 0, z: 0 },

  setOrbitalAngle: (angle) => set({ orbitalAngle: angle }),

  toggleOrbiting: () => set((state) => ({ isOrbiting: !state.isOrbiting })),

  setTimeWarp: (warp) => set({ timeWarp: warp }),

  setCameraMode: (mode) => set({ cameraMode: mode }),

  selectSatellite: (satellite) => set({ selectedSatellite: satellite }),

  setHoveredSatellite: (satellite) => set({ hoveredSatellite: satellite }),

  setLoadedSatellites: (satellites) => set({ loadedSatellites: satellites }),

  toggleCategory: (category) => set((state) => {
    const newCategories = new Set(state.visibleCategories);
    if (newCategories.has(category)) {
      newCategories.delete(category);
    } else {
      newCategories.add(category);
    }
    return { visibleCategories: newCategories };
  }),

  setCubeSatPosition: (pos) => set({ cubeSatPosition: pos }),

  tick: (deltaTime) => {
    const { isOrbiting, orbitalSpeed, timeWarp, orbitalAngle, missionElapsedTime, orbitalParams } = get();
    if (isOrbiting) {
      const newAngle = (orbitalAngle + orbitalSpeed * deltaTime * timeWarp) % (2 * Math.PI);

      // Calculate CubeSat position with inclination
      const incRad = (orbitalParams.inclination * Math.PI) / 180;
      const orbitRadius = 1.15;
      const x = Math.cos(newAngle) * orbitRadius;
      const z = Math.sin(newAngle) * orbitRadius;
      const y = z * Math.sin(incRad);
      const adjustedZ = z * Math.cos(incRad);

      set({
        orbitalAngle: newAngle,
        missionElapsedTime: missionElapsedTime + deltaTime * timeWarp,
        cubeSatPosition: { x, y, z: adjustedZ },
      });
    }
  },

  reset: () => set({
    orbitalAngle: 0,
    isOrbiting: true,
    timeWarp: 100,
    missionElapsedTime: 0,
    cameraMode: 'free',
    selectedSatellite: null,
    hoveredSatellite: null,
    cubeSatPosition: { x: 1.15, y: 0, z: 0 },
    visibleCategories: new Set(ALL_CATEGORIES),
  }),
}));
