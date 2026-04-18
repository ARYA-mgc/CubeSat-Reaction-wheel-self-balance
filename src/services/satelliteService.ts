/**
 * satelliteService.ts
 * Real-time satellite data from CelesTrak TLE database
 */

// TLE data structure
export interface TLEData {
    name: string;
    line1: string;
    line2: string;
}

// Complete satellite data with orbital elements
export interface SatelliteData {
    id: string;
    name: string;
    noradId: number;
    intlDesignator: string;
    epochYear: number;
    epochDay: number;
    inclination: number;
    raan: number;
    eccentricity: number;
    argOfPerigee: number;
    meanAnomaly: number;
    meanMotion: number;
    altitude: number;
    period: number;
    category: 'starlink' | 'oneweb' | 'iridium' | 'science' | 'weather' | 'iss' | 'other';
}

// CelesTrak API endpoints
const CELESTRAK_BASE = 'https://celestrak.org/NORAD/elements/gp.php';
const CORS_PROXY = 'https://api.allorigins.win/raw?url=';

// Calculate altitude from mean motion using Kepler's law
function calculateAltitude(meanMotion: number): number {
    const GM = 3.986e14; // Earth's gravitational parameter
    const earthRadius = 6371;
    const period = (24 * 60 * 60) / meanMotion;
    const semiMajorAxis = Math.pow((GM * period * period) / (4 * Math.PI * Math.PI), 1 / 3) / 1000;
    return Math.max(0, semiMajorAxis - earthRadius);
}

// Calculate period in minutes
function calculatePeriod(meanMotion: number): number {
    return (24 * 60) / meanMotion;
}

// Parse a single TLE entry
function parseTLE(tle: TLEData, forceCategory?: SatelliteData['category']): SatelliteData | null {
    try {
        const { name, line1, line2 } = tle;

        // Validate TLE lines
        if (!line1.startsWith('1 ') || !line2.startsWith('2 ')) {
            return null;
        }

        const noradId = parseInt(line1.substring(2, 7).trim());
        const intlDesignator = line1.substring(9, 17).trim();
        const epochYear = parseInt(line1.substring(18, 20));
        const epochDay = parseFloat(line1.substring(20, 32));

        const inclination = parseFloat(line2.substring(8, 16).trim());
        const raan = parseFloat(line2.substring(17, 25).trim());
        const eccentricity = parseFloat('0.' + line2.substring(26, 33).trim());
        const argOfPerigee = parseFloat(line2.substring(34, 42).trim());
        const meanAnomaly = parseFloat(line2.substring(43, 51).trim());
        const meanMotion = parseFloat(line2.substring(52, 63).trim());

        // Validate parsed values
        if (isNaN(noradId) || isNaN(meanMotion) || meanMotion <= 0) {
            return null;
        }

        const altitude = calculateAltitude(meanMotion);

        // Determine category from name if not forced
        let category = forceCategory || 'other';
        if (!forceCategory) {
            const upperName = name.toUpperCase();
            if (upperName.includes('STARLINK')) category = 'starlink';
            else if (upperName.includes('ONEWEB')) category = 'oneweb';
            else if (upperName.includes('IRIDIUM')) category = 'iridium';
            else if (upperName.includes('ISS') || upperName.includes('ZARYA') || upperName.includes('TIANHE') || upperName.includes('TIANGONG') || upperName.includes('CSS')) category = 'iss';
            else if (upperName.includes('NOAA') || upperName.includes('GOES') || upperName.includes('METEO') || upperName.includes('METEOR')) category = 'weather';
            else if (upperName.includes('HUBBLE') || upperName.includes('TERRA') || upperName.includes('AQUA') || upperName.includes('LANDSAT') || upperName.includes('SENTINEL')) category = 'science';
        }

        return {
            id: `sat-${noradId}`,
            name: name.trim(),
            noradId,
            intlDesignator,
            epochYear: epochYear + (epochYear > 50 ? 1900 : 2000),
            epochDay,
            inclination,
            raan,
            eccentricity,
            argOfPerigee,
            meanAnomaly,
            meanMotion,
            altitude,
            period: calculatePeriod(meanMotion),
            category,
        };
    } catch (e) {
        return null;
    }
}

// Fetch TLE data from CelesTrak
async function fetchTLEGroup(group: string, forceCategory?: SatelliteData['category']): Promise<SatelliteData[]> {
    const satellites: SatelliteData[] = [];
    const url = `${CELESTRAK_BASE}?GROUP=${group}&FORMAT=tle`;

    try {
        // Try direct fetch first
        let response: Response;
        try {
            response = await fetch(url, {
                signal: AbortSignal.timeout(8000),
            });
        } catch {
            // Fallback to CORS proxy if direct fetch fails
            console.log(`Using CORS proxy for ${group}...`);
            response = await fetch(`${CORS_PROXY}${encodeURIComponent(url)}`, {
                signal: AbortSignal.timeout(10000),
            });
        }

        if (!response.ok) {
            console.warn(`Failed to fetch ${group}: ${response.status}`);
            return [];
        }

        const text = await response.text();
        const lines = text.split(/\r?\n/).filter(line => line.trim());

        // Parse TLE triplets
        for (let i = 0; i < lines.length - 2; i += 3) {
            const name = lines[i];
            const line1 = lines[i + 1];
            const line2 = lines[i + 2];

            if (line1 && line2) {
                const sat = parseTLE({ name, line1, line2 }, forceCategory);
                if (sat && sat.altitude > 100 && sat.altitude < 2500) {
                    satellites.push(sat);
                }
            }
        }

        console.log(`✓ Fetched ${satellites.length} satellites from ${group}`);
        return satellites;
    } catch (error) {
        console.error(`Failed to fetch ${group}:`, error);
        return [];
    }
}

// Main fetch function - loads all satellite categories
export async function fetchAllLEOSatellites(): Promise<SatelliteData[]> {
    console.log('🛰️ Fetching real satellite TLE data from CelesTrak...');

    // Fetch all categories in parallel
    const results = await Promise.allSettled([
        fetchTLEGroup('stations', 'iss'),       // ISS, Tiangong, etc.
        fetchTLEGroup('starlink'),              // SpaceX Starlink
        fetchTLEGroup('oneweb'),                // OneWeb constellation
        fetchTLEGroup('iridium-NEXT', 'iridium'), // Iridium NEXT
        fetchTLEGroup('weather'),               // Weather satellites
        fetchTLEGroup('science'),               // Science satellites
        fetchTLEGroup('active'),                // Other active satellites
    ]);

    const allSatellites: SatelliteData[] = [];

    for (const result of results) {
        if (result.status === 'fulfilled' && result.value.length > 0) {
            allSatellites.push(...result.value);
        }
    }

    // Remove duplicates by NORAD ID
    const unique = new Map<number, SatelliteData>();
    for (const sat of allSatellites) {
        if (!unique.has(sat.noradId)) {
            unique.set(sat.noradId, sat);
        }
    }

    // Limit for performance but keep variety
    let result: SatelliteData[] = [];
    const byCategory = new Map<string, SatelliteData[]>();

    for (const sat of unique.values()) {
        if (!byCategory.has(sat.category)) {
            byCategory.set(sat.category, []);
        }
        byCategory.get(sat.category)!.push(sat);
    }

    // Take samples from each category
    const limits: Record<string, number> = {
        iss: 10,
        starlink: 40,
        oneweb: 25,
        iridium: 20,
        weather: 15,
        science: 15,
        other: 25,
    };

    for (const [category, sats] of byCategory) {
        const limit = limits[category] || 20;
        result.push(...sats.slice(0, limit));
    }

    // Log summary
    const summary = {
        total: result.length,
        iss: result.filter(s => s.category === 'iss').length,
        starlink: result.filter(s => s.category === 'starlink').length,
        oneweb: result.filter(s => s.category === 'oneweb').length,
        iridium: result.filter(s => s.category === 'iridium').length,
        weather: result.filter(s => s.category === 'weather').length,
        science: result.filter(s => s.category === 'science').length,
        other: result.filter(s => s.category === 'other').length,
    };

    console.log('📊 Satellite counts:', summary);

    if (result.length === 0) {
        console.warn('⚠️ No satellites loaded from API, using fallback data');
        return getSampleSatellites();
    }

    console.log(`✅ Loaded ${result.length} real satellites from CelesTrak TLE data`);
    return result;
}

// Calculate real-time satellite position from TLE orbital elements
export function calculateSatellitePosition(
    satellite: SatelliteData,
    elapsedSeconds: number
): { x: number; y: number; z: number; lat: number; lon: number } {
    const EARTH_RADIUS = 1;

    // Mean motion in radians per second
    const meanMotionRadPerSec = (satellite.meanMotion * 2 * Math.PI) / (24 * 60 * 60);

    // Current position along orbit
    const currentMeanAnomaly = (satellite.meanAnomaly * Math.PI / 180) + (meanMotionRadPerSec * elapsedSeconds);
    const trueAnomaly = currentMeanAnomaly; // Simplified for circular orbit

    // Orbital elements in radians
    const incRad = satellite.inclination * Math.PI / 180;
    const raanRad = satellite.raan * Math.PI / 180;
    const argPerigeeRad = satellite.argOfPerigee * Math.PI / 180;

    const u = trueAnomaly + argPerigeeRad;

    // Altitude scaled for visualization
    const altitudeScale = satellite.altitude / 6371;
    const r = EARTH_RADIUS * (1 + altitudeScale * 0.15);

    // Earth rotation (sidereal day)
    const earthRotationRate = (2 * Math.PI) / (23.9344696 * 60 * 60);
    const gmst = earthRotationRate * elapsedSeconds;
    const adjustedRaan = raanRad - gmst;

    // Transform to 3D coordinates
    const x = r * (Math.cos(adjustedRaan) * Math.cos(u) - Math.sin(adjustedRaan) * Math.sin(u) * Math.cos(incRad));
    const y = r * Math.sin(u) * Math.sin(incRad);
    const z = r * (Math.sin(adjustedRaan) * Math.cos(u) + Math.cos(adjustedRaan) * Math.sin(u) * Math.cos(incRad));

    // Geographic coordinates
    const lat = Math.asin(Math.min(1, Math.max(-1, y / r))) * 180 / Math.PI;
    const lon = Math.atan2(z, x) * 180 / Math.PI;

    return { x, y, z, lat, lon };
}

// Fallback sample data for offline/error mode
export function getSampleSatellites(): SatelliteData[] {
    const samples: SatelliteData[] = [
        // ISS and Stations
        { id: 'iss', name: 'ISS (ZARYA)', noradId: 25544, intlDesignator: '98067A', epochYear: 2024, epochDay: 1, inclination: 51.6, raan: 100, eccentricity: 0.0004, argOfPerigee: 45, meanAnomaly: 315, meanMotion: 15.5, altitude: 408, period: 92.68, category: 'iss' },
        { id: 'tiangong', name: 'CSS (TIANHE)', noradId: 48274, intlDesignator: '21035A', epochYear: 2024, epochDay: 1, inclination: 41.5, raan: 200, eccentricity: 0.0002, argOfPerigee: 120, meanAnomaly: 240, meanMotion: 15.58, altitude: 390, period: 92.3, category: 'iss' },
    ];

    // Generate varied Starlink
    for (let i = 0; i < 15; i++) {
        samples.push({
            id: `starlink-${i}`,
            name: `STARLINK-${1000 + i * 100}`,
            noradId: 44700 + i,
            intlDesignator: `20001${String.fromCharCode(65 + i)}`,
            epochYear: 2024, epochDay: 1,
            inclination: 53 + (i % 3),
            raan: (i * 25) % 360,
            eccentricity: 0.0001,
            argOfPerigee: (i * 45) % 360,
            meanAnomaly: (i * 30) % 360,
            meanMotion: 15.06,
            altitude: 550,
            period: 95.6,
            category: 'starlink',
        });
    }

    // Iridium
    for (let i = 0; i < 8; i++) {
        samples.push({
            id: `iridium-${i}`,
            name: `IRIDIUM ${100 + i * 6}`,
            noradId: 43570 + i,
            intlDesignator: `18061${String.fromCharCode(65 + i)}`,
            epochYear: 2024, epochDay: 1,
            inclination: 86.4,
            raan: (i * 45) % 360,
            eccentricity: 0.0002,
            argOfPerigee: (i * 90) % 360,
            meanAnomaly: (i * 45) % 360,
            meanMotion: 14.34,
            altitude: 780,
            period: 100.4,
            category: 'iridium',
        });
    }

    // OneWeb
    for (let i = 0; i < 8; i++) {
        samples.push({
            id: `oneweb-${i}`,
            name: `ONEWEB-${i * 20 + 10}`,
            noradId: 44050 + i,
            intlDesignator: `19010${String.fromCharCode(65 + i)}`,
            epochYear: 2024, epochDay: 1,
            inclination: 87.9,
            raan: (i * 40) % 360,
            eccentricity: 0.0001,
            argOfPerigee: (i * 60) % 360,
            meanAnomaly: (i * 50) % 360,
            meanMotion: 13.15,
            altitude: 1200,
            period: 109.5,
            category: 'oneweb',
        });
    }

    // Weather
    samples.push(
        { id: 'noaa-18', name: 'NOAA 18', noradId: 28654, intlDesignator: '05018A', epochYear: 2024, epochDay: 1, inclination: 98.7, raan: 45, eccentricity: 0.0012, argOfPerigee: 120, meanAnomaly: 240, meanMotion: 14.11, altitude: 854, period: 102.1, category: 'weather' },
        { id: 'noaa-19', name: 'NOAA 19', noradId: 33591, intlDesignator: '09005A', epochYear: 2024, epochDay: 1, inclination: 98.7, raan: 135, eccentricity: 0.0013, argOfPerigee: 220, meanAnomaly: 140, meanMotion: 14.12, altitude: 850, period: 102.0, category: 'weather' },
        { id: 'meteor-m2', name: 'METEOR-M 2', noradId: 40069, intlDesignator: '14037A', epochYear: 2024, epochDay: 1, inclination: 98.5, raan: 200, eccentricity: 0.0005, argOfPerigee: 80, meanAnomaly: 300, meanMotion: 14.21, altitude: 820, period: 101.3, category: 'weather' }
    );

    // Science
    samples.push(
        { id: 'landsat-8', name: 'LANDSAT 8', noradId: 39084, intlDesignator: '13008A', epochYear: 2024, epochDay: 1, inclination: 98.2, raan: 60, eccentricity: 0.0001, argOfPerigee: 90, meanAnomaly: 270, meanMotion: 14.57, altitude: 705, period: 98.9, category: 'science' },
        { id: 'terra', name: 'TERRA', noradId: 25994, intlDesignator: '99068A', epochYear: 2024, epochDay: 1, inclination: 98.2, raan: 150, eccentricity: 0.0001, argOfPerigee: 180, meanAnomaly: 180, meanMotion: 14.57, altitude: 705, period: 98.9, category: 'science' },
        { id: 'aqua', name: 'AQUA', noradId: 27424, intlDesignator: '02022A', epochYear: 2024, epochDay: 1, inclination: 98.2, raan: 220, eccentricity: 0.0002, argOfPerigee: 270, meanAnomaly: 90, meanMotion: 14.57, altitude: 705, period: 98.9, category: 'science' }
    );

    return samples;
}
