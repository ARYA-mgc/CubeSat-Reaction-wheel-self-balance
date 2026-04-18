/**
 * App.tsx
 * Main application component - Mission Control Dashboard
 */

import Toolbar from './components/Toolbar';
import ControlPanel from './components/ControlPanel';
import SatVisualizer from './components/SatVisualizer';
import TelemetryStream from './components/TelemetryStream';
import StatusPanel from './components/StatusPanel';
import LogConsole from './components/LogConsole';

function App() {
  return (
    <div className="min-h-screen bg-mission-bg text-white overflow-hidden relative">
      {/* Toolbar */}
      <Toolbar />

      {/* Main Grid Layout */}
      <div className="h-[calc(100vh-72px)] grid grid-cols-[300px_1fr_300px] grid-rows-[1fr_260px] gap-3 p-3">
        {/* Left Panel - Control Settings */}
        <div className="row-span-2 overflow-hidden">
          <ControlPanel />
        </div>

        {/* Center Top - 3D Visualizer */}
        <div className="relative overflow-hidden rounded-xl">
          <SatVisualizer />
        </div>

        {/* Right Panel - Status */}
        <div className="row-span-2 overflow-hidden">
          <StatusPanel />
        </div>

        {/* Center Bottom - Split: Telemetry Charts & Log */}
        <div className="grid grid-cols-[1fr_350px] gap-3 overflow-hidden">
          <TelemetryStream />
          <LogConsole />
        </div>
      </div>
    </div>
  );
}

export default App;
