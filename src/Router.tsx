/**
 * Router.tsx
 * Application routing configuration
 */

import { BrowserRouter, Routes, Route } from 'react-router-dom';
import App from './App';
import TrackingPage from './pages/TrackingPage';
import SDRPage from './pages/SDRPage';
import UplinkPage from './pages/UplinkPage';
import CubeSatChat from './components/CubeSatChat';

export default function Router() {
    return (
        <BrowserRouter>
            <Routes>
                <Route path="/" element={<App />} />
                <Route path="/tracking" element={<TrackingPage />} />
                <Route path="/sdr" element={<SDRPage />} />
                <Route path="/uplink" element={<UplinkPage />} />
            </Routes>
            <CubeSatChat />
        </BrowserRouter>
    );
}
