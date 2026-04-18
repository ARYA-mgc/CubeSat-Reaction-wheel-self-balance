/**
 * ThemeToggle.tsx
 * Creative animated theme toggle switch
 */

import { useThemeStore } from '../store/themeStore';
import { cn } from '../utils/cn';

export default function ThemeToggle() {
  const { theme, toggleTheme } = useThemeStore();
  const isDark = theme === 'dark';

  return (
    <button
      onClick={toggleTheme}
      className={cn(
        "relative w-16 h-8 rounded-full p-1 transition-all duration-500 ease-in-out",
        "border-2 overflow-hidden group",
        isDark 
          ? "bg-slate-900 border-slate-700 hover:border-cyan-500/50" 
          : "bg-sky-100 border-sky-300 hover:border-amber-400"
      )}
      title={`Switch to ${isDark ? 'light' : 'dark'} mode`}
    >
      {/* Background Stars (dark mode) */}
      <div className={cn(
        "absolute inset-0 transition-opacity duration-500",
        isDark ? "opacity-100" : "opacity-0"
      )}>
        {[...Array(6)].map((_, i) => (
          <div
            key={i}
            className="absolute w-0.5 h-0.5 bg-white rounded-full animate-pulse"
            style={{
              top: `${20 + Math.random() * 60}%`,
              left: `${10 + Math.random() * 40}%`,
              animationDelay: `${i * 0.3}s`,
            }}
          />
        ))}
      </div>

      {/* Background Clouds (light mode) */}
      <div className={cn(
        "absolute inset-0 transition-opacity duration-500",
        isDark ? "opacity-0" : "opacity-100"
      )}>
        <div className="absolute top-1 right-2 w-4 h-2 bg-white/80 rounded-full" />
        <div className="absolute top-3 right-4 w-3 h-1.5 bg-white/60 rounded-full" />
      </div>

      {/* Toggle Circle - Sun/Moon */}
      <div
        className={cn(
          "relative w-6 h-6 rounded-full transition-all duration-500 ease-in-out transform",
          isDark 
            ? "translate-x-0 bg-slate-300" 
            : "translate-x-8 bg-amber-400"
        )}
      >
        {/* Moon craters (dark mode) */}
        <div className={cn(
          "absolute inset-0 transition-opacity duration-300",
          isDark ? "opacity-100" : "opacity-0"
        )}>
          <div className="absolute top-1 left-1.5 w-1.5 h-1.5 bg-slate-400 rounded-full" />
          <div className="absolute top-3 left-3 w-1 h-1 bg-slate-400 rounded-full" />
          <div className="absolute top-1.5 right-1 w-0.5 h-0.5 bg-slate-400 rounded-full" />
        </div>

        {/* Sun rays (light mode) */}
        <div className={cn(
          "absolute inset-0 transition-opacity duration-300",
          isDark ? "opacity-0" : "opacity-100"
        )}>
          {[...Array(8)].map((_, i) => (
            <div
              key={i}
              className="absolute w-0.5 h-1.5 bg-amber-300 rounded-full origin-center"
              style={{
                top: '50%',
                left: '50%',
                transform: `translate(-50%, -50%) rotate(${i * 45}deg) translateY(-10px)`,
              }}
            />
          ))}
        </div>
      </div>
    </button>
  );
}
