/**
 * LogConsole.tsx
 * System event log display - Enhanced UI with color-coded messages
 */

import { useEffect, useRef, useState } from 'react';
import { Terminal, ChevronDown, Filter } from 'lucide-react';
import { useSimulationStore } from '../store/simulationStore';
import { cn } from '../utils/cn';

type LogLevel = 'info' | 'warn' | 'error' | 'success' | 'system';

function getLogLevel(message: string): LogLevel {
  const lowerMsg = message.toLowerCase();
  if (lowerMsg.includes('error') || lowerMsg.includes('fail')) return 'error';
  if (lowerMsg.includes('warn') || lowerMsg.includes('dropped') || lowerMsg.includes('lost')) return 'warn';
  if (lowerMsg.includes('success') || lowerMsg.includes('complete') || lowerMsg.includes('target')) return 'success';
  if (lowerMsg.includes('init') || lowerMsg.includes('start') || lowerMsg.includes('stop') || lowerMsg.includes('reset')) return 'system';
  return 'info';
}

function getLogStyles(level: LogLevel) {
  switch (level) {
    case 'error':
      return {
        badge: 'bg-red-500/20 text-red-400 border-red-500/30',
        text: 'text-red-300',
        dot: 'bg-red-400',
        label: 'ERR'
      };
    case 'warn':
      return {
        badge: 'bg-amber-500/20 text-amber-400 border-amber-500/30',
        text: 'text-amber-300',
        dot: 'bg-amber-400',
        label: 'WRN'
      };
    case 'success':
      return {
        badge: 'bg-emerald-500/20 text-emerald-400 border-emerald-500/30',
        text: 'text-emerald-300',
        dot: 'bg-emerald-400',
        label: 'OK'
      };
    case 'system':
      return {
        badge: 'bg-cyan-500/20 text-cyan-400 border-cyan-500/30',
        text: 'text-cyan-300',
        dot: 'bg-cyan-400',
        label: 'SYS'
      };
    default:
      return {
        badge: 'bg-white/10 text-gray-400 border-white/10',
        text: 'text-gray-400',
        dot: 'bg-gray-400',
        label: 'INF'
      };
  }
}

export default function LogConsole() {
  const logs = useSimulationStore((state) => state.logs);
  const consoleRef = useRef<HTMLDivElement>(null);
  const [autoScroll, setAutoScroll] = useState(true);
  const [filter, setFilter] = useState<LogLevel | 'all'>('all');
  
  useEffect(() => {
    if (consoleRef.current && autoScroll) {
      consoleRef.current.scrollTop = consoleRef.current.scrollHeight;
    }
  }, [logs, autoScroll]);
  
  const handleScroll = () => {
    if (consoleRef.current) {
      const { scrollTop, scrollHeight, clientHeight } = consoleRef.current;
      const isAtBottom = scrollHeight - scrollTop - clientHeight < 50;
      setAutoScroll(isAtBottom);
    }
  };

  const filteredLogs = filter === 'all' 
    ? logs 
    : logs.filter(log => getLogLevel(log.message) === filter);
  
  return (
    <div className="w-full h-full glass-panel rounded-xl overflow-hidden flex flex-col">
      {/* Header */}
      <div className="px-4 py-2 border-b border-white/5 bg-black/20 flex items-center justify-between">
        <div className="flex items-center gap-2">
          <div className="p-1.5 rounded-lg bg-emerald-500/10">
            <Terminal className="w-3.5 h-3.5 text-emerald-400" />
          </div>
          <span className="text-xs font-medium text-white">System Log</span>
          <span className="text-[9px] font-mono text-gray-500 bg-black/30 px-1.5 py-0.5 rounded">
            {filteredLogs.length} entries
          </span>
        </div>
        
        <div className="flex items-center gap-2">
          {/* Filter dropdown */}
          <div className="relative group">
            <button className="flex items-center gap-1 px-2 py-1 text-[10px] font-mono text-gray-400 bg-black/30 hover:bg-black/50 border border-white/10 rounded-lg transition-colors">
              <Filter className="w-3 h-3" />
              {filter === 'all' ? 'All' : filter.toUpperCase()}
              <ChevronDown className="w-3 h-3" />
            </button>
            <div className="absolute right-0 mt-1 w-24 bg-black/90 backdrop-blur-sm border border-white/10 rounded-lg overflow-hidden opacity-0 invisible group-hover:opacity-100 group-hover:visible transition-all z-10">
              {(['all', 'info', 'warn', 'error', 'success', 'system'] as const).map((level) => (
                <button
                  key={level}
                  onClick={() => setFilter(level)}
                  className={cn(
                    "w-full px-3 py-1.5 text-left text-[10px] font-mono hover:bg-white/5 transition-colors",
                    filter === level ? "text-cyan-400 bg-cyan-500/10" : "text-gray-400"
                  )}
                >
                  {level === 'all' ? 'All Logs' : level.toUpperCase()}
                </button>
              ))}
            </div>
          </div>
          
          {/* Auto-scroll indicator */}
          <button
            onClick={() => setAutoScroll(!autoScroll)}
            className={cn(
              "p-1.5 rounded-lg border transition-colors",
              autoScroll 
                ? "bg-emerald-500/10 border-emerald-500/30 text-emerald-400"
                : "bg-black/30 border-white/10 text-gray-500"
            )}
            title={autoScroll ? "Auto-scroll ON" : "Auto-scroll OFF"}
          >
            <ChevronDown className="w-3 h-3" />
          </button>
        </div>
      </div>
      
      {/* Log content */}
      <div
        ref={consoleRef}
        onScroll={handleScroll}
        className="flex-1 overflow-y-auto scrollbar-thin scrollbar-thumb-white/10 scrollbar-track-transparent"
      >
        {filteredLogs.length === 0 ? (
          <div className="h-full flex flex-col items-center justify-center text-gray-500">
            <Terminal className="w-6 h-6 mb-2 opacity-50" />
            <span className="text-xs font-mono">
              {filter === 'all' ? 'Waiting for simulation data...' : `No ${filter} logs`}
            </span>
          </div>
        ) : (
          <div className="p-2 space-y-0.5">
            {filteredLogs.map((log, index) => {
              const level = getLogLevel(log.message);
              const styles = getLogStyles(level);
              
              return (
                <div 
                  key={index} 
                  className="flex items-start gap-2 px-2 py-1.5 rounded hover:bg-white/5 transition-colors group"
                >
                  {/* Level badge */}
                  <span className={cn(
                    "text-[8px] font-mono px-1.5 py-0.5 rounded border shrink-0 mt-0.5",
                    styles.badge
                  )}>
                    {styles.label}
                  </span>
                  
                  {/* Timestamp */}
                  <span className="text-[10px] font-mono text-gray-600 shrink-0">
                    {log.timestamp.toLocaleTimeString('en-US', { 
                      hour12: false,
                      hour: '2-digit',
                      minute: '2-digit',
                      second: '2-digit'
                    })}
                    <span className="text-gray-700">
                      .{log.timestamp.getMilliseconds().toString().padStart(3, '0')}
                    </span>
                  </span>
                  
                  {/* Message */}
                  <span className={cn("text-[11px] font-mono flex-1", styles.text)}>
                    {log.message}
                  </span>
                </div>
              );
            })}
          </div>
        )}
      </div>
      
      {/* Footer */}
      <div className="px-3 py-1.5 border-t border-white/5 bg-black/20 flex items-center justify-between">
        <span className="text-[9px] font-mono text-gray-600">
          {autoScroll ? '● Live' : '○ Paused'}
        </span>
        <span className="text-[9px] font-mono text-gray-600">
          {logs.length} total • {filteredLogs.length} shown
        </span>
      </div>
    </div>
  );
}
