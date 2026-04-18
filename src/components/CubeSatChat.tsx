import { useState, useRef, useEffect } from 'react';
import { Send, User, Bot, X, MessageSquare, Loader2, Minimize2, Zap, ExternalLink } from 'lucide-react';
import { useChatStore } from '../store/chatStore';
import { useNavigate } from 'react-router-dom';
import { clsx } from 'clsx';
import { twMerge } from 'tailwind-merge';

function cn(...inputs: (string | undefined | null | false)[]) {
    return twMerge(clsx(inputs));
}

export default function CubeSatChat() {
    const { isOpen, messages, isLoading, toggleChat, sendMessage, closeChat } = useChatStore();
    const [input, setInput] = useState('');
    const messagesEndRef = useRef<HTMLDivElement>(null);
    const navigate = useNavigate();
    const lastNavIndexRef = useRef<number>(-1);

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    };

    useEffect(() => {
        scrollToBottom();
    }, [messages, isOpen]);

    // Command Parsing Logic - Fixed to prevent navigation loops
    useEffect(() => {
        const lastIdx = messages.length - 1;
        if (lastIdx < 0 || lastIdx <= lastNavIndexRef.current) return;

        const lastMsg = messages[lastIdx];
        if (lastMsg?.role === 'assistant' && lastMsg.content.includes('[NAVIGATE:')) {
            const match = lastMsg.content.match(/\[NAVIGATE:(.*?)\]/);
            if (match && match[1]) {
                lastNavIndexRef.current = lastIdx; // Mark this message as processed
                console.log("CUBE-D Navigating to:", match[1]);
                navigate(match[1]);
            }
        }
    }, [messages, navigate]);

    const handleSubmit = async (e: React.FormEvent) => {
        e.preventDefault();
        if (!input.trim() || isLoading) return;
        const query = input;
        setInput('');
        await sendMessage(query);
    };

    const handleChipClick = (query: string) => {
        setInput('');
        sendMessage(query);
    };

    // Helper to remove command tags from display
    const cleanContent = (content: string) => {
        return content.replace(/\[NAVIGATE:.*?\]/g, '').trim();
    };

    // Improved Link Rendering
    const renderContent = (content: string) => {
        const cleaned = cleanContent(content);

        // Regex for markdown links: [text](url)
        const markdownLinkRegex = /\[([^\]]+)\]\((https?:\/\/[^\)]+)\)/g;
        // Regex for raw URLs (that aren't part of the markdown link above)
        // specific lookbehind/ahead is hard in JS regex across all browsers, so we'll split by space/newlines

        // Strategy: Split by markdown links first.
        const parts = cleaned.split(markdownLinkRegex);

        const elements: React.ReactNode[] = [];
        let i = 0;
        while (i < parts.length) {
            // Even indices are text (or empty)
            const textSegment = parts[i];
            if (textSegment) {
                // Process raw URLs in this text segment
                const urlRegex = /(https?:\/\/[^\s\)]+)/g;
                const subParts = textSegment.split(urlRegex);
                subParts.forEach((subPart, subIdx) => {
                    // Every odd subPart is a URL
                    if (subIdx % 2 === 1) {
                        elements.push(
                            <a key={`${i}-${subIdx}`} href={subPart} target="_blank" rel="noopener noreferrer" className="text-cyan-400 underline underline-offset-2 hover:text-cyan-200 break-all">
                                {subPart} <ExternalLink className="w-3 h-3 inline" />
                            </a>
                        );
                    } else {
                        elements.push(<span key={`${i}-${subIdx}`}>{subPart}</span>);
                    }
                });
            }

            // Odd indices are the captured groups from markdownRegex (Text, URL)
            if (i + 2 < parts.length) {
                const linkText = parts[i + 1];
                const linkUrl = parts[i + 2];
                elements.push(
                    <a key={`md-${i}`} href={linkUrl} target="_blank" rel="noopener noreferrer" className="text-cyan-300 font-semibold hover:text-white transition-colors border-b border-cyan-500/50 hover:border-cyan-400 flex items-center inline-flex gap-1 group">
                        {linkText} <ExternalLink className="w-3 h-3 group-hover:-translate-y-0.5 transition-transform" />
                    </a>
                );
                i += 2; // Skip the two captured groups
            }
            i++;
        }

        return elements;
    };

    return (
        <div className="fixed bottom-6 right-6 z-50 flex flex-col items-end pointer-events-none font-sans">
            {/* Chat Window */}
            <div
                className={cn(
                    "w-[400px] h-[600px] bg-[#0A0F1C]/95 backdrop-blur-xl border border-cyan-500/30 rounded-2xl shadow-[0_0_30px_rgba(6,182,212,0.15)] flex flex-col overflow-hidden transition-all duration-300 ease-in-out origin-bottom-right pointer-events-auto",
                    isOpen ? "scale-100 opacity-100 translate-y-0" : "scale-0 opacity-0 translate-y-10 h-0 w-0"
                )}
            >
                {/* Header - Sci-Fi Style */}
                <div className="relative p-4 overflow-hidden group">
                    <div className="absolute inset-0 bg-gradient-to-r from-cyan-900/40 to-blue-900/40"></div>
                    <div className="absolute bottom-0 left-0 right-0 h-px bg-gradient-to-r from-transparent via-cyan-500/50 to-transparent"></div>

                    <div className="relative flex justify-between items-center z-10">
                        <div className="flex items-center gap-3">
                            <div className="relative">
                                <div className="absolute inset-0 bg-cyan-500 rounded-lg animate-pulse opacity-20 blur-sm"></div>
                                <div className="bg-cyan-950/80 border border-cyan-500/30 p-2 rounded-lg relative">
                                    <Zap className="w-5 h-5 text-cyan-400" />
                                </div>
                            </div>
                            <div>
                                <h3 className="font-bold text-white text-base tracking-widest font-mono">CUBE-D</h3>
                                <div className="flex items-center gap-2">
                                    <span className="w-1.5 h-1.5 rounded-full bg-cyan-400 animate-pulse shadow-[0_0_8px_rgba(34,211,238,0.8)]"></span>
                                    <span className="text-[10px] text-cyan-300/80 uppercase tracking-wider">System Online</span>
                                </div>
                            </div>
                        </div>
                        <button
                            onClick={closeChat}
                            className="p-2 hover:bg-cyan-950/50 rounded-lg transition-colors text-cyan-400/70 hover:text-cyan-200"
                        >
                            <X className="w-5 h-5" />
                        </button>
                    </div>
                </div>

                {/* Messages */}
                <div className="flex-1 overflow-y-auto p-4 space-y-5 custom-scrollbar bg-gradient-to-b from-[#0A0F1C] to-[#060910]">
                    {messages.length === 0 && (
                        <div className="text-center py-10 opacity-50">
                            <Bot className="w-12 h-12 mx-auto text-cyan-500/50 mb-4" />
                            <p className="text-cyan-400/50 text-sm">Awaiting Input...</p>
                        </div>
                    )}

                    {messages.map((msg, idx) => {
                        return (
                            <div
                                key={idx}
                                className={cn(
                                    "flex gap-3 max-w-[90%] animate-in fade-in slide-in-from-bottom-2 duration-300 fill-mode-forwards",
                                    msg.role === 'user' ? "ml-auto flex-row-reverse" : ""
                                )}
                            >
                                <div className={cn(
                                    "w-8 h-8 rounded-lg flex items-center justify-center shrink-0 border shadow-lg",
                                    msg.role === 'user'
                                        ? "bg-blue-600/20 border-blue-400/30"
                                        : "bg-cyan-600/20 border-cyan-400/30"
                                )}>
                                    {msg.role === 'user' ? <User className="w-4 h-4 text-blue-300" /> : <Bot className="w-4 h-4 text-cyan-300" />}
                                </div>

                                <div className={cn(
                                    "p-3 rounded-xl text-sm leading-relaxed shadow-lg backdrop-blur-sm",
                                    msg.role === 'user'
                                        ? "bg-blue-600/20 text-blue-50 border border-blue-500/20 rounded-tr-none"
                                        : "bg-cyan-950/40 text-cyan-50 border border-cyan-500/20 rounded-tl-none shadow-[0_4px_20px_-10px_rgba(6,182,212,0.15)]"
                                )}>
                                    {msg.role === 'assistant' && idx === 0 && (
                                        <span className="block mb-2 font-mono text-[10px] text-cyan-500 uppercase tracking-wider border-b border-cyan-500/20 pb-1">
                                            Initialization Sequence Complete
                                        </span>
                                    )}
                                    {/* Typing effect only for the latest AI message if we want, but simple render is safer for links.
                                    Let's just use smooth CSS animations for entry.
                                    If we type, links break during typing. Let's stick to processed content for stability.
                                */}
                                    <div className="whitespace-pre-wrap">
                                        {renderContent(msg.content)}
                                    </div>
                                </div>
                            </div>
                        );
                    })}

                    {isLoading && (
                        <div className="flex gap-3 max-w-[85%]">
                            <div className="w-8 h-8 rounded-lg bg-cyan-600/20 border border-cyan-400/30 flex items-center justify-center shrink-0">
                                <Bot className="w-4 h-4 text-cyan-300" />
                            </div>
                            <div className="bg-cyan-950/30 border border-cyan-500/20 p-3 rounded-xl rounded-tl-none flex items-center gap-2">
                                <Loader2 className="w-4 h-4 text-cyan-400 animate-spin" />
                                <span className="text-xs text-cyan-400/70 font-mono animate-pulse">Processing...</span>
                            </div>
                        </div>
                    )}
                    <div ref={messagesEndRef} />
                </div>

                {/* Suggested Actions Chips */}
                {!isLoading && messages.length < 3 && (
                    <div className="px-4 pb-2 flex gap-2 overflow-x-auto no-scrollbar mask-gradient">
                        {[
                            "Open SDR View 📡",
                            "Explain Telemetry 🔢",
                            "What is RTL-SDR? 📻",
                            "Start Simulation 🚀"
                        ].map((chip) => (
                            <button
                                key={chip}
                                onClick={() => handleChipClick(chip)}
                                className="whitespace-nowrap px-3 py-1.5 bg-cyan-950/30 hover:bg-cyan-900/50 border border-cyan-500/20 hover:border-cyan-400/50 rounded-full text-xs text-cyan-300 transition-all cursor-pointer hover:scale-105 active:scale-95 transform"
                            >
                                {chip}
                            </button>
                        ))}
                    </div>
                )}

                {/* Input Area */}
                <div className="p-4 bg-[#0A0F1C] border-t border-cyan-500/20 relative z-20">
                    <form onSubmit={handleSubmit} className="flex gap-2">
                        <div className="relative flex-1">
                            <input
                                type="text"
                                value={input}
                                onChange={(e) => setInput(e.target.value)}
                                placeholder="Enter command or query..."
                                className="w-full bg-[#060910] border border-cyan-500/20 rounded-xl px-4 py-3 text-sm text-cyan-50 placeholder-cyan-700/50 focus:outline-none focus:border-cyan-500/50 focus:shadow-[0_0_15px_-3px_rgba(6,182,212,0.2)] transition-all font-mono"
                            />
                            <div className="absolute right-2 top-2 px-1.5 py-0.5 bg-cyan-900/20 rounded text-[9px] text-cyan-600/50 font-mono pointer-events-none mt-1">
                                AI-8B
                            </div>
                        </div>
                        <button
                            type="submit"
                            disabled={isLoading || !input.trim()}
                            className="group bg-cyan-600 hover:bg-cyan-500 disabled:bg-cyan-900/20 text-white p-3 rounded-xl disabled:opacity-50 disabled:cursor-not-allowed transition-all transform active:scale-95 shadow-[0_0_15px_rgba(8,145,178,0.4)]"
                        >
                            <Send className="w-5 h-5 group-hover:translate-x-0.5 group-hover:-translate-y-0.5 transition-transform" />
                        </button>
                    </form>
                </div>
            </div>

            {/* Floating Toggle Button */}
            <button
                onClick={toggleChat}
                className={cn(
                    "mt-4 p-4 rounded-xl shadow-[0_0_20px_rgba(6,182,212,0.3)] transition-all duration-300 transform hover:scale-105 active:scale-95 pointer-events-auto border relative overflow-hidden group",
                    isOpen
                        ? "bg-[#0A0F1C] border-cyan-500/50 text-cyan-400 rotate-90"
                        : "bg-cyan-600 border-cyan-400 text-white hover:shadow-cyan-500/50"
                )}
            >
                <div className="absolute inset-0 bg-gradient-to-tr from-cyan-400/0 via-white/10 to-transparent opacity-0 group-hover:opacity-100 transition-opacity"></div>
                {isOpen ? <Minimize2 className="w-6 h-6" /> : <MessageSquare className="w-6 h-6" />}
            </button>
        </div>
    );
}
