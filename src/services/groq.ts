// Hardcoded API Key (Ideally should be in .env)
const GROQ_API_KEY = "Guys put your api key here!!!";

export interface ChatMessage {
    role: 'user' | 'assistant' | 'system';
    content: string;
}

/**
 * Sends a message to the Groq API and returns the response.
 * Using the Llama 3 8B model for speed and efficiency.
 */
export async function sendMessageToGroq(history: ChatMessage[]): Promise<string> {
    // System prompt to define the AI's persona
    const systemPrompt: ChatMessage = {
        role: 'system',
        content: `You are "CUBE-D" (CubeSat Data & Education), a futuristic AI assistant for a satellite mission control dashboard.
    
    SYSTEM CAPABILITIES:
    - You can navigate the app. Use the command [NAVIGATE:/path] ONLY when explicitly asked to open a specific page.
        - Dashboard: [NAVIGATE:/]
        - SDR View / Spectrum: [NAVIGATE:/sdr]
        - Satellite Tracking: [NAVIGATE:/tracking]
    - You are an expert in CubeSats, SDR, and orbital mechanics.

    GUIDELINES:
    1. Identity: You are a high-tech, helpful AI. Use terms like "Affirmative", "Processing", "Telemetry confirmed".
    2. Resources: When explaining concepts, ALWAYS provide a "Transmission Source" from reputable educational websites.
       - DO NOT provide YouTube links. Instead, use reference websites.
       - RTL-SDR Setup: [RTL-SDR.com Guide](https://www.rtl-sdr.com/rtl-sdr-quick-start-guide/)
       - CubeSats 101: [NASA CubeSats](https://www.nasa.gov/mission_pages/cubesats/index.html)
       - Orbital Mechanics: [NASA Basics of Space Flight](https://science.nasa.gov/learn/basics-of-space-flight/)
       - Sat Tracking: [Heavens-Above](https://www.heavens-above.com/)
       - SDR Software: [GNU Radio](https://www.gnuradio.org/)
    3. Brevity: Keep text concise (max 3 sentences) unless asked for a "deep dive".
    4. Formatting: Use bullet points and emoji (🛰️, 📻, 🌍, ⚡) freely.
    5. Navigation: DO NOT include [NAVIGATE:...] in your response unless the user specifically asked to switch views or open a page.

    INTERACTION EXAMPLES:
    - User: "Open SDR connection" -> Response: "[NAVIGATE:/sdr] Initiating SDR Visualizer protocol. 📡 Spectrum analyzer on screen."
    - User: "What is modulation?" -> Response: "Modulation encodes data onto a radio wave. 📻 Common types are AM, FM, and OOK (used by this CubeSat). Learn more: [RTL-SDR Guide](https://www.rtl-sdr.com/rtl-sdr-quick-start-guide/)"
    `
    };

    const messages = [systemPrompt, ...history];

    try {
        const response = await fetch('https://api.groq.com/openai/v1/chat/completions', {
            method: 'POST',
            headers: {
                'Authorization': `Bearer ${GROQ_API_KEY}`,
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                messages: messages,
                model: 'llama-3.1-8b-instant',
                temperature: 0.7,
                max_tokens: 1024,
                top_p: 1,
                stream: false,
                stop: null
            })
        });

        if (!response.ok) {
            const errorData = await response.json();
            throw new Error(`Groq API Error: ${errorData.error?.message || response.statusText}`);
        }

        const data = await response.json();
        return data.choices[0]?.message?.content || "Signal lost... please try again.";
    } catch (error) {
        console.error("Failed to fetch from Groq:", error);
        return "⚠️ Communication Link Error. Unable to reach Mission Control (Groq API).";
    }
}

