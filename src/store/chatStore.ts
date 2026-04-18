import { create } from 'zustand';
import { ChatMessage, sendMessageToGroq } from '../services/groq';

interface ChatState {
    isOpen: boolean;
    messages: ChatMessage[];
    isLoading: boolean;
    toggleChat: () => void;
    openChat: () => void;
    closeChat: () => void;
    addMessage: (content: string, role: 'user' | 'assistant') => void;
    sendMessage: (content: string) => Promise<void>;
    clearChat: () => void;
}

export const useChatStore = create<ChatState>((set, get) => ({
    isOpen: false, // Start closed
    messages: [
        { role: 'assistant', content: "Hello! Mission Control here. 🛰️ I can help you with CubeSat operations, telemetry analysis, or general satellite questions. What's on your mind?" }
    ],
    isLoading: false,

    toggleChat: () => set((state) => ({ isOpen: !state.isOpen })),
    openChat: () => set({ isOpen: true }),
    closeChat: () => set({ isOpen: false }),

    addMessage: (content, role) => set((state) => ({
        messages: [...state.messages, { role, content }]
    })),

    sendMessage: async (content) => {
        const { addMessage } = get();

        // add user message
        addMessage(content, 'user');
        set({ isLoading: true });

        // prepare history for API (exclude the optimistic user message we just added to state to avoid duplication if we re-read state, 
        // but actually we need to pass it to the API. 
        // The 'messages' in state already has the new user message because addMessage runs synchronously)
        const currentHistory = get().messages;

        try {
            const response = await sendMessageToGroq(currentHistory);
            addMessage(response, 'assistant');
        } catch (error) {
            addMessage("⚠️ Error: Failed to receive transmission.", 'assistant');
        } finally {
            set({ isLoading: false });
        }
    },

    clearChat: () => set({
        messages: [{ role: 'assistant', content: "Chat cleared. Ready for new instructions. 🛰️" }]
    })
}));
