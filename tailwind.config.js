/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        mission: {
          bg: '#0a0a0a',
          panel: '#1a1a1a',
          border: 'rgba(255, 255, 255, 0.1)',
          accent: {
            cyan: '#00d4ff',
            green: '#00ff88',
            amber: '#ffaa00',
            red: '#ff4444',
          },
        },
      },
      fontFamily: {
        mono: ['JetBrains Mono', 'Courier New', 'monospace'],
        sans: ['Inter', 'system-ui', 'sans-serif'],
      },
    },
  },
  plugins: [],
}
