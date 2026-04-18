CUBESAT SIMULATION - RUN INSTRUCTIONS

STEP 1: SETUP
1. Open the "Web_Simulation" folder in VS Code.
2. Open a terminal and run: npm install
   (This downloads all the missing libraries)

STEP 2: UPLOAD ARDUINO
1. Open the code in "Arduino_Code".
2. IMPORTANT: Change the "pcIp" line to YOUR laptop's IP address!
3. Upload to ESP32.

STEP 3: RUN THE BRIDGE (Terminal 1)
node udp-bridge/server.cjs

STEP 4: RUN THE APP (Terminal 2)
npm run dev

STEP 5: VISUALIZE
Open http://localhost:5173
Turn on "IMU Mode"