@echo off
echo --- REAL MODE START ---
SET PATH=C:\Program Files\UHD\bin;%PATH%

echo [1] Checking Hardware...
uhd_find_devices.exe

echo [2] Launching Python Server...
python usrp-bridge\usrp_server.py

echo.
echo --- CRASH OR TERMINATION DETECTED ---
pause
