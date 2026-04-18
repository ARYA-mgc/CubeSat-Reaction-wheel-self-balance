@echo off
REM CubeSat USRP Bridge Launcher - FINAL FIX
SET UHD_BIN=C:\Program Files\UHD\bin
SET PATH=%UHD_BIN%;%PATH%

echo ===========================================
echo   USRP HARDWARE: READY
echo ===========================================
echo.

echo [1] Running USRP Bridge...
REM We use the GNURadio Python because it definitely has the library files
SET GRO_PYTHON="C:\GNURadio-3.10\python\python.exe"

IF EXIST %GRO_PYTHON% (
    %GRO_PYTHON% usrp-bridge\usrp_server.py
) ELSE (
    python usrp-bridge\usrp_server.py
)

echo.
echo ===========================================
echo  If you see "MOCK MODE" or "DLL FAILED":
echo  1. Close this window.
echo  2. Run: pip install uhd
echo  3. Run this launcher again.
echo ===========================================
pause
