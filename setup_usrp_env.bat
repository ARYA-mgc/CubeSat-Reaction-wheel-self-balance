@echo off
SET GRO_PYTHON="C:\GNURadio-3.10\python\python.exe"

echo [SETUP] Bootstrapping pip...
%GRO_PYTHON% -m ensurepip

echo [SETUP] Installing websockets and uhd...
%GRO_PYTHON% -m pip install websockets uhd

echo [SETUP] Done!
pause
