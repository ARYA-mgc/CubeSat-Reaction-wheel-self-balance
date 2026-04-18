@echo off
echo --- HARDWARE DISCOVERY TEST ---
SET PATH=C:\Program Files\UHD\bin;%PATH%
"C:\Program Files\UHD\bin\uhd_find_devices.exe"
echo.
echo Return Code: %ERRORLEVEL%
echo --- END OF TEST ---
pause
