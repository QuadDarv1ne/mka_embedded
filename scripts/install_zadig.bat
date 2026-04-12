@echo off
echo === RTL-SDR V4 Zadig Installer ===
echo.
echo Downloading Zadig...
powershell -Command "Invoke-WebRequest -Uri 'https://github.com/pbatard/libwdi/releases/download/v1.5.0/zadig-2.9.exe' -OutFile '%TEMP%\zadig.exe'"
if exist "%TEMP%\zadig.exe" (
    echo Zadig downloaded: %TEMP%\zadig.exe
    echo.
    echo Starting Zadig...
    echo.
    echo Instructions:
    echo   1. Options -^> List All Devices
    echo   2. Select "RTL2838UHIDIR" or "RTL-SDR Blog V4"
    echo   3. Select driver "WinUSB"  
    echo   4. Click "Replace Driver"
    echo   5. Reconnect device
    echo.
    start "" "%TEMP%\zadig.exe"
) else (
    echo ERROR: Failed to download Zadig
    echo Download manually from: https://zadig.akeo.ie/
)
echo.
pause
