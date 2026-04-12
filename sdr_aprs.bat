@echo off
REM Приём APRS пакетов
REM Использование: sdr_aprs.bat

echo ========================================
echo   Приём APRS пакетов
echo ========================================
echo.

python scripts\sdr_ax25.py --decode --monitor

pause
