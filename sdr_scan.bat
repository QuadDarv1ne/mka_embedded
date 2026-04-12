@echo off
REM Сканер спектра RTL-SDR
REM Использование: sdr_scan.bat [начало] [конец] [шаг]

set START=%1
set STOP=%2
set STEP=%3

if "%START%"=="" (
    set START=88
)

if "%STOP%"=="" (
    set STOP=108
)

if "%STEP%"=="" (
    set STEP=0.1
)

echo ========================================
echo   Сканер спектра SDR
echo ========================================
echo.
echo Сканирование: %START% - %STOP% МГц (шаг %STEP% МГц)
echo.

python scripts\sdr_scanner.py --start %START% --stop %STOP% --step %STEP% --plot

pause
