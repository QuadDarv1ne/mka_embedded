@echo off
REM Приём спутников NOAA
REM Использование: sdr_noaa.bat [спутник]

set SAT=%1

if "%SAT%"=="" (
    set SAT=noaa19
)

echo ========================================
echo   Приём спутника NOAA
echo ========================================
echo.
echo Спутник: %SAT%
echo.

python scripts\sdr_noaa.py --sat %SAT% --gain 40 --scan

pause
