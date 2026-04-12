@echo off
REM Быстрый запуск SDR скриптов
REM Использование: sdr.bat [частота] [усиление]
REM Пример: sdr.bat 145.8 40

set FREQ=%1
set GAIN=%2

if "%FREQ%"=="" (
    echo ========================================
    echo   Быстрый запуск SDR
    echo ========================================
    echo.
    echo Использование: sdr.bat [частота] [усиление]
    echo.
    echo Примеры:
    echo   sdr.bat 100.0        FM радио
    echo   sdr.bat 145.8 40     ISS
    echo   sdr.bat 144.8        APRS
    echo   sdr.bat 433.92 30    ISM 70cm
    echo.
    exit /b 1
)

if "%GAIN%"=="" (
    set GAIN=0
)

echo Запуск приёма на частоте %FREQ% МГц с усилением %GAIN% dB...
python scripts\sdr_capture_enhanced.py --freq %FREQ% --gain %GAIN% --plot --peaks --snr

pause
