@echo off
REM Быстрая проверка RTL-SDR
REM Использование: sdr_check.bat

echo ========================================
echo   Быстрая проверка RTL-SDR
echo ========================================
echo.

echo [1/3] Проверка DLL...
if exist "C:\SDRSharp\tmp\x64\rtlsdr.dll" (
    echo [OK] rtlsdr.dll найден в C:\SDRSharp\tmp\x64\
) else if exist "C:\SDRSharp\rtlsdr.dll" (
    echo [OK] rtlsdr.dll найден в C:\SDRSharp\
) else (
    echo [ERROR] rtlsdr.dll не найден!
    echo Установите SDR# или скачайте rtlsdr.dll
    pause
    exit /b 1
)

echo.
echo [2/3] Тест устройства...
python scripts\test_rtlsdr.py

if errorlevel 1 (
    echo.
    echo [ERROR] Устройство не обнаружено!
    echo.
    echo Решение:
    echo 1. Проверьте подключение USB
    echo 2. Установите драйвер WinUSB через Zadig
    echo    https://zadig.akeo.ie/
    echo 3. Перезапустите скрипт
    pause
    exit /b 1
)

echo.
echo [3/3] Тест приёма...
echo Получение тестовых сэмплов на 100 МГц...
python scripts\sdr_capture_enhanced.py --freq 100.0 --samples 4096 --gain 30

echo.
echo ========================================
echo   Проверка завершена!
echo ========================================
pause
