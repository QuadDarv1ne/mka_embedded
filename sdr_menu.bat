@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

:menu
cls
echo ================================================
echo         MKA Embedded - SDR Launcher
echo ================================================
echo.
echo  Выберите действие:
echo.
echo  1.  Проверка RTL-SDR
echo  2.  Универсальный приём сигналов
echo  3.  Приём спутников NOAA
echo  4.  Приём APRS пакетов
echo  5.  Сканирование спектра
echo  6.  Калькулятор антенн
echo  7.  Быстрый приём ISS
echo  8.  Быстрый приём FM радио
echo  9.  Быстрый приём NOAA 19
echo  10. Выход
echo.
echo ================================================

set /p choice="Введите номер (1-10): "

if "%choice%"=="1" goto check
if "%choice%"=="2" goto capture
if "%choice%"=="3" goto noaa
if "%choice%"=="4" goto aprs
if "%choice%"=="5" goto scan
if "%choice%"=="6" goto antenna
if "%choice%"=="7" goto iss
if "%choice%"=="8" goto fm
if "%choice%"=="9" goto noaa19
if "%choice%"=="10" goto end
echo Неверный выбор!
pause
goto menu

:check
echo.
call sdr_check.bat
goto menu

:capture
echo.
echo ================================================
echo   Универсальный приём сигналов
echo ================================================
echo.
set /p freq="Введите частоту (МГц): "
set /p gain="Введите усиление (0=авто): "
if "%gain%"=="" set gain=0
python scripts\sdr_capture_enhanced.py --freq %freq% --gain %gain% --plot --peaks --snr
pause
goto menu

:noaa
echo.
echo ================================================
echo   Приём спутников NOAA
echo ================================================
echo.
echo 1. NOAA 18 (137.9125 МГц)
echo 2. NOAA 19 (137.1000 МГц)
echo.
set /p sat_choice="Выберите спутник (1-2): "
if "%sat_choice%"=="1" (
    python scripts\sdr_noaa.py --sat noaa18 --gain 40 --scan
) else if "%sat_choice%"=="2" (
    python scripts\sdr_noaa.py --sat noaa19 --gain 40 --scan
) else (
    echo Неверный выбор!
)
pause
goto menu

:aprs
echo.
echo ================================================
echo   Приём APRS пакетов
echo ================================================
echo.
python scripts\sdr_ax25.py --decode --monitor
pause
goto menu

:scan
echo.
echo ================================================
echo   Сканирование спектра
echo ================================================
echo.
set /p start="Начальная частота (МГц): "
set /p stop="Конечная частота (МГц): "
if "%start%"=="" set start=88
if "%stop%"=="" set stop=108
python scripts\sdr_scanner.py --start %start% --stop %stop% --plot
pause
goto menu

:antenna
echo.
echo ================================================
echo   Калькулятор антенн
echo ================================================
echo.
set /p freq="Введите частоту (МГц): "
python scripts\sdr_antenna.py --freq %freq%
pause
goto menu

:iss
echo.
echo ================================================
echo   Быстрый приём ISS (145.800 МГц)
echo ================================================
echo.
echo Примечание: ISS должен быть в зоне видимости!
echo Отслеживайте через heavens-above.com
echo.
python scripts\sdr_capture_enhanced.py --freq 145.8 --gain 40 --plot --peaks --snr
pause
goto menu

:fm
echo.
echo ================================================
echo   Быстрый приём FM радио (100.0 МГц)
echo ================================================
echo.
python scripts\sdr_capture_enhanced.py --freq 100.0 --gain 30 --plot --wav fm_radio.wav
echo.
if exist fm_radio.wav (
    echo [OK] Аудио сохранено в fm_radio.wav
    echo Откройте файл в любом медиаплеере
)
pause
goto menu

:noaa19
echo.
echo ================================================
echo   Быстрый приём NOAA 19 (137.1000 МГц)
echo ================================================
echo.
python scripts\sdr_noaa.py --sat noaa19 --gain 40 --output noaa19.bin
pause
goto menu

:end
echo.
echo Завершение работы...
echo Спасибо за использование MKA Embedded SDR!
echo.
exit /b 0
