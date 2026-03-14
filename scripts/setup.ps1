# MKA Embedded - Установка зависимостей
# Скрипт для Windows PowerShell

$ErrorActionPreference = "Stop"

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  MKA Embedded - Установка зависимостей" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Проверка Chocolatey
if (-not (Get-Command "choco" -ErrorAction SilentlyContinue)) {
    Write-Host "Chocolatey не найден. Устанавливаем..." -ForegroundColor Yellow
    Set-ExecutionPolicy Bypass -Scope Process -Force
    [System.Net.ServicePointManager]::SecurityProtocol = `
        [System.Net.ServicePointManager]::SecurityProtocol -bor 3072
    iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
}

# Установка Ninja
& "$PSScriptRoot\install_ninja.ps1"

# Проверка CMake
if (-not (Get-Command "cmake" -ErrorAction SilentlyContinue)) {
    Write-Host "Установка CMake..." -ForegroundColor Cyan
    choco install cmake -y --no-progress
} else {
    $cmakeVersion = cmake --version | Select-Object -First 1
    Write-Host "$cmakeVersion уже установлен" -ForegroundColor Green
}

# Проверка GCC (MinGW)
if (-not (Get-Command "g++" -ErrorAction SilentlyContinue)) {
    Write-Host "Установка MinGW..." -ForegroundColor Cyan
    choco install mingw -y --no-progress
} else {
    $gccVersion = g++ --version | Select-Object -First 1
    Write-Host "GCC: $gccVersion" -ForegroundColor Green
}

# Python зависимости
if (Get-Command "python" -ErrorAction SilentlyContinue) {
    Write-Host ""
    Write-Host "Установка Python зависимостей..." -ForegroundColor Cyan
    python -m pip install --upgrade pip
    python -m pip install -r "$PSScriptRoot\..\src\python\requirements.txt"
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Green
Write-Host "  Готово!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host ""
Write-Host "Проверка версий:" -ForegroundColor Cyan
cmake --version | Select-Object -First 1
ninja --version
Write-Host ""
Write-Host "Для сборки:" -ForegroundColor Cyan
Write-Host "  cmake --preset=host-debug"
Write-Host "  cmake --build build/host-debug"
Write-Host ""
