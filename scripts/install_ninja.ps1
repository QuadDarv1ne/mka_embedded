#!/usr/bin/env pwsh
# Установка Ninja build system для Windows

param(
    [switch]$Force
)

$ErrorActionPreference = "Stop"
$NinjaVersion = "1.13.2"
$NinjaDir = "$env:ChocolateyInstall\lib\ninja\tools\ninja"
$NinjaExe = "$NinjaDir\ninja.exe"

Write-Host "=== Установка Ninja $NinjaVersion ===" -ForegroundColor Cyan

# Проверка установки Chocolatey
if (-not (Get-Command "choco" -ErrorAction SilentlyContinue)) {
    Write-Host "Chocolatey не найден. Устанавливаем..." -ForegroundColor Yellow
    Set-ExecutionPolicy Bypass -Scope Process -Force
    [System.Net.ServicePointManager]::SecurityProtocol = `
        [System.Net.ServicePointManager]::SecurityProtocol -bor 3072
    iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
}

# Проверка установленной версии
if (Test-Path $NinjaExe) {
    $currentVersion = & $NinjaExe --version
    Write-Host "Ninja $currentVersion уже установлен" -ForegroundColor Green
    
    if ($Force) {
        Write-Host "Принудительная переустановка..." -ForegroundColor Yellow
        choco uninstall ninja -y
    } else {
        exit 0
    }
}

# Установка через Chocolatey
Write-Host "Установка Ninja через Chocolatey..." -ForegroundColor Cyan
choco install ninja -y --no-progress

# Распаковка архива (если нужно)
if (-not (Test-Path $NinjaExe)) {
    $zipPath = "$env:ChocolateyInstall\lib\ninja\tools\ninja-win_x32.zip"
    if (Test-Path $zipPath) {
        Write-Host "Распаковка Ninja..." -ForegroundColor Cyan
        Expand-Archive -Path $zipPath -DestinationPath $NinjaDir -Force
    }
}

# Проверка установки
if (Test-Path $NinjaExe) {
    $version = & $NinjaExe --version
    Write-Host "Ninja $version успешно установлен" -ForegroundColor Green
    
    # Добавление в PATH (если нет)
    $userPath = [Environment]::GetEnvironmentVariable("Path", "User")
    if ($userPath -notlike "*$NinjaDir*") {
        Write-Host "Добавление в PATH..." -ForegroundColor Cyan
        [Environment]::SetEnvironmentVariable(
            "Path",
            "$userPath;$NinjaDir",
            "User"
        )
        Write-Host "PATH обновлён. Перезапустите терминал для применения." -ForegroundColor Yellow
    }
    
    Write-Host "`nПроверка: ninja --version" -ForegroundColor Cyan
    & $NinjaExe --version
} else {
    Write-Host "Ошибка установки Ninja" -ForegroundColor Red
    exit 1
}
