#!/bin/bash
# Скрипт установки всех зависимостей для проекта

set -e

echo "========================================"
echo "  MKA Embedded - Установка зависимостей"
echo "========================================"
echo ""

OS="$(uname -s)"

case "$OS" in
    Linux*)
        echo "Определена Linux..."
        
        # Обновление пакетов
        if command -v apt &> /dev/null; then
            sudo apt update
            # CMake
            if ! command -v cmake &> /dev/null; then
                echo "Установка CMake..."
                sudo apt install -y cmake
            fi
            # Ninja
            if ! command -v ninja &> /dev/null; then
                echo "Установка Ninja..."
                sudo apt install -y ninja-build
            fi
            # GCC
            if ! command -v g++ &> /dev/null; then
                echo "Установка GCC..."
                sudo apt install -y g++
            fi
        elif command -v dnf &> /dev/null; then
            sudo dnf install -y cmake ninja-build gcc-c++
        fi
        ;;
    Darwin*)
        echo "Определена macOS..."
        if ! command -v brew &> /dev/null; then
            echo "Установите Homebrew: https://brew.sh"
            exit 1
        fi
        brew install cmake ninja gcc
        ;;
    MINGW*|MSYS*|CYGWIN*)
        echo "Определена Windows (MSYS/MinGW)..."
        echo "Запустите scripts/install_ninja.ps1 в PowerShell"
        exit 1
        ;;
    *)
        echo "Неподдерживаемая ОС: $OS"
        exit 1
        ;;
esac

# Python зависимости
if command -v python3 &> /dev/null; then
    echo ""
    echo "Установка Python зависимостей..."
    python3 -m pip install --upgrade pip
    python3 -m pip install -r src/python/requirements.txt
fi

echo ""
echo "========================================"
echo "  Готово!"
echo "========================================"
echo ""
echo "Проверка версий:"
cmake --version | head -1
ninja --version
echo ""
echo "Для сборки:"
echo "  cmake --preset=host-debug"
echo "  cmake --build build/host-debug"
echo ""
