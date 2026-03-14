#!/bin/bash
# Установка Ninja build system для Linux/macOS

set -e

NINJA_VERSION="1.13.2"

echo "=== Установка Ninja $NINJA_VERSION ==="

# Проверка наличия Ninja
if command -v ninja &> /dev/null; then
    CURRENT_VERSION=$(ninja --version)
    echo "Ninja $CURRENT_VERSION уже установлен"
    exit 0
fi

# Определение ОС
OS="$(uname -s)"

case "$OS" in
    Linux*)
        echo "Определена Linux..."
        # Проверка менеджера пакетов
        if command -v apt &> /dev/null; then
            echo "Установка через apt..."
            sudo apt update
            sudo apt install -y ninja-build
        elif command -v dnf &> /dev/null; then
            echo "Установка через dnf..."
            sudo dnf install -y ninja-build
        elif command -v pacman &> /dev/null; then
            echo "Установка через pacman..."
            sudo pacman -S --noconfirm ninja
        else
            # Ручная установка
            echo "Ручная установка..."
            TMPDIR=$(mktemp -d)
            cd "$TMPDIR"
            wget "https://github.com/ninja-build/ninja/releases/download/v${NINJA_VERSION}/ninja-linux.zip"
            unzip ninja-linux.zip
            sudo cp ninja /usr/local/bin/
            sudo chmod +x /usr/local/bin/ninja
            rm -rf "$TMPDIR"
        fi
        ;;
    Darwin*)
        echo "Определена macOS..."
        if command -v brew &> /dev/null; then
            echo "Установка через Homebrew..."
            brew install ninja
        elif command -v port &> /dev/null; then
            echo "Установка через MacPorts..."
            sudo port install ninja
        else
            echo "Установите Homebrew: https://brew.sh"
            exit 1
        fi
        ;;
    *)
        echo "Неподдерживаемая ОС: $OS"
        exit 1
        ;;
esac

# Проверка установки
if command -v ninja &> /dev/null; then
    VERSION=$(ninja --version)
    echo "Ninja $VERSION успешно установлен"
else
    echo "Ошибка установки Ninja"
    exit 1
fi
