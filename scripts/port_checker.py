#!/usr/bin/env python3
"""
Port checker and project launcher for MKA Embedded
Проверка доступности порта и запуск компонентов проекта
"""

import socket
import sys
import subprocess
import platform
from typing import Optional, List, Tuple


def check_port(port: int, host: str = 'localhost') -> bool:
    """
    Проверка занятости порта
    
    Args:
        port: Номер порта для проверки
        host: Хост для проверки (по умолчанию localhost)
    
    Returns:
        True если порт занят, False если свободен
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1)
    try:
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except socket.error:
        return False


def find_free_port(start_port: int, end_port: int, host: str = 'localhost') -> Optional[int]:
    """
    Поиск свободного порта в диапазоне
    
    Args:
        start_port: Начальный порт диапазона
        end_port: Конечный порт диапазона
        host: Хост для проверки
    
    Returns:
        Номер свободного порта или None если не найдено
    """
    for port in range(start_port, end_port + 1):
        if not check_port(port, host):
            return port
    return None


def get_process_on_port(port: int, host: str = 'localhost') -> Optional[str]:
    """
    Получение информации о процессе на порту
    
    Args:
        port: Номер порта
        host: Хост
    
    Returns:
        Информация о процессе или None
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        result = sock.connect_ex((host, port))
        sock.close()
        if result == 0:
            return f"Port {port} is in use"
    except socket.error as e:
        return f"Error: {e}"
    return None


def run_tests(build_dir: str) -> bool:
    """
    Запуск тестов
    
    Args:
        build_dir: Директория сборки
    
    Returns:
        True если все тесты прошли
    """
    try:
        # Определяем команду для запуска тестов
        if platform.system() == 'Windows':
            cmd = ['ctest', '--output-on-failure']
        else:
            cmd = ['ctest', '--output-on-failure']
        
        result = subprocess.run(
            cmd,
            cwd=build_dir,
            capture_output=True,
            text=True
        )
        
        print(result.stdout)
        if result.stderr:
            print(result.stderr, file=sys.stderr)
        
        return result.returncode == 0
    except FileNotFoundError:
        print("Error: ctest not found. Make sure CMake is installed.")
        return False
    except Exception as e:
        print(f"Error running tests: {e}")
        return False


def run_python_tests(python_dir: str) -> bool:
    """
    Запуск Python тестов
    
    Args:
        python_dir: Директория с Python кодом
    
    Returns:
        True если все тесты прошли
    """
    try:
        cmd = [sys.executable, '-m', 'pytest', 'tests/', '-v']
        
        result = subprocess.run(
            cmd,
            cwd=python_dir,
            capture_output=True,
            text=True
        )
        
        print(result.stdout)
        if result.stderr:
            print(result.stderr, file=sys.stderr)
        
        return result.returncode == 0
    except FileNotFoundError:
        print("Error: pytest not found. Install with: pip install pytest")
        return False
    except Exception as e:
        print(f"Error running Python tests: {e}")
        return False


def check_common_ports() -> List[Tuple[int, bool]]:
    """
    Проверка распространённых портов
    
    Returns:
        Список кортежей (порт, занят)
    """
    common_ports = [
        (22, 'SSH'),
        (80, 'HTTP'),
        (443, 'HTTPS'),
        (3306, 'MySQL'),
        (5432, 'PostgreSQL'),
        (8080, 'HTTP Alt'),
        (9000, 'Common Dev Port'),
    ]
    
    results = []
    for port, name in common_ports:
        is_busy = check_port(port)
        results.append((port, is_busy))
        status = "BUSY" if is_busy else "FREE"
        print(f"  Port {port:5d} ({name:15s}): {status}")
    
    return results


def main():
    """Основная функция"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='MKA Embedded Port Checker and Launcher'
    )
    parser.add_argument(
        '--check-port', '-p',
        type=int,
        help='Проверить конкретный порт'
    )
    parser.add_argument(
        '--range', '-r',
        nargs=2,
        type=int,
        metavar=('START', 'END'),
        help='Диапазон портов для проверки'
    )
    parser.add_argument(
        '--find-free', '-f',
        nargs=2,
        type=int,
        metavar=('START', 'END'),
        help='Найти свободный порт в диапазоне'
    )
    parser.add_argument(
        '--test', '-t',
        action='store_true',
        help='Запустить C++ тесты'
    )
    parser.add_argument(
        '--py-test',
        action='store_true',
        help='Запустить Python тесты'
    )
    parser.add_argument(
        '--build-dir', '-b',
        default='build',
        help='Директория сборки (по умолчанию: build)'
    )
    parser.add_argument(
        '--python-dir',
        default='src/python',
        help='Директория Python кода (по умолчанию: src/python)'
    )
    parser.add_argument(
        '--common', '-c',
        action='store_true',
        help='Проверить распространённые порты'
    )
    
    args = parser.parse_args()
    
    # Проверка конкретного порта
    if args.check_port:
        is_busy = check_port(args.check_port)
        status = "BUSY" if is_busy else "FREE"
        print(f"Port {args.check_port}: {status}")
        if is_busy:
            info = get_process_on_port(args.check_port)
            if info:
                print(f"  {info}")
        return 0 if not is_busy else 1
    
    # Проверка диапазона
    if args.range:
        start, end = args.range
        print(f"Checking ports {start} to {end}:")
        for port in range(start, end + 1):
            is_busy = check_port(port)
            status = "BUSY" if is_busy else "FREE"
            print(f"  Port {port}: {status}")
        return 0
    
    # Поиск свободного порта
    if args.find_free:
        start, end = args.find_free
        free_port = find_free_port(start, end)
        if free_port:
            print(f"Free port found: {free_port}")
            return 0
        else:
            print(f"No free ports in range {start}-{end}")
            return 1
    
    # Проверка распространённых портов
    if args.common:
        print("Checking common ports:")
        check_common_ports()
        return 0
    
    # Запуск тестов
    if args.test:
        print("Running C++ tests...")
        success = run_tests(args.build_dir)
        return 0 if success else 1
    
    if args.py_test:
        print("Running Python tests...")
        success = run_python_tests(args.python_dir)
        return 0 if success else 1
    
    # По умолчанию - показать справку
    parser.print_help()
    return 0


if __name__ == '__main__':
    sys.exit(main())
