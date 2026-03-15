#!/usr/bin/env python3
"""
MKA Embedded Project Launcher
Запуск проекта с автоматической проверкой и выбором порта
"""

import socket
import sys
import subprocess
import os
from pathlib import Path


class PortChecker:
    """Проверка и выбор порта"""
    
    DEFAULT_PORT = 5000
    MAX_PORT = 5100
    
    @staticmethod
    def is_port_available(port: int, host: str = '127.0.0.1') -> bool:
        """Проверка доступности порта"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind((host, port))
                return True
        except socket.error:
            return False
    
    @staticmethod
    def find_available_port(start_port: int = DEFAULT_PORT, 
                           max_port: int = MAX_PORT) -> int:
        """Поиск доступного порта"""
        for port in range(start_port, max_port + 1):
            if PortChecker.is_port_available(port):
                print(f"✓ Found available port: {port}")
                return port
        
        raise RuntimeError(
            f"No available ports in range {start_port}-{max_port}"
        )


class ProjectLauncher:
    """Запуск проекта MKA Embedded"""
    
    def __init__(self, project_root: str):
        self.project_root = Path(project_root)
        self.build_dir = self.project_root / 'build'
        self.python_dir = self.project_root / 'src' / 'python'
        
    def check_build(self) -> bool:
        """Проверка наличия сборки"""
        if not self.build_dir.exists():
            print("✗ Build directory not found. Running CMake...")
            return self.configure_build()
        
        # Проверка наличия тестов
        test_exe = self.build_dir / 'test_telemetry.exe'
        if not test_exe.exists():
            # Попробуем Unix путь
            test_exe = self.build_dir / 'test_telemetry'
        
        if not test_exe.exists():
            print("✗ Tests not built. Rebuilding...")
            return self.build_project()
        
        print("✓ Build directory OK")
        return True
    
    def configure_build(self) -> bool:
        """Конфигурация CMake"""
        try:
            print("Configuring CMake...")
            subprocess.run(
                ['cmake', '-B', str(self.build_dir), 
                 '-DBUILD_TESTS=ON', '-DHOST_BUILD=ON'],
                cwd=self.project_root,
                check=True
            )
            return True
        except subprocess.CalledProcessError as e:
            print(f"✗ CMake configuration failed: {e}")
            return False
        except FileNotFoundError:
            print("✗ CMake not found. Please install CMake.")
            return False
    
    def build_project(self) -> bool:
        """Сборка проекта"""
        try:
            print("Building project...")
            subprocess.run(
                ['cmake', '--build', str(self.build_dir), '--parallel'],
                cwd=self.project_root,
                check=True
            )
            return True
        except subprocess.CalledProcessError as e:
            print(f"✗ Build failed: {e}")
            return False
    
    def run_tests(self) -> bool:
        """Запуск тестов"""
        try:
            print("\n" + "="*50)
            print("Running C++ tests...")
            print("="*50)
            
            result = subprocess.run(
                ['ctest', '--output-on-failure', '--parallel'],
                cwd=self.build_dir,
                capture_output=True,
                text=True
            )
            
            print(result.stdout)
            if result.stderr:
                print(result.stderr, file=sys.stderr)
            
            if result.returncode == 0:
                print("✓ All C++ tests passed")
                return True
            else:
                print("✗ Some tests failed")
                return False
                
        except FileNotFoundError:
            print("✗ ctest not found")
            return False
        except Exception as e:
            print(f"✗ Error running tests: {e}")
            return False
    
    def run_python_tests(self) -> bool:
        """Запуск Python тестов"""
        try:
            print("\n" + "="*50)
            print("Running Python tests...")
            print("="*50)
            
            result = subprocess.run(
                [sys.executable, '-m', 'pytest', 'tests/', '-v'],
                cwd=self.python_dir,
                capture_output=True,
                text=True
            )
            
            print(result.stdout)
            if result.stderr:
                print(result.stderr, file=sys.stderr)
            
            if result.returncode == 0:
                print("✓ All Python tests passed")
                return True
            else:
                print("✗ Some Python tests failed")
                return False
                
        except FileNotFoundError:
            print("✗ pytest not found. Install: pip install pytest")
            return False
        except Exception as e:
            print(f"✗ Error running Python tests: {e}")
            return False
    
    def run_all(self) -> bool:
        """Запуск всех проверок"""
        print("="*50)
        print("MKA Embedded Project Launcher")
        print("="*50)
        
        # Проверка и настройка сборки
        if not self.check_build():
            if not self.configure_build():
                return False
            if not self.build_project():
                return False
        
        # Запуск тестов
        cpp_ok = self.run_tests()
        py_ok = self.run_python_tests()
        
        print("\n" + "="*50)
        print("Summary:")
        print(f"  C++ Tests:   {'✓ PASS' if cpp_ok else '✗ FAIL'}")
        print(f"  Python Tests: {'✓ PASS' if py_ok else '✗ FAIL'}")
        print("="*50)
        
        return cpp_ok and py_ok


def main():
    """Основная функция"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='MKA Embedded Project Launcher'
    )
    parser.add_argument(
        '--project-root', '-p',
        default='.',
        help='Корень проекта (по умолчанию: текущая директория)'
    )
    parser.add_argument(
        '--check-port', '-c',
        type=int,
        nargs='?',
        const=5000,
        help='Проверить порт (по умолчанию 5000)'
    )
    parser.add_argument(
        '--find-port', '-f',
        action='store_true',
        help='Найти свободный порт'
    )
    parser.add_argument(
        '--test', '-t',
        action='store_true',
        help='Запустить только тесты'
    )
    parser.add_argument(
        '--build', '-b',
        action='store_true',
        help='Только собрать проект'
    )
    
    args = parser.parse_args()
    
    project_root = Path(args.project_root).resolve()
    
    # Проверка порта
    if args.check_port is not None:
        port = args.check_port
        checker = PortChecker()
        if checker.is_port_available(port):
            print(f"✓ Port {port} is available")
            return 0
        else:
            print(f"✗ Port {port} is busy")
            return 1
    
    # Поиск порта
    if args.find_port:
        try:
            port = PortChecker.find_available_port()
            print(f"✓ Available port: {port}")
            return 0
        except RuntimeError as e:
            print(f"✗ {e}")
            return 1
    
    launcher = ProjectLauncher(project_root)
    
    # Только сборка
    if args.build:
        if not launcher.check_build():
            if not launcher.configure_build():
                return 1
        if not launcher.build_project():
            return 1
        return 0
    
    # Только тесты
    if args.test:
        if not launcher.check_build():
            print("✗ Project not built. Use --build first.")
            return 1
        return launcher.run_tests()
    
    # Полный запуск
    success = launcher.run_all()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
