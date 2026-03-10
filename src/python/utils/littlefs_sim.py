#!/usr/bin/env python3
"""
LittleFS Simulation for Testing
Симуляция файловой системы LittleFS для тестирования бортового ПО

LittleFS - лёгкая файловая система для микроконтроллеров:
- Малогабаритная (минимальный overhead)
- Отказоустойчивая (power-loss resilient)
- Wear leveling (равномерный износ Flash)
- Поддержка директорий

Формат Superblock:
┌────────────────────────────────────────────────────────────────────┐
│ Magic (8B) │ Version (4B) │ Block Size (4B) │ Block Count (4B)     │
├────────────────────────────────────────────────────────────────────┤
│ Name Max (4B) │ File Max (8B) │ Attr Max (4B) │ CRC (4B)           │
└────────────────────────────────────────────────────────────────────┘
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, BinaryIO
from enum import IntEnum
import struct
import time
import hashlib
import json


class LittleFSError(Exception):
    """Исключение LittleFS"""
    pass


class LittleFSErrors(IntEnum):
    """Коды ошибок LittleFS"""
    OK = 0
    CORRUPT = -1
    NOT_FOUND = -2
    EXIST = -3
    NOT_DIR = -4
    IS_DIR = -5
    NOT_EMPTY = -6
    BAD_NAME = -7
    NO_SPACE = -8
    NO_MEM = -9
    NO_ATTR = -10
    NAME_TOO_LONG = -11


# ============================================================================
# Константы LittleFS
# ============================================================================

LFS_MAGIC = b'littlefs'
LFS_VERSION = (2, 0)  # Major.Minor

# Типы тегов
class TagType(IntEnum):
    LFS_TYPE_DIR = 0x400
    LFS_TYPE_REG = 0x000
    LFS_TYPE_SUPERBLOCK = 0x0FE
    LFS_TYPE_DIRSTRUCT = 0x401
    LFS_TYPE_CTZSTRUCT = 0x001
    LFS_TYPE_INLINESTRUCT = 0x002
    LFS_TYPE_SOFTTAIL = 0x600
    LFS_TYPE_HARDTAIL = 0x601
    LFS_TYPE_MOVE = 0x7FF
    LFS_TYPE_DELETE = 0x7FD
    LFS_TYPE_NAME = 0x300
    LFS_TYPE_ATTR = 0x301
    LFS_TYPE_USERATTR = 0x302
    LFS_TYPE_CRC = 0x7FE


# ============================================================================
# Структуры данных
# ============================================================================

@dataclass
class LFSConfig:
    """Конфигурация LittleFS"""
    block_size: int = 4096
    block_count: int = 256     # 1 MB при block_size=4096
    name_max: int = 255
    file_max: int = 2147483647  # 2GB
    attr_max: int = 1022
    read_size: int = 1
    prog_size: int = 4
    lookahead_size: int = 8
    block_cycles: int = 512    # Wear leveling cycles
    
    @property
    def total_size(self) -> int:
        return self.block_size * self.block_count


@dataclass
class LFSFile:
    """Файл в LittleFS"""
    name: str
    size: int = 0
    data: bytearray = field(default_factory=bytearray)
    attributes: Dict[str, bytes] = field(default_factory=dict)
    created_time: float = field(default_factory=time.time)
    modified_time: float = field(default_factory=time.time)
    
    def read(self, offset: int = 0, size: int = -1) -> bytes:
        if size < 0:
            size = len(self.data) - offset
        return bytes(self.data[offset:offset + size])
    
    def write(self, offset: int, data: bytes) -> int:
        # Расширение файла если нужно
        while offset + len(data) > len(self.data):
            self.data.append(0)
        
        for i, byte in enumerate(data):
            self.data[offset + i] = byte
        
        self.size = len(self.data)
        self.modified_time = time.time()
        return len(data)
    
    def truncate(self, size: int) -> None:
        if size < len(self.data):
            self.data = self.data[:size]
        else:
            self.data.extend([0] * (size - len(self.data)))
        self.size = len(self.data)
        self.modified_time = time.time()


@dataclass 
class LFSDirectory:
    """Директория в LittleFS"""
    name: str
    files: Dict[str, LFSFile] = field(default_factory=dict)
    directories: Dict[str, 'LFSDirectory'] = field(default_factory=dict)
    created_time: float = field(default_factory=time.time)
    
    def get_entry(self, path: List[str]) -> Optional[any]:
        """Получение элемента по пути"""
        if not path:
            return self
        
        name = path[0]
        rest = path[1:]
        
        if name in self.files:
            return self.files[name] if not rest else None
        
        if name in self.directories:
            return self.directories[name].get_entry(rest)
        
        return None


@dataclass
class LFSStats:
    """Статистика файловой системы"""
    total_blocks: int = 0
    used_blocks: int = 0
    free_blocks: int = 0
    total_files: int = 0
    total_dirs: int = 0
    read_count: int = 0
    write_count: int = 0
    erase_count: int = 0


# ============================================================================
# Симулятор LittleFS
# ============================================================================

class LittleFS:
    """
    Симулятор файловой системы LittleFS.
    
    Используется для тестирования бортового ПО на хост-машине.
    Реализует основные операции файловой системы.
    """
    
    def __init__(self, config: LFSConfig = None):
        """
        Args:
            config: Конфигурация файловой системы
        """
        self.config = config or LFSConfig()
        
        # Корневая директория
        self.root = LFSDirectory(name="")
        
        # Состояние
        self.mounted = False
        self.write_buffer: bytearray = bytearray()
        
        # Статистика
        self.stats = LFSStats()
        self.stats.total_blocks = self.config.block_count
        
        # Wear leveling tracking
        self.block_erase_count: Dict[int, int] = {}
        
        # Bad blocks
        self.bad_blocks: set = set()
        
        # Журнал операций для восстановления
        self.journal: List[dict] = []
    
    # Константа успеха
    LFS_OK = 0
    
    def format(self) -> int:
        """
        Форматирование файловой системы.
        
        Returns:
            0 если успешно (LFS_OK)
        """
        self.root = LFSDirectory(name="")
        self.mounted = True
        self.write_buffer = bytearray()
        self.stats = LFSStats()
        self.stats.total_blocks = self.config.block_count
        self.block_erase_count = {}
        self.bad_blocks = set()
        self.journal = []
        
        # Создание системных директорий
        self.mkdir("/sys")
        self.mkdir("/log")
        self.mkdir("/data")
        self.mkdir("/tmp")
        self.mkdir("/config")
        
        # Создание системных файлов
        self._write_file("/sys/version", b"LittleFS v2.0")
        self._write_file("/sys/created", struct.pack('<d', time.time()))
        
        return self.LFS_OK
    
    def mount(self) -> int:
        """
        Монтирование файловой системы.
        
        Returns:
            0 (LFS_OK) если успешно
        """
        if not self.mounted:
            # Проверка superblock (в реальной системе)
            self.mounted = True
        
        return self.LFS_OK
    
    def unmount(self) -> None:
        """Размонтирование файловой системы."""
        # Flush всех буферов
        self._sync()
        self.mounted = False
    
    # ========================================================================
    # Операции с файлами
    # ========================================================================
    
    def open(self, path: str, mode: str = 'r') -> 'LFSFileHandle':
        """
        Открытие файла.
        
        Args:
            path: Путь к файлу
            mode: Режим открытия (r, w, a, r+, w+, a+)
        
        Returns:
            Дескриптор файла
        """
        if not self.mounted:
            raise LittleFSError("Filesystem not mounted")
        
        # Нормализация пути
        path = self._normalize_path(path)
        
        # Разбор пути
        dir_path, file_name = self._split_path(path)
        directory = self._get_directory(dir_path, create=False)
        
        if directory is None:
            raise LittleFSError(f"Directory not found: {dir_path}")
        
        # Получение или создание файла
        file = directory.files.get(file_name)
        
        if 'r' in mode and '+' not in mode:
            # Только чтение
            if file is None:
                raise LittleFSError(f"File not found: {path}")
        elif 'w' in mode:
            # Запись (очистка файла)
            if file is None:
                file = LFSFile(name=file_name)
                directory.files[file_name] = file
                self.stats.total_files += 1
            else:
                file.data = bytearray()
                file.size = 0
        
        if 'a' in mode:
            # Дополнение
            if file is None:
                file = LFSFile(name=file_name)
                directory.files[file_name] = file
                self.stats.total_files += 1
        
        return LFSFileHandle(self, file, path, mode)
    
    def remove(self, path: str) -> bool:
        """Удаление файла."""
        path = self._normalize_path(path)
        dir_path, file_name = self._split_path(path)
        directory = self._get_directory(dir_path, create=False)
        
        if directory is None or file_name not in directory.files:
            raise LittleFSError(f"File not found: {path}")
        
        del directory.files[file_name]
        self.stats.total_files -= 1
        
        self._log_operation('remove', path)
        return True
    
    def rename(self, old_path: str, new_path: str) -> bool:
        """Переименование/перемещение файла."""
        old_path = self._normalize_path(old_path)
        new_path = self._normalize_path(new_path)
        
        # Чтение содержимого
        with self.open(old_path, 'r') as f:
            data = f.read()
        
        # Удаление старого
        self.remove(old_path)
        
        # Создание нового
        with self.open(new_path, 'w') as f:
            f.write(data)
        
        return True
    
    def stat(self, path: str) -> dict:
        """Получение информации о файле."""
        path = self._normalize_path(path)
        
        entry = self._get_entry(path)
        if entry is None:
            raise LittleFSError(f"Not found: {path}")
        
        if isinstance(entry, LFSFile):
            return {
                'type': 'file',
                'size': entry.size,
                'created': entry.created_time,
                'modified': entry.modified_time,
                'attributes': list(entry.attributes.keys())
            }
        elif isinstance(entry, LFSDirectory):
            return {
                'type': 'directory',
                'size': 0,
                'created': entry.created_time,
                'entries': len(entry.files) + len(entry.directories)
            }
        
        raise LittleFSError(f"Unknown type: {path}")
    
    # ========================================================================
    # Операции с директориями
    # ========================================================================
    
    def mkdir(self, path: str) -> bool:
        """Создание директории."""
        path = self._normalize_path(path)
        
        dir_path, dir_name = self._split_path(path)
        parent = self._get_directory(dir_path, create=True)
        
        if dir_name in parent.directories:
            return False  # Already exists
        
        parent.directories[dir_name] = LFSDirectory(name=dir_name)
        self.stats.total_dirs += 1
        
        return True
    
    def rmdir(self, path: str) -> bool:
        """Удаление пустой директории."""
        path = self._normalize_path(path)
        
        dir_path, dir_name = self._split_path(path)
        parent = self._get_directory(dir_path, create=False)
        
        if parent is None or dir_name not in parent.directories:
            raise LittleFSError(f"Directory not found: {path}")
        
        directory = parent.directories[dir_name]
        
        if directory.files or directory.directories:
            raise LittleFSError("Directory not empty")
        
        del parent.directories[dir_name]
        self.stats.total_dirs -= 1
        
        return True
    
    def listdir(self, path: str = "/") -> List[str]:
        """Получение списка файлов и директорий."""
        path = self._normalize_path(path)
        directory = self._get_directory(path, create=False)
        
        if directory is None:
            raise LittleFSError(f"Directory not found: {path}")
        
        entries = list(directory.files.keys()) + list(directory.directories.keys())
        return sorted(entries)
    
    def walk(self, path: str = "/") -> List[Tuple[str, List[str], List[str]]]:
        """
        Рекурсивный обход директорий.
        
        Yields:
            (dirpath, dirnames, filenames)
        """
        path = self._normalize_path(path)
        directory = self._get_directory(path, create=False)
        
        if directory is None:
            return []
        
        result = []
        
        # Текущая директория
        dirnames = sorted(directory.directories.keys())
        filenames = sorted(directory.files.keys())
        result.append((path, dirnames, filenames))
        
        # Рекурсия
        for dirname in dirnames:
            subdir_path = f"{path}/{dirname}".replace("//", "/")
            result.extend(self.walk(subdir_path))
        
        return result
    
    # ========================================================================
    # Атрибуты файлов
    # ========================================================================
    
    def setattr(self, path: str, name: str, value: bytes) -> bool:
        """Установка атрибута файла."""
        path = self._normalize_path(path)
        entry = self._get_entry(path)
        
        if entry is None or not isinstance(entry, LFSFile):
            raise LittleFSError(f"File not found: {path}")
        
        entry.attributes[name] = value
        return True
    
    def getattr(self, path: str, name: str) -> Optional[bytes]:
        """Получение атрибута файла."""
        path = self._normalize_path(path)
        entry = self._get_entry(path)
        
        if entry is None or not isinstance(entry, LFSFile):
            raise LittleFSError(f"File not found: {path}")
        
        return entry.attributes.get(name)
    
    # ========================================================================
    # Wear Leveling
    # ========================================================================
    
    def get_wear_statistics(self) -> Dict[int, int]:
        """Получение статистики износа блоков."""
        return dict(self.block_erase_count)
    
    def get_wear_level(self) -> Tuple[int, int, float]:
        """
        Получение уровня износа.
        
        Returns:
            (min_erases, max_erases, avg_erases)
        """
        if not self.block_erase_count:
            return (0, 0, 0.0)
        
        values = list(self.block_erase_count.values())
        return (min(values), max(values), sum(values) / len(values))
    
    # ========================================================================
    # Сохранение и загрузка
    # ========================================================================
    
    def save_to_file(self, filename: str) -> bool:
        """Сохранение образа файловой системы."""
        def serialize_dir(directory: LFSDirectory) -> dict:
            return {
                'name': directory.name,
                'created': directory.created_time,
                'files': {
                    name: {
                        'data': list(f.data),
                        'size': f.size,
                        'created': f.created_time,
                        'modified': f.modified_time,
                        'attrs': {k: list(v) for k, v in f.attributes.items()}
                    }
                    for name, f in directory.files.items()
                },
                'directories': {
                    name: serialize_dir(d)
                    for name, d in directory.directories.items()
                }
            }
        
        image = {
            'config': {
                'block_size': self.config.block_size,
                'block_count': self.config.block_count,
                'name_max': self.config.name_max,
            },
            'stats': {
                'total_files': self.stats.total_files,
                'total_dirs': self.stats.total_dirs,
            },
            'root': serialize_dir(self.root),
            'wear': self.block_erase_count
        }
        
        with open(filename, 'w') as f:
            json.dump(image, f)
        
        return True
    
    def load_from_file(self, filename: str) -> bool:
        """Загрузка образа файловой системы."""
        with open(filename, 'r') as f:
            image = json.load(f)
        
        # Конфигурация
        self.config.block_size = image['config']['block_size']
        self.config.block_count = image['config']['block_count']
        
        # Статистика
        self.stats.total_files = image['stats']['total_files']
        self.stats.total_dirs = image['stats']['total_dirs']
        
        # Дерево файлов
        def deserialize_dir(data: dict) -> LFSDirectory:
            directory = LFSDirectory(
                name=data['name'],
                created_time=data['created']
            )
            
            for name, fdata in data['files'].items():
                directory.files[name] = LFSFile(
                    name=name,
                    data=bytearray(fdata['data']),
                    size=fdata['size'],
                    created_time=fdata['created'],
                    modified_time=fdata['modified'],
                    attributes={k: bytes(v) for k, v in fdata['attrs'].items()}
                )
            
            for name, ddata in data['directories'].items():
                directory.directories[name] = deserialize_dir(ddata)
            
            return directory
        
        self.root = deserialize_dir(image['root'])
        self.block_erase_count = {int(k): v for k, v in image.get('wear', {}).items()}
        
        return True
    
    # ========================================================================
    # Внутренние методы
    # ========================================================================
    
    def _normalize_path(self, path: str) -> str:
        """Нормализация пути."""
        # Удаление ведущих/конечных пробелов
        path = path.strip()
        
        # Замена обратных слешей
        path = path.replace('\\', '/')
        
        # Удаление повторяющихся слешей
        while '//' in path:
            path = path.replace('//', '/')
        
        # Добавление ведущего слеша
        if not path.startswith('/'):
            path = '/' + path
        
        return path
    
    def _split_path(self, path: str) -> Tuple[str, str]:
        """Разделение пути на директорию и имя файла."""
        parts = path.rstrip('/').split('/')
        
        if len(parts) <= 1:
            return ('', parts[-1] if parts else '')
        
        return ('/'.join(parts[:-1]), parts[-1])
    
    def _get_directory(self, path: str, create: bool = False) -> Optional[LFSDirectory]:
        """Получение директории по пути."""
        if not path or path == '/':
            return self.root
        
        parts = path.strip('/').split('/')
        current = self.root
        
        for part in parts:
            if not part:
                continue
            
            if part not in current.directories:
                if create:
                    current.directories[part] = LFSDirectory(name=part)
                    self.stats.total_dirs += 1
                else:
                    return None
            
            current = current.directories[part]
        
        return current
    
    def _get_entry(self, path: str) -> Optional[any]:
        """Получение элемента по пути."""
        path = path.rstrip('/')
        
        if path == '/' or not path:
            return self.root
        
        dir_path, name = self._split_path(path)
        directory = self._get_directory(dir_path)
        
        if directory is None:
            return None
        
        if name in directory.files:
            return directory.files[name]
        
        if name in directory.directories:
            return directory.directories[name]
        
        return None
    
    def _write_file(self, path: str, data: bytes) -> bool:
        """Внутренний метод записи файла."""
        with self.open(path, 'w') as f:
            f.write(data)
        return True
    
    def _sync(self) -> None:
        """Синхронизация буферов."""
        # В реальной системе - flush на носитель
        pass
    
    def _log_operation(self, op: str, path: str, **kwargs) -> None:
        """Логирование операции для восстановления."""
        self.journal.append({
            'op': op,
            'path': path,
            'time': time.time(),
            **kwargs
        })

    # ========================================================================
    # Упрощённые методы для совместимости
    # ========================================================================

    def write_file(self, path: str, data: bytes) -> bool:
        """Запись файла (упрощённый метод)."""
        with self.open(path, 'w') as f:
            f.write(data)
        return True
    
    def read_file(self, path: str) -> Optional[bytes]:
        """Чтение файла (упрощённый метод)."""
        try:
            with self.open(path, 'r') as f:
                return f.read()
        except LittleFSError:
            return None
    
    def exists(self, path: str) -> bool:
        """Проверка существования файла или директории."""
        try:
            return self._get_entry(path) is not None
        except:
            return False


# ============================================================================
# Дескриптор файла
# ============================================================================

class LFSFileHandle:
    """Дескриптор открытого файла."""
    
    def __init__(self, fs: LittleFS, file: LFSFile, path: str, mode: str):
        self.fs = fs
        self.file = file
        self.path = path
        self.mode = mode
        self.position = 0
        self.closed = False
    
    def read(self, size: int = -1) -> bytes:
        """Чтение данных."""
        if self.closed:
            raise LittleFSError("File closed")
        
        if 'r' not in self.mode and '+' not in self.mode:
            raise LittleFSError("File not open for reading")
        
        if size < 0:
            size = len(self.file.data) - self.position
        
        data = self.file.read(self.position, size)
        self.position += len(data)
        self.fs.stats.read_count += 1
        
        return data
    
    def write(self, data: bytes) -> int:
        """Запись данных."""
        if self.closed:
            raise LittleFSError("File closed")
        
        if 'w' not in self.mode and 'a' not in self.mode and '+' not in self.mode:
            raise LittleFSError("File not open for writing")
        
        written = self.file.write(self.position, data)
        self.position += written
        self.fs.stats.write_count += 1
        
        return written
    
    def seek(self, offset: int, whence: int = 0) -> int:
        """
        Перемещение позиции.
        
        Args:
            offset: Смещение
            whence: 0=от начала, 1=текущая, 2=от конца
        """
        if whence == 0:
            self.position = offset
        elif whence == 1:
            self.position += offset
        elif whence == 2:
            self.position = len(self.file.data) + offset
        
        return self.position
    
    def tell(self) -> int:
        """Текущая позиция."""
        return self.position
    
    def truncate(self, size: int = None) -> None:
        """Обрезка файла."""
        if size is None:
            size = self.position
        
        self.file.truncate(size)
    
    def close(self) -> None:
        """Закрытие файла."""
        self.closed = True
    
    def __enter__(self):
        return self
    
    def __exit__(self, *args):
        self.close()
    
    def __repr__(self):
        return f"<LFSFile {self.path} pos={self.position}>"


# ============================================================================
# Примеры использования
# ============================================================================

def example_lfs_basic():
    """Базовый пример работы с LittleFS."""
    print("=== LittleFS Basic Example ===\n")
    
    # Создание файловой системы
    config = LFSConfig(block_size=4096, block_count=256)  # 1 MB
    fs = LittleFS(config)
    
    # Форматирование
    fs.format()
    print(f"Formatted: {config.block_count} blocks × {config.block_size} bytes = {config.total_size} bytes")
    
    # Создание файлов
    with fs.open("/data/telemetry.bin", 'w') as f:
        f.write(b'TM_DATA_V1.0')
        f.write(struct.pack('<I', 12345))  # Counter
        f.write(struct.pack('<f', 25.5))   # Temperature
    
    print(f"Created: /data/telemetry.bin")
    
    # Чтение файла
    with fs.open("/data/telemetry.bin", 'r') as f:
        header = f.read(10)
        counter = struct.unpack('<I', f.read(4))[0]
        temp = struct.unpack('<f', f.read(4))[0]
    
    print(f"Read: header={header}, counter={counter}, temp={temp}°C")
    
    # Список файлов
    print(f"\nFiles in /data:")
    for name in fs.listdir("/data"):
        stat = fs.stat(f"/data/{name}")
        print(f"  {name}: {stat['size']} bytes")


def example_lfs_logging():
    """Пример логирования в файловую систему."""
    print("\n=== LittleFS Logging Example ===\n")
    
    fs = LittleFS()
    fs.format()
    
    # Создание директории для логов
    fs.mkdir("/log")
    
    # Запись логов с ротацией
    for i in range(5):
        log_file = f"/log/session_{i}.log"
        
        with fs.open(log_file, 'w') as f:
            for j in range(10):
                log_entry = f"[{i*10+j:04d}] System nominal, temp={20+i}\n"
                f.write(log_entry.encode())
    
    # Статистика
    print("Log files:")
    total_size = 0
    for name in fs.listdir("/log"):
        stat = fs.stat(f"/log/{name}")
        total_size += stat['size']
        print(f"  {name}: {stat['size']} bytes")
    
    print(f"\nTotal log size: {total_size} bytes")


def example_lfs_config():
    """Пример хранения конфигурации."""
    print("\n=== LittleFS Configuration Example ===\n")
    
    fs = LittleFS()
    fs.format()
    
    # Сохранение конфигурации
    config = {
        'node_address': 1,
        'baud_rate': 115200,
        'tx_power': 27,
        'beacon_interval': 60,
        'sensors': {
            'imu_enabled': True,
            'mag_enabled': True,
            'gps_enabled': False
        }
    }
    
    with fs.open("/config/system.json", 'w') as f:
        f.write(json.dumps(config, indent=2).encode())
    
    print("Configuration saved")
    
    # Чтение конфигурации
    with fs.open("/config/system.json", 'r') as f:
        loaded = json.loads(f.read().decode())
    
    print(f"Loaded config: node={loaded['node_address']}, baud={loaded['baud_rate']}")
    
    # Атрибуты файла (например, контрольная сумма)
    checksum = hashlib.md5(json.dumps(config).encode()).digest()
    fs.setattr("/config/system.json", "md5", checksum)
    
    stored_md5 = fs.getattr("/config/system.json", "md5")
    print(f"Stored MD5: {stored_md5.hex()}")


def example_lfs_walk():
    """Пример обхода файловой системы."""
    print("\n=== LittleFS Walk Example ===\n")
    
    fs = LittleFS()
    fs.format()
    
    # Создание структуры
    fs.mkdir("/data/hk")
    fs.mkdir("/data/payload")
    
    with fs.open("/data/hk/eps.csv", 'w') as f:
        f.write(b'time,vbat,itemp\n')
    
    with fs.open("/data/hk/adcs.csv", 'w') as f:
        f.write(b'time,q0,q1,q2,q3\n')
    
    with fs.open("/data/payload/image.raw", 'w') as f:
        f.write(bytes([0xFF] * 100))
    
    # Обход
    print("File system tree:")
    for dirpath, dirnames, filenames in fs.walk():
        print(f"{dirpath}/")
        for d in dirnames:
            print(f"  [DIR]  {d}/")
        for f in filenames:
            stat = fs.stat(f"{dirpath}/{f}")
            print(f"  [FILE] {f} ({stat['size']} bytes)")


if __name__ == '__main__':
    example_lfs_basic()
    example_lfs_logging()
    example_lfs_config()
    example_lfs_walk()
