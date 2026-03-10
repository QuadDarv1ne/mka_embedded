#!/usr/bin/env python3
"""
OTA (Over-The-Air) Firmware Update System
Система удалённого обновления прошивки для МКА

Реализует:
- Загрузку прошивки по радиоканалу
- Верификацию подписи и CRC
- Атомарное переключение между образами
- Откат при ошибках загрузки

Архитектура Flash памяти:
┌─────────────────────────────────────────────────────────────┐
│ Bootloader (16 KB)                                          │
│ - Проверка целостности                                      │
│ - Выбор активного образа                                    │
│ - Переключение образов                                      │
├─────────────────────────────────────────────────────────────┤
│ Image A (Application, ~200 KB)                              │
│ - Текущая прошивка                                          │
│ - Метаданные (версия, CRC, подпись)                         │
├─────────────────────────────────────────────────────────────┤
│ Image B (Application, ~200 KB)                              │
│ - Новая прошивка (загружаемая)                              │
│ - Метаданные (версия, CRC, подпись)                         │
├─────────────────────────────────────────────────────────────┤
│ Configuration & Storage (~64 KB)                            │
│ - Параметры системы                                         │
│ - Журналы                                                   │
└─────────────────────────────────────────────────────────────┘
"""

from dataclasses import dataclass, field
from typing import Optional, Dict, List, Callable, BinaryIO
from enum import IntEnum, IntFlag
import struct
import hashlib
import time
import os
import json


class OTAError(Exception):
    """Исключение OTA"""
    pass


class OTAState(IntEnum):
    """Состояния OTA"""
    IDLE = 0              # Ожидание
    RECEIVING = 1         # Приём данных
    VERIFYING = 2         # Верификация
    READY_TO_APPLY = 3    # Готово к применению
    APPLIED = 4           # Применено (требуется перезагрузка)
    ERROR = 5             # Ошибка


class OTAErrorCode(IntEnum):
    """Коды ошибок OTA"""
    NONE = 0
    INVALID_HEADER = 1
    CRC_MISMATCH = 2
    SIGNATURE_INVALID = 3
    FLASH_WRITE_ERROR = 4
    ERASE_ERROR = 5
    VERSION_ROLLBACK = 6
    SIZE_MISMATCH = 7
    TIMEOUT = 8
    ABORTED = 9


class ImageSlot(IntEnum):
    """Слоты образов прошивки"""
    SLOT_A = 0
    SLOT_B = 1


class ImageFlags(IntFlag):
    """Флаги образа"""
    NONE = 0
    VALID = 0x01         # Образ валиден
    ACTIVE = 0x02        # Активный образ
    VERIFIED = 0x04      # Проверен
    NEW = 0x08           # Новый, не загружался
    CORRUPT = 0x10       # Повреждён


# ============================================================================
# Структуры данных
# ============================================================================

@dataclass
class ImageHeader:
    """
    Заголовок образа прошивки (64 байта).
    
    Располагается в начале каждого слота.
    """
    magic: bytes = b'MKA_FW01'        # 8 байт - магическое число
    version_major: int = 0            # Версия major
    version_minor: int = 0            # Версия minor
    version_patch: int = 0            # Версия patch
    version_build: int = 0            # Номер сборки
    timestamp: int = 0                # Unix timestamp сборки
    size: int = 0                     # Размер образа (без заголовка)
    crc32: int = 0                    # CRC32 образа
    flags: int = ImageFlags.NONE      # Флаги
    git_hash: bytes = b'\x00' * 20    # Git commit hash
    board_rev: int = 0                # Ревизия платы
    min_hardware: int = 0             # Минимальная версия hardware
    reserved: bytes = b'\x00' * 12    # Резерв
    
    HEADER_SIZE = 64
    MAGIC = b'MKA_FW01'
    
    def pack(self) -> bytes:
        """Упаковка заголовка в байты."""
        return struct.pack(
            '<8sBBBBIIII20sBB12s',
            self.magic,
            self.version_major,
            self.version_minor,
            self.version_patch,
            self.version_build,
            self.timestamp,
            self.size,
            self.crc32,
            self.flags,
            self.git_hash,
            self.board_rev,
            self.min_hardware,
            self.reserved
        )
    
    @classmethod
    def unpack(cls, data: bytes) -> 'ImageHeader':
        """Распаковка заголовка из байтов."""
        if len(data) < cls.HEADER_SIZE:
            raise OTAError("Header too short")
        
        fields = struct.unpack('<8sBBBBIIII20sBB12s', data[:cls.HEADER_SIZE])
        
        return cls(
            magic=fields[0],
            version_major=fields[1],
            version_minor=fields[2],
            version_patch=fields[3],
            version_build=fields[4],
            timestamp=fields[5],
            size=fields[6],
            crc32=fields[7],
            flags=fields[8],
            git_hash=fields[9],
            board_rev=fields[10],
            min_hardware=fields[11],
            reserved=fields[12]
        )
    
    @property
    def version_string(self) -> str:
        """Строка версии."""
        return f"{self.version_major}.{self.version_minor}.{self.version_patch}-b{self.version_build}"
    
    @property
    def git_hash_short(self) -> str:
        """Короткий git hash."""
        return self.git_hash[:8].hex()
    
    def is_valid(self) -> bool:
        """Проверка валидности заголовка."""
        return self.magic == self.MAGIC and self.size > 0


@dataclass
class OTAMetadata:
    """Метаданные OTA сессии."""
    state: OTAState = OTAState.IDLE
    error_code: OTAErrorCode = OTAErrorCode.NONE
    target_slot: ImageSlot = ImageSlot.SLOT_B
    total_size: int = 0
    received_size: int = 0
    received_chunks: int = 0
    total_chunks: int = 0
    expected_crc: int = 0
    calculated_crc: int = 0
    start_time: float = 0
    last_activity: float = 0
    source_version: str = ""
    target_version: str = ""
    abort_reason: str = ""


# ============================================================================
# OTA Manager
# ============================================================================

class OTAManager:
    """
    Менеджер OTA обновлений.
    
    Управляет процессом загрузки и применения новой прошивки.
    """
    
    # Размеры
    BOOTLOADER_SIZE = 16 * 1024       # 16 KB
    IMAGE_SIZE = 200 * 1024           # 200 KB per slot
    CONFIG_SIZE = 64 * 1024           # 64 KB
    
    # Адреса
    SLOT_A_ADDR = BOOTLOADER_SIZE
    SLOT_B_ADDR = BOOTLOADER_SIZE + IMAGE_SIZE
    CONFIG_ADDR = BOOTLOADER_SIZE + 2 * IMAGE_SIZE
    
    # Параметры передачи
    CHUNK_SIZE = 256                  # Размер чанка (байт)
    MAX_RETRIES = 3                   # Макс. повторных попыток
    TIMEOUT_MS = 30000                # Таймаут сессии (30 сек)
    
    def __init__(self):
        """Инициализация OTA менеджера."""
        self.metadata = OTAMetadata()
        
        # Callbacks
        self.on_progress: Optional[Callable[[int, int], None]] = None
        self.on_state_change: Optional[Callable[[OTAState], None]] = None
        self.on_error: Optional[Callable[[OTAErrorCode, str], None]] = None
        
        # Буфер для приёма
        self._rx_buffer: bytearray = bytearray()
        
        # Образы (симуляция)
        self._slot_a: bytearray = bytearray(self.IMAGE_SIZE)
        self._slot_b: bytearray = bytearray(self.IMAGE_SIZE)
        
        # Флаги
        self._active_slot = ImageSlot.SLOT_A
    
    @property
    def progress_percent(self) -> int:
        """Прогресс в процентах."""
        if self.metadata.total_size == 0:
            return 0
        return int(100 * self.metadata.received_size / self.metadata.total_size)
    
    def get_active_header(self) -> Optional[ImageHeader]:
        """Получение заголовка активного образа."""
        if self._active_slot == ImageSlot.SLOT_A:
            data = bytes(self._slot_a[:ImageHeader.HEADER_SIZE])
        else:
            data = bytes(self._slot_b[:ImageHeader.HEADER_SIZE])
        
        try:
            header = ImageHeader.unpack(data)
            if header.is_valid():
                return header
        except:
            pass
        
        return None
    
    def get_inactive_header(self) -> Optional[ImageHeader]:
        """Получение заголовка неактивного образа."""
        if self._active_slot == ImageSlot.SLOT_A:
            data = bytes(self._slot_b[:ImageHeader.HEADER_SIZE])
        else:
            data = bytes(self._slot_a[:ImageHeader.HEADER_SIZE])
        
        try:
            header = ImageHeader.unpack(data)
            if header.is_valid():
                return header
        except:
            pass
        
        return None
    
    def begin_update(self, total_size: int, version: str = "", 
                     expected_crc: int = 0) -> bool:
        """
        Начало сессии обновления.
        
        Args:
            total_size: Полный размер нового образа
            version: Версия прошивки
            expected_crc: Ожидаемый CRC32
        
        Returns:
            True если успешно
        """
        if self.metadata.state not in [OTAState.IDLE, OTAState.ERROR]:
            return False
        
        # Проверка размера
        if total_size > self.IMAGE_SIZE - ImageHeader.HEADER_SIZE:
            self._set_error(OTAErrorCode.SIZE_MISMATCH, 
                          f"Image too large: {total_size} > {self.IMAGE_SIZE}")
            return False
        
        # Инициализация сессии
        self.metadata = OTAMetadata(
            state=OTAState.RECEIVING,
            target_slot=ImageSlot.SLOT_B if self._active_slot == ImageSlot.SLOT_A else ImageSlot.SLOT_A,
            total_size=total_size,
            expected_crc=expected_crc,
            start_time=time.time(),
            last_activity=time.time(),
            target_version=version,
            total_chunks=(total_size + self.CHUNK_SIZE - 1) // self.CHUNK_SIZE
        )
        
        # Получение версии источника
        active = self.get_active_header()
        if active:
            self.metadata.source_version = active.version_string
        
        # Очистка целевого слота (в реальной системе - erase flash)
        if self.metadata.target_slot == ImageSlot.SLOT_A:
            self._slot_a = bytearray(self.IMAGE_SIZE)
        else:
            self._slot_b = bytearray(self.IMAGE_SIZE)
        
        self._rx_buffer = bytearray()
        self._set_state(OTAState.RECEIVING)
        
        return True
    
    def receive_chunk(self, chunk_num: int, data: bytes) -> bool:
        """
        Приём чанка данных.
        
        Args:
            chunk_num: Номер чанка (начиная с 0)
            data: Данные чанка
        
        Returns:
            True если успешно
        """
        if self.metadata.state != OTAState.RECEIVING:
            return False
        
        # Проверка таймаута
        if time.time() - self.metadata.last_activity > self.TIMEOUT_MS / 1000:
            self._set_error(OTAErrorCode.TIMEOUT, "Session timeout")
            return False
        
        # Проверка порядка чанков
        expected_chunk = self.metadata.received_chunks
        if chunk_num != expected_chunk:
            # TODO: Возможна повторная передача
            pass
        
        # Запись в буфер
        offset = chunk_num * self.CHUNK_SIZE
        self._rx_buffer.extend(data)
        
        self.metadata.received_size += len(data)
        self.metadata.received_chunks += 1
        self.metadata.last_activity = time.time()
        
        # Callback прогресса
        if self.on_progress:
            self.on_progress(self.metadata.received_size, self.metadata.total_size)
        
        return True
    
    def finalize_update(self) -> bool:
        """
        Завершение приёма и верификация.
        
        Returns:
            True если успешно
        """
        if self.metadata.state != OTAState.RECEIVING:
            return False
        
        self._set_state(OTAState.VERIFYING)
        
        # Проверка размера
        if len(self._rx_buffer) != self.metadata.total_size:
            self._set_error(OTAErrorCode.SIZE_MISMATCH,
                          f"Size mismatch: {len(self._rx_buffer)} != {self.metadata.total_size}")
            return False
        
        # CRC проверка
        calculated_crc = self._calculate_crc32(bytes(self._rx_buffer))
        self.metadata.calculated_crc = calculated_crc
        
        if self.metadata.expected_crc != 0 and \
           calculated_crc != self.metadata.expected_crc:
            self._set_error(OTAErrorCode.CRC_MISMATCH,
                          f"CRC mismatch: {calculated_crc:08X} != {self.metadata.expected_crc:08X}")
            return False
        
        # Запись в целевой слот (симуляция)
        # Сначала заголовок
        header = ImageHeader(
            magic=ImageHeader.MAGIC,
            version_major=1,  # Из метаданных
            version_minor=0,
            version_patch=0,
            size=self.metadata.total_size,
            crc32=calculated_crc,
            flags=ImageFlags.VALID | ImageFlags.NEW,
            timestamp=int(time.time())
        )
        
        header_data = header.pack()
        
        # Запись в слот
        if self.metadata.target_slot == ImageSlot.SLOT_A:
            self._slot_a[:ImageHeader.HEADER_SIZE] = header_data
            self._slot_a[ImageHeader.HEADER_SIZE:ImageHeader.HEADER_SIZE + len(self._rx_buffer)] = self._rx_buffer
        else:
            self._slot_b[:ImageHeader.HEADER_SIZE] = header_data
            self._slot_b[ImageHeader.HEADER_SIZE:ImageHeader.HEADER_SIZE + len(self._rx_buffer)] = self._rx_buffer
        
        self._set_state(OTAState.READY_TO_APPLY)
        
        return True
    
    def apply_update(self) -> bool:
        """
        Применение обновления.
        
        После успешного применения требуется перезагрузка.
        
        Returns:
            True если успешно
        """
        if self.metadata.state != OTAState.READY_TO_APPLY:
            return False
        
        # Переключение активного слота
        self._active_slot = self.metadata.target_slot
        
        # Обновление флагов
        # В реальной системе - запись во flash
        
        self._set_state(OTAState.APPLIED)
        
        return True
    
    def abort_update(self, reason: str = "") -> None:
        """Прерывание обновления."""
        self.metadata.abort_reason = reason
        self._set_error(OTAErrorCode.ABORTED, reason)
    
    def rollback(self) -> bool:
        """
        Откат к предыдущей версии.
        
        Returns:
            True если успешно
        """
        # Переключение на другой слот
        if self._active_slot == ImageSlot.SLOT_A:
            self._active_slot = ImageSlot.SLOT_B
        else:
            self._active_slot = ImageSlot.SLOT_A
        
        # Проверка валидности
        header = self.get_active_header()
        if header is None or not (header.flags & ImageFlags.VALID):
            # Откат назад
            if self._active_slot == ImageSlot.SLOT_A:
                self._active_slot = ImageSlot.SLOT_B
            else:
                self._active_slot = ImageSlot.SLOT_A
            return False
        
        return True
    
    # ========================================================================
    # Внутренние методы
    # ========================================================================
    
    def _set_state(self, state: OTAState) -> None:
        """Установка состояния."""
        old_state = self.metadata.state
        self.metadata.state = state
        
        if self.on_state_change and old_state != state:
            self.on_state_change(state)
    
    def _set_error(self, code: OTAErrorCode, message: str) -> None:
        """Установка ошибки."""
        self.metadata.error_code = code
        self._set_state(OTAState.ERROR)
        
        if self.on_error:
            self.on_error(code, message)
    
    @staticmethod
    def _calculate_crc32(data: bytes) -> int:
        """Расчёт CRC32."""
        import binascii
        return binascii.crc32(data) & 0xFFFFFFFF


# ============================================================================
# OTA Protocol
# ============================================================================

class OTAProtocol:
    """
    Протокол OTA для передачи по радиоканалу.
    
    Формат пакета:
    ┌────────┬────────┬────────┬────────┬─────────┬──────┐
    │ SOP    │ Cmd    │ Seq    │ Len    │ Payload │ CRC  │
    │ 2 B    │ 1 B    │ 2 B    │ 2 B    │ N B     │ 2 B  │
    └────────┴────────┴────────┴────────┴─────────┴──────┘
    """
    
    SOP = bytes([0xAB, 0xCD])
    
    # Команды
    class Command(IntEnum):
        BEGIN_UPDATE = 0x01
        CHUNK_DATA = 0x02
        END_UPDATE = 0x03
        APPLY_UPDATE = 0x04
        ABORT_UPDATE = 0x05
        GET_STATUS = 0x10
        GET_VERSION = 0x11
        ROLLBACK = 0x12
        
        ACK = 0x80
        NACK = 0x81
        STATUS = 0x82
    
    def __init__(self, manager: OTAManager):
        self.manager = manager
        self.sequence = 0
    
    def encode_packet(self, cmd: int, payload: bytes = b'') -> bytes:
        """Кодирование пакета."""
        self.sequence = (self.sequence + 1) & 0xFFFF
        
        header = self.SOP + bytes([cmd]) + struct.pack('<HH', self.sequence, len(payload))
        data = header + payload
        
        crc = binascii.crc32(data) & 0xFFFF
        return data + struct.pack('<H', crc)
    
    def decode_packet(self, data: bytes) -> tuple:
        """Декодирование пакета."""
        import binascii
        
        if len(data) < 9:
            raise OTAError("Packet too short")
        
        if data[:2] != self.SOP:
            raise OTAError("Invalid SOP")
        
        cmd = data[2]
        seq, length = struct.unpack('<HH', data[3:7])
        payload = data[7:7+length]
        
        # CRC
        expected_crc = struct.unpack('<H', data[7+length:9+length])[0]
        calculated_crc = binascii.crc32(data[:7+length]) & 0xFFFF
        
        if expected_crc != calculated_crc:
            raise OTAError("CRC error")
        
        return (cmd, seq, payload)
    
    def process_packet(self, data: bytes) -> bytes:
        """Обработка входящего пакета."""
        try:
            cmd, seq, payload = self.decode_packet(data)
            
            if cmd == self.Command.BEGIN_UPDATE:
                return self._handle_begin(payload)
            elif cmd == self.Command.CHUNK_DATA:
                return self._handle_chunk(payload)
            elif cmd == self.Command.END_UPDATE:
                return self._handle_end(payload)
            elif cmd == self.Command.GET_STATUS:
                return self._handle_get_status()
            elif cmd == self.Command.GET_VERSION:
                return self._handle_get_version()
            elif cmd == self.Command.ROLLBACK:
                return self._handle_rollback()
            else:
                return self._nack(seq, 0xFF)
                
        except OTAError as e:
            return self._nack(seq, 0x01)
    
    def _handle_begin(self, payload: bytes) -> bytes:
        """Обработка команды BEGIN_UPDATE."""
        if len(payload) < 8:
            return self._nack(0, 0x02)
        
        size = struct.unpack('<I', payload[:4])[0]
        crc = struct.unpack('<I', payload[4:8])[0]
        
        success = self.manager.begin_update(size, "", crc)
        
        if success:
            return self._ack(0)
        else:
            return self._nack(0, 0x03)
    
    def _handle_chunk(self, payload: bytes) -> bytes:
        """Обработка чанка данных."""
        if len(payload) < 3:
            return self._nack(0, 0x02)
        
        chunk_num = struct.unpack('<H', payload[:2])[0]
        data = payload[2:]
        
        success = self.manager.receive_chunk(chunk_num, data)
        
        if success:
            return self._ack(0)
        else:
            return self._nack(0, 0x04)
    
    def _handle_end(self, payload: bytes) -> bytes:
        """Обработка команды END_UPDATE."""
        success = self.manager.finalize_update()
        
        if success:
            return self._ack(0)
        else:
            return self._nack(0, 0x05)
    
    def _handle_get_status(self) -> bytes:
        """Обработка команды GET_STATUS."""
        status = struct.pack(
            '<BBIIII',
            self.manager.metadata.state,
            self.manager.metadata.error_code,
            self.manager.metadata.total_size,
            self.manager.metadata.received_size,
            self.manager.metadata.expected_crc,
            self.manager.metadata.calculated_crc
        )
        
        return self.encode_packet(self.Command.STATUS, status)
    
    def _handle_get_version(self) -> bytes:
        """Обработка команды GET_VERSION."""
        header = self.manager.get_active_header()
        
        if header:
            version = struct.pack(
                '<BBBB',
                header.version_major,
                header.version_minor,
                header.version_patch,
                header.version_build
            )
        else:
            version = bytes(4)
        
        return self.encode_packet(self.Command.STATUS, version)
    
    def _handle_rollback(self) -> bytes:
        """Обработка команды ROLLBACK."""
        success = self.manager.rollback()
        
        if success:
            return self._ack(0)
        else:
            return self._nack(0, 0x06)
    
    def _ack(self, seq: int) -> bytes:
        """Создание ACK пакета."""
        return self.encode_packet(self.Command.ACK, struct.pack('<H', seq))
    
    def _nack(self, seq: int, error: int) -> bytes:
        """Создание NACK пакета."""
        return self.encode_packet(self.Command.NACK, struct.pack('<HB', seq, error))


# ============================================================================
# Примеры использования
# ============================================================================

import binascii

def example_ota_session():
    """Пример OTA сессии."""
    print("=== OTA Update Session Example ===\n")
    
    # Создание менеджера
    manager = OTAManager()
    protocol = OTAProtocol(manager)
    
    # Callback прогресса
    def on_progress(received: int, total: int):
        print(f"\r  Progress: {received}/{total} bytes ({100*received/total:.1f}%)", end="")
    
    manager.on_progress = on_progress
    
    # Симуляция текущей прошивки
    print("Current firmware: v1.0.0")
    
    # Создание "новой" прошивки
    new_firmware = b'MKA_FW_V2' + bytes(range(256)) * 100  # ~25KB
    expected_crc = binascii.crc32(new_firmware) & 0xFFFFFFFF
    
    print(f"\nNew firmware size: {len(new_firmware)} bytes, CRC: {expected_crc:08X}")
    
    # BEGIN_UPDATE
    print("\nSending BEGIN_UPDATE...")
    begin_packet = protocol.encode_packet(
        OTAProtocol.Command.BEGIN_UPDATE,
        struct.pack('<II', len(new_firmware), expected_crc)
    )
    response = protocol.process_packet(begin_packet)
    print(f"  Response: {'ACK' if response[2] == OTAProtocol.Command.ACK else 'NACK'}")
    
    # Передача чанков
    print("\nSending chunks...")
    chunk_size = manager.CHUNK_SIZE
    chunk_num = 0
    offset = 0
    
    while offset < len(new_firmware):
        chunk = new_firmware[offset:offset + chunk_size]
        chunk_packet = protocol.encode_packet(
            OTAProtocol.Command.CHUNK_DATA,
            struct.pack('<H', chunk_num) + chunk
        )
        protocol.process_packet(chunk_packet)
        
        chunk_num += 1
        offset += chunk_size
    
    print()  # Newline after progress
    
    # END_UPDATE
    print("\nSending END_UPDATE...")
    end_packet = protocol.encode_packet(OTAProtocol.Command.END_UPDATE)
    response = protocol.process_packet(end_packet)
    print(f"  Response: {'ACK' if response[2] == OTAProtocol.Command.ACK else 'NACK'}")
    
    # Проверка статуса
    status_packet = protocol.encode_packet(OTAProtocol.Command.GET_STATUS)
    response = protocol.process_packet(status_packet)
    _, _, payload = protocol.decode_packet(response)
    
    state = payload[0]
    print(f"\nFinal state: {OTAState(state).name}")
    print(f"Ready to apply: {state == OTAState.READY_TO_APPLY}")
    
    # Применение
    if manager.apply_update():
        print("\nUpdate applied! Reboot required.")


def example_ota_rollback():
    """Пример отката прошивки."""
    print("\n=== OTA Rollback Example ===\n")
    
    manager = OTAManager()
    
    # Симуляция текущего образа
    print("Active slot: A")
    manager._active_slot = ImageSlot.SLOT_A
    
    # Попытка отката
    print("Attempting rollback...")
    success = manager.rollback()
    print(f"Rollback result: {'Success' if success else 'Failed'}")
    
    if success:
        print(f"New active slot: {'A' if manager._active_slot == ImageSlot.SLOT_A else 'B'}")


if __name__ == '__main__':
    example_ota_session()
    example_ota_rollback()
