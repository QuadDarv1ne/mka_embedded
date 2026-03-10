#!/usr/bin/env python3
"""
CCSDS Protocol Implementation
Реализация протоколов CCSDS для космической связи

CCSDS (Consultative Committee for Space Data Systems) — международный
стандарт для космической связи, используемый большинством космических
агентств и операторов спутников.

Основные протоколы:
- Space Packet Protocol (SPP) — пакеты данных
- TM (Telemetry) Transfer Frame — кадры телеметрии
- TC (Telecommand) Transfer Frame — кадры команд
- AOS (Advanced Orbiting Systems) — расширенный протокол
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Any
from enum import IntEnum, IntFlag
import struct
import time


class CCSDSError(Exception):
    """Исключение протокола CCSDS"""
    pass


# ============================================================================
# Space Packet Protocol
# ============================================================================

class PacketType(IntEnum):
    """Тип пакета"""
    TELEMETRY = 0
    TELECOMMAND = 1


class SecondaryHeaderFlag(IntEnum):
    """Наличие вторичного заголовка"""
    ABSENT = 0
    PRESENT = 1


@dataclass
class SpacePacketPrimaryHeader:
    """
    Первичный заголовок Space Packet (6 байт)
    
    Структура:
    ┌────────┬────────┬────────┬────────┬────────┬────────┐
    │ Version│ Type   │ SecHdr │ APID   │ Seq    │ Length │
    │ 3 bits │ 1 bit  │ 1 bit  │ 11 bits│ 16 bits│ 16 bits│
    └────────┴────────┴────────┴────────┴────────┴────────┘
    """
    version: int = 0                    # Версия (0=basic)
    packet_type: int = 0                # 0=TM, 1=TC
    secondary_header_flag: int = 0      # Наличие вторичного заголовка
    apid: int = 0                       # Application Process ID (0-2047)
    sequence_flags: int = 3             # 3=standalone, 1=first, 0=continuation, 2=last
    packet_sequence_count: int = 0      # Счётчик последовательности (0-16383)
    packet_data_length: int = 0         # Длина данных - 1
    
    def encode(self) -> bytes:
        """Кодирование заголовка в байты"""
        # Byte 0-1: Version, Type, SecHdrFlag, APID
        word0 = ((self.version & 0x07) << 13) | \
                ((self.packet_type & 0x01) << 12) | \
                ((self.secondary_header_flag & 0x01) << 11) | \
                (self.apid & 0x07FF)
        
        # Byte 2-3: SeqFlags, SeqCount
        word1 = ((self.sequence_flags & 0x03) << 14) | \
                (self.packet_sequence_count & 0x3FFF)
        
        # Byte 4-5: Data Length
        word2 = self.packet_data_length & 0xFFFF
        
        return struct.pack('>HHH', word0, word1, word2)
    
    @classmethod
    def decode(cls, data: bytes) -> 'SpacePacketPrimaryHeader':
        """Декодирование заголовка из байтов"""
        if len(data) < 6:
            raise CCSDSError("Header too short")
        
        word0, word1, word2 = struct.unpack('>HHH', data[:6])
        
        return cls(
            version=(word0 >> 13) & 0x07,
            packet_type=(word0 >> 12) & 0x01,
            secondary_header_flag=(word0 >> 11) & 0x01,
            apid=word0 & 0x07FF,
            sequence_flags=(word1 >> 14) & 0x03,
            packet_sequence_count=word1 & 0x3FFF,
            packet_data_length=word2
        )


@dataclass
class SpacePacket:
    """
    Space Packet
    
    Полный пакет данных по стандарту CCSDS.
    """
    primary_header: SpacePacketPrimaryHeader
    secondary_header: bytes = b''
    user_data: bytes = b''
    
    def encode(self) -> bytes:
        """Кодирование пакета в байты"""
        # Обновление длины данных
        total_data_len = len(self.secondary_header) + len(self.user_data)
        self.primary_header.packet_data_length = total_data_len - 1 if total_data_len > 0 else 0
        
        result = bytearray()
        result.extend(self.primary_header.encode())
        
        if self.secondary_header:
            result.extend(self.secondary_header)
        
        result.extend(self.user_data)
        
        return bytes(result)
    
    @classmethod
    def decode(cls, data: bytes) -> 'SpacePacket':
        """Декодирование пакета из байтов"""
        header = SpacePacketPrimaryHeader.decode(data)
        
        data_start = 6
        secondary_header = b''
        
        if header.secondary_header_flag:
            # Минимум вторичный заголовок TM (время)
            if header.packet_type == PacketType.TELEMETRY:
                secondary_header = data[6:14]  # 8 байт времени
                data_start = 14
            else:
                secondary_header = data[6:10]  # 4 байка для TC
                data_start = 10
        
        user_data = data[data_start:6 + header.packet_data_length + 1]
        
        return cls(
            primary_header=header,
            secondary_header=secondary_header,
            user_data=user_data
        )
    
    @property
    def total_length(self) -> int:
        """Полная длина пакета"""
        return 7 + self.primary_header.packet_data_length


# ============================================================================
# TM Transfer Frame
# ============================================================================

@dataclass
class TMTransferFrame:
    """
    TM Transfer Frame (кадр телеметрии)
    
    Структура (синхронизация + первичный заголовок):
    ┌────────┬────────┬────────┬────────┬────────┬────────┬────────┐
    │ Sync   │ Version│ Spacec │ VCID   │ MC     │ VC      │ OCF   │
    │ 4 bytes│ 2 bits │ 10 bits│ 3 bits │ Counter│ Counter │ 1 bit │
    └────────┴────────┴────────┴────────┴────────┴─────────┴───────┘
    
    Типичные размеры: 892 байта (AOS), 1115 байт (TM), 2232 байта
    """
    version: int = 0
    spacecraft_id: int = 0
    virtual_channel_id: int = 0
    master_channel_frame_count: int = 0
    virtual_channel_frame_count: int = 0
    operational_control_field_flag: int = 0
    master_channel_frame_count_flag: int = 0
    virtual_channel_frame_count_flag: int = 0
    secondary_header_flag: int = 0
    sync_flag: int = 0
    packet_order_flag: int = 0
    segment_length_id: int = 3  # 3 = 2048 bytes
    first_header_pointer: int = 0
    ocf: bytes = b''
    data: bytes = b''
    
    # ASM (Attached Synchronization Marker)
    ASM = bytes([0x1A, 0xCF, 0xFC, 0x1D])
    
    def encode(self) -> bytes:
        """Кодирование кадра"""
        result = bytearray()
        
        # ASM
        result.extend(self.ASM)
        
        # Primary Header (6 байт)
        word0 = ((self.version & 0x03) << 14) | (self.spacecraft_id & 0x03FF)
        word1 = ((self.virtual_channel_id & 0x07) << 8) | \
                ((self.operational_control_field_flag & 0x01) << 7) | \
                ((self.master_channel_frame_count_flag & 0x01) << 6) | \
                ((self.virtual_channel_frame_count_flag & 0x01) << 5) | \
                ((self.secondary_header_flag & 0x01) << 4) | \
                ((self.sync_flag & 0x01) << 3) | \
                ((self.packet_order_flag & 0x01) << 2) | \
                ((self.segment_length_id & 0x03))
        word2 = self.first_header_pointer & 0x3FF
        
        result.extend(struct.pack('>HHH', word0, word1, word2))
        
        # Master Channel Frame Count (1 байт)
        result.append(self.master_channel_frame_count & 0xFF)
        
        # Virtual Channel Frame Count (1 байт)
        result.append(self.virtual_channel_frame_count & 0xFF)
        
        # Data
        result.extend(self.data)
        
        # OCF (4 байта, если есть)
        if self.operational_control_field_flag and self.ocf:
            result.extend(self.ocf)
        
        # Frame Error Control (2 байта, CRC-16)
        crc = self._calculate_crc(bytes(result))
        result.extend(struct.pack('>H', crc))
        
        return bytes(result)
    
    @staticmethod
    def _calculate_crc(data: bytes) -> int:
        """Расчёт CRC-16 для кадра"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc


# ============================================================================
# TC Transfer Frame
# ============================================================================

@dataclass
class TCTransferFrame:
    """
    TC Transfer Frame (кадр телекоманд)
    
    Структура:
    ┌────────┬────────┬────────┬────────┬────────┬────────┐
    │ Version│ Type   │ SecHdr │ Spacec │ VCID   │ Length │
    │ 2 bits │ 1 bit  │ 1 bit  │ 10 bits│ 6 bits │ 16 bits│
    └────────┴────────┴────────┴────────┴────────┴────────┘
    """
    version: int = 0
    bypass_flag: int = 0
    control_command_flag: int = 0
    secondary_header_flag: int = 0
    spacecraft_id: int = 0
    virtual_channel_id: int = 0
    frame_sequence_number: int = 0
    data: bytes = b''
    
    def encode(self) -> bytes:
        """Кодирование кадра TC"""
        result = bytearray()
        
        # Header (5 байт)
        word0 = ((self.version & 0x03) << 14) | \
                ((self.bypass_flag & 0x01) << 13) | \
                ((self.control_command_flag & 0x01) << 12) | \
                ((self.secondary_header_flag & 0x01) << 11) | \
                (self.spacecraft_id & 0x03FF)
        
        word1 = ((self.virtual_channel_id & 0x3F) << 10) | \
                (self.frame_sequence_number & 0x03FF)
        
        result.extend(struct.pack('>HH', word0, word1))
        
        # Frame Length (включая заголовок и CRC)
        frame_length = 5 + len(self.data) + 2
        result.append(frame_length - 1)
        
        # Data
        result.extend(self.data)
        
        # CRC-16
        crc = TMTransferFrame._calculate_crc(bytes(result))
        result.extend(struct.pack('>H', crc))
        
        return bytes(result)


# ============================================================================
# APID Definitions
# ============================================================================

class APIDCategory(IntEnum):
    """Категории APID"""
    GROUND_COMMAND = 0x000      # Команды с земли
    OBC = 0x001                 # Бортовой компьютер
    EPS = 0x002                 # Система электропитания
    ADCS = 0x003                # Система ориентации
    COMM = 0x004                # Система связи
    TCS = 0x005                 # Терморегулирование
    PAYLOAD = 0x010             # Полезная нагрузка
    HOUSEKEEPING = 0x100        # Домашние данные
    SCIENCE_DATA = 0x200        # Научные данные


# ============================================================================
# CCSDS Packet Builder
# ============================================================================

class CCSDSPacketBuilder:
    """
    Построитель CCSDS пакетов
    """
    
    def __init__(self, spacecraft_id: int, apid_base: int):
        self.spacecraft_id = spacecraft_id
        self.apid_base = apid_base
        self.sequence_count = 0
    
    def create_tm_packet(self, apid: int, data: bytes, 
                         secondary_header: bytes = None,
                         time_code: int = None) -> SpacePacket:
        """
        Создание пакета телеметрии
        
        Args:
            apid: APID (0-2047)
            data: Пользовательские данные
            secondary_header: Вторичный заголовок
            time_code: Временная метка (если None, текущее время)
        """
        # Формирование вторичного заголовка (время)
        if secondary_header is None:
            if time_code is None:
                time_code = int(time.time())
            # CCSDS Unsegmented Time Code (8 байт)
            secondary_header = struct.pack('>Q', time_code << 16)
        
        header = SpacePacketPrimaryHeader(
            version=0,
            packet_type=PacketType.TELEMETRY,
            secondary_header_flag=1,
            apid=self.apid_base + apid,
            sequence_flags=3,  # Standalone
            packet_sequence_count=self._get_next_sequence()
        )
        
        self.sequence_count = (self.sequence_count + 1) & 0x3FFF
        
        return SpacePacket(
            primary_header=header,
            secondary_header=secondary_header,
            user_data=data
        )
    
    def create_tc_packet(self, apid: int, data: bytes) -> SpacePacket:
        """
        Создание пакета телекоманды
        """
        header = SpacePacketPrimaryHeader(
            version=0,
            packet_type=PacketType.TELECOMMAND,
            secondary_header_flag=0,
            apid=self.apid_base + apid,
            sequence_flags=3,
            packet_sequence_count=self._get_next_sequence()
        )
        
        self.sequence_count = (self.sequence_count + 1) & 0x3FFF
        
        return SpacePacket(
            primary_header=header,
            user_data=data
        )
    
    def _get_next_sequence(self) -> int:
        return self.sequence_count


# ============================================================================
# Примеры использования
# ============================================================================

def example_space_packet():
    """Пример создания Space Packet"""
    print("=== Space Packet Example ===\n")
    
    # Создание заголовка
    header = SpacePacketPrimaryHeader(
        version=0,
        packet_type=PacketType.TELEMETRY,
        secondary_header_flag=1,
        apid=0x001,
        sequence_flags=3,
        packet_sequence_count=0
    )
    
    # Временная метка (CUC format)
    timestamp = struct.pack('>Q', int(time.time()) << 16)
    
    # Пакет
    packet = SpacePacket(
        primary_header=header,
        secondary_header=timestamp,
        user_data=b'Hello from satellite!'
    )
    
    encoded = packet.encode()
    print(f"Packet length: {len(encoded)} bytes")
    print(f"Hex: {encoded.hex()}")
    
    # Декодирование
    decoded = SpacePacket.decode(encoded)
    print(f"APID: {decoded.primary_header.apid:#05x}")
    print(f"Data: {decoded.user_data}")


def example_tm_frame():
    """Пример создания TM Transfer Frame"""
    print("\n=== TM Transfer Frame Example ===\n")
    
    frame = TMTransferFrame(
        spacecraft_id=123,
        virtual_channel_id=0,
        master_channel_frame_count=0,
        virtual_channel_frame_count=0,
        data=b'Telemetry data...'
    )
    
    encoded = frame.encode()
    print(f"Frame length: {len(encoded)} bytes")
    print(f"ASM: {encoded[:4].hex()}")


def example_ccsds_builder():
    """Пример использования построителя пакетов"""
    print("\n=== CCSDS Packet Builder Example ===\n")
    
    builder = CCSDSPacketBuilder(
        spacecraft_id=123,
        apid_base=0x100
    )
    
    # Создание нескольких пакетов
    for i in range(3):
        packet = builder.create_tm_packet(
            apid=APIDCategory.HOUSEKEEPING,
            data=f"HK data {i}".encode()
        )
        encoded = packet.encode()
        print(f"Packet {i}: APID={packet.primary_header.apid:#05x}, "
              f"Seq={packet.primary_header.packet_sequence_count}, "
              f"Len={len(encoded)}")


if __name__ == '__main__':
    example_space_packet()
    example_tm_frame()
    example_ccsds_builder()
