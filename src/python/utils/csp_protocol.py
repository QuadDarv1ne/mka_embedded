#!/usr/bin/env python3
"""
CubeSat Space Protocol (CSP) Implementation
Реализация CSP для бортовой сети CubeSat

CSP — легковесный протокол для связи между подсистемами спутника.
Разработан в Aalborg University для CubeSat проектов.

Формат CSP пакета:
┌────────────────────────────────────────────────────────────────────┐
│  Header (4 bytes)                                                  │
├────────┬────────┬─────────┬─────────┬────────┬───────┬───────────┤
│ Prio   │ Source │ Dst     │ DstPort │ SrcPort│ Flags │ HMAC/Len  │
│ 2 bits │ 5 bits │ 5 bits  │ 6 bits  │ 6 bits │ 8 bits│ 8/16 bits │
└────────┴────────┴─────────┴─────────┴────────┴───────┴───────────┘
┌────────────────────────────────────────────────────────────────────┐
│  Data (variable length, max 65535 bytes)                          │
└────────────────────────────────────────────────────────────────────┘
"""

from dataclasses import dataclass, field
from typing import Optional, Dict, Callable, Any
from enum import IntFlag, IntEnum
import struct
import hashlib
import hmac


class CSPError(Exception):
    """Исключение CSP"""
    pass


class CSPPriority(IntEnum):
    """Приоритеты CSP"""
    CRITICAL = 0   # Критические сообщения (reset, emergency)
    HIGH = 1       # Высокий приоритет (telemetry, commands)
    NORMAL = 2     # Обычный приоритет
    LOW = 3        # Низкий приоритет (logging, debug)


class CSPFlags(IntFlag):
    """Флаги CSP пакета"""
    NONE = 0
    MORE = 0x01       # More fragments
    HMAC1 = 0x02      # HMAC type 1
    HMAC2 = 0x04      # HMAC type 2
    XTEA = 0x08       # XTEA encryption
    RDP = 0x10        # Reliable Data Protocol
    CRC32 = 0x20      # CRC32 checksum


# ============================================================================
# Стандартные порты CSP
# ============================================================================

class CSPPorts(IntEnum):
    """Стандартные CSP порты"""
    # Зарезервированные порты (0-7)
    CMP = 0           # CSP Management Protocol
    PING = 1          # Ping service
    PS = 2            # Process list
    MEMFREE = 3       # Memory free
    REBOOT = 4        # Reboot request
    BUF_FREE = 5      # Buffer free
    UART1 = 6         # UART1 router
    CAN = 7           # CAN router
    
    # Порты служб (8-47)
    # Зависят от миссии
    
    # Порты пользователя (48-205)
    TELEMETRY = 10    # Телеметрия
    COMMAND = 11      # Команды
    ADCS = 20         # ADCS subsystem
    EPS = 21          # EPS subsystem
    COMM = 22         # COMM subsystem
    PAYLOAD = 23      # Payload
    
    # Зарезервировано (206-255)
    PROMISC = 255     # Promiscuous mode


# ============================================================================
# CSP Packet
# ============================================================================

@dataclass
class CSPPacket:
    """CSP пакет"""
    priority: int = CSPPriority.NORMAL
    source: int = 0         # Node address (0-31)
    destination: int = 0    # Node address (0-31)
    dest_port: int = 0      # Port (0-63)
    source_port: int = 0    # Port (0-63)
    flags: int = CSPFlags.NONE
    data: bytes = b''
    
    # HMAC/Length field interpretation
    use_hmac: bool = False
    hmac_value: int = 0
    
    def __post_init__(self):
        if self.priority not in range(4):
            raise CSPError(f"Invalid priority: {self.priority}")
        if self.source not in range(32):
            raise CSPError(f"Invalid source: {self.source}")
        if self.destination not in range(32):
            raise CSPError(f"Invalid destination: {self.destination}")
        if self.dest_port not in range(64):
            raise CSPError(f"Invalid dest_port: {self.dest_port}")
        if self.source_port not in range(64):
            raise CSPError(f"Invalid source_port: {self.source_port}")
    
    def encode(self) -> bytes:
        """Кодирование пакета в байты"""
        # Header (4 байта)
        # Byte 0: Priority(2) + Source(5) + Dest(5) upper bits
        # Byte 1: Dest(5) lower bits + DestPort(6) + SrcPort(6) upper bits
        # Byte 2: SrcPort(6) lower bits + Flags(8)
        # Byte 3: HMAC/Length field
        
        byte0 = ((self.priority & 0x03) << 6) | ((self.source & 0x1F) << 1) | ((self.destination >> 4) & 0x01)
        byte1 = ((self.destination & 0x0F) << 4) | ((self.dest_port >> 2) & 0x0F)
        byte2 = ((self.dest_port & 0x03) << 6) | (self.source_port & 0x3F)
        byte3 = self.flags
        
        header = bytes([byte0, byte1, byte2, byte3])
        
        # HMAC/Length field
        if self.flags & (CSPFlags.HMAC1 | CSPFlags.HMAC2):
            # HMAC field (2 байта)
            length_field = struct.pack('>H', self.hmac_value)
        else:
            # Length field (1 байт)
            length_field = bytes([len(self.data) & 0xFF])
        
        # Полный пакет
        packet = header + length_field + self.data
        
        # CRC32 если нужно
        if self.flags & CSPFlags.CRC32:
            crc = self._calculate_crc32(packet)
            packet += struct.pack('<I', crc)
        
        return packet
    
    @classmethod
    def decode(cls, data: bytes) -> 'CSPPacket':
        """Декодирование пакета из байтов"""
        if len(data) < 5:  # Минимум header + length
            raise CSPError("Packet too short")
        
        # Header
        byte0, byte1, byte2, byte3 = data[:4]
        
        priority = (byte0 >> 6) & 0x03
        source = (byte0 >> 1) & 0x1F
        destination = ((byte0 & 0x01) << 4) | ((byte1 >> 4) & 0x0F)
        dest_port = ((byte1 & 0x0F) << 2) | ((byte2 >> 6) & 0x03)
        source_port = byte2 & 0x3F
        flags = byte3
        
        # HMAC/Length field
        offset = 4
        use_hmac = bool(flags & (CSPFlags.HMAC1 | CSPFlags.HMAC2))
        
        if use_hmac:
            hmac_value = struct.unpack('>H', data[offset:offset+2])[0]
            offset += 2
            payload = data[offset:]
        else:
            length = data[offset]
            offset += 1
            payload = data[offset:offset+length]
            hmac_value = 0
        
        # CRC32 проверка
        if flags & CSPFlags.CRC32:
            if len(payload) < 4:
                raise CSPError("Missing CRC32")
            
            received_crc = struct.unpack('<I', payload[-4:])[0]
            payload = payload[:-4]
            
            # Проверка CRC
            packet_without_crc = data[:offset] + payload
            calculated_crc = cls._calculate_crc32(packet_without_crc)
            
            if received_crc != calculated_crc:
                raise CSPError(f"CRC32 error: {received_crc:08X} vs {calculated_crc:08X}")
        
        return cls(
            priority=priority,
            source=source,
            destination=destination,
            dest_port=dest_port,
            source_port=source_port,
            flags=flags,
            data=payload,
            use_hmac=use_hmac,
            hmac_value=hmac_value
        )
    
    @staticmethod
    def _calculate_crc32(data: bytes) -> int:
        """Расчёт CRC32"""
        import binascii
        return binascii.crc32(data) & 0xFFFFFFFF
    
    def __str__(self) -> str:
        return (f"CSP[src={self.source}:{self.source_port}, "
                f"dst={self.destination}:{self.dest_port}, "
                f"prio={self.priority}, flags={self.flags}, "
                f"len={len(self.data)}]")


# ============================================================================
# CSP Router
# ============================================================================

class CSPRouter:
    """
    Маршрутизатор CSP пакетов.
    
    Управляет маршрутизацией пакетов между локальными службами
    и удалёнными узлами.
    """
    
    # Адрес broadcast
    BROADCAST_ADDR = 31
    
    def __init__(self, node_address: int, node_name: str = ""):
        """
        Args:
            node_address: Адрес узла (0-30, 31=broadcast)
            node_name: Имя узла
        """
        if node_address not in range(32):
            raise CSPError(f"Invalid node address: {node_address}")
        
        self.address = node_address
        self.name = node_name
        
        # Зарегистрированные службы
        self.services: Dict[int, Callable[[CSPPacket], Optional[bytes]]] = {}
        
        # Обработчики исходящих пакетов
        self.tx_handlers: list = []
        
        # Таблица маршрутизации: address -> (interface, next_hop)
        self.routes: Dict[int, tuple] = {}
        
        # Статистика
        self.stats = {
            'rx_packets': 0,
            'tx_packets': 0,
            'rx_bytes': 0,
            'tx_bytes': 0,
            'rx_errors': 0,
            'tx_errors': 0
        }
    
    def register_service(self, port: int, 
                         handler: Callable[[CSPPacket], Optional[bytes]]) -> None:
        """
        Регистрация службы на порту.
        
        Args:
            port: Порт (0-255)
            handler: Функция обработки пакетов
        """
        self.services[port] = handler
    
    def unregister_service(self, port: int) -> None:
        """Отмена регистрации службы."""
        self.services.pop(port, None)
    
    def add_route(self, address: int, interface: Any, next_hop: int = None) -> None:
        """
        Добавление маршрута.
        
        Args:
            address: Адрес назначения
            interface: Интерфейс для отправки
            next_hop: Адрес следующего узла (если None = напрямую)
        """
        self.routes[address] = (interface, next_hop)
    
    def add_tx_handler(self, handler: Callable[[bytes], bool]) -> None:
        """
        Добавление обработчика исходящих пакетов.
        
        Args:
            handler: Функция, принимающая bytes и возвращающая bool
        """
        self.tx_handlers.append(handler)
    
    def receive_packet(self, data: bytes) -> Optional[bytes]:
        """
        Обработка входящего пакета.
        
        Args:
            data: Сырые данные пакета
        
        Returns:
            Ответный пакет или None
        """
        try:
            packet = CSPPacket.decode(data)
            self.stats['rx_packets'] += 1
            self.stats['rx_bytes'] += len(data)
            
            # Проверка адресации
            if packet.destination != self.address and \
               packet.destination != self.BROADCAST_ADDR:
                # Пакет не для нас - маршрутизация
                return self._route_packet(packet)
            
            # Обработка локальной службой
            if packet.dest_port in self.services:
                response = self.services[packet.dest_port](packet)
                if response is not None:
                    return self._create_response(packet, response)
            
            return None
            
        except CSPError as e:
            self.stats['rx_errors'] += 1
            print(f"CSP receive error: {e}")
            return None
    
    def send_packet(self, dest: int, dest_port: int, data: bytes,
                   priority: int = CSPPriority.NORMAL,
                   source_port: int = CSPPorts.TELEMETRY) -> bool:
        """
        Отправка пакета.
        
        Args:
            dest: Адрес назначения
            dest_port: Порт назначения
            data: Данные
            priority: Приоритет
            source_port: Исходный порт
        
        Returns:
            Успешность отправки
        """
        packet = CSPPacket(
            priority=priority,
            source=self.address,
            destination=dest,
            dest_port=dest_port,
            source_port=source_port,
            data=data
        )
        
        return self._route_packet(packet) is not None
    
    def _route_packet(self, packet: CSPPacket) -> Optional[bytes]:
        """Маршрутизация пакета."""
        encoded = packet.encode()
        
        success = False
        for handler in self.tx_handlers:
            if handler(encoded):
                success = True
                break
        
        if success:
            self.stats['tx_packets'] += 1
            self.stats['tx_bytes'] += len(encoded)
        else:
            self.stats['tx_errors'] += 1
        
        return None
    
    def _create_response(self, request: CSPPacket, data: bytes) -> bytes:
        """Создание ответного пакета."""
        response = CSPPacket(
            priority=request.priority,
            source=self.address,
            destination=request.source,
            dest_port=request.source_port,
            source_port=request.dest_port,
            data=data
        )
        return response.encode()


# ============================================================================
# CSP Services
# ============================================================================

class CSPServicePing:
    """CSP Ping Service"""
    
    def __init__(self, router: CSPRouter):
        self.router = router
        router.register_service(CSPPorts.PING, self.handle)
    
    def handle(self, packet: CSPPacket) -> bytes:
        """Обработка ping запроса."""
        # Echo back
        return packet.data


class CSPServiceMemory:
    """CSP Memory Info Service"""
    
    def __init__(self, router: CSPRouter):
        router.register_service(CSPPorts.MEMFREE, self.handle)
    
    def handle(self, packet: CSPPacket) -> bytes:
        """Возврат информации о памяти."""
        import gc
        free = gc.mem_free() if hasattr(gc, 'mem_free') else 65536
        return struct.pack('<I', free)


class CSPServiceReboot:
    """CSP Reboot Service"""
    
    def __init__(self, router: CSPRouter):
        self.router = router
        router.register_service(CSPPorts.REBOOT, self.handle)
    
    def handle(self, packet: CSPPacket) -> bytes:
        """Обработка запроса перезагрузки."""
        if len(packet.data) >= 4:
            magic = struct.unpack('<I', packet.data[:4])[0]
            if magic == 0x80078007:  # Magic number
                # В реальной системе здесь была бы перезагрузка
                print("REBOOT requested!")
                return b'ACK'
        
        return b'NAK'


# ============================================================================
# Примеры использования
# ============================================================================

def example_csp_basic():
    """Базовый пример CSP"""
    print("=== CSP Basic Example ===\n")
    
    # Создание пакета
    packet = CSPPacket(
        priority=CSPPriority.HIGH,
        source=1,          # OBC
        destination=2,     # ADCS
        dest_port=CSPPorts.ADCS,
        source_port=CSPPorts.COMMAND,
        data=b'SET_MODE:SAFE'
    )
    
    print(f"Created: {packet}")
    
    # Кодирование
    encoded = packet.encode()
    print(f"Encoded ({len(encoded)} bytes): {encoded.hex()}")
    
    # Декодирование
    decoded = CSPPacket.decode(encoded)
    print(f"Decoded: {decoded}")
    print(f"Data: {decoded.data.decode()}")


def example_csp_router():
    """Пример маршрутизатора"""
    print("\n=== CSP Router Example ===\n")
    
    # Создание маршрутизатора для OBC (адрес 1)
    obc_router = CSPRouter(node_address=1, node_name="OBC")
    
    # Регистрация службы телеметрии
    def telemetry_handler(packet: CSPPacket) -> bytes:
        print(f"Telemetry request from node {packet.source}")
        return b'VBAT=7.2;TEMP=25;MODE=NOMINAL'
    
    obc_router.register_service(CSPPorts.TELEMETRY, telemetry_handler)
    
    # Регистрация ping службы
    CSPServicePing(obc_router)
    
    # Создание тестового пакета
    test_packet = CSPPacket(
        source=2,           # Ground station
        destination=1,      # OBC
        dest_port=CSPPorts.TELEMETRY,
        source_port=CSPPorts.COMMAND,
        data=b'GET_HK'
    )
    
    # Обработка
    response = obc_router.receive_packet(test_packet.encode())
    if response:
        print(f"Response: {response}")


def example_csp_with_crc():
    """Пример с CRC32"""
    print("\n=== CSP with CRC32 Example ===\n")
    
    packet = CSPPacket(
        source=1,
        destination=2,
        dest_port=10,
        source_port=10,
        flags=CSPFlags.CRC32,
        data=b'Important telemetry data'
    )
    
    encoded = packet.encode()
    print(f"With CRC32 ({len(encoded)} bytes): {encoded.hex()}")
    
    # Имитация ошибки
    corrupted = bytearray(encoded)
    corrupted[-5] ^= 0xFF  # Повреждение данных
    
    try:
        CSPPacket.decode(bytes(corrupted))
    except CSPError as e:
        print(f"CRC detected corruption: {e}")
    
    # Правильный пакет декодируется
    decoded = CSPPacket.decode(encoded)
    print(f"Valid packet decoded: {decoded.data.decode()}")


if __name__ == '__main__':
    example_csp_basic()
    example_csp_router()
    example_csp_with_crc()
