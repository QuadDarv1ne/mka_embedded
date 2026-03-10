#!/usr/bin/env python3
"""
AX.25 Protocol Implementation
Реализация протокола AX.25 для любительской спутниковой радиосвязи

AX.25 — протокол канального уровня, используемый в любительской
пакетной радиосвязи и многих CubeSat миссиях.

Формат кадра:
┌──────────┬──────────┬──────────┬──────────┬─────────┬─────────┬──────┐
│  Flag    │ Address  │ Control  │  PID     │  Info   │  FCS    │ Flag │
│  01111110│  (N*8 B) │  1-2 B   │  1 B     │  0-256B │  2 B    │      │
└──────────┴──────────┴──────────┴──────────┴─────────┴─────────┴──────┘
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from enum import IntEnum
import struct


class AX25Error(Exception):
    """Исключение протокола AX.25"""
    pass


class FrameType(IntEnum):
    """Типы кадров"""
    I_FRAME = 0x00    # Information
    S_FRAME = 0x01    # Supervisory
    U_FRAME = 0x03    # Unnumbered


class SupervisoryType(IntEnum):
    """Типы S-кадров"""
    RR = 0x01   # Receiver Ready
    RNR = 0x05  # Receiver Not Ready
    REJ = 0x09  # Reject


class UnnumberedType(IntEnum):
    """Типы U-кадров"""
    SABM = 0x2F   # Set Asynchronous Balanced Mode
    DISC = 0x43   # Disconnect
    UA = 0x63     # Unnumbered Acknowledge
    DM = 0x0F     # Disconnect Mode
    UI = 0x03     # Unnumbered Information
    FRMR = 0x87   # Frame Reject


@dataclass
class AX25Address:
    """Адрес станции AX.25"""
    callsign: str
    ssid: int = 0
    
    def __post_init__(self):
        if len(self.callsign) > 6:
            raise AX25Error("Callsign too long (max 6 chars)")
        if self.ssid < 0 or self.ssid > 15:
            raise AX25Error("SSID must be 0-15")
    
    def encode(self, is_last: bool = False) -> bytes:
        """Кодирование адреса в байты"""
        # Callsign: 6 байт, сдвинуты влево на 1 бит
        result = bytearray(7)
        
        # Дополнение пробелами до 6 символов
        padded = self.callsign.ljust(6)
        
        for i, char in enumerate(padded):
            result[i] = ord(char) << 1
        
        # SSID байт
        ssid_byte = 0x60 | (self.ssid << 1)
        if is_last:
            ssid_byte |= 0x01  # Extension bit
        
        result[6] = ssid_byte
        return bytes(result)
    
    @classmethod
    def decode(cls, data: bytes) -> Tuple['AX25Address', bool]:
        """Декодирование адреса из байтов"""
        if len(data) < 7:
            raise AX25Error("Address field too short")
        
        # Декодирование callsign
        callsign = ""
        for i in range(6):
            char = chr(data[i] >> 1)
            if char != ' ':
                callsign += char
        
        # SSID
        ssid = (data[6] >> 1) & 0x0F
        
        # Extension bit (last address)
        is_last = bool(data[6] & 0x01)
        
        return cls(callsign, ssid), is_last


@dataclass
class AX25Frame:
    """Кадр AX.25"""
    dest: AX25Address
    source: AX25Address
    repeaters: List[AX25Address] = field(default_factory=list)
    control: int = 0x03  # UI frame
    pid: int = 0xF0      # No layer 3 protocol
    info: bytes = b''
    
    # Флаги
    FLAG = bytes([0x7E])
    
    def encode(self) -> bytes:
        """Кодирование кадра в байты"""
        frame = bytearray()
        
        # Флаг начала
        frame.extend(self.FLAG)
        
        # Поле адреса
        # Destination (not last if repeaters or source follows)
        all_addresses = [self.dest, self.source] + self.repeaters
        for i, addr in enumerate(all_addresses):
            is_last = (i == len(all_addresses) - 1)
            frame.extend(addr.encode(is_last))
        
        # Control field
        frame.append(self.control)
        
        # PID (только для I и UI кадров)
        if self._has_pid():
            frame.append(self.pid)
        
        # Info field
        if self.info:
            frame.extend(self.info)
        
        # FCS (Frame Check Sequence) - CRC-16-CCITT
        fcs = self._calculate_fcs(bytes(frame[1:]))  # Без флага
        frame.extend(struct.pack('<H', fcs))
        
        # Флаг конца
        frame.extend(self.FLAG)
        
        # Bit stuffing (побитовое заполнение)
        return self._bit_stuff(bytes(frame))
    
    @classmethod
    def decode(cls, data: bytes) -> 'AX25Frame':
        """Декодирование кадра из байтов"""
        # Удаление bit stuffing
        data = cls._bit_unstuff(data)
        
        # Проверка флагов
        if len(data) < 18:  # Минимальный размер
            raise AX25Error("Frame too short")
        
        if data[0] != 0x7E or data[-1] != 0x7E:
            raise AX25Error("Invalid flags")
        
        # Удаление флагов
        data = data[1:-1]
        
        # Проверка FCS
        fcs_received = struct.unpack('<H', data[-2:])[0]
        fcs_calculated = cls._calculate_fcs(data[:-2])
        
        if fcs_received != fcs_calculated:
            raise AX25Error(f"CRC error: received {fcs_received:04X}, calculated {fcs_calculated:04X}")
        
        # Парсинг адресов
        offset = 0
        addresses = []
        
        while True:
            if offset + 7 > len(data):
                raise AX25Error("Incomplete address field")
            
            addr, is_last = AX25Address.decode(data[offset:offset+7])
            addresses.append(addr)
            offset += 7
            
            if is_last:
                break
        
        if len(addresses) < 2:
            raise AX25Error("Missing source/destination")
        
        dest = addresses[0]
        source = addresses[1]
        repeaters = addresses[2:] if len(addresses) > 2 else []
        
        # Control field
        if offset >= len(data) - 2:
            raise AX25Error("Missing control field")
        
        control = data[offset]
        offset += 1
        
        # PID field (если есть)
        pid = 0xF0
        if cls._control_has_pid(control):
            if offset >= len(data) - 2:
                raise AX25Error("Missing PID field")
            pid = data[offset]
            offset += 1
        
        # Info field
        info = data[offset:-2]  # До FCS
        
        return cls(
            dest=dest,
            source=source,
            repeaters=repeaters,
            control=control,
            pid=pid,
            info=info
        )
    
    def _has_pid(self) -> bool:
        """Проверка наличия PID поля"""
        return self._control_has_pid(self.control)
    
    @staticmethod
    def _control_has_pid(control: int) -> bool:
        """Проверка наличия PID по control полю"""
        # I-frame или UI-frame
        frame_type = control & 0x01
        if frame_type == 0:  # I-frame
            return True
        if control & 0xEF == 0x03:  # UI-frame
            return True
        return False
    
    @staticmethod
    def _calculate_fcs(data: bytes) -> int:
        """Расчёт CRC-16-CCITT"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0x8408  # Polynomial reversed
                else:
                    crc >>= 1
        return crc ^ 0xFFFF
    
    @staticmethod
    def _bit_stuff(data: bytes) -> bytes:
        """
        Bit stuffing: после 5 единиц подряд вставляется 0.
        Это предотвращает появление флага 0x7E в данных.
        """
        result = bytearray()
        bit_buffer = 0
        bit_count = 0
        ones_count = 0
        
        for byte in data:
            for i in range(8):
                bit = (byte >> i) & 1
                
                bit_buffer |= (bit << bit_count)
                bit_count += 1
                
                if bit == 1:
                    ones_count += 1
                    if ones_count == 5:
                        # Вставляем 0 после 5 единиц
                        bit_count += 1
                        ones_count = 0
                else:
                    ones_count = 0
                
                if bit_count >= 8:
                    result.append(bit_buffer & 0xFF)
                    bit_buffer >>= 8
                    bit_count -= 8
        
        if bit_count > 0:
            result.append(bit_buffer)
        
        return bytes(result)
    
    @staticmethod
    def _bit_unstuff(data: bytes) -> bytes:
        """Обратная операция bit stuffing"""
        result = bytearray()
        bit_buffer = 0
        bit_count = 0
        ones_count = 0
        
        for byte in data:
            for i in range(8):
                bit = (byte >> i) & 1
                
                if ones_count == 5:
                    # Пропускаем stuffed bit
                    ones_count = 0
                    continue
                
                bit_buffer |= (bit << bit_count)
                bit_count += 1
                
                if bit == 1:
                    ones_count += 1
                else:
                    ones_count = 0
                
                if bit_count >= 8:
                    result.append(bit_buffer & 0xFF)
                    bit_buffer >>= 8
                    bit_count -= 8
        
        return bytes(result)
    
    @property
    def frame_type(self) -> FrameType:
        """Определение типа кадра"""
        if self.control & 0x01 == 0:
            return FrameType.I_FRAME
        elif self.control & 0x03 == 0x01:
            return FrameType.S_FRAME
        else:
            return FrameType.U_FRAME
    
    def get_info_string(self) -> str:
        """Получение info поля как строки"""
        try:
            return self.info.decode('ascii', errors='replace')
        except:
            return self.info.hex()


class AX25Encoder:
    """Кодировщик/декодировщик AX.25 для использования в бортовом ПО"""
    
    @staticmethod
    def create_ui_frame(
        dest_callsign: str,
        dest_ssid: int,
        source_callsign: str,
        source_ssid: int,
        info: bytes,
        repeaters: List[Tuple[str, int]] = None
    ) -> bytes:
        """Создание UI кадра (Unnumbered Information)"""
        dest = AX25Address(dest_callsign, dest_ssid)
        source = AX25Address(source_callsign, source_ssid)
        
        rep = []
        if repeaters:
            for callsign, ssid in repeaters:
                rep.append(AX25Address(callsign, ssid))
        
        frame = AX25Frame(
            dest=dest,
            source=source,
            repeaters=rep,
            control=0x03,  # UI frame
            pid=0xF0,      # No layer 3
            info=info
        )
        
        return frame.encode()
    
    @staticmethod
    def parse_frame(data: bytes) -> Optional[AX25Frame]:
        """Парсинг кадра с обработкой ошибок"""
        try:
            return AX25Frame.decode(data)
        except AX25Error as e:
            print(f"AX.25 error: {e}")
            return None


# ============================================================================
# Примеры использования
# ============================================================================

def example_beacon():
    """Пример создания маякового сообщения"""
    print("=== AX.25 Beacon Example ===\n")
    
    # Создание маяка от спутника
    info = b">CQ This is MKA-1 beacon, all systems nominal"
    
    frame_data = AX25Encoder.create_ui_frame(
        dest_callsign="CQ",
        dest_ssid=0,
        source_callsign="MKA1",
        source_ssid=0,
        info=info
    )
    
    print(f"Beacon frame ({len(frame_data)} bytes):")
    print(f"  Hex: {frame_data.hex()}")
    
    # Декодирование
    frame = AX25Frame.decode(frame_data)
    print(f"\nDecoded:")
    print(f"  From: {frame.source.callsign}-{frame.source.ssid}")
    print(f"  To: {frame.dest.callsign}-{frame.dest.ssid}")
    print(f"  Info: {frame.get_info_string()}")


def example_digipeater():
    """Пример с дигипитером"""
    print("\n=== AX.25 Digipeater Example ===\n")
    
    # Сообщение через дигипитер
    info = b"Hello from CubeSat!"
    
    frame_data = AX25Encoder.create_ui_frame(
        dest_callsign="GROUND",
        dest_ssid=1,
        source_callsign="SAT",
        source_ssid=0,
        info=info,
        repeaters=[("RELAY", 0), ("WIDE2", 2)]
    )
    
    print(f"Frame with digipeaters ({len(frame_data)} bytes):")
    
    frame = AX25Frame.decode(frame_data)
    print(f"  Route: {frame.source.callsign}")
    for rep in frame.repeaters:
        print(f"    -> {rep.callsign}-{rep.ssid}")
    print(f"    -> {frame.dest.callsign}-{frame.dest.ssid}")


def example_aprs_position():
    """Пример APRS позиции"""
    print("\n=== APRS Position Example ===\n")
    
    # APRS формат позиции: !DDMM.mmN/DDDMM.mmW$comments
    # Москва: 55°45'N, 37°37'E
    lat = "5545.00N"
    lon = "03737.00E"
    
    # APRS позиция с символом спутника
    aprs_info = f"!{lat}/{lon}O".encode()  # O = balloon/satellite symbol
    
    frame_data = AX25Encoder.create_ui_frame(
        dest_callsign="APRS",
        dest_ssid=0,
        source_callsign="MKA1",
        source_ssid=0,
        info=aprs_info
    )
    
    frame = AX25Frame.decode(frame_data)
    print(f"APRS Position: {frame.get_info_string()}")


if __name__ == '__main__':
    example_beacon()
    example_digipeater()
    example_aprs_position()
