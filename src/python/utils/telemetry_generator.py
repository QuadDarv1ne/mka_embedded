#!/usr/bin/env python3
"""
Telemetry Generator for MKA
Генератор телеметрии для тестирования и симуляции

Создаёт реалистичные данные телеметрии спутника в форматах:
- Бинарный формат (для передачи по радиоканалу)
- JSON формат (для отладки и визуализации)
- CCSDS Space Packets
"""

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Tuple
from enum import IntEnum
import struct
import time
import random
import math
import json


class TelemetryError(Exception):
    pass


# ============================================================================
# Идентификаторы телеметрии (APID)
# ============================================================================

class TelemetryAPID(IntEnum):
    """APID для различных типов телеметрии"""
    HOUSEKEEPING = 0x100
    ATTITUDE = 0x101
    POWER = 0x102
    THERMAL = 0x103
    COMM_STATUS = 0x104
    PAYLOAD = 0x105
    ADCS_DETAIL = 0x110
    ORBIT = 0x111
    EVENT = 0x200


# ============================================================================
# Структуры данных телеметрии
# ============================================================================

@dataclass
class HousekeepingData:
    """Housekeeping телеметрия (общее состояние)"""
    uptime: int = 0                    # секунды
    mode: int = 0                      # режим работы
    reset_count: int = 0               # количество сбросов
    last_reset_reason: int = 0         # причина последнего сброса
    free_memory: int = 0               # свободная память (байты)
    cpu_load: float = 0.0              # загрузка CPU (%)
    task_count: int = 0                # количество задач RTOS
    
    def to_binary(self) -> bytes:
        return struct.pack('>IHBBHfB',
            self.uptime, self.mode, self.reset_count, self.last_reset_reason,
            self.free_memory, self.cpu_load, self.task_count)
    
    def to_dict(self) -> Dict:
        return {
            'uptime': self.uptime,
            'mode': self.mode,
            'reset_count': self.reset_count,
            'last_reset_reason': self.last_reset_reason,
            'free_memory': self.free_memory,
            'cpu_load': self.cpu_load,
            'task_count': self.task_count
        }


@dataclass
class PowerData:
    """Телеметрия системы электропитания"""
    battery_voltage: float = 7.4       # Вольт
    battery_current: float = 0.5       # Ампер
    battery_charge: float = 80.0       # Процент
    solar_voltage: float = 8.0         # Вольт
    solar_current: float = 0.3         # Ампер
    bus_3v3: float = 3.3               # Вольт
    bus_5v: float = 5.0                # Вольт
    temperature: float = 25.0          # °C
    power_rails: int = 0x0F            # Битовая маска включённых линий
    
    def to_binary(self) -> bytes:
        return struct.pack('>fffffffB',
            self.battery_voltage, self.battery_current, self.battery_charge,
            self.solar_voltage, self.solar_current, self.bus_3v3, self.bus_5v,
            self.power_rails)
    
    def to_dict(self) -> Dict:
        return {
            'battery_voltage': round(self.battery_voltage, 3),
            'battery_current': round(self.battery_current, 3),
            'battery_charge': round(self.battery_charge, 1),
            'solar_voltage': round(self.solar_voltage, 3),
            'solar_current': round(self.solar_current, 3),
            'bus_3v3': round(self.bus_3v3, 3),
            'bus_5v': round(self.bus_5v, 3),
            'temperature': round(self.temperature, 1),
            'power_rails': self.power_rails
        }


@dataclass
class AttitudeData:
    """Телеметрия ориентации"""
    # Кватернион [w, x, y, z]
    quaternion: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    # Угловая скорость (рад/с)
    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Магнитное поле (μT)
    magnetic_field: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Вектор солнца (в body frame)
    sun_vector: Tuple[float, float, float] = (1.0, 0.0, 0.0)
    sun_visible: bool = True
    # Режим ADCS
    adcs_mode: int = 0
    adcs_status: int = 0
    
    def to_binary(self) -> bytes:
        return struct.pack('>ffffffffffBB',
            *self.quaternion, *self.angular_velocity,
            self.sun_visible, self.adcs_mode, self.adcs_status)
    
    def to_dict(self) -> Dict:
        return {
            'quaternion': list(self.quaternion),
            'angular_velocity': [round(v, 6) for v in self.angular_velocity],
            'magnetic_field': list(self.magnetic_field),
            'sun_vector': list(self.sun_vector),
            'sun_visible': self.sun_visible,
            'adcs_mode': self.adcs_mode,
            'adcs_status': self.adcs_status
        }


@dataclass
class ThermalData:
    """Телеметрия температур"""
    obc_temp: float = 25.0             # OBC
    battery_temp: float = 20.0         # Батарея
    solar_panel_temp: float = 30.0     # Солнечная панель
    external_temp: float = -10.0       # Внешняя
    radio_temp: float = 35.0           # Радиомодуль
    payload_temp: float = 25.0         # Полезная нагрузка
    
    def to_binary(self) -> bytes:
        return struct.pack('>ffffff',
            self.obc_temp, self.battery_temp, self.solar_panel_temp,
            self.external_temp, self.radio_temp, self.payload_temp)
    
    def to_dict(self) -> Dict:
        return {k: round(v, 1) for k, v in {
            'obc_temp': self.obc_temp,
            'battery_temp': self.battery_temp,
            'solar_panel_temp': self.solar_panel_temp,
            'external_temp': self.external_temp,
            'radio_temp': self.radio_temp,
            'payload_temp': self.payload_temp
        }.items()}


@dataclass
class OrbitData:
    """Орбитальная информация"""
    timestamp: int = 0                 # Unix time
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # ECEF, км
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # км/с
    latitude: float = 0.0              # °
    longitude: float = 0.0             # °
    altitude: float = 400.0            # км
    orbital_period: float = 92.0       # минуты
    
    def to_binary(self) -> bytes:
        return struct.pack('>Ifffffff',
            self.timestamp, *self.position, *self.velocity,
            self.altitude, self.orbital_period)
    
    def to_dict(self) -> Dict:
        return {
            'timestamp': self.timestamp,
            'position': list(self.position),
            'velocity': list(self.velocity),
            'latitude': round(self.latitude, 4),
            'longitude': round(self.longitude, 4),
            'altitude': round(self.altitude, 2),
            'orbital_period': round(self.orbital_period, 2)
        }


# ============================================================================
# Генератор телеметрии
# ============================================================================

class TelemetryGenerator:
    """
    Генератор телеметрии МКА
    
    Создаёт реалистичные данные телеметрии с учётом:
    - Динамики спутника
    - Орбитального положения
    - Времени суток (освещённость)
    - Шума датчиков
    """
    
    def __init__(self, spacecraft_id: int = 123):
        self.spacecraft_id = spacecraft_id
        self.sequence_count = 0
        
        # Внутреннее состояние
        self.start_time = time.time()
        self.uptime = 0
        self.mode = 4  # NOMINAL
        self.orbit_phase = 0.0  # Фаза орбиты (0-1)
        
        # Параметры симуляции
        self.orbit_period = 92.0 * 60  # секунды
        self.altitude = 400.0  # км
        self.inclination = 51.6  # °
        
        # Текущее состояние
        self.power = PowerData()
        self.attitude = AttitudeData()
        self.thermal = ThermalData()
        self.orbit = OrbitData()
        self.hk = HousekeepingData()
        
    def update(self, dt: float = 1.0):
        """Обновление состояния спутника"""
        self.uptime += dt
        self.orbit_phase = (self.orbit_phase + dt / self.orbit_period) % 1.0
        
        # Обновление орбиты
        self._update_orbit()
        
        # Обновление питания
        self._update_power(dt)
        
        # Обновление ориентации
        self._update_attitude(dt)
        
        # Обновление температур
        self._update_thermal(dt)
        
        # Обновление HK
        self._update_housekeeping()
    
    def _update_orbit(self):
        """Обновление орбитальных параметров"""
        # Положение на орбите (круговая орбита)
        angle = self.orbit_phase * 2 * math.pi
        
        # Позиция в ECEF (упрощённо)
        r = 6371 + self.altitude  # км
        self.orbit.position = (
            r * math.cos(angle) * math.cos(math.radians(self.inclination)),
            r * math.sin(angle),
            r * math.cos(angle) * math.sin(math.radians(self.inclination))
        )
        
        # Скорость
        v = math.sqrt(398600.4 / r)  # км/с
        self.orbit.velocity = (
            -v * math.sin(angle),
            v * math.cos(angle),
            0.0
        )
        
        # Широта/долгота
        self.orbit.latitude = math.degrees(math.asin(self.orbit.position[2] / r))
        self.orbit.longitude = math.degrees(math.atan2(self.orbit.position[1], 
                                                        self.orbit.position[0]))
        
        self.orbit.altitude = self.altitude
        self.orbit.timestamp = int(self.start_time + self.uptime)
        self.orbit.orbital_period = self.orbit_period / 60
    
    def _update_power(self, dt: float):
        """Обновление параметров питания"""
        # Освещённость (простая модель)
        # Солнце освещает спутник ~60% орбиты
        sun_fraction = 0.6
        in_sunlight = self.orbit_phase < sun_fraction or self.orbit_phase > (1 - sun_fraction/2)
        
        if in_sunlight:
            # Зарядка
            self.power.solar_voltage = 8.0 + random.gauss(0, 0.1)
            self.power.solar_current = 0.3 + random.gauss(0, 0.02)
            self.power.battery_charge = min(100, self.power.battery_charge + 0.01 * dt)
        else:
            # Тень
            self.power.solar_voltage = 0.0
            self.power.solar_current = 0.0
            self.power.battery_charge = max(0, self.power.battery_charge - 0.02 * dt)
        
        # Напряжение батареи зависит от заряда
        self.power.battery_voltage = 6.0 + 2.4 * (self.power.battery_charge / 100)
        self.power.battery_voltage += random.gauss(0, 0.05)
        
        # Ток потребления
        self.power.battery_current = 0.5 + random.gauss(0, 0.05)
        
        # Шины питания
        self.power.bus_3v3 = 3.3 + random.gauss(0, 0.02)
        self.power.bus_5v = 5.0 + random.gauss(0, 0.03)
        
        # Температура батареи
        self.power.temperature = self.thermal.battery_temp
    
    def _update_attitude(self, dt: float):
        """Обновление параметров ориентации"""
        # Малое случайное вращение
        wx, wy, wz = self.attitude.angular_velocity
        
        # Случайное блуждание угловой скорости
        wx += random.gauss(0, 0.0001)
        wy += random.gauss(0, 0.0001)
        wz += random.gauss(0, 0.0001)
        
        # Затухание (магнитное демпфирование)
        damping = 0.999
        wx *= damping
        wy *= damping
        wz *= damping
        
        self.attitude.angular_velocity = (wx, wy, wz)
        
        # Обновление кватерниона (упрощённо)
        q = list(self.attitude.quaternion)
        dq = 0.5 * dt
        q[1] += wx * dq
        q[2] += wy * dq
        q[3] += wz * dq
        
        # Нормализация
        norm = math.sqrt(sum(x*x for x in q))
        q = [x/norm for x in q]
        self.attitude.quaternion = tuple(q)
        
        # Магнитное поле (зависит от положения)
        B0 = 30  # μT на поверхности
        r = 6371 + self.altitude
        B = B0 * (6371/r)**3
        self.attitude.magnetic_field = (
            B * math.cos(self.orbit_phase * 2 * math.pi) + random.gauss(0, 0.5),
            random.gauss(0, 0.5),
            B * 0.5 + random.gauss(0, 0.5)
        )
        
        # Вектор солнца
        in_sunlight = self.power.solar_voltage > 1.0
        self.attitude.sun_visible = in_sunlight
        
        if in_sunlight:
            self.attitude.sun_vector = (
                0.8 + random.gauss(0, 0.05),
                0.1 + random.gauss(0, 0.05),
                0.5 + random.gauss(0, 0.05)
            )
            norm = math.sqrt(sum(x*x for x in self.attitude.sun_vector))
            self.attitude.sun_vector = tuple(x/norm for x in self.attitude.sun_vector)
    
    def _update_thermal(self, dt: float):
        """Обновление температур"""
        in_sunlight = self.power.solar_voltage > 1.0
        
        # Внешняя температура зависит от освещённости
        target_external = 50.0 if in_sunlight else -50.0
        self.thermal.external_temp += (target_external - self.thermal.external_temp) * 0.01
        
        # OBC температура (стабильная)
        self.thermal.obc_temp = 25 + random.gauss(0, 0.5)
        
        # Батарея
        self.thermal.battery_temp = 20 + self.power.battery_current * 5 + random.gauss(0, 0.3)
        
        # Радиомодуль
        self.thermal.radio_temp = 30 + random.gauss(0, 1)
        
        # Солнечная панель
        self.thermal.solar_panel_temp = 40 if in_sunlight else -20
        self.thermal.solar_panel_temp += random.gauss(0, 2)
    
    def _update_housekeeping(self):
        """Обновление housekeeping данных"""
        self.hk.uptime = int(self.uptime)
        self.hk.mode = self.mode
        self.hk.free_memory = 45000 + random.randint(-1000, 1000)
        self.hk.cpu_load = 20 + random.gauss(0, 5)
        self.hk.cpu_load = max(0, min(100, self.hk.cpu_load))
        self.hk.task_count = 8
    
    # ========================================================================
    # Генерация пакетов
    # ========================================================================
    
    def generate_packet(self, apid: TelemetryAPID) -> bytes:
        """Генерация CCSDS Space Packet"""
        # Первичный заголовок (6 байт)
        header = struct.pack('>HHH',
            (0 << 13) | (0 << 12) | (1 << 11) | apid,  # Version, Type, SecHdr, APID
            (3 << 14) | self.sequence_count,           # SeqFlags, SeqCount
            0                                            # Placeholder length
        )
        
        # Вторичный заголовок (время, 8 байт)
        time_code = int(time.time()) << 16
        secondary_header = struct.pack('>Q', time_code)
        
        # Данные в зависимости от APID
        if apid == TelemetryAPID.HOUSEKEEPING:
            data = self.hk.to_binary()
        elif apid == TelemetryAPID.POWER:
            data = self.power.to_binary()
        elif apid == TelemetryAPID.ATTITUDE:
            data = self.attitude.to_binary()
        elif apid == TelemetryAPID.THERMAL:
            data = self.thermal.to_binary()
        elif apid == TelemetryAPID.ORBIT:
            data = self.orbit.to_binary()
        else:
            data = b''
        
        # Полный пакет
        packet = header + secondary_header + data
        
        # Обновление длины
        packet_len = len(packet) - 7  # Length field = total - 7
        packet = header[:4] + struct.pack('>H', packet_len) + \
                 header[6:] + secondary_header + data
        
        self.sequence_count = (self.sequence_count + 1) & 0x3FFF
        
        return packet
    
    def generate_all_packets(self) -> Dict[int, bytes]:
        """Генерация всех типов телеметрии"""
        return {
            TelemetryAPID.HOUSEKEEPING: self.generate_packet(TelemetryAPID.HOUSEKEEPING),
            TelemetryAPID.POWER: self.generate_packet(TelemetryAPID.POWER),
            TelemetryAPID.ATTITUDE: self.generate_packet(TelemetryAPID.ATTITUDE),
            TelemetryAPID.THERMAL: self.generate_packet(TelemetryAPID.THERMAL),
            TelemetryAPID.ORBIT: self.generate_packet(TelemetryAPID.ORBIT),
        }
    
    def generate_json(self) -> Dict[str, Any]:
        """Генерация JSON телеметрии"""
        return {
            'spacecraft_id': self.spacecraft_id,
            'timestamp': int(time.time()),
            'uptime': int(self.uptime),
            'mode': self.mode,
            'housekeeping': self.hk.to_dict(),
            'power': self.power.to_dict(),
            'attitude': self.attitude.to_dict(),
            'thermal': self.thermal.to_dict(),
            'orbit': self.orbit.to_dict()
        }


# ============================================================================
# Демо
# ============================================================================

def main():
    """Демонстрация генератора телеметрии"""
    print("=== MKA Telemetry Generator Demo ===\n")
    
    gen = TelemetryGenerator(spacecraft_id=123)
    
    # Симуляция на несколько минут
    for i in range(5):
        gen.update(dt=60)  # 1 минута
        
        print(f"--- Uptime: {gen.uptime:.0f}s ---")
        print(f"Orbit phase: {gen.orbit_phase:.2%}")
        print(f"Battery: {gen.power.battery_charge:.1f}% @ {gen.power.battery_voltage:.2f}V")
        print(f"In sunlight: {gen.power.solar_voltage > 1.0}")
        print(f"Temperature: OBC={gen.thermal.obc_temp:.1f}°C, Ext={gen.thermal.external_temp:.1f}°C")
        print()
    
    # Генерация пакетов
    print("=== Generated Packets ===")
    packets = gen.generate_all_packets()
    for apid, data in packets.items():
        print(f"APID 0x{apid:03X}: {len(data)} bytes")
        print(f"  Hex: {data[:20].hex()}...")
        print()
    
    # JSON вывод
    print("=== JSON Telemetry ===")
    telemetry = gen.generate_json()
    print(json.dumps(telemetry, indent=2))


if __name__ == '__main__':
    main()
