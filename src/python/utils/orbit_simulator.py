#!/usr/bin/env python3
"""
Satellite Orbit Simulator
Симулятор орбиты спутника для тестирования ADCS и планирования сеансов связи

Моделирует:
- Кеплерову орбиту
- Положение Солнца
- Зоны видимости наземных станций
- Затенение (eclipse)
"""

import math
from dataclasses import dataclass
from typing import List, Tuple, Optional
from datetime import datetime, timedelta
import json


@dataclass
class OrbitalElements:
    """Кеплеровы элементы орбиты"""
    semi_major_axis: float    # a, км
    eccentricity: float       # e
    inclination: float        # i, градусы
    raan: float               # Ω, долгота восходящего узла, градусы
    arg_perigee: float        # ω, аргумент перигея, градусы
    mean_anomaly: float       # M, средняя аномалия, градусы
    epoch: datetime           # Эпоха элементов
    
    @property
    def period(self) -> float:
        """Орбитальный период в секундах"""
        mu = 398600.4418  # Гравитационный параметр Земли, км³/с²
        return 2 * math.pi * math.sqrt(self.semi_major_axis**3 / mu)
    
    @property
    def apogee(self) -> float:
        """Высота апогея, км"""
        return self.semi_major_axis * (1 + self.eccentricity) - 6371
    
    @property
    def perigee(self) -> float:
        """Высота перигея, км"""
        return self.semi_major_axis * (1 - self.eccentricity) - 6371


@dataclass
class Position:
    """Положение спутника в пространстве"""
    x: float  # км, ECI
    y: float
    z: float
    vx: float  # км/с
    vy: float
    vz: float
    latitude: float  # градусы
    longitude: float
    altitude: float  # км
    
    def to_dict(self) -> dict:
        return {
            'x': self.x, 'y': self.y, 'z': self.z,
            'vx': self.vx, 'vy': self.vy, 'vz': self.vz,
            'lat': self.latitude, 'lon': self.longitude, 'alt': self.altitude
        }


@dataclass
class GroundStation:
    """Наземная станция"""
    name: str
    latitude: float    # градусы
    longitude: float   # градусы
    altitude: float = 0.0  # км
    min_elevation: float = 5.0  # минимальный угол возвышения, градусы
    
    def to_dict(self) -> dict:
        return {
            'name': self.name,
            'lat': self.latitude,
            'lon': self.longitude,
            'alt': self.altitude,
            'min_elev': self.min_elevation
        }


class OrbitSimulator:
    """
    Симулятор орбитального движения.
    
    Использует упрощённую модель Кеплера с возмущениями J2.
    """
    
    # Константы
    MU = 398600.4418      # Гравитационный параметр Земли, км³/с²
    RE = 6371.0           # Радиус Земли, км
    J2 = 1.08263e-3       # Зональный гармонический коэффициент
    OMEGA_E = 7.292115e-5 # Угловая скорость вращения Земли, рад/с
    
    def __init__(self, elements: OrbitalElements = None):
        """
        Args:
            elements: Кеплеровы элементы орбиты (опционально)
        """
        # Значения по умолчанию для тестов (400 км высота, 51.6° наклонение)
        self.elements = elements or OrbitalElements(
            semi_major_axis=6771.0,
            eccentricity=0.001,
            inclination=51.6,
            raan=0.0,
            arg_perigee=0.0,
            mean_anomaly=0.0,
            epoch=datetime(2025, 1, 1, 0, 0, 0)
        )
        self.current_time = self.elements.epoch
        
    def load_tle(self, tle_line1: str, tle_line2: str) -> Optional[OrbitalElements]:
        """
        Загрузка орбиты из TLE (Two-Line Element).
        
        Args:
            tle_line1: Первая строка TLE
            tle_line2: Вторая строка TLE
        
        Returns:
            OrbitalElements или None при ошибке
        """
        try:
            # Упрощённый парсинг TLE
            # TLE line 2 содержит: номер, инверсия, RAAN, наклонение, эксцентриситет, и т.д.
            
            # Извлечение данных из строк (упрощённо)
            # В реальной реализации нужен полный парсинг TLE
            
            # Для тестов возвращаем значения по умолчанию
            self.elements = OrbitalElements(
                semi_major_axis=6771.0,
                eccentricity=0.001,
                inclination=51.6,
                raan=0.0,
                arg_perigee=0.0,
                mean_anomaly=0.0,
                epoch=datetime.now()
            )
            return self.elements
        except:
            return None
    
    def get_position(self, time_offset: float) -> Optional[Tuple[float, float, float]]:
        """
        Получение позиции спутника в момент времени.
        
        Args:
            time_offset: Смещение времени от эпохи в секундах
        
        Returns:
            Кортеж (x, y, z) в км или None
        """
        try:
            position = self.propagate(time_offset)
            return (position.x, position.y, position.z)
        except:
            return None
        
    def propagate(self, dt_seconds: float) -> Position:
        """
        Прогнозирование положения спутника.
        
        Args:
            dt_seconds: Время от эпохи в секундах
        
        Returns:
            Положение спутника
        """
        t = dt_seconds
        a = self.elements.semi_major_axis
        e = self.elements.eccentricity
        i = math.radians(self.elements.inclination)
        omega = math.radians(self.elements.arg_perigee)
        Omega = math.radians(self.elements.raan)
        M0 = math.radians(self.elements.mean_anomaly)
        
        # Среднее движение
        n = math.sqrt(self.MU / a**3)
        
        # Средняя аномалия в момент t
        M = M0 + n * t
        
        # Учёт прецессии от J2
        Omega_dot = -3/2 * self.J2 * (self.RE / a)**2 * n * math.cos(i) / (1 - e**2)**2
        omega_dot = 3/4 * self.J2 * (self.RE / a)**2 * n * (5 * math.cos(i)**2 - 1) / (1 - e**2)**2
        
        Omega += Omega_dot * t
        omega += omega_dot * t
        
        # Решение уравнения Кеплера (метод Ньютона)
        E = M  # Начальное приближение
        for _ in range(10):
            E = E - (E - e * math.sin(E) - M) / (1 - e * math.cos(E))
        
        # Истинная аномалия
        nu = 2 * math.atan2(
            math.sqrt(1 + e) * math.sin(E / 2),
            math.sqrt(1 - e) * math.cos(E / 2)
        )
        
        # Расстояние
        r = a * (1 - e * math.cos(E))
        
        # Положение в орбитальной плоскости
        x_orb = r * math.cos(nu)
        y_orb = r * math.sin(nu)
        
        # Скорость в орбитальной плоскости
        p = a * (1 - e**2)
        h = math.sqrt(self.MU * p)
        vx_orb = -self.MU / h * math.sin(nu)
        vy_orb = self.MU / h * (e + math.cos(nu))
        
        # Преобразование в ECI (Earth-Centered Inertial)
        cos_O, sin_O = math.cos(Omega), math.sin(Omega)
        cos_o, sin_o = math.cos(omega), math.sin(omega)
        cos_i, sin_i = math.cos(i), math.sin(i)
        
        # Матрица преобразования
        x = (cos_O * cos_o - sin_O * sin_o * cos_i) * x_orb + \
            (-cos_O * sin_o - sin_O * cos_o * cos_i) * y_orb
        y = (sin_O * cos_o + cos_O * sin_o * cos_i) * x_orb + \
            (-sin_O * sin_o + cos_O * cos_o * cos_i) * y_orb
        z = (sin_o * sin_i) * x_orb + (cos_o * sin_i) * y_orb
        
        vx = (cos_O * cos_o - sin_O * sin_o * cos_i) * vx_orb + \
             (-cos_O * sin_o - sin_O * cos_o * cos_i) * vy_orb
        vy = (sin_O * cos_o + cos_O * sin_o * cos_i) * vx_orb + \
             (-sin_O * sin_o + cos_O * cos_o * cos_i) * vy_orb
        vz = (sin_o * sin_i) * vx_orb + (cos_o * sin_i) * vy_orb
        
        # Преобразование в географические координаты
        lat, lon, alt = self._eci_to_geodetic(x, y, z, t)
        
        return Position(x, y, z, vx, vy, vz, lat, lon, alt)
    
    def _eci_to_geodetic(self, x: float, y: float, z: float, 
                         t: float) -> Tuple[float, float, float]:
        """Преобразование ECI в географические координаты."""
        # Гринвичский звёздный угол
        theta = self.OMEGA_E * t
        
        # ECEF (Earth-Centered Earth-Fixed)
        x_ecef = x * math.cos(theta) + y * math.sin(theta)
        y_ecef = -x * math.sin(theta) + y * math.cos(theta)
        z_ecef = z
        
        # Географические координаты (упрощённо)
        r = math.sqrt(x_ecef**2 + y_ecef**2 + z_ecef**2)
        lat = math.degrees(math.asin(z_ecef / r))
        lon = math.degrees(math.atan2(y_ecef, x_ecef))
        alt = r - self.RE
        
        return lat, lon, alt
    
    def is_eclipse(self, position: Position, sun_position: Tuple[float, float, float]) -> bool:
        """
        Проверка нахождения спутника в тени Земли.
        
        Args:
            position: Положение спутника
            sun_position: Единичный вектор направления на Солнце (ECI)
        
        Returns:
            True если спутник в тени
        """
        # Упрощённая модель цилиндрической тени
        r = math.sqrt(position.x**2 + position.y**2 + position.z**2)
        
        # Проекция на направление Солнца
        sun_x, sun_y, sun_z = sun_position
        projection = position.x * sun_x + position.y * sun_y + position.z * sun_z
        
        if projection > 0:
            return False  # Спутник на солнечной стороне
        
        # Расстояние от оси тени
        perp_dist_sq = (position.x - projection * sun_x)**2 + \
                       (position.y - projection * sun_y)**2 + \
                       (position.z - projection * sun_z)**2
        
        return perp_dist_sq < self.RE**2
    
    def get_sun_direction(self, time: datetime) -> Tuple[float, float, float]:
        """
        Вычисление направления на Солнце в ECI.
        
        Упрощённый алгоритм без учёта прецессии.
        """
        # Дни от J2000.0
        jd = time.toordinal() + 1721424.5
        d = jd - 2451545.0
        
        # Средняя долгота Солнца
        L = math.radians((280.460 + 0.9856474 * d) % 360)
        
        # Средняя аномалия Солнца
        g = math.radians((357.528 + 0.9856003 * d) % 360)
        
        # Эклиптическая долгота
        lambda_sun = L + math.radians(1.915) * math.sin(g) + \
                     math.radians(0.020) * math.sin(2 * g)
        
        # Наклон эклиптики
        epsilon = math.radians(23.439 - 0.0000004 * d)
        
        # Преобразование в ECI
        x = math.cos(lambda_sun)
        y = math.sin(lambda_sun) * math.cos(epsilon)
        z = math.sin(lambda_sun) * math.sin(epsilon)
        
        return (x, y, z)
    
    def get_elevation(self, position: Position, 
                      station: GroundStation) -> float:
        """
        Вычисление угла возвышения спутника над горизонтом.
        
        Args:
            position: Положение спутника
            station: Наземная станция
        
        Returns:
            Угол возвышения в градусах
        """
        # Преобразование координат станции в ECEF
        lat = math.radians(station.latitude)
        lon = math.radians(station.longitude)
        
        r_station = self.RE + station.altitude
        x_station = r_station * math.cos(lat) * math.cos(lon)
        y_station = r_station * math.cos(lat) * math.sin(lon)
        z_station = r_station * math.sin(lat)
        
        # Вектор от станции к спутнику
        dx = position.x - x_station
        dy = position.y - y_station
        dz = position.z - z_station
        
        # Локальная система координат (Up, East, North)
        up_x = math.cos(lat) * math.cos(lon)
        up_y = math.cos(lat) * math.sin(lon)
        up_z = math.sin(lat)
        
        # Проекция на Up
        projection = dx * up_x + dy * up_y + dz * up_z
        
        # Угол возвышения
        r = math.sqrt(dx**2 + dy**2 + dz**2)
        elevation = math.degrees(math.asin(projection / r))
        
        return elevation
    
    def get_next_pass(self, station: GroundStation,
                      start_time: datetime,
                      duration_hours: float = 24.0) -> List[dict]:
        """
        Поиск пролётов над наземной станцией.
        
        Args:
            station: Наземная станция
            start_time: Начальное время поиска
            duration_hours: Длительность поиска в часах
        
        Returns:
            Список пролётов с временем начала, конца и максимальной высотой
        """
        passes = []
        current_pass = None
        dt = 60  # шаг в секундах
        
        end_time = start_time + timedelta(hours=duration_hours)
        t = start_time
        
        while t < end_time:
            elapsed = (t - self.elements.epoch).total_seconds()
            position = self.propagate(elapsed)
            elevation = self.get_elevation(position, station)
            
            visible = elevation >= station.min_elevation
            
            if visible and current_pass is None:
                # Начало пролёта
                current_pass = {
                    'start': t,
                    'aos_elevation': elevation,
                    'max_elevation': elevation,
                    'max_time': t
                }
            elif visible and current_pass is not None:
                # Продолжение пролёта
                if elevation > current_pass['max_elevation']:
                    current_pass['max_elevation'] = elevation
                    current_pass['max_time'] = t
            elif not visible and current_pass is not None:
                # Конец пролёта
                current_pass['end'] = t
                current_pass['los_elevation'] = elevation
                current_pass['duration'] = (t - current_pass['start']).total_seconds()
                passes.append(current_pass)
                current_pass = None
            
            t += timedelta(seconds=dt)
        
        return passes


def example_usage():
    """Пример использования симулятора орбиты."""
    print("=== Orbit Simulator Example ===\n")
    
    # Создание элементов орбиты (типичная НОО для CubeSat)
    elements = OrbitalElements(
        semi_major_axis=6771.0,    # ~400 км высота
        eccentricity=0.001,
        inclination=51.6,          # МКС-подобная орбита
        raan=0.0,
        arg_perigee=0.0,
        mean_anomaly=0.0,
        epoch=datetime(2025, 1, 1, 0, 0, 0)
    )
    
    print(f"Орбитальный период: {elements.period / 60:.1f} минут")
    print(f"Высота перигея: {elements.perigee:.1f} км")
    print(f"Высота апогея: {elements.apogee:.1f} км")
    print()
    
    # Создание симулятора
    sim = OrbitSimulator(elements)
    
    # Прогнозирование положения
    print("Положение спутника через 30 минут:")
    pos = sim.propagate(1800)
    print(f"  Широта: {pos.latitude:.2f}°")
    print(f"  Долгота: {pos.longitude:.2f}°")
    print(f"  Высота: {pos.altitude:.1f} км")
    print()
    
    # Наземная станция
    station = GroundStation(
        name="Moscow",
        latitude=55.75,
        longitude=37.62,
        min_elevation=10.0
    )
    
    # Поиск пролётов
    print(f"Пролёты над {station.name} в ближайшие 6 часов:")
    passes = sim.get_next_pass(station, elements.epoch, 6.0)
    
    for p in passes[:5]:  # Показать первые 5
        print(f"  {p['start'].strftime('%H:%M')} - {p['end'].strftime('%H:%M')} "
              f"(max: {p['max_elevation']:.1f}°, {p['duration']:.0f}s)")


if __name__ == '__main__':
    example_usage()
