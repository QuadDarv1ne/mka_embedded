#!/usr/bin/env python3
"""
HIL (Hardware-in-the-Loop) Testing Framework
Фреймворк для тестирования бортового ПО с симуляцией оборудования

HIL тестирование позволяет проверять поведение бортового ПО в условиях,
максимально приближённых к реальным, без необходимости использования
физического оборудования.

Компоненты:
- Симуляторы подсистем (EPS, ADCS, COMM)
- Генераторы данных датчиков
- Инжекторы неисправностей
- Система сценариев тестирования
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Callable, Any, Tuple
from enum import IntEnum
import time
import threading
import queue
import json
import struct


class HILError(Exception):
    """Исключение HIL фреймворка"""
    pass


# ============================================================================
# Симулятор подсистемы электропитания (EPS)
# ============================================================================

@dataclass
class EPSState:
    """Состояние EPS"""
    battery_voltage: float = 7.4        # Вольт
    battery_current: float = 0.5        # Ампер
    battery_charge: float = 80.0        # Процент
    solar_voltage: float = 0.0          # Вольт
    solar_current: float = 0.0          # Ампер
    bus_3v3_voltage: float = 3.3        # Вольт
    bus_5v_voltage: float = 5.0         # Вольт
    temperature: float = 25.0           # Градусы Цельсия
    charging: bool = True
    
    # Состояния линий питания
    power_rails: Dict[int, bool] = field(default_factory=lambda: {
        0: True,   # OBC
        1: True,   # ADCS
        2: True,   # COMM TX
        3: False,  # PAYLOAD
    })


class EPSSimulator:
    """
    Симулятор системы электропитания
    
    Моделирует поведение EPS с учётом:
    - Потребления тока нагрузками
    - Зарядки от солнечных панелей
    - Разрядки батареи
    - Температурных эффектов
    """
    
    # Параметры батареи
    BATTERY_CAPACITY_MAH = 5200
    BATTERY_NOMINAL_VOLTAGE = 7.4
    BATTERY_MIN_VOLTAGE = 6.0
    BATTERY_MAX_VOLTAGE = 8.4
    
    # Потребление подсистем (мА)
    POWER_CONSUMPTION = {
        'OBC': 150,
        'ADCS_IDLE': 50,
        'ADCS_ACTIVE': 500,
        'COMM_RX': 100,
        'COMM_TX': 2000,
        'PAYLOAD_IDLE': 10,
        'PAYLOAD_ACTIVE': 800,
    }
    
    def __init__(self):
        self.state = EPSState()
        self.fault_mode = False
        self.solar_panel_efficiency = 1.0
        self.running = False
        self._thread = None
        self._callbacks: List[Callable[[EPSState], None]] = []
    
    def start(self):
        """Запуск симуляции"""
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        """Остановка симуляции"""
        self.running = False
        if self._thread:
            self._thread.join(timeout=1.0)
    
    def add_callback(self, callback: Callable[[EPSState], None]):
        """Добавление callback для обновления состояния"""
        self._callbacks.append(callback)
    
    def set_illumination(self, illuminated: bool, efficiency: float = 1.0):
        """Установка освещённости солнечных панелей"""
        if illuminated:
            # Типичная мощность солнечных панелей CubeSat: ~2-3 Вт
            self.state.solar_voltage = 8.0 * efficiency * self.solar_panel_efficiency
            self.state.solar_current = 0.3 * efficiency * self.solar_panel_efficiency
            self.state.charging = True
        else:
            self.state.solar_voltage = 0.0
            self.state.solar_current = 0.0
            self.state.charging = False
        
        self.solar_panel_efficiency = efficiency
    
    def set_power_rail(self, rail_id: int, enabled: bool):
        """Управление линией питания"""
        self.state.power_rails[rail_id] = enabled
    
    def inject_fault(self, fault_type: str):
        """Инъекция неисправности"""
        if fault_type == "battery_low":
            self.state.battery_charge = 5.0
            self.state.battery_voltage = 6.2
        elif fault_type == "battery_critical":
            self.state.battery_charge = 2.0
            self.state.battery_voltage = 5.8
        elif fault_type == "solar_panel_failure":
            self.solar_panel_efficiency = 0.0
            self.state.solar_voltage = 0.0
            self.state.solar_current = 0.0
        elif fault_type == "overcurrent":
            self.state.battery_current = 3.0
        elif fault_type == "overtemperature":
            self.state.temperature = 65.0
        self.fault_mode = True
    
    def clear_fault(self):
        """Сброс режима неисправности"""
        self.fault_mode = False
        self.solar_panel_efficiency = 1.0
    
    def _simulation_loop(self):
        """Основной цикл симуляции"""
        dt = 0.1  # 100 ms
        
        while self.running:
            # Расчёт потребления тока
            total_current_ma = self.POWER_CONSUMPTION['OBC']
            
            if self.state.power_rails.get(1, False):  # ADCS
                total_current_ma += self.POWER_CONSUMPTION['ADCS_IDLE']
            
            if self.state.power_rails.get(2, False):  # COMM
                total_current_ma += self.POWER_CONSUMPTION['COMM_RX']
            
            if self.state.power_rails.get(3, False):  # PAYLOAD
                total_current_ma += self.POWER_CONSUMPTION['PAYLOAD_IDLE']
            
            # Заряд/разряд батареи
            net_current_ma = self.state.solar_current * 1000 - total_current_ma
            delta_charge_mah = net_current_ma * (dt / 3600)
            
            self.state.battery_charge += delta_charge_mah / self.BATTERY_CAPACITY_MAH * 100
            self.state.battery_charge = max(0, min(100, self.state.battery_charge))
            
            # Напряжение батареи (упрощённая модель)
            soc = self.state.battery_charge / 100
            self.state.battery_voltage = self.BATTERY_MIN_VOLTAGE + \
                (self.BATTERY_MAX_VOLTAGE - self.BATTERY_MIN_VOLTAGE) * soc
            
            # Ток батареи
            self.state.battery_current = total_current_ma / 1000
            
            # Уведомление подписчиков
            for callback in self._callbacks:
                callback(self.state)
            
            time.sleep(dt)


# ============================================================================
# Симулятор системы ориентации (ADCS)
# ============================================================================

@dataclass
class ADCSState:
    """Состояние ADCS"""
    # Угловая скорость (рад/с)
    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Кватернион ориентации [w, x, y, z]
    quaternion: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    # Магнитное поле (Тесла)
    magnetic_field: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Данные солнечного датчика
    sun_vector: Tuple[float, float, float] = (1.0, 0.0, 0.0)
    sun_visible: bool = True
    # Температура
    temperature: float = 25.0
    # Режим работы
    mode: str = "DETUMBLING"  # DETUMBLING, SUN_POINTING, TARGET_POINTING, SAFE


class ADCSSimulator:
    """
    Симулятор системы ориентации
    
    Моделирует динамику спутника:
    - Угловое движение
    - Атмосферное торможение (на низких орбитах)
    - Гравитационный градиент
    - Воздействие магнитных катушек
    """
    
    # Параметры спутника
    MOMENT_OF_INERTIA = (0.1, 0.1, 0.05)  # kg·m²
    
    def __init__(self):
        self.state = ADCSState()
        self.running = False
        self._thread = None
        self._callbacks: List[Callable[[ADCSState], None]] = []
        
        # Магнитные катушки
        self.magnetorquer_output = (0.0, 0.0, 0.0)
        
        # Маховики
        self.wheel_speed = (0.0, 0.0, 0.0)
    
    def start(self):
        """Запуск симуляции"""
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self._thread.start()
    
    def stop(self):
        """Остановка симуляции"""
        self.running = False
        if self._thread:
            self._thread.join(timeout=1.0)
    
    def add_callback(self, callback: Callable[[ADCSState], None]):
        self._callbacks.append(callback)
    
    def set_magnetorquer(self, x: float, y: float, z: float):
        """Установка тока магнитных катушек (А·м²)"""
        self.magnetorquer_output = (x, y, z)
    
    def set_initial_tumble(self, rate: float = 0.1):
        """Установка начального вращения"""
        import random
        self.state.angular_velocity = (
            rate * (random.random() - 0.5) * 2,
            rate * (random.random() - 0.5) * 2,
            rate * (random.random() - 0.5) * 2
        )
    
    def inject_fault(self, fault_type: str):
        """Инъекция неисправности ADCS"""
        if fault_type == "sensor_failure":
            self.state.sun_visible = False
        elif fault_type == "wheel_failure":
            self.wheel_speed = (0.0, 0.0, 0.0)
        elif fault_type == "excessive_tumble":
            self.state.angular_velocity = (0.5, 0.5, 0.5)
    
    def _simulation_loop(self):
        """Основной цикл симуляции динамики"""
        dt = 0.05  # 50 ms
        
        while self.running:
            # Момент от магнитных катушек: τ = m × B
            tau = self._cross_product(
                self.magnetorquer_output,
                self.state.magnetic_field
            )
            
            # Угловое ускорение: α = τ / I
            alpha = (
                tau[0] / self.MOMENT_OF_INERTIA[0],
                tau[1] / self.MOMENT_OF_INERTIA[1],
                tau[2] / self.MOMENT_OF_INERTIA[2]
            )
            
            # Обновление угловой скорости
            omega = self.state.angular_velocity
            omega = (
                omega[0] + alpha[0] * dt,
                omega[1] + alpha[1] * dt,
                omega[2] + alpha[2] * dt
            )
            self.state.angular_velocity = omega
            
            # Обновление кватерниона (упрощённое)
            q = self.state.quaternion
            dq = self._quaternion_derivative(q, omega)
            q = (
                q[0] + dq[0] * dt,
                q[1] + dq[1] * dt,
                q[2] + dq[2] * dt,
                q[3] + dq[3] * dt
            )
            # Нормализация
            norm = (q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)**0.5
            q = (q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm)
            self.state.quaternion = q
            
            # Уведомление подписчиков
            for callback in self._callbacks:
                callback(self.state)
            
            time.sleep(dt)
    
    @staticmethod
    def _cross_product(a, b):
        return (
            a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]
        )
    
    @staticmethod
    def _quaternion_derivative(q, omega):
        """Производная кватерниона по времени"""
        return (
            0.5 * (-q[1]*omega[0] - q[2]*omega[1] - q[3]*omega[2]),
            0.5 * ( q[0]*omega[0] - q[3]*omega[1] + q[2]*omega[2]),
            0.5 * ( q[3]*omega[0] + q[0]*omega[1] - q[1]*omega[2]),
            0.5 * (-q[2]*omega[0] + q[1]*omega[1] + q[0]*omega[2])
        )


# ============================================================================
# Система сценариев тестирования
# ============================================================================

@dataclass
class TestStep:
    """Шаг тестового сценария"""
    name: str
    description: str
    action: Callable[['HILTestRunner'], None]
    expected_result: Optional[Callable[[Any], bool]] = None
    timeout_ms: int = 5000


@dataclass
class TestResult:
    """Результат выполнения шага"""
    step_name: str
    success: bool
    duration_ms: int
    message: str
    data: Any = None


class HILTestRunner:
    """
    Исполнитель тестовых сценариев HIL
    """
    
    def __init__(self, eps: EPSSimulator, adcs: ADCSSimulator):
        self.eps = eps
        self.adcs = adcs
        self.steps: List[TestStep] = []
        self.results: List[TestResult] = []
        self.current_step = 0
        self.test_data: Dict[str, Any] = {}
    
    def add_step(self, step: TestStep):
        """Добавление шага теста"""
        self.steps.append(step)
    
    def run(self) -> bool:
        """Выполнение всех шагов теста"""
        self.results = []
        all_passed = True
        
        for i, step in enumerate(self.steps):
            self.current_step = i
            start_time = time.time()
            
            try:
                step.action(self)
                
                # Проверка ожидаемого результата
                if step.expected_result:
                    success = step.expected_result(self.test_data)
                else:
                    success = True
                
                duration = int((time.time() - start_time) * 1000)
                
                result = TestResult(
                    step_name=step.name,
                    success=success,
                    duration_ms=duration,
                    message="OK" if success else "Failed"
                )
                
            except Exception as e:
                duration = int((time.time() - start_time) * 1000)
                result = TestResult(
                    step_name=step.name,
                    success=False,
                    duration_ms=duration,
                    message=str(e)
                )
                all_passed = False
            
            self.results.append(result)
            
            if not result.success:
                all_passed = False
                # Можно прервать тест при первой ошибке
                # break
        
        return all_passed
    
    def get_report(self) -> str:
        """Получение текстового отчёта"""
        lines = []
        lines.append("=" * 60)
        lines.append("HIL Test Report")
        lines.append("=" * 60)
        
        total_time = sum(r.duration_ms for r in self.results)
        passed = sum(1 for r in self.results if r.success)
        
        lines.append(f"Total steps: {len(self.results)}")
        lines.append(f"Passed: {passed}")
        lines.append(f"Failed: {len(self.results) - passed}")
        lines.append(f"Total time: {total_time} ms")
        lines.append("")
        
        for r in self.results:
            status = "✓ PASS" if r.success else "✗ FAIL"
            lines.append(f"  {status} [{r.duration_ms:4d}ms] {r.step_name}")
            if not r.success:
                lines.append(f"         {r.message}")
        
        return "\n".join(lines)


# ============================================================================
# Пример тестового сценария
# ============================================================================

def create_eps_test_scenario() -> HILTestRunner:
    """Создание тестового сценария для EPS"""
    
    eps = EPSSimulator()
    adcs = ADCSSimulator()
    runner = HILTestRunner(eps, adcs)
    
    # Шаг 1: Инициализация
    def init_action(r: HILTestRunner):
        r.eps.start()
        time.sleep(0.2)
        assert r.eps.state.battery_voltage > 6.0, "Battery voltage too low"
    
    runner.add_step(TestStep(
        name="EPS Initialization",
        description="Start EPS simulator and check initial state",
        action=init_action,
        timeout_ms=1000
    ))
    
    # Шаг 2: Проверка зарядки
    def charging_action(r: HILTestRunner):
        initial_charge = r.eps.state.battery_charge
        r.eps.set_illumination(True)
        time.sleep(1.0)
        r.test_data['charge_after_sun'] = r.eps.state.battery_charge
        assert r.eps.state.charging, "Not charging in sunlight"
    
    runner.add_step(TestStep(
        name="Solar Charging",
        description="Verify solar panel charging",
        action=charging_action,
        expected_result=lambda data: data.get('charge_after_sun', 0) > 80,
        timeout_ms=2000
    ))
    
    # Шаг 3: Инъекция неисправности
    def fault_action(r: HILTestRunner):
        r.eps.inject_fault("battery_low")
        time.sleep(0.1)
        r.test_data['low_battery'] = r.eps.state.battery_voltage
    
    runner.add_step(TestStep(
        name="Battery Low Fault",
        description="Inject low battery fault",
        action=fault_action,
        expected_result=lambda data: data.get('low_battery', 10) < 7.0,
        timeout_ms=500
    ))
    
    # Шаг 4: Восстановление
    def recovery_action(r: HILTestRunner):
        r.eps.clear_fault()
        time.sleep(0.1)
        r.test_data['recovered'] = r.eps.state.battery_voltage > 6.0
    
    runner.add_step(TestStep(
        name="Fault Recovery",
        description="Clear fault and verify recovery",
        action=recovery_action,
        timeout_ms=500
    ))
    
    # Остановка
    def cleanup(r: HILTestRunner):
        r.eps.stop()
    
    runner.add_step(TestStep(
        name="Cleanup",
        description="Stop simulators",
        action=cleanup,
        timeout_ms=100
    ))
    
    return runner


if __name__ == '__main__':
    print("Running HIL Test Scenario...")
    print()
    
    runner = create_eps_test_scenario()
    success = runner.run()
    
    print(runner.get_report())
    print()
    print(f"Overall result: {'PASS' if success else 'FAIL'}")
