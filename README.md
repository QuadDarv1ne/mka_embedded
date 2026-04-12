# MKA Embedded Complete

Полный набор инструментов, драйверов и документации для разработки бортового ПО малых космических аппаратов (МКА) формата CubeSat.

## Содержание

- [Структура проекта](#структура-проекта)
- [Компоненты](#компоненты)
- [Быстрый старт](#быстрый-старт)
- [Документация](#документация)
- [Сборка](#сборка)
- [Git](#git)

## Структура проекта

```
mka_embedded/
├── src/
│   ├── cpp/
│   │   ├── hal/
│   │   │   └── hal_full.hpp              # Полные HAL интерфейсы
│   │   ├── drivers/
│   │   │   ├── sensors_drivers.hpp       # Драйверы BMI160, LIS3MDL, GPS, BMP388, LSM6DSO
│   │   │   ├── eeprom_driver.hpp         # Драйвер EEPROM 24LC256
│   │   │   ├── radio_driver.hpp          # Драйвер радиомодуля SI4463
│   │   │   └── sun_sensor.hpp            # Драйвер солнечного сенсора
│   │   ├── algorithms/
│   │   │   ├── adcs_algorithms.hpp       # Алгоритмы ориентации (EKF, UKF, Madgwick)
│   │   │   ├── anomaly_detector.hpp      # Детектор аномалий (Isolation Forest)
│   │   │   └── sgp4.hpp                  # SGP4 орбитальный пропагатор
│   │   ├── systems/
│   │   │   ├── fdir.hpp                  # Fault Detection, Isolation, Recovery
│   │   │   ├── state_machine.hpp         # Машина состояний спутника
│   │   │   ├── log_system.hpp            # Система логирования
│   │   │   ├── memory_pool.hpp           # Пул памяти для RTOS
│   │   │   ├── telemetry.hpp             # Генератор телеметрии и команд
│   │   │   ├── file_system.hpp           # Файловая система LittleFS
│   │   │   ├── ota_updater.hpp           # OTA обновление прошивки
│   │   │   ├── command_handler.hpp       # Обработчик команд
│   │   │   ├── param_store.hpp           # Хранилище параметров
│   │   │   ├── watchdog_manager.hpp      # Менеджер сторожевых таймеров
│   │   │   ├── canopen.hpp               # CANopen стек
│   │   │   ├── power_manager.hpp         # Управление питанием (MPPT)
│   │   │   ├── task_scheduler.hpp        # Планировщик задач
│   │   │   ├── health_monitor.hpp        # Мониторинг здоровья
│   │   │   ├── moscow_time.hpp           # Система Московского времени
│   │   │   └── auto_actualization.hpp    # Автоматическая актуализация
│   │   ├── rtos/
│   │   │   └── freertos_wrapper.hpp      # Обёртки FreeRTOS
│   │   └── utils/
│   │       ├── callback.hpp              # Легковесные callback
│   │       ├── result.hpp                # Result<T, E> тип
│   │       └── span.hpp                  # Span utility
│   └── python/
│       ├── utils/
│       │   ├── littlefs_sim.py           # Симулятор файловой системы
│       │   ├── ota_update.py             # OTA загрузчик
│       │   ├── orbit_simulator.py        # Симулятор орбиты
│       │   ├── ax25_protocol.py          # Протокол AX.25
│       │   ├── csp_protocol.py           # CubeSat Space Protocol
│       │   └── ccsds_protocol.py         # CCSDS протоколы
│       ├── visualization/
│       │   └── telemetry_visualizer.py   # Визуализация телеметрии
│       └── hil/
│           └── hil_framework.py          # HIL тестирование
├── tests/                                # Юнит-тесты (40 файлов, 450+ тестов)
├── scripts/                                # SDR скрипты и утилиты
│   ├── sdr_capture_enhanced.py             # Улучшенный приём и анализ сигналов
│   ├── sdr_noaa.py                         # Приём метеоспутников NOAA (APT)
│   ├── sdr_ax25.py                         # Приём AX.25 пакетов (APRS)
│   ├── sdr_scanner.py                      # Сканер спектра
│   ├── sdr_antenna.py                      # Калькулятор антенн
│   ├── test_rtlsdr.py                      # Тест RTL-SDR устройства
│   ├── SDR_README.md                       # Документация по SDR скриптам
│   └── sdr_direwolf.conf                   # Конфигурация Dire Wolf
├── exercises/
│   └── labs.md                           # Практические задания
├── cheatsheets/
│   ├── stm32_quickref.md                 # Шпаргалка по STM32
│   └── freertos_quickref.md              # Шпаргалка по FreeRTOS
├── CMakeLists.txt                        # Конфигурация CMake
└── README.md
```

## Компоненты

### C++ компоненты

#### HAL интерфейсы (`hal_full.hpp`)

Полный набор абстрактных интерфейсов:
- **ISystemTime** — системное время
- **IWatchdog** — сторожевой таймер
- **IGPIO** — цифровые входы/выходы
- **IUART** — последовательный порт
- **ISPI** — SPI интерфейс
- **II2C** — I2C интерфейс
- **ICAN** — CAN шина
- **IADC** — аналого-цифровой преобразователь
- **IDAC** — цифро-аналоговый преобразователь
- **ITimer** — таймеры
- **IFlash** — Flash память
- **ISDCard** — SD карта
- **IPowerSystem** — система электропитания
- **ICRC** — аппаратный CRC
- **IRNG** — генератор случайных чисел
- **IDMA** — DMA контроллер

#### Драйверы датчиков (`sensors_drivers.hpp`)

| Датчик | Тип | Особенности |
|--------|-----|-------------|
| **BMI160** | IMU (6 DOF) | Акселерометр ±2/4/8/16g, Гироскоп ±125-2000°/s, I2C/SPI |
| **LIS3MDL** | Магнитометр | ±4/8/12/16 Гаусс, 16-бит, I2C/SPI |
| **u-blox NEO-M8** | GPS | GPS+GLONASS, UART, NMEA/UBX |
| **Sun Sensor** | Оптический | 4 фотодиода, АЦП |

#### Алгоритмы ориентации (`adcs_algorithms.hpp`)

- **Quaternion** — математика кватернионов
- **Фильтры оценки ориентации:**
  - MadgwickFilter — фильтр слияния IMU
  - Extended Kalman Filter (EKF) — оценка ориентации + смещения гироскопа
  - Unscented Kalman Filter (UKF) — нелинейная оценка ориентации
  - Complementary filter — альтернатива Madgwick
- **Определение ориентации:**
  - TRIAD метод — построение DCM из двух векторов
  - QUEST метод — оптимальный алгоритм Wahba
- **Контроллеры ориентации:**
  - PIDController — регулятор с anti-windup
  - BDotController — гашение вращения (detumbling)
  - PD Controller — кватернионная ошибка, демпфирование
  - SlidingModeController — робастный нелинейный контроллер
- **AttitudeController** — контроллер ориентации

#### Система FDIR (`fdir.hpp`)

Fault Detection, Isolation and Recovery:
- Мониторинг параметров с порогами
- Детектор аномалий (статистический)
- Журнал событий
- Автоматическое восстановление
- Детекторы: Frozen Value, Stuck Bit, Glitch

#### Машина состояний (`state_machine.hpp`)

Управление режимами спутника:
- OFF → INIT → SAFE → STANDBY → NOMINAL → MISSION
- Валидация переходов
- Автономные переходы по условиям
- История переходов

#### Система логирования (`log_system.hpp`)

**Структурированное логирование:**

- **Уровни:** `DEBUG`, `INFO`, `NOTICE`, `WARNING`, `ERROR`, `CRITICAL`
- Категории по подсистемам
- Форматирование с временными метками
- Кольцевой буфер

#### Пул памяти (`memory_pool.hpp`)

**Безопасное управление памятью для RTOS:**

- `Fixed Block Pool` — O(1) выделение
- `Memory Pool Manager` — пулы разных размеров
- `RAII` обёртка

### Python утилиты

#### Протоколы связи

| Протокол | Файл | Описание |
|----------|------|----------|
| AX.25 | `ax25_protocol.py` | Любительская пакетная радиосвязь |
| CSP | `csp_protocol.py` | CubeSat Space Protocol |
| CCSDS | `ccsds_protocol.py` | Space Packet, TM/TC Transfer Frames |

#### Утилиты

- **LittleFS Simulator** (`littlefs_sim.py`) — симуляция файловой системы
- **OTA Update** (`ota_update.py`) — удалённое обновление ПО
- **Orbit Simulator** (`orbit_simulator.py`) — прогноз пролётов

#### HIL Testing (`hil_framework.py`)

**Hardware-in-the-Loop тестирование:**

- Симулятор `EPS` (электропитание)
- Симулятор `ADCS` (ориентация)
- Инъекция неисправностей
- Сценарии тестирования

#### SDR инструменты (`scripts/`)

**Приём и анализ радиосигналов для CubeSat:**

| Скрипт | Назначение |
|--------|------------|
| `sdr_capture_enhanced.py` | Универсальный приём с FFT, SNR, детекцией пиков |
| `sdr_noaa.py` | Приём APT изображений с NOAA спутников |
| `sdr_ax25.py` | Приём APRS пакетов (AX.25) |
| `sdr_scanner.py` | Сканирование спектра |
| `sdr_antenna.py` | Калькулятор антенн |

**Примеры использования:**

```bash
# Приём ISS (145.8 МГц)
python scripts\sdr_capture_enhanced.py --freq 145.8 --plot

# Приём NOAA 19
python scripts\sdr_noaa.py --sat noaa19

# Мониторинг APRS
python scripts\sdr_ax25.py --monitor --decode

# Сканирование VHF диапазона
python scripts\sdr_scanner.py --start 137 --stop 146

# Расчёт антенны для 145.8 МГц
python scripts\sdr_antenna.py --freq 145.8
```

**Подробнее:** См. [scripts/SDR_README.md](scripts/SDR_README.md)

### Документация

#### Шпаргалки

- `stm32_quickref.md` — GPIO, UART, SPI, I2C, CAN, DMA, таймеры, ADC
- `freertos_quickref.md` — Задачи, очереди, семафоры, таймеры

#### Практические задания

**5 лабораторных работ:**

1. HAL и драйверы
2. CAN шина
3. Система ориентации
4. RTOS задачи
5. Телеметрия и команды

## Быстрый старт

### Python тесты

```bash
# Установка зависимостей
pip install pytest

# Запуск тестов
cd src/python
python -m pytest tests/ -v

# Демо скрипты
python utils/ax25_protocol.py
python utils/csp_protocol.py
python utils/ccsds_protocol.py
python hil/hil_framework.py
```

### Интеграция C++

```cpp
#include "hal_full.hpp"
#include "sensors_drivers.hpp"
#include "adcs_algorithms.hpp"
#include "fdir.hpp"
#include "state_machine.hpp"

// Создание драйверов
BMI160Driver imu(i2c);
imu.init();

// Чтение данных
IMUData data;
imu.readData(data);

// Фильтр ориентации
MadgwickFilter filter(100.0f, 0.1f);
filter.updateIMU(data.gyro, data.accel, dt);

// FDIR мониторинг
mka::fdir::FDIRManager fdir;
auto paramId = fdir.registerParameter({
    .nominalValue = 7.4f,
    .warningLow = 6.5f,
    .warningHigh = 8.5f,
    .errorLow = 6.0f,
    .errorHigh = 9.0f,
    .criticalLow = 5.5f,
    .criticalHigh = 10.0f
});

// State Machine
mka::statemachine::SatelliteStateMachine sm;
sm.forceTransition(mka::statemachine::SatelliteMode::INIT, 
                   mka::statemachine::TransitionReason::COMMAND);
```

## Архитектура бортового ПО

```textline
┌─────────────────────────────────────────────────────────────────────┐
│                         Application Layer                           │
│  ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌───────────────────┐  │
│  │ Mission   │ │ Payload   │ │ Telemetry │ │ Command Processor │  │
│  │ Manager   │ │ Control   │ │ Generator │ │                   │  │
│  └───────────┘ └───────────┘ └───────────┘ └───────────────────┘  │
├─────────────────────────────────────────────────────────────────────┤
│                          Service Layer                              │
│  ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌───────────────────┐  │
│  │ File      │ │ Log       │ │ Protocol  │ │ FDIR Manager      │  │
│  │ System    │ │ Manager   │ │ Stack     │ │ State Machine     │  │
│  └───────────┘ └───────────┘ └───────────┘ └───────────────────┘  │
├─────────────────────────────────────────────────────────────────────┤
│                           Driver Layer                              │
│  ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌───────────────────┐  │
│  │ IMU       │ │ Magnetom. │ │ GPS       │ │ Radio Module      │  │
│  │ Driver    │ │ Driver    │ │ Driver    │ │ Driver            │  │
│  └───────────┘ └───────────┘ └───────────┘ └───────────────────┘  │
├─────────────────────────────────────────────────────────────────────┤
│                        HAL (Hardware Abstraction)                   │
│  ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌───────────────────┐  │
│  │ GPIO      │ │ UART/SPI  │ │ I2C/CAN   │ │ ADC/DMA           │  │
│  └───────────┘ └───────────┘ └───────────┘ └───────────────────┘  │
├─────────────────────────────────────────────────────────────────────┤
│                         Hardware (STM32F4/F7/H7)                    │
└─────────────────────────────────────────────────────────────────────┘
```

## Сборка

### CMake (рекомендуется)

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Опции CMake

| Опция | Описание | По умолчанию |
|-------|----------|--------------|
| `STM32F4` | Сборка для STM32F4 | OFF |
| `STM32F7` | Сборка для STM32F7 | OFF |
| `STM32H7` | Сборка для STM32H7 | OFF |
| `HOST_BUILD` | Сборка для хоста | ON |
| `BUILD_TESTS` | Сборка тестов | ON |
| `BUILD_DOCS` | Генерация документации | OFF |
| `ENABLE_CPPCHECK` | Статический анализ | OFF |

### Целевые платформы

```bash
# Для STM32F4
cmake .. -DSTM32F4=ON -DBUILD_TESTS=OFF

# Для STM32H7
cmake .. -DSTM32H7=ON -DBUILD_TESTS=OFF

# Для тестирования на хосте
cmake .. -DHOST_BUILD=ON -DBUILD_TESTS=ON
```

## Git

### Конфигурация

**Проект настроен для корректной работы с Git:**

- **.gitignore** — исключает артефакты сборки, IDE-файлы, Python-виртуальные окружения, бинарные файлы
- **.gitattributes** — принудительное использование LF (Unix) окончаний строк для исходного кода; бинарные файлы не преобразуются

### Рекомендуемые настройки Git

```bash
# Включение поддержки файлов >100MB (если нужно)
git lfs install

# Клонирование репозитория
git clone <url>
cd mka_embedded_complete
```

## Требования

### C++ разработка
- Компилятор C++17+
- STM32CubeIDE или IAR/Keil
- FreeRTOS 10.x (опционально)

### Python разработка
- Python 3.8+
- pytest (для тестирования)

### Сборка
- CMake 3.20+
- Make или Ninja

## Лицензия

Образовательный материал для подготовки Embedded-программистов космической отрасли.

---

*Разработано для использования в образовательных целях и для реальных проектов CubeSat.*
