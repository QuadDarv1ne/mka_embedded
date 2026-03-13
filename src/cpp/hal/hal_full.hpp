/**
 * @file hal_full.hpp
 * @brief Complete Hardware Abstraction Layer for MKA
 * 
 * Полный HAL для бортового ПО малого космического аппарата.
 * Включает все интерфейсы периферии и системные сервисы.
 */

#ifndef HAL_FULL_HPP
#define HAL_FULL_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <span>

#include "../utils/callback.hpp"

namespace mka {
namespace hal {

// ============================================================================
// Базовые типы и константы
// ============================================================================

/// Статус операции
enum class Status : uint8_t {
    OK = 0,
    ERROR = 1,
    TIMEOUT = 2,
    BUSY = 3,
    INVALID_PARAM = 4,
    NOT_INITIALIZED = 5,
    BUFFER_OVERFLOW = 6,
    CRC_ERROR = 7,
    NOT_SUPPORTED = 8
};

/// Режим энергопотребления
enum class PowerMode : uint8_t {
    ACTIVE,
    LOW_POWER,
    SLEEP,
    DEEP_SLEEP,
    SHUTDOWN
};

/// Результат сравнения
enum class CompareResult : int8_t {
    LESS = -1,
    EQUAL = 0,
    GREATER = 1
};

// ============================================================================
// Интерфейс системного времени
// ============================================================================

/**
 * @brief Интерфейс системного таймера
 */
class ISystemTime {
public:
    virtual ~ISystemTime() = default;
    
    /// Получить текущее время в миллисекундах
    virtual uint64_t getMs() const = 0;
    
    /// Получить текущее время в микросекундах
    virtual uint64_t getUs() const = 0;
    
    /// Задержка в миллисекундах
    virtual void delayMs(uint32_t ms) = 0;
    
    /// Задержка в микросекундах
    virtual void delayUs(uint32_t us) = 0;
    
    /// Получить время запуска системы
    virtual uint64_t getBootTime() const = 0;
    
    /// Установить системное время (для синхронизации)
    virtual void setTime(uint64_t unixTimeMs) = 0;
};

// ============================================================================
// Интерфейс Watchdog
// ============================================================================

/**
 * @brief Интерфейс сторожевого таймера
 */
class IWatchdog {
public:
    virtual ~IWatchdog() = default;
    
    /// Инициализация с периодом в миллисекундах
    virtual Status init(uint32_t timeoutMs) = 0;
    
    /// Обновить (кикнуть) watchdog
    virtual void refresh() = 0;
    
    /// Получить剩余 время до сброса
    virtual uint32_t getRemainingTime() const = 0;
    
    /// Проверить причину последнего сброса
    virtual bool wasResetByWatchdog() const = 0;
};

// ============================================================================
// Интерфейс GPIO (расширенный)
// ============================================================================

enum class GPIOMode : uint8_t {
    INPUT,
    INPUT_PULL_UP,
    INPUT_PULL_DOWN,
    OUTPUT_PUSH_PULL,
    OUTPUT_OPEN_DRAIN,
    ALTERNATE_FUNCTION,
    ANALOG
};

enum class GPIOSpeed : uint8_t {
    LOW,
    MEDIUM,
    HIGH,
    VERY_HIGH
};

enum class GPIOState : uint8_t {
    LOW = 0,
    HIGH = 1
};

struct GPIOConfig {
    GPIOMode mode = GPIOMode::INPUT;
    GPIOSpeed speed = GPIOSpeed::HIGH;
    uint8_t alternateFunction = 0;
    bool initialState = false;
};

/**
 * @brief Интерфейс GPIO
 */
class IGPIO {
public:
    using InterruptCallback = Callback<void(uint8_t pin)>;

    virtual ~IGPIO() = default;
    
    /// Инициализация пина
    virtual Status init(uint8_t pin, const GPIOConfig& config) = 0;
    
    /// Деинициализация пина
    virtual Status deinit(uint8_t pin) = 0;
    
    /// Записать состояние
    virtual Status write(uint8_t pin, GPIOState state) = 0;
    
    /// Прочитать состояние
    virtual GPIOState read(uint8_t pin) const = 0;
    
    /// Переключить состояние
    virtual Status toggle(uint8_t pin) = 0;
    
    /// Установить callback для прерывания
    virtual Status setInterruptCallback(uint8_t pin, 
                                        InterruptCallback callback,
                                        uint8_t triggerEdge) = 0;
};

// ============================================================================
// Интерфейс UART (расширенный)
// ============================================================================

struct UARTConfig {
    uint32_t baudRate = 115200;
    uint8_t dataBits = 8;
    uint8_t stopBits = 1;
    uint8_t parity = 0;  // 0=none, 1=odd, 2=even
    bool flowControl = false;
    bool dmaEnable = true;
    uint16_t txBufferSize = 512;
    uint16_t rxBufferSize = 512;
};

struct UARTStatistics {
    uint32_t txBytes = 0;
    uint32_t rxBytes = 0;
    uint32_t txErrors = 0;
    uint32_t rxErrors = 0;
    uint32_t parityErrors = 0;
    uint32_t framingErrors = 0;
    uint32_t overrunErrors = 0;
};

/**
 * @brief Интерфейс UART
 */
class IUART {
public:
    using RxCallback = Callback<void(std::span<const uint8_t> data)>;
    using TxCompleteCallback = Callback<void()>;

    virtual ~IUART() = default;
    
    /// Инициализация
    virtual Status init(const UARTConfig& config) = 0;
    
    /// Деинициализация
    virtual void deinit() = 0;
    
    /// Передача (блокирующая)
    virtual Status transmit(std::span<const uint8_t> data, 
                           uint32_t timeoutMs) = 0;
    
    /// Приём (блокирующий)
    virtual Status receive(std::span<uint8_t> buffer,
                          uint32_t timeoutMs,
                          size_t& received) = 0;
    
    /// Передача (неблокирующая)
    virtual Status transmitAsync(std::span<const uint8_t> data,
                                 TxCompleteCallback callback) = 0;
    
    /// Установка callback для приёма
    virtual void setRxCallback(RxCallback callback) = 0;
    
    /// Запуск непрерывного приёма
    virtual Status startContinuousRx() = 0;
    
    /// Остановка приёма
    virtual void stopContinuousRx() = 0;
    
    /// Получить количество байт в буфере приёма
    virtual size_t available() const = 0;
    
    /// Сброс буферов
    virtual void flush() = 0;
    
    /// Получить статистику
    virtual UARTStatistics getStatistics() const = 0;
    
    /// Сброс статистики
    virtual void resetStatistics() = 0;
};

// ============================================================================
// Интерфейс SPI (расширенный)
// ============================================================================

enum class SPIMode : uint8_t {
    MODE_0,  // CPOL=0, CPHA=0
    MODE_1,  // CPOL=0, CPHA=1
    MODE_2,  // CPOL=1, CPHA=0
    MODE_3   // CPOL=1, CPHA=1
};

enum class SPIBitOrder : uint8_t {
    MSB_FIRST,
    LSB_FIRST
};

struct SPIConfig {
    uint32_t clockSpeed = 1000000;
    SPIMode mode = SPIMode::MODE_0;
    SPIBitOrder bitOrder = SPIBitOrder::MSB_FIRST;
    bool masterMode = true;
    bool dmaEnable = true;
    uint8_t csPin = 0xFF;  // Использовать аппаратный CS
};

/**
 * @brief Интерфейс SPI
 */
class ISPI {
public:
    using TransferCallback = Callback<void(std::span<uint8_t> rxData)>;

    virtual ~ISPI() = default;
    
    /// Инициализация
    virtual Status init(const SPIConfig& config) = 0;
    
    /// Деинициализация
    virtual void deinit() = 0;
    
    /// Выбрать устройство (CS low)
    virtual Status selectDevice() = 0;
    
    /// Отпустить устройство (CS high)
    virtual void deselectDevice() = 0;
    
    /// Полнодуплексный обмен (блокирующий)
    virtual Status transfer(std::span<const uint8_t> txData,
                           std::span<uint8_t> rxData,
                           uint32_t timeoutMs) = 0;
    
    /// Передача (блокирующая)
    virtual Status transmit(std::span<const uint8_t> data,
                           uint32_t timeoutMs) = 0;
    
    /// Приём (блокирующий)
    virtual Status receive(std::span<uint8_t> data,
                          uint32_t timeoutMs) = 0;
    
    /// Асинхронный обмен
    virtual Status transferAsync(std::span<const uint8_t> txData,
                                 std::span<uint8_t> rxData,
                                 TransferCallback callback) = 0;
    
    /// Установить скорость на лету
    virtual Status setClockSpeed(uint32_t speed) = 0;
};

// ============================================================================
// Интерфейс I2C (расширенный)
// ============================================================================

struct I2CConfig {
    uint32_t clockSpeed = 100000;
    uint8_t ownAddress = 0;
    bool generalCall = false;
    bool clockStretch = true;
    uint8_t maxRetries = 3;
};

struct I2CStatistics {
    uint32_t txBytes = 0;
    uint32_t rxBytes = 0;
    uint32_t nackCount = 0;
    uint32_t timeoutCount = 0;
    uint32_t busErrors = 0;
    uint32_t arbitrationLost = 0;
};

/**
 * @brief Интерфейс I2C
 */
class II2C {
public:
    virtual ~II2C() = default;
    
    /// Инициализация
    virtual Status init(const I2CConfig& config) = 0;
    
    /// Деинициализация
    virtual void deinit() = 0;
    
    /// Запись в регистр
    virtual Status writeRegister(uint8_t devAddress,
                                uint8_t regAddress,
                                std::span<const uint8_t> data,
                                uint32_t timeoutMs) = 0;
    
    /// Чтение из регистра
    virtual Status readRegister(uint8_t devAddress,
                               uint8_t regAddress,
                               std::span<uint8_t> data,
                               uint32_t timeoutMs) = 0;
    
    /// Запись 16-битного регистра
    virtual Status writeRegister16(uint8_t devAddress,
                                   uint16_t regAddress,
                                   std::span<const uint8_t> data,
                                   uint32_t timeoutMs) = 0;
    
    /// Чтение 16-битного регистра
    virtual Status readRegister16(uint8_t devAddress,
                                  uint16_t regAddress,
                                  std::span<uint8_t> data,
                                  uint32_t timeoutMs) = 0;
    
    /// Сканирование шины
    virtual size_t scanBus(std::span<uint8_t> foundDevices) = 0;
    
    /// Проверка наличия устройства
    virtual bool isDevicePresent(uint8_t devAddress) = 0;
    
    /// Восстановление шины при зависании
    virtual Status recoverBus() = 0;
    
    /// Получить статистику
    virtual I2CStatistics getStatistics() const = 0;
};

// ============================================================================
// Интерфейс CAN (расширенный)
// ============================================================================

struct CANMessage {
    uint32_t id;
    bool extended;
    bool remote;
    uint8_t dlc;
    std::array<uint8_t, 8> data;
    uint32_t timestamp;
};

struct CANFilter {
    uint32_t id;
    uint32_t mask;
    bool extended;
};

struct CANStatistics {
    uint32_t txCount = 0;
    uint32_t rxCount = 0;
    uint32_t txErrorCount = 0;
    uint32_t rxErrorCount = 0;
    uint32_t busOffCount = 0;
    uint32_t overrunCount = 0;
};

struct CANConfig {
    uint32_t baudRate = 500000;
    bool loopback = false;
    bool listenOnly = false;
    bool autoRetransmit = true;
    bool autoBusOffRecovery = true;
};

/**
 * @brief Интерфейс CAN
 */
class ICAN {
public:
    using RxCallback = Callback<void(const CANMessage& msg)>;
    using TxCompleteCallback = Callback<void(uint32_t mailbox)>;

    virtual ~ICAN() = default;
    
    /// Инициализация
    virtual Status init(const CANConfig& config) = 0;
    
    /// Деинициализация
    virtual void deinit() = 0;
    
    /// Передача сообщения
    virtual Status transmit(const CANMessage& msg, 
                           uint32_t timeoutMs) = 0;
    
    /// Асинхронная передача
    virtual Status transmitAsync(const CANMessage& msg,
                                TxCompleteCallback callback) = 0;
    
    /// Приём сообщения (блокирующий)
    virtual Status receive(CANMessage& msg, uint32_t timeoutMs) = 0;
    
    /// Установка callback для приёма
    virtual void setRxCallback(RxCallback callback) = 0;
    
    /// Добавить фильтр приёма
    virtual Status addFilter(const CANFilter& filter, uint8_t fifo = 0) = 0;
    
    /// Очистить все фильтры
    virtual void clearFilters() = 0;
    
    /// Получить статус шины
    virtual bool isBusOff() const = 0;
    
    /// Восстановление после bus-off
    virtual Status recoverBus() = 0;
    
    /// Получить статистику
    virtual CANStatistics getStatistics() const = 0;
    
    /// Сброс статистики
    virtual void resetStatistics() = 0;
};

// ============================================================================
// Интерфейс ADC
// ============================================================================

struct ADCConfig {
    uint8_t resolution = 12;  // бит
    uint32_t sampleTime = 1;  // мкс
    bool continuous = false;
    bool dmaEnable = true;
};

/**
 * @brief Интерфейс АЦП
 */
class IADC {
public:
    using ConversionCallback = Callback<void(uint8_t channel, uint16_t value)>;

    virtual ~IADC() = default;
    
    /// Инициализация
    virtual Status init(const ADCConfig& config) = 0;
    
    /// Калибровка
    virtual Status calibrate() = 0;
    
    /// Одиночное преобразование
    virtual Status read(uint8_t channel, uint16_t& value, 
                       uint32_t timeoutMs) = 0;
    
    /// Многоканальное преобразование
    virtual Status readMulti(std::span<const uint8_t> channels,
                            std::span<uint16_t> values,
                            uint32_t timeoutMs) = 0;
    
    /// Запуск непрерывного преобразования
    virtual Status startContinuous(std::span<const uint8_t> channels,
                                   ConversionCallback callback) = 0;
    
    /// Остановка непрерывного преобразования
    virtual void stopContinuous() = 0;
    
    /// Преобразование в вольты
    virtual float toVolts(uint16_t rawValue, uint8_t channel) const = 0;
};

// ============================================================================
// Интерфейс DAC
// ============================================================================

struct DACConfig {
    uint8_t resolution = 12;
    bool dmaEnable = true;
    uint32_t updateRate = 100000;  // Hz
};

/**
 * @brief Интерфейс ЦАП
 */
class IDAC {
public:
    virtual ~IDAC() = default;
    
    /// Инициализация
    virtual Status init(const DACConfig& config) = 0;
    
    /// Установка выходного значения
    virtual Status setValue(uint8_t channel, uint16_t value) = 0;
    
    /// Установка выходного напряжения
    virtual Status setVoltage(uint8_t channel, float voltage) = 0;
    
    /// Вывод сигнала по DMA
    virtual Status outputWaveform(uint8_t channel,
                                  std::span<const uint16_t> samples,
                                  bool repeat) = 0;
    
    /// Остановка вывода
    virtual void stopWaveform(uint8_t channel) = 0;
};

// ============================================================================
// Интерфейс таймеров
// ============================================================================

enum class TimerMode : uint8_t {
    PERIODIC,
    ONE_SHOT,
    PWM,
    INPUT_CAPTURE,
    OUTPUT_COMPARE,
    ENCODER
};

struct TimerConfig {
    TimerMode mode = TimerMode::PERIODIC;
    uint32_t period = 1000;  // мкс
    uint32_t prescaler = 0;
    bool interruptEnable = true;
    bool dmaEnable = false;
};

/**
 * @brief Интерфейс таймера
 */
class ITimer {
public:
    using Callback = Callback<void()>;

    virtual ~ITimer() = default;
    
    /// Инициализация
    virtual Status init(const TimerConfig& config) = 0;
    
    /// Установка callback
    virtual void setCallback(Callback callback) = 0;
    
    /// Запуск
    virtual void start() = 0;
    
    /// Остановка
    virtual void stop() = 0;
    
    /// Перезапуск с новым периодом
    virtual Status setPeriod(uint32_t periodUs) = 0;
    
    /// Получить текущее значение счётчика
    virtual uint32_t getCount() const = 0;
    
    /// Установка PWM (для PWM режима)
    virtual Status setPWM(uint8_t channel, float dutyCycle) = 0;
    
    /// Получение измеренного значения (для input capture)
    virtual uint32_t getCaptureValue(uint8_t channel) const = 0;
};

// ============================================================================
// Интерфейс Flash памяти
// ============================================================================

struct FlashRegion {
    uint32_t address;
    uint32_t size;
    uint32_t eraseSize;
    bool writeProtected;
};

/**
 * @brief Интерфейс Flash памяти
 */
class IFlash {
public:
    virtual ~IFlash() = default;
    
    /// Получить информацию о регионах
    virtual size_t getRegions(std::span<FlashRegion> regions) const = 0;
    
    /// Стирание сектора
    virtual Status erase(uint32_t address, uint32_t size) = 0;
    
    /// Запись
    virtual Status write(uint32_t address, 
                        std::span<const uint8_t> data) = 0;
    
    /// Чтение
    virtual Status read(uint32_t address, 
                       std::span<uint8_t> data) const = 0;
    
    /// Проверка на пустоту
    virtual bool isErased(uint32_t address, uint32_t size) const = 0;
    
    /// Установка защиты
    virtual Status setWriteProtection(uint32_t address, 
                                      bool protect) = 0;
};

// ============================================================================
// Интерфейс EEPROM/FRAM
// ============================================================================

/**
 * @brief Интерфейс энергонезависимой памяти
 */
class INonVolatileMemory {
public:
    virtual ~INonVolatileMemory() = default;
    
    /// Размер памяти
    virtual size_t getSize() const = 0;
    
    /// Размер страницы
    virtual size_t getPageSize() const = 0;
    
    /// Чтение
    virtual Status read(uint32_t address, 
                       std::span<uint8_t> data) = 0;
    
    /// Запись
    virtual Status write(uint32_t address,
                        std::span<const uint8_t> data) = 0;
    
    /// Стирание страницы
    virtual Status erasePage(uint32_t address) = 0;
    
    /// Полное стирание
    virtual Status eraseAll() = 0;
};

// ============================================================================
// Интерфейс SD карты
// ============================================================================

struct SDCardInfo {
    uint8_t cardType;
    uint32_t sectorCount;
    uint32_t sectorSize;
    uint32_t blockSize;
    char manufacturer[16];
    char product[16];
};

/**
 * @brief Интерфейс SD карты
 */
class ISDCard {
public:
    virtual ~ISDCard() = default;
    
    /// Инициализация
    virtual Status init() = 0;
    
    /// Проверка наличия карты
    virtual bool isPresent() const = 0;
    
    /// Получить информацию
    virtual Status getInfo(SDCardInfo& info) = 0;
    
    /// Чтение секторов
    virtual Status readSectors(uint32_t sector,
                              std::span<uint8_t> data,
                              uint32_t count) = 0;
    
    /// Запись секторов
    virtual Status writeSectors(uint32_t sector,
                               std::span<const uint8_t> data,
                               uint32_t count) = 0;
    
    /// Синхронизация (flush)
    virtual Status sync() = 0;
    
    /// Получить размер в байтах
    virtual uint64_t getSize() const = 0;
};

// ============================================================================
// Интерфейс питания
// ============================================================================

struct PowerStatus {
    float batteryVoltage;
    float batteryCurrent;
    float batteryCharge;      // %
    float solarVoltage;
    float solarCurrent;
    float bus3V3Voltage;
    float bus5VVoltage;
    float temperature;
    bool charging;
    bool externalPower;
};

struct PowerConfig {
    float batteryLowThreshold = 6.5f;
    float batteryCriticalThreshold = 5.8f;
    float overcurrentThreshold = 3.0f;
    bool enableMPPT = true;
};

/**
 * @brief Интерфейс системы электропитания
 */
class IPowerSystem {
public:
    using AlertCallback = Callback<void(uint8_t alertType)>;

    virtual ~IPowerSystem() = default;
    
    /// Инициализация
    virtual Status init(const PowerConfig& config) = 0;
    
    /// Получить статус
    virtual Status getStatus(PowerStatus& status) = 0;
    
    /// Установка callback для предупреждений
    virtual void setAlertCallback(AlertCallback callback) = 0;
    
    /// Включение/выключение линии питания
    virtual Status setPowerRail(uint8_t rail, bool enabled) = 0;
    
    /// Переключение в режим пониженного потребления
    virtual Status enterLowPower(PowerMode mode) = 0;
    
    /// Выход из режима пониженного потребления
    virtual void exitLowPower() = 0;
    
    /// Получить текущий режим
    virtual PowerMode getCurrentMode() const = 0;
};

// ============================================================================
// Интерфейс CRC
// ============================================================================

enum class CRCType : uint8_t {
    CRC8,
    CRC16_CCITT,
    CRC16_MODBUS,
    CRC32,
    CRC32C
};

/**
 * @brief Интерфейс вычисления CRC
 */
class ICRC {
public:
    virtual ~ICRC() = default;
    
    /// Вычисление CRC
    virtual uint32_t calculate(CRCType type,
                              std::span<const uint8_t> data,
                              uint32_t initialValue = 0xFFFFFFFF) = 0;
    
    /// Проверка CRC
    virtual bool verify(CRCType type,
                       std::span<const uint8_t> data,
                       uint32_t expected) = 0;
};

// ============================================================================
// Интерфейс RNG
// ============================================================================

/**
 * @brief Интерфейс генератора случайных чисел
 */
class IRNG {
public:
    virtual ~IRNG() = default;
    
    /// Получить случайное 32-битное число
    virtual Status getRandom(uint32_t& value) = 0;
    
    /// Заполнить буфер случайными данными
    virtual Status fillRandom(std::span<uint8_t> buffer) = 0;
    
    /// Получить случайное число в диапазоне
    virtual Status getRandomRange(uint32_t min, uint32_t max, 
                                  uint32_t& value) = 0;
};

// ============================================================================
// Интерфейс DMA
// ============================================================================

enum class DMADirection : uint8_t {
    MEM_TO_MEM,
    MEM_TO_PERIPH,
    PERIPH_TO_MEM,
    PERIPH_TO_PERIPH
};

struct DMAConfig {
    DMADirection direction;
    uint8_t channel;
    bool memIncrement = true;
    bool periphIncrement = false;
    uint8_t memDataSize = 1;    // 1, 2, or 4 bytes
    uint8_t periphDataSize = 1;
    bool circular = false;
    uint8_t priority = 0;
};

/**
 * @brief Интерфейс DMA
 */
class IDMA {
public:
    using TransferCallback = Callback<void()>;

    virtual ~IDMA() = default;
    
    /// Конфигурация
    virtual Status configure(uint8_t stream, const DMAConfig& config) = 0;
    
    /// Запуск передачи
    virtual Status startTransfer(uint8_t stream,
                                void* src, void* dst,
                                size_t length,
                                TransferCallback callback = nullptr) = 0;
    
    /// Остановка
    virtual void stop(uint8_t stream) = 0;
    
    /// Проверка завершения
    virtual bool isComplete(uint8_t stream) const = 0;
    
    /// Получение оставшихся данных
    virtual size_t getRemaining(uint8_t stream) const = 0;
};

// ============================================================================
// Интерфейс прерываний
// ============================================================================

/**
 * @brief Интерфейс управления прерываниями
 */
class IInterrupts {
public:
    virtual ~IInterrupts() = default;
    
    /// Глобальное включение
    virtual void enable() = 0;
    
    /// Глобальное выключение
    virtual void disable() = 0;
    
    /// Получить текущее состояние
    virtual bool isEnabled() const = 0;
    
    /// Установить приоритет
    virtual void setPriority(IRQn_Type irq, uint8_t priority) = 0;
    
    /// Включить прерывание
    virtual void enableIRQ(IRQn_Type irq) = 0;
    
    /// Выключить прерывание
    virtual void disableIRQ(IRQn_Type irq) = 0;
    
    /// Очистить флаг прерывания
    virtual void clearPending(IRQn_Type irq) = 0;
};

// ============================================================================
// Интерфейс NVIC (заглушка для IRQn_Type)
// ============================================================================

using IRQn_Type = int;

} // namespace hal
} // namespace mka

#endif // HAL_FULL_HPP
