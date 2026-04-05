/**
 * @file radio_driver.hpp
 * @brief Radio Transceiver Driver (SI4463/CC1125 compatible)
 * 
 * Драйвер радиопередатчика для спутниковой связи.
 * Поддерживает частоты UHF/VHF и модуляции GFSK/2FSK.
 */

#ifndef RADIO_DRIVER_HPP
#define RADIO_DRIVER_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <chrono>

#include "utils/callback.hpp"
#include "utils/span.hpp"

namespace mka {
namespace radio {

// ============================================================================
// Типы и константы
// ============================================================================

/// Статус операции
enum class RadioStatus : uint8_t {
    OK = 0,
    ERROR = 1,
    TIMEOUT = 2,
    BUSY = 3,
    TX_BUSY = 4,
    RX_BUSY = 5,
    INVALID_PARAM = 6,
    NOT_INITIALIZED = 7,
    HARDWARE_ERROR = 8
};

/// Режим работы
enum class RadioMode : uint8_t {
    SLEEP,          // Минимальное потребление
    STANDBY,        // Готовность к работе
    TX,             // Передача
    RX,             // Приём
    RX_CONTINUOUS   // Непрерывный приём
};

/// Тип модуляции
enum class Modulation : uint8_t {
    FSK = 0,
    GFSK = 1,
    OOK = 2,
    LORA = 3
};

/// Ширина полосы
enum class Bandwidth : uint8_t {
    BW_12_5_KHZ = 0,
    BW_25_KHZ = 1,
    BW_50_KHZ = 2,
    BW_100_KHZ = 3,
    BW_150_KHZ = 4,
    BW_200_KHZ = 5,
    BW_250_KHZ = 6,
    BW_300_KHZ = 7
};

/// Мощность передачи
enum class TxPower : uint8_t {
    MIN = 0,        // ~0 dBm
    LOW = 1,        // ~10 dBm
    MEDIUM = 2,     // ~20 dBm
    HIGH = 3,       // ~30 dBm
    MAX = 4         // Максимальная
};

// ============================================================================
// Конфигурация
// ============================================================================

struct RadioConfig {
    // Частоты
    uint32_t frequencyHz;           // Несущая частота (Гц)
    uint32_t channelSpacing;        // Шаг каналов (Гц)
    uint8_t channel;                // Номер канала
    
    // Модуляция
    Modulation modulation;
    Bandwidth rxBandwidth;
    uint32_t datarate;              // Скорость данных (бит/с)
    uint16_t deviation;             // Девиация частоты (Гц)
    
    // Передача
    TxPower txPower;
    uint16_t preambleLength;        // Длина преамбулы (биты)
    uint16_t syncWord;              // Синхрослово
    uint8_t syncWordLength;         // Длина синхрослова (биты)
    
    // Приём
    uint16_t rxTimeout;             // Таймаут приёма (мс)
    int16_t rssiThreshold;          // Порог RSSI (dBm)
    
    // Пакеты
    uint8_t maxPacketLength;
    bool crcEnable;
    bool whiteningEnable;
    bool manchesterEnable;
};

// ============================================================================
// Статус радио
// ============================================================================

struct RadioState {
    RadioMode mode;
    int16_t rssi;                   // Текущий RSSI (dBm)
    int16_t lastRssi;               // RSSI последнего пакета
    int8_t snr;                     // Отношение сигнал/шум (dB)
    int8_t frequencyError;          // Ошибка частоты (kHz)
    uint32_t txCount;               // Счётчик передач
    uint32_t rxCount;               // Счётчик приёмов
    uint32_t rxErrors;              // Ошибки приёма
    uint32_t txErrors;              // Ошибки передачи
    bool txBusy;
    bool rxBusy;
    bool channelFree;
};

// ============================================================================
// Пакет данных
// ============================================================================

struct RadioPacket {
    uint8_t length;
    int16_t rssi;
    int8_t snr;
    std::array<uint8_t, 256> data;
};

// ============================================================================
// Интерфейс SPI (абстракция)
// ============================================================================

class IRadioSPI {
public:
    virtual ~IRadioSPI() = default;
    virtual bool transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx) = 0;
    virtual void select() = 0;
    virtual void deselect() = 0;
};

// ============================================================================
// Интерфейс GPIO
// ============================================================================

class IRadioGPIO {
public:
    virtual ~IRadioGPIO() = default;
    virtual void setReset(bool state) = 0;
    virtual bool getInterrupt() = 0;
    virtual void setTxEnable(bool state) = 0;
    virtual void setRxEnable(bool state) = 0;
};

// ============================================================================
// Драйвер SI4463
// ============================================================================

/**
 * @brief Драйвер радиомодуля SI446x
 * 
 * Совместим с чипами SI4463, SI4464, SI4468.
 * Диапазон частот: 142-175, 420-525, 850-1050 МГц.
 */
class SI4463Driver {
public:
    static constexpr uint8_t FIFO_SIZE = 64;
    static constexpr uint16_t MAX_PACKET_SIZE = 255;

    using RxCallback = Callback<void(const RadioPacket&)>;
    using TxCallback = Callback<void(bool success)>;

    SI4463Driver(IRadioSPI& spi, IRadioGPIO& gpio)
        : spi_(spi), gpio_(gpio) {}
    
    // ========================================================================
    // Управление
    // ========================================================================
    
    /**
     * @brief Инициализация радио
     */
    RadioStatus init(const RadioConfig& config) {
        config_ = config;
        
        // Сброс чипа
        gpio_.setReset(true);
        delayMs(10);
        gpio_.setReset(false);
        delayMs(20);
        gpio_.setReset(true);
        delayMs(50);
        
        // Проверка наличия чипа
        uint8_t partInfo[8];
        if (!getPartInfo(partInfo)) {
            return RadioStatus::HARDWARE_ERROR;
        }
        
        // Загрузка конфигурации
        if (!loadConfig()) {
            return RadioStatus::ERROR;
        }
        
        initialized_ = true;
        return RadioStatus::OK;
    }
    
    /**
     * @brief Установка частоты
     */
    RadioStatus setFrequency(uint32_t freqHz) {
        if (!initialized_) return RadioStatus::NOT_INITIALIZED;
        
        config_.frequencyHz = freqHz;
        // Расчёт делителя частоты
        uint32_t freq = freqHz / 1000;  // kHz
        uint8_t params[4];
        params[0] = (freq >> 16) & 0xFF;
        params[1] = (freq >> 8) & 0xFF;
        params[2] = freq & 0xFF;
        
        return sendCommand(0x11, params, 3, nullptr, 0) 
            ? RadioStatus::OK : RadioStatus::ERROR;
    }
    
    /**
     * @brief Установка мощности передачи
     */
    RadioStatus setTxPower(TxPower power) {
        if (!initialized_) return RadioStatus::NOT_INITIALIZED;
        
        config_.txPower = power;
        
        // Конвертация в dBm
        int8_t dBm = powerToDbm(power);
        uint8_t params[2] = {0x00, static_cast<uint8_t>(dBm + 127)};
        
        return sendCommand(0x11, params, 2, nullptr, 0)
            ? RadioStatus::OK : RadioStatus::ERROR;
    }
    
    /**
     * @brief Установка режима
     */
    RadioStatus setMode(RadioMode mode) {
        if (!initialized_) return RadioStatus::NOT_INITIALIZED;
        
        state_.mode = mode;
        
        switch (mode) {
            case RadioMode::SLEEP:
                return sendCommand(0x13, nullptr, 0, nullptr, 0)
                    ? RadioStatus::OK : RadioStatus::ERROR;
            case RadioMode::STANDBY:
                return sendCommand(0x12, nullptr, 0, nullptr, 0)
                    ? RadioStatus::OK : RadioStatus::ERROR;
            case RadioMode::TX:
                // TX запускается при передаче пакета
                break;
            case RadioMode::RX:
            case RadioMode::RX_CONTINUOUS:
                return startRx()
                    ? RadioStatus::OK : RadioStatus::ERROR;
        }
        
        return RadioStatus::OK;
    }
    
    // ========================================================================
    // Передача
    // ========================================================================
    
    /**
     * @brief Передача пакета (блокирующая)
     */
    RadioStatus transmit(std::span<const uint8_t> data, uint32_t timeoutMs) {
        if (!initialized_) return RadioStatus::NOT_INITIALIZED;
        if (data.size() > MAX_PACKET_SIZE) return RadioStatus::INVALID_PARAM;
        if (state_.txBusy) return RadioStatus::TX_BUSY;
        
        state_.txBusy = true;
        
        // Заполнение TX FIFO
        if (!writeTxFifo(data)) {
            state_.txBusy = false;
            return RadioStatus::ERROR;
        }
        
        // Запуск передачи
        if (!startTx()) {
            state_.txBusy = false;
            return RadioStatus::ERROR;
        }
        
        // Ожидание завершения
        uint32_t start = getTickMs();
        while (getTickMs() - start < timeoutMs) {
            uint8_t status;
            if (getTxStatus(status)) {
                if (status == 0x00) {  // TX complete
                    state_.txCount++;
                    state_.txBusy = false;
                    return RadioStatus::OK;
                }
            }
            delayMs(1);
        }
        
        state_.txErrors++;
        state_.txBusy = false;
        return RadioStatus::TIMEOUT;
    }
    
    /**
     * @brief Асинхронная передача
     */
    RadioStatus transmitAsync(std::span<const uint8_t> data, TxCallback callback) {
        if (!initialized_) return RadioStatus::NOT_INITIALIZED;
        if (data.size() > MAX_PACKET_SIZE) return RadioStatus::INVALID_PARAM;
        if (state_.txBusy) return RadioStatus::TX_BUSY;
        
        txCallback_ = callback;
        state_.txBusy = true;
        
        // Заполнение FIFO и запуск TX
        if (!writeTxFifo(data) || !startTx()) {
            state_.txBusy = false;
            return RadioStatus::ERROR;
        }
        
        return RadioStatus::OK;
    }
    
    // ========================================================================
    // Приём
    // ========================================================================
    
    /**
     * @brief Приём пакета (блокирующий)
     */
    RadioStatus receive(RadioPacket& packet, uint32_t timeoutMs) {
        if (!initialized_) return RadioStatus::NOT_INITIALIZED;
        if (state_.rxBusy) return RadioStatus::RX_BUSY;
        
        // Запуск приёма
        if (!startRx()) {
            return RadioStatus::ERROR;
        }
        
        state_.rxBusy = true;
        
        // Ожидание пакета
        uint32_t start = getTickMs();
        while (getTickMs() - start < timeoutMs) {
            uint8_t intStatus[8];
            if (getInterruptStatus(intStatus)) {
                if (intStatus[0] & 0x02) {  // Packet received
                    if (readRxFifo(packet)) {
                        packet.rssi = state_.lastRssi;
                        packet.snr = state_.snr;
                        state_.rxCount++;
                        state_.rxBusy = false;
                        return RadioStatus::OK;
                    }
                }
            }
            delayMs(1);
        }
        
        state_.rxErrors++;
        state_.rxBusy = false;
        return RadioStatus::TIMEOUT;
    }
    
    /**
     * @brief Запуск непрерывного приёма с callback
     */
    RadioStatus startContinuousRx(RxCallback callback) {
        if (!initialized_) return RadioStatus::NOT_INITIALIZED;
        
        rxCallback_ = callback;
        return setMode(RadioMode::RX_CONTINUOUS);
    }
    
    /**
     * @brief Обработка прерывания (вызывать из ISR)
     */
    void handleInterrupt() {
        uint8_t intStatus[8];
        if (!getInterruptStatus(intStatus)) return;
        
        // TX complete
        if (intStatus[0] & 0x01) {
            state_.txCount++;
            state_.txBusy = false;
            if (txCallback_) {
                txCallback_(true);
            }
        }
        
        // RX complete
        if (intStatus[0] & 0x02) {
            RadioPacket packet;
            if (readRxFifo(packet)) {
                packet.rssi = state_.lastRssi;
                packet.snr = state_.snr;
                state_.rxCount++;
                if (rxCallback_) {
                    rxCallback_(packet);
                }
            }
            
            // Перезапуск RX в continuous режиме
            if (state_.mode == RadioMode::RX_CONTINUOUS) {
                startRx();
            }
        }
    }
    
    // ========================================================================
    // Информация
    // ========================================================================
    
    /**
     * @brief Получение состояния
     */
    const RadioState& getState() const { return state_; }
    
    /**
     * @brief Получение RSSI
     */
    int16_t getRssi() {
        uint8_t params[2] = {0x00, 0x00};
        uint8_t response[4];
        
        if (sendCommand(0x14, params, 2, response, 4)) {
            state_.rssi = -((response[2] << 8 | response[3]) / 2);
        }
        
        return state_.rssi;
    }
    
    /**
     * @brief Проверка свободного канала (CCA)
     */
    bool isChannelFree() {
        int16_t rssi = getRssi();
        return rssi < config_.rssiThreshold;
    }
    
private:
    IRadioSPI& spi_;
    IRadioGPIO& gpio_;
    
    RadioConfig config_{};
    RadioState state_{};
    bool initialized_ = false;
    
    RxCallback rxCallback_;
    TxCallback txCallback_;
    
    // ========================================================================
    // Низкоуровневые функции
    // ========================================================================
    
    bool sendCommand(uint8_t cmd, const uint8_t* params, size_t paramLen,
                     uint8_t* response, size_t respLen) {
        spi_.select();

        // Отправка команды
        bool success = spi_.transfer({&cmd, 1}, {});
        if (!success) {
            spi_.deselect();
            return false;
        }

        if (paramLen > 0 && params) {
            success = spi_.transfer({params, paramLen}, {});
            if (!success) {
                spi_.deselect();
                return false;
            }
        }

        // Чтение ответа
        if (respLen > 0 && response) {
            delayMs(1);  // Wait for processing
            success = spi_.transfer({}, {response, respLen});
        }

        spi_.deselect();
        return success;
    }
    
    bool getPartInfo(uint8_t* info) {
        return sendCommand(0x10, nullptr, 0, info, 8);
    }
    
    bool loadConfig() {
        // Загрузка конфигурации из массива (упрощённо)
        // В реальном проекте используется API config от SiLabs
        return true;
    }
    
    bool writeTxFifo(std::span<const uint8_t> data) {
        uint8_t cmd = 0x12;  // Write TX FIFO
        spi_.select();
        bool success = spi_.transfer({&cmd, 1}, {});
        if (success) {
            success = spi_.transfer({data.data(), data.size()}, {});
        }
        spi_.deselect();
        return success;
    }
    
    bool readRxFifo(RadioPacket& packet) {
        uint8_t cmd = 0x13;  // Read RX FIFO
        uint8_t len;

        spi_.select();
        bool success = spi_.transfer({&cmd, 1}, {});
        if (!success) {
            spi_.deselect();
            return false;
        }

        success = spi_.transfer({}, {&len, 1});
        if (!success) {
            spi_.deselect();
            return false;
        }

        // Проверка что длина не превышает размер буфера
        if (len > packet.data.size()) {
            spi_.deselect();
            return false;  // Переполнение буфера - отбрасываем пакет
        }

        packet.length = len;
        success = spi_.transfer({}, {packet.data.data(), len});
        spi_.deselect();

        return success;
    }
    
    bool startTx() {
        uint8_t params[4] = {0x00, 0x00, 0x00, 0x00};
        return sendCommand(0x31, params, 4, nullptr, 0);
    }
    
    bool startRx() {
        uint8_t params[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
        return sendCommand(0x32, params, 5, nullptr, 0);
    }
    
    bool getTxStatus(uint8_t& status) {
        uint8_t cmd = 0x22;
        return sendCommand(cmd, nullptr, 0, &status, 1);
    }
    
    bool getInterruptStatus(uint8_t* status) {
        uint8_t cmd = 0x20;
        uint8_t params[2] = {0x00, 0x00};
        return sendCommand(cmd, params, 2, status, 8);
    }
    
    int8_t powerToDbm(TxPower power) {
        switch (power) {
            case TxPower::MIN: return 0;
            case TxPower::LOW: return 10;
            case TxPower::MEDIUM: return 20;
            case TxPower::HIGH: return 27;
            case TxPower::MAX: return 30;
            default: return 10;
        }
    }
    
    void delayMs(uint32_t ms) {
        // Platform-specific delay
    }

    uint32_t getTickMs() {
        // Platform-specific tick
        // Для host build используем стандартное время
        // Для STM32 использовать HAL_GetTick() или SysTick
#ifdef __GNUC__
        // На bare-metal это должна быть реализация через SysTick
        // Заглушка - возвращаем 0, но с комментарием что это требует реализации
        return 0;  // TODO: реализовать через HAL_GetTick() или SysTick
#else
        // На хосте используем chrono
        auto now = std::chrono::steady_clock::now();
        return static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count()
        );
#endif
    }
};

// ============================================================================
// Константы для спутниковой связи
// ============================================================================

namespace satellite {

// UHF Amateur Band
constexpr uint32_t UHF_FREQ_MIN = 430000000;
constexpr uint32_t UHF_FREQ_MAX = 440000000;
constexpr uint32_t UHF_DEFAULT = 436700000;  // 436.7 MHz

// VHF Amateur Band
constexpr uint32_t VHF_FREQ_MIN = 144000000;
constexpr uint32_t VHF_FREQ_MAX = 146000000;
constexpr uint32_t VHF_DEFAULT = 145800000;  // 145.8 MHz

// Типичные параметры
constexpr uint32_t DATARATE_1200 = 1200;    // AX.25 AFSK
constexpr uint32_t DATARATE_9600 = 9600;    // GFSK
constexpr uint32_t DATARATE_38K4 = 38400;   // High-speed
constexpr uint32_t DATARATE_115K2 = 115200; // Very high-speed

} // namespace satellite

} // namespace radio
} // namespace mka

#endif // RADIO_DRIVER_HPP
