/**
 * @file hal_mocks.hpp
 * @brief Mock-классы для HAL интерфейсов для модульного тестирования
 *
 * Позволяют тестировать драйверы без реального оборудования.
 */

#ifndef HAL_MOCKS_HPP
#define HAL_MOCKS_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <span>
#include <vector>
#include <unordered_map>

#include "hal/hal_full.hpp"

namespace mka {
namespace hal {
namespace mocks {

// ============================================================================
// Mock SystemTime
// ============================================================================

class MockSystemTime : public ISystemTime {
public:
    void setCurrentTime(uint64_t ms) { currentTime_ = ms; }
    void advanceTime(uint64_t ms) { currentTime_ += ms; }

    uint64_t getMs() const override { return currentTime_; }
    uint64_t getUs() const override { return currentTime_ * 1000; }

    void delayMs(uint32_t ms) override { currentTime_ += ms; }
    void delayUs(uint32_t us) override { currentTime_ += us / 1000; }

    uint64_t getBootTime() const override { return bootTime_; }
    void setTime(uint64_t unixTimeMs) override { currentTime_ = unixTimeMs; }

private:
    uint64_t currentTime_ = 0;
    uint64_t bootTime_ = 0;
};

// ============================================================================
// Mock GPIO
// ============================================================================

class MockGPIO : public IGPIO {
public:
    struct PinState {
        GPIOConfig config{};
        GPIOState state = GPIOState::LOW;
        bool initialized = false;
    };

    Status init(uint8_t pin, const GPIOConfig& config) override {
        if (pin >= MAX_PINS) return Status::INVALID_PARAM;
        pins_[pin] = {config, GPIOState::LOW, true};
        return Status::OK;
    }

    Status deinit(uint8_t pin) override {
        if (pin >= MAX_PINS) return Status::INVALID_PARAM;
        pins_[pin].initialized = false;
        return Status::OK;
    }

    Status write(uint8_t pin, GPIOState state) override {
        if (pin >= MAX_PINS || !pins_[pin].initialized) return Status::INVALID_PARAM;
        pins_[pin].state = state;
        writeHistory_.push_back({pin, state});
        return Status::OK;
    }

    GPIOState read(uint8_t pin) const override {
        if (pin >= MAX_PINS || !pins_[pin].initialized) return GPIOState::LOW;
        return pins_[pin].state;
    }

    Status toggle(uint8_t pin) override {
        if (pin >= MAX_PINS || !pins_[pin].initialized) return Status::INVALID_PARAM;
        pins_[pin].state = (pins_[pin].state == GPIOState::LOW) ? GPIOState::HIGH : GPIOState::LOW;
        return Status::OK;
    }

    Status setInterruptCallback(uint8_t pin, InterruptCallback callback,
                                uint8_t triggerEdge) override {
        if (pin >= MAX_PINS) return Status::INVALID_PARAM;
        callbacks_[pin] = {callback, triggerEdge};
        return Status::OK;
    }

    // Методы для тестирования
    void simulateInterrupt(uint8_t pin) {
        if (pin < MAX_PINS && callbacks_[pin].callback) {
            callbacks_[pin].callback(pin);
        }
    }

    struct WriteRecord {
        uint8_t pin;
        GPIOState state;
    };

    const std::vector<WriteRecord>& getWriteHistory() const { return writeHistory_; }
    void clearHistory() { writeHistory_.clear(); }

private:
    static constexpr size_t MAX_PINS = 64;
    PinState pins_[MAX_PINS] = {};

    struct CallbackData {
        InterruptCallback callback;
        uint8_t triggerEdge;
    };
    CallbackData callbacks_[MAX_PINS] = {};

    std::vector<WriteRecord> writeHistory_;
};

// ============================================================================
// Mock I2C
// ============================================================================

class MockI2C : public II2C {
public:
    struct RegisterMap {
        std::array<uint8_t, 256> registers{};
    };

    struct Device {
        uint8_t address;
        RegisterMap registers;
        bool present = true;
    };

    Status init(const I2CConfig& config) override {
        config_ = config;
        initialized_ = true;
        return Status::OK;
    }

    void deinit() override {
        initialized_ = false;
    }

    Status writeRegister(uint8_t devAddress, uint8_t regAddress,
                        std::span<const uint8_t> data,
                        uint32_t timeoutMs) override {
        if (!initialized_) return Status::NOT_INITIALIZED;

        auto it = devices_.find(devAddress);
        if (it == devices_.end() || !it->second.present) {
            stats_.nackCount++;
            return Status::ERROR;
        }

        if (regAddress + data.size() <= 256) {
            for (size_t i = 0; i < data.size(); i++) {
                it->second.registers.registers[regAddress + i] = data[i];
            }
            stats_.txBytes += data.size();
        }

        writeHistory_.push_back({devAddress, regAddress, {data.begin(), data.end()}});
        return Status::OK;
    }

    Status readRegister(uint8_t devAddress, uint8_t regAddress,
                       std::span<uint8_t> data,
                       uint32_t timeoutMs) override {
        if (!initialized_) return Status::NOT_INITIALIZED;

        auto it = devices_.find(devAddress);
        if (it == devices_.end() || !it->second.present) {
            stats_.nackCount++;
            return Status::ERROR;
        }

        if (regAddress + data.size() <= 256) {
            for (size_t i = 0; i < data.size(); i++) {
                data[i] = it->second.registers.registers[regAddress + i];
            }
            stats_.rxBytes += data.size();
        }

        return Status::OK;
    }

    Status writeRegister16(uint8_t devAddress, uint16_t regAddress,
                          std::span<const uint8_t> data,
                          uint32_t timeoutMs) override {
        // Упрощённая реализация
        return writeRegister(devAddress, static_cast<uint8_t>(regAddress), data, timeoutMs);
    }

    Status readRegister16(uint8_t devAddress, uint16_t regAddress,
                         std::span<uint8_t> data,
                         uint32_t timeoutMs) override {
        // Упрощённая реализация
        return readRegister(devAddress, static_cast<uint8_t>(regAddress), data, timeoutMs);
    }

    size_t scanBus(std::span<uint8_t> foundDevices) override {
        size_t count = 0;
        for (const auto& [addr, dev] : devices_) {
            if (dev.present && count < foundDevices.size()) {
                foundDevices[count++] = addr;
            }
        }
        return count;
    }

    bool isDevicePresent(uint8_t devAddress) override {
        auto it = devices_.find(devAddress);
        return it != devices_.end() && it->second.present;
    }

    Status recoverBus() override {
        return Status::OK;
    }

    I2CStatistics getStatistics() const override {
        return stats_;
    }

    // Методы для тестирования
    void addDevice(uint8_t address) {
        devices_[address] = Device{address, {}, true};
    }

    void removeDevice(uint8_t address) {
        devices_.erase(address);
    }

    void setDevicePresent(uint8_t address, bool present) {
        auto it = devices_.find(address);
        if (it != devices_.end()) {
            it->second.present = present;
        }
    }

    void setRegisterValue(uint8_t address, uint8_t reg, uint8_t value) {
        auto it = devices_.find(address);
        if (it != devices_.end()) {
            it->second.registers.registers[reg] = value;
        }
    }

    struct WriteRecord {
        uint8_t address;
        uint8_t registerAddr;
        std::vector<uint8_t> data;
    };

    const std::vector<WriteRecord>& getWriteHistory() const { return writeHistory_; }
    void clearHistory() { writeHistory_.clear(); }

private:
    I2CConfig config_{};
    bool initialized_ = false;
    I2CStatistics stats_{};
    std::vector<WriteRecord> writeHistory_;
    std::unordered_map<uint8_t, Device> devices_;
};

// ============================================================================
// Mock SPI
// ============================================================================

class MockSPI : public ISPI {
public:
    Status init(const SPIConfig& config) override {
        config_ = config;
        initialized_ = true;
        return Status::OK;
    }

    void deinit() override {
        initialized_ = false;
    }

    Status selectDevice() override {
        if (!initialized_) return Status::NOT_INITIALIZED;
        selected_ = true;
        return Status::OK;
    }

    void deselectDevice() override {
        selected_ = false;
    }

    Status transfer(std::span<const uint8_t> txData,
                   std::span<uint8_t> rxData,
                   uint32_t timeoutMs) override {
        if (!initialized_) return Status::NOT_INITIALIZED;
        if (!selected_) return Status::ERROR;

        size_t len = std::min(txData.size(), rxData.size());
        for (size_t i = 0; i < len; i++) {
            // Эмуляция эхо-ответа
            rxData[i] = txData[i];
        }
        return Status::OK;
    }

    Status transmit(std::span<const uint8_t> data, uint32_t timeoutMs) override {
        if (!initialized_) return Status::NOT_INITIALIZED;
        txHistory_.insert(txHistory_.end(), data.begin(), data.end());
        return Status::OK;
    }

    Status receive(std::span<uint8_t> data, uint32_t timeoutMs) override {
        if (!initialized_) return Status::NOT_INITIALIZED;
        // Заполняем нулями по умолчанию
        std::fill(data.begin(), data.end(), 0);
        return Status::OK;
    }

    Status transferAsync(std::span<const uint8_t> txData,
                        std::span<uint8_t> rxData,
                        TransferCallback callback) override {
        // Синхронная эмуляция
        auto status = transfer(txData, rxData, 1000);
        if (callback) callback(rxData);
        return status;
    }

    Status setClockSpeed(uint32_t speed) override {
        config_.clockSpeed = speed;
        return Status::OK;
    }

    // Методы для тестирования
    const std::vector<uint8_t>& getTxHistory() const { return txHistory_; }
    void clearHistory() { txHistory_.clear(); }

    bool isSelected() const { return selected_; }

private:
    SPIConfig config_{};
    bool initialized_ = false;
    bool selected_ = false;
    std::vector<uint8_t> txHistory_;
};

// ============================================================================
// Mock UART
// ============================================================================

class MockUART : public IUART {
public:
    Status init(const UARTConfig& config) override {
        config_ = config;
        initialized_ = true;
        return Status::OK;
    }

    void deinit() override {
        initialized_ = false;
    }

    Status transmit(std::span<const uint8_t> data, uint32_t timeoutMs) override {
        if (!initialized_) return Status::NOT_INITIALIZED;
        rxBuffer_.insert(rxBuffer_.end(), data.begin(), data.end());
        stats_.txBytes += data.size();
        return Status::OK;
    }

    Status receive(std::span<uint8_t> buffer, uint32_t timeoutMs,
                  size_t& received) override {
        if (!initialized_) return Status::NOT_INITIALIZED;

        received = std::min(buffer.size(), txBuffer_.size());
        std::memcpy(buffer.data(), txBuffer_.data(), received);
        txBuffer_.erase(txBuffer_.begin(), txBuffer_.begin() + received);
        stats_.rxBytes += received;
        return Status::OK;
    }

    Status transmitAsync(std::span<const uint8_t> data,
                        TxCompleteCallback callback) override {
        auto status = transmit(data, 1000);
        if (callback) callback();
        return status;
    }

    void setRxCallback(RxCallback callback) override {
        rxCallback_ = callback;
    }

    Status startContinuousRx() override {
        continuousRx_ = true;
        return Status::OK;
    }

    void stopContinuousRx() override {
        continuousRx_ = false;
    }

    size_t available() const override {
        return txBuffer_.size();
    }

    void flush() override {
        txBuffer_.clear();
        rxBuffer_.clear();
    }

    UARTStatistics getStatistics() const override {
        return stats_;
    }

    void resetStatistics() override {
        stats_ = {};
    }

    // Методы для тестирования
    void writeTxBuffer(std::span<const uint8_t> data) {
        txBuffer_.insert(txBuffer_.end(), data.begin(), data.end());
    }

    const std::vector<uint8_t>& getRxBuffer() const { return rxBuffer_; }

    void simulateRxData(std::span<const uint8_t> data) {
        for (auto byte : data) {
            txBuffer_.push_back(byte);
        }
        if (rxCallback_ && continuousRx_) {
            rxCallback_(data);
        }
    }

private:
    UARTConfig config_{};
    bool initialized_ = false;
    bool continuousRx_ = false;
    UARTStatistics stats_{};
    RxCallback rxCallback_;
    std::vector<uint8_t> txBuffer_;
    std::vector<uint8_t> rxBuffer_;
};

// ============================================================================
// Mock ADC
// ============================================================================

class MockADC : public IADC {
public:
    Status init(const ADCConfig& config) override {
        config_ = config;
        initialized_ = true;
        return Status::OK;
    }

    Status calibrate() override {
        calibrated_ = true;
        return Status::OK;
    }

    Status read(uint8_t channel, uint16_t& value, uint32_t timeoutMs) override {
        if (!initialized_) return Status::NOT_INITIALIZED;
        auto it = channelValues_.find(channel);
        value = (it != channelValues_.end()) ? it->second : 0;
        return Status::OK;
    }

    Status readMulti(std::span<const uint8_t> channels,
                    std::span<uint16_t> values,
                    uint32_t timeoutMs) override {
        if (!initialized_) return Status::NOT_INITIALIZED;
        for (size_t i = 0; i < channels.size() && i < values.size(); i++) {
            values[i] = channelValues_[channels[i]];
        }
        return Status::OK;
    }

    Status startContinuous(std::span<const uint8_t> channels,
                          ConversionCallback callback) override {
        continuousChannels_.assign(channels.begin(), channels.end());
        continuousCallback_ = callback;
        continuous_ = true;
        return Status::OK;
    }

    void stopContinuous() override {
        continuous_ = false;
        continuousChannels_.clear();
    }

    float toVolts(uint16_t rawValue, uint8_t channel) const override {
        // Предполагаем 3.3V референс и 12 бит
        return (rawValue / 4095.0f) * 3.3f;
    }

    // Методы для тестирования
    void setChannelValue(uint8_t channel, uint16_t value) {
        channelValues_[channel] = value;
    }

    void simulateConversion(uint8_t channel, uint16_t value) {
        if (continuous_ && continuousCallback_) {
            continuousCallback_(channel, value);
        }
    }

private:
    ADCConfig config_{};
    bool initialized_ = false;
    bool calibrated_ = false;
    bool continuous_ = false;
    std::unordered_map<uint8_t, uint16_t> channelValues_;
    std::vector<uint8_t> continuousChannels_;
    ConversionCallback continuousCallback_;
};

} // namespace mocks
} // namespace hal
} // namespace mka

#endif // HAL_MOCKS_HPP
