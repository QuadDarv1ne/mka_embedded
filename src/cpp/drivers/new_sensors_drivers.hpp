/**
 * @file new_sensors_drivers.hpp
 * @brief Новые драйверы датчиков для МКА
 *
 * Содержит драйверы:
 * - HMC5883L: 3-осевой магнитометр с калибровкой
 * - MAX31865: Прецизионный датчик температуры (PT100/PT1000)
 */

#ifndef NEW_SENSORS_DRIVERS_HPP
#define NEW_SENSORS_DRIVERS_HPP

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <span>
#include <vector>
#include "hal/hal_full.hpp"

namespace mka {
namespace sensors {

// Локальное объявление MagData чтобы избежать зависимости от sensors_drivers.hpp
struct MagData {
    float mag[3];
    bool overflow;
    uint64_t timestamp;
};

// ============================================================================
// HMC5883L - 3-осевой магнитометр
// ============================================================================

/**
 * @brief Драйвер 3-осевого магнитометра HMC5883L (Honeywell)
 *
 * Особенности:
 * - I2C интерфейс (адрес 0x1E)
 * - Разрешение 1-2 мГauss
 * - Диапазон: ±1.3 до ±8.1 Гаусс
 * - Встроенный 12-битный АЦП
 * - Калибровка hard/soft iron
 *
 * @note HMC5883L устарел, рекомендуется QMC5883L как замена
 */
class HMC5883LDriver {
public:
    static constexpr uint8_t I2C_ADDR = 0x1E;
    static constexpr uint8_t ID_A = 0x48;
    static constexpr uint8_t ID_B = 0x34;
    static constexpr uint8_t ID_C = 0x33;

    // Регистры
    enum Register : uint8_t {
        CONFIG_A       = 0x00,
        CONFIG_B       = 0x01,
        MODE           = 0x02,
        DATA_X_MSB     = 0x03,
        DATA_X_LSB     = 0x04,
        DATA_Z_MSB     = 0x05,
        DATA_Z_LSB     = 0x06,
        DATA_Y_MSB     = 0x07,
        DATA_Y_LSB     = 0x08,
        STATUS         = 0x09,
        ID_A_REG       = 0x0A,
        ID_B_REG       = 0x0B,
        ID_C_REG       = 0x0C,
    };

    // Диапазоны измерения
    enum class Scale : uint8_t {
        SCALE_0_88G = 0x00,  // ±0.88 Гаусс
        SCALE_1_3G  = 0x20,  // ±1.3 Гаусс (по умолчанию)
        SCALE_1_9G  = 0x40,  // ±1.9 Гаусс
        SCALE_2_5G  = 0x60,  // ±2.5 Гаусс
        SCALE_4_0G  = 0x80,  // ±4.0 Гаусс
        SCALE_4_7G  = 0xA0,  // ±4.7 Гаусс
        SCALE_5_6G  = 0xC0,  // ±5.6 Гаусс
        SCALE_8_1G  = 0xE0,  // ±8.1 Гаусс
    };

    // Режимы работы
    enum class Mode : uint8_t {
        CONTINUOUS = 0x00,
        SINGLE     = 0x01,
        IDLE       = 0x03,
    };

    // Скорость обновления (ODR)
    enum class DataRate : uint8_t {
        RATE_0_75HZ  = 0x00,
        RATE_1_5HZ   = 0x04,
        RATE_3HZ     = 0x08,
        RATE_7_5HZ   = 0x0C,
        RATE_15HZ    = 0x10,
        RATE_30HZ    = 0x14,
        RATE_75HZ    = 0x18,
    };

    // samples per measurement
    enum class Samples : uint8_t {
        SAMPLES_1 = 0x00,
        SAMPLES_2 = 0x40,
        SAMPLES_4 = 0x80,
        SAMPLES_8 = 0xC0,
    };

    // Калибровочные параметры
    struct CalibrationData {
        float offset[3] = {0.0f, 0.0f, 0.0f};
        float softIronMatrix[9] = {
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f
        };
        bool isCalibrated = false;
    };

    struct Config {
        Scale scale = Scale::SCALE_1_3G;
        Mode mode = Mode::CONTINUOUS;
        DataRate dataRate = DataRate::RATE_15HZ;
        Samples samples = Samples::SAMPLES_1;
        bool autoCalibrate = false;
    };

    HMC5883LDriver(hal::II2C& i2c, uint8_t address = I2C_ADDR)
        : i2c_(i2c), address_(address) {}

    hal::Status init() {
        Config config;
        return init(config);
    }
    
    hal::Status init(const Config& config) {
        config_ = config;

        // Проверка ID
        uint8_t idA, idB, idC;
        hal::Status st;
        
        st = i2c_.readRegister(address_, ID_A_REG, std::span<uint8_t>(&idA, 1), 100);
        if (st != hal::Status::OK) { errorCount_++; return st; }
        
        st = i2c_.readRegister(address_, ID_B_REG, std::span<uint8_t>(&idB, 1), 100);
        if (st != hal::Status::OK) { errorCount_++; return st; }
        
        st = i2c_.readRegister(address_, ID_C_REG, std::span<uint8_t>(&idC, 1), 100);
        if (st != hal::Status::OK) { errorCount_++; return st; }

        if (idA != ID_A || idB != ID_B || idC != ID_C) {
            return hal::Status::ERROR;
        }

        uint8_t cfgA = static_cast<uint8_t>(config.samples) | static_cast<uint8_t>(config.dataRate);
        st = i2c_.writeRegister(address_, CONFIG_A, std::span<const uint8_t>(&cfgA, 1), 100);
        if (st != hal::Status::OK) { errorCount_++; return st; }

        uint8_t cfgB = static_cast<uint8_t>(config.scale);
        st = i2c_.writeRegister(address_, CONFIG_B, std::span<const uint8_t>(&cfgB, 1), 100);
        if (st != hal::Status::OK) { errorCount_++; return st; }

        uint8_t mode = static_cast<uint8_t>(config.mode);
        st = i2c_.writeRegister(address_, MODE, std::span<const uint8_t>(&mode, 1), 100);
        if (st != hal::Status::OK) { errorCount_++; return st; }

        updateSensitivity(config.scale);

        if (config.autoCalibrate) {
            return calibrateHardIron();
        }

        return hal::Status::OK;
    }

    hal::Status readData(MagData& data) {
        uint8_t status;
        hal::Status st = i2c_.readRegister(address_, STATUS, std::span<uint8_t>(&status, 1), 100);
        if (st != hal::Status::OK) { errorCount_++; return st; }

        if (!(status & 0x01)) { return hal::Status::BUSY; }

        uint8_t buffer[6];
        st = i2c_.readRegister(address_, DATA_X_MSB | 0x80, std::span<uint8_t>(buffer, 6), 100);
        if (st != hal::Status::OK) { errorCount_++; return st; }

        int16_t x = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
        int16_t z = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
        int16_t y = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);

        data.overflow = (x == -4096 || y == -4096 || z == -4096);
        
        float rawMag[3] = {x * sensitivity_, y * sensitivity_, z * sensitivity_};

        if (calibration_.isCalibrated) {
            applyCalibration(rawMag, data.mag);
        } else {
            data.mag[0] = rawMag[0];
            data.mag[1] = rawMag[1];
            data.mag[2] = rawMag[2];
        }

        data.timestamp = 0;
        return hal::Status::OK;
    }

    hal::Status calibrateHardIron() {
        float minValues[3] = {999999.0f, 999999.0f, 999999.0f};
        float maxValues[3] = {-999999.0f, -999999.0f, -999999.0f};
        isCalibrating_ = true;
        calibrationSamples_ = 0;

        for (int i = 0; i < 100; ++i) {
            MagData data;
            hal::Status st = readRawData(data);
            if (st == hal::Status::OK) {
                for (int axis = 0; axis < 3; ++axis) {
                    if (data.mag[axis] < minValues[axis]) minValues[axis] = data.mag[axis];
                    if (data.mag[axis] > maxValues[axis]) maxValues[axis] = data.mag[axis];
                }
                calibrationSamples_++;
            }
        }

        if (calibrationSamples_ < 10) {
            isCalibrating_ = false;
            return hal::Status::ERROR;
        }

        for (int axis = 0; axis < 3; ++axis) {
            calibration_.offset[axis] = (maxValues[axis] + minValues[axis]) / 2.0f;
        }

        calibration_.isCalibrated = true;
        isCalibrating_ = false;
        return hal::Status::OK;
    }

    hal::Status setSoftIronCalibration(const float matrix[9]) {
        if (!matrix) return hal::Status::INVALID_PARAM;
        for (int i = 0; i < 9; ++i) calibration_.softIronMatrix[i] = matrix[i];
        calibration_.isCalibrated = true;
        return hal::Status::OK;
    }

    void setCalibration(const CalibrationData& cal) { calibration_ = cal; }
    const CalibrationData& getCalibration() const { return calibration_; }
    void resetCalibration() { calibration_ = CalibrationData{}; }

    hal::Status setMode(Mode mode) {
        uint8_t modeReg = static_cast<uint8_t>(mode);
        hal::Status st = i2c_.writeRegister(address_, MODE, std::span<const uint8_t>(&modeReg, 1), 100);
        if (st != hal::Status::OK) errorCount_++;
        config_.mode = mode;
        return st;
    }

    hal::Status setScale(Scale scale) {
        uint8_t cfgB = static_cast<uint8_t>(scale);
        hal::Status st = i2c_.writeRegister(address_, CONFIG_B, std::span<const uint8_t>(&cfgB, 1), 100);
        if (st != hal::Status::OK) { errorCount_++; return st; }
        config_.scale = scale;
        updateSensitivity(scale);
        return hal::Status::OK;
    }

    uint32_t getErrorCount() const { return errorCount_; }
    uint32_t getTimeoutCount() const { return timeoutCount_; }
    uint16_t getCalibrationSamples() const { return calibrationSamples_; }
    bool isCalibrating() const { return isCalibrating_; }
    void resetErrorCounters() { errorCount_ = 0; timeoutCount_ = 0; }

private:
    hal::II2C& i2c_;
    uint8_t address_;
    Config config_;
    float sensitivity_ = 0.0f;
    CalibrationData calibration_;
    uint32_t errorCount_ = 0;
    uint32_t timeoutCount_ = 0;
    float minValues_[3] = {999999.0f, 999999.0f, 999999.0f};
    float maxValues_[3] = {-999999.0f, -999999.0f, -999999.0f};
    bool isCalibrating_ = false;
    uint16_t calibrationSamples_ = 0;

    void updateSensitivity(Scale scale) {
        switch (scale) {
            case Scale::SCALE_0_88G: sensitivity_ = 0.73f / 1000.0f; break;
            case Scale::SCALE_1_3G:  sensitivity_ = 0.92f / 1000.0f; break;
            case Scale::SCALE_1_9G:  sensitivity_ = 1.22f / 1000.0f; break;
            case Scale::SCALE_2_5G:  sensitivity_ = 1.52f / 1000.0f; break;
            case Scale::SCALE_4_0G:  sensitivity_ = 2.27f / 1000.0f; break;
            case Scale::SCALE_4_7G:  sensitivity_ = 2.56f / 1000.0f; break;
            case Scale::SCALE_5_6G:  sensitivity_ = 3.03f / 1000.0f; break;
            case Scale::SCALE_8_1G:  sensitivity_ = 4.35f / 1000.0f; break;
        }
    }

    hal::Status readRawData(MagData& data) {
        uint8_t status;
        hal::Status st = i2c_.readRegister(address_, STATUS, std::span<uint8_t>(&status, 1), 100);
        if (st != hal::Status::OK) return st;
        if (!(status & 0x01)) return hal::Status::BUSY;

        uint8_t buffer[6];
        st = i2c_.readRegister(address_, DATA_X_MSB | 0x80, std::span<uint8_t>(buffer, 6), 100);
        if (st != hal::Status::OK) return st;

        int16_t x = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
        int16_t z = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
        int16_t y = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);

        data.mag[0] = x * sensitivity_;
        data.mag[1] = y * sensitivity_;
        data.mag[2] = z * sensitivity_;
        data.overflow = (x == -4096 || y == -4096 || z == -4096);
        data.timestamp = 0;
        return hal::Status::OK;
    }

    void applyCalibration(const float raw[3], float output[3]) {
        float corrected[3] = {
            raw[0] - calibration_.offset[0],
            raw[1] - calibration_.offset[1],
            raw[2] - calibration_.offset[2]
        };
        for (int row = 0; row < 3; ++row) {
            output[row] = 0.0f;
            for (int col = 0; col < 3; ++col) {
                output[row] += calibration_.softIronMatrix[row * 3 + col] * corrected[col];
            }
        }
    }
};

// ============================================================================
// MAX31865 - прецизионный датчик температуры (PT100/PT1000)
// ============================================================================

/**
 * @brief Драйвер прецизионного датчика температуры MAX31865
 *
 * Особенности:
 * - SPI интерфейс (до 5 МГц)
 * - Поддержка PT100, PT1000, PT500
 * - Разрешение 15-бит (0.03125°C)
 * - Точность ±0.5°C (0-500°C для PT100)
 * - Компенсация сопротивления проводов
 * - Детекция обрыва/КЗ датчика
 */
class MAX31865Driver {
public:
    enum class RTDType : uint8_t { PT100 = 0, PT500 = 1, PT1000 = 2 };

    enum Register : uint8_t {
        CONFIG_REG = 0x00, RTD_MSB = 0x01, RTD_LSB = 0x02,
        HIGH_FAULT_MSB = 0x03, HIGH_FAULT_LSB = 0x04,
        LOW_FAULT_MSB = 0x05, LOW_FAULT_LSB = 0x06, FAULT_STATUS = 0x07,
    };

    enum ConfigBit : uint8_t {
        BIAS_ON = 0x80, AUTO_CONV = 0x40, ONE_SHOT = 0x20,
        WIRE_3 = 0x10, FAULT_STAT_CLEAR = 0x02,
    };

    struct CallendarCoefficients {
        float A = 3.9083e-3f;
        float B = -5.775e-7f;
        float C = -4.183e-12f;
    };

    struct Config {
        RTDType rtdType = RTDType::PT100;
        float rref = 4300.0f;
        uint8_t wires = 4;
        bool autoConvert = true;
        CallendarCoefficients coeffs;
    };

    struct TempData {
        float temperature;
        float resistance;
        uint16_t rtdCode;
        bool faultDetected;
        uint8_t faultStatus;
        uint64_t timestamp;
    };

    MAX31865Driver(hal::ISPI& spi, uint8_t csPin = 0)
        : spi_(spi), csPin_(csPin) {}

    hal::Status init() {
        Config config;
        return init(config);
    }
    
    hal::Status init(const Config& config) {
        config_ = config;
        updateCoefficients(config.rtdType);

        uint8_t cfg = BIAS_ON;
        if (config.autoConvert) cfg |= AUTO_CONV;
        if (config.wires == 3) cfg |= WIRE_3;
        cfg |= FAULT_STAT_CLEAR;

        hal::Status st = writeRegister(CONFIG_REG, cfg);
        if (st != hal::Status::OK) { errorCount_++; return st; }

        uint8_t fault;
        st = readRegister(FAULT_STATUS, &fault);
        if (st != hal::Status::OK) { errorCount_++; return st; }

        return hal::Status::OK;
    }

    hal::Status readTemperature(TempData& data) {
        if (!config_.autoConvert) {
            uint8_t cfg;
            hal::Status st = readRegister(CONFIG_REG, &cfg);
            if (st != hal::Status::OK) { errorCount_++; return st; }
            cfg |= ONE_SHOT;
            st = writeRegister(CONFIG_REG, cfg);
            if (st != hal::Status::OK) { errorCount_++; return st; }
        }

        uint8_t fault;
        hal::Status st = readRegister(FAULT_STATUS, &fault);
        if (st != hal::Status::OK) { errorCount_++; return st; }

        data.faultStatus = fault;
        data.faultDetected = (fault & 0xFC) != 0;

        uint8_t rtdBuffer[2];
        st = readRegister(RTD_MSB, rtdBuffer, 2);
        if (st != hal::Status::OK) { errorCount_++; return st; }

        data.rtdCode = static_cast<uint16_t>((rtdBuffer[0] << 8) | rtdBuffer[1]);
        data.rtdCode >>= 1;
        data.resistance = (static_cast<float>(data.rtdCode) / 32768.0f) * config_.rref;
        data.temperature = calculateTemperature(data.resistance);
        data.timestamp = 0;

        return hal::Status::OK;
    }

    hal::Status setFaultThresholds(float highTemp, float lowTemp) {
        if (highTemp <= lowTemp) return hal::Status::INVALID_PARAM;

        float highResistance = temperatureToResistance(highTemp);
        float lowResistance = temperatureToResistance(lowTemp);

        uint16_t highCode = static_cast<uint16_t>((highResistance / config_.rref) * 32768.0f);
        uint16_t lowCode = static_cast<uint16_t>((lowResistance / config_.rref) * 32768.0f);

        hal::Status st = writeRegister(HIGH_FAULT_MSB, (highCode >> 8) & 0xFF);
        if (st != hal::Status::OK) { errorCount_++; return st; }
        st = writeRegister(HIGH_FAULT_LSB, highCode & 0xFF);
        if (st != hal::Status::OK) { errorCount_++; return st; }

        st = writeRegister(LOW_FAULT_MSB, (lowCode >> 8) & 0xFF);
        if (st != hal::Status::OK) { errorCount_++; return st; }
        st = writeRegister(LOW_FAULT_LSB, lowCode & 0xFF);
        if (st != hal::Status::OK) { errorCount_++; return st; }

        return hal::Status::OK;
    }

    hal::Status clearFaultStatus() {
        uint8_t cfg;
        hal::Status st = readRegister(CONFIG_REG, &cfg);
        if (st != hal::Status::OK) { errorCount_++; return st; }
        cfg |= FAULT_STAT_CLEAR;
        st = writeRegister(CONFIG_REG, cfg);
        if (st != hal::Status::OK) errorCount_++;
        return st;
    }

    const Config& getConfig() const { return config_; }
    uint32_t getErrorCount() const { return errorCount_; }
    void resetErrorCounters() { errorCount_ = 0; }

private:
    hal::ISPI& spi_;
    uint8_t csPin_;
    Config config_;
    CallendarCoefficients coeffs_;
    uint32_t errorCount_ = 0;

    hal::Status readRegister(uint8_t reg, uint8_t* data, size_t len = 1) {
        uint8_t cmd = reg & 0x7F;
        std::vector<uint8_t> tx(len + 1, 0), rx(len + 1, 0);
        tx[0] = cmd;
        
        hal::Status st = spi_.transfer(std::span<const uint8_t>(tx.data(), len + 1),
                                       std::span<uint8_t>(rx.data(), len + 1), 100);
        if (st != hal::Status::OK) return st;

        for (size_t i = 0; i < len; ++i) data[i] = rx[i + 1];
        return hal::Status::OK;
    }

    hal::Status writeRegister(uint8_t reg, uint8_t data) {
        uint8_t cmd = reg | 0x80;
        uint8_t txBuffer[2] = {cmd, data};
        uint8_t rxBuffer[2];
        
        return spi_.transfer(std::span<const uint8_t>(txBuffer, 2),
                            std::span<uint8_t>(rxBuffer, 2), 100);
    }

    float calculateTemperature(float resistance) {
        float rRatio = resistance / 100.0f;

        if (resistance >= 100.0f) {
            float a = coeffs_.B;
            float b = coeffs_.A;
            float c = 1.0f - rRatio;

            float discriminant = b * b - 4.0f * a * c;
            if (discriminant < 0.0f) return 0.0f;

            float t = (-b + std::sqrt(discriminant)) / (2.0f * a);
            return t;
        } else {
            float t = (rRatio - 1.0f) / coeffs_.A;
            for (int i = 0; i < 4; ++i) {
                float t2 = t * t;
                float t3 = t2 * t;
                float rCalc = 1.0f + coeffs_.A * t + coeffs_.B * t2 + 
                             coeffs_.C * (t - 100.0f) * t3;
                
                float error = rRatio - rCalc;
                float derivative = coeffs_.A + 2.0f * coeffs_.B * t + 
                                  coeffs_.C * (4.0f * t3 - 300.0f * t2);
                
                if (std::abs(derivative) < 1e-10f) break;
                t += error / derivative;
            }
            return t;
        }
    }

    float temperatureToResistance(float temperature) {
        if (temperature >= 0.0f) {
            float t2 = temperature * temperature;
            return 100.0f * (1.0f + coeffs_.A * temperature + coeffs_.B * t2);
        } else {
            float t2 = temperature * temperature;
            float t3 = t2 * temperature;
            return 100.0f * (1.0f + coeffs_.A * temperature + coeffs_.B * t2 + 
                           coeffs_.C * (temperature - 100.0f) * t3);
        }
    }

    void updateCoefficients(RTDType type) {
        switch (type) {
            case RTDType::PT100:
            case RTDType::PT500:
            case RTDType::PT1000:
                coeffs_ = CallendarCoefficients{3.9083e-3f, -5.775e-7f, -4.183e-12f};
                break;
        }
    }
};

} // namespace sensors
} // namespace mka

#endif // NEW_SENSORS_DRIVERS_HPP
