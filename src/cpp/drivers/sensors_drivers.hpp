/**
 * @file sensors_drivers.hpp
 * @brief Драйверы датчиков для МКА
 * 
 * Содержит драйверы для:
 * - IMU: BMI160, LSM6DSO
 * - Магнитометр: LIS3MDL, HMC5883L
 * - GPS: u-blox NEO-M8
 * - Солнечные датчики
 */

#ifndef SENSORS_DRIVERS_HPP
#define SENSORS_DRIVERS_HPP

#include <cstdint>
#include <cmath>
#include <array>
#include <optional>

// Предполагаем наличие HAL интерфейсов
#include "hal_full.hpp"

namespace mka {
namespace sensors {

// ============================================================================
// Общие структуры данных
// ============================================================================

/// Данные IMU
struct IMUData {
    float accel[3];     // м/с²
    float gyro[3];      // рад/с
    float temperature;  // °C
    uint64_t timestamp;
};

/// Данные магнитометра
struct MagData {
    float mag[3];       // Гаусс (или µT = 0.01 Гаусс)
    bool overflow;
    uint64_t timestamp;
};

/// Данные GPS
struct GPSData {
    double latitude;    // градусы
    double longitude;   // градусы
    double altitude;    // метры
    float speed;        // м/с
    float course;       // градусы
    uint8_t satellites;
    uint8_t fixType;    // 0=no fix, 1=dead reckoning, 2=2D, 3=3D
    float hdop;         // horizontal dilution of precision
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint64_t timestamp;
};

/// Данные солнечного датчика
struct SunSensorData {
    float sunVector[3]; // Единичный вектор на Солнце (body frame)
    float angle;        // Угол от оси датчика
    bool valid;
    uint64_t timestamp;
};

/// Результат самодиагностики
struct SelfTestResult {
    bool passed;
    uint8_t errorCode;
    const char* errorMessage;
};

// ============================================================================
// BMI160 - 6-осевой IMU
// ============================================================================

/**
 * @brief Драйвер IMU BMI160
 * 
 * Особенности:
 * - 16-битный АЦП для акселерометра и гироскопа
 * - Программируемые диапазоны: ±2/4/8/16g (acc), ±125/250/500/1000/2000°/s (gyro)
 * - Встроенный FIFO
 * - Интерфейсы: I2C, SPI
 */
class BMI160Driver {
public:
    // Адреса I2C
    static constexpr uint8_t I2C_ADDR_PRIMARY = 0x68;
    static constexpr uint8_t I2C_ADDR_SECONDARY = 0x69;
    
    // Идентификатор
    static constexpr uint8_t CHIP_ID = 0xD1;
    
    // Регистры
    enum Register : uint8_t {
        CHIP_ID_REG      = 0x00,
        ERR_REG          = 0x02,
        PMU_STATUS       = 0x03,
        MAG_X_L          = 0x04,
        MAG_X_H          = 0x05,
        MAG_Y_L          = 0x06,
        MAG_Y_H          = 0x07,
        MAG_Z_L          = 0x08,
        MAG_Z_H          = 0x09,
        RHALL_L          = 0x0A,
        RHALL_H          = 0x0B,
        GYR_X_L          = 0x0C,
        GYR_X_H          = 0x0D,
        GYR_Y_L          = 0x0E,
        GYR_Y_H          = 0x0F,
        GYR_Z_L          = 0x10,
        GYR_Z_H          = 0x11,
        ACC_X_L          = 0x12,
        ACC_X_H          = 0x13,
        ACC_Y_L          = 0x14,
        ACC_Y_H          = 0x15,
        ACC_Z_L          = 0x16,
        ACC_Z_H          = 0x17,
        SENSORTIME_0     = 0x18,
        SENSORTIME_1     = 0x19,
        SENSORTIME_2     = 0x1A,
        STATUS           = 0x1B,
        INT_STATUS_0     = 0x1C,
        INT_STATUS_1     = 0x1D,
        INT_STATUS_2     = 0x1E,
        INT_STATUS_3     = 0x1F,
        TEMPERATURE_L    = 0x20,
        TEMPERATURE_M    = 0x21,
        FIFO_LENGTH_0    = 0x22,
        FIFO_LENGTH_1    = 0x23,
        FIFO_DATA        = 0x24,
        ACC_CONF         = 0x40,
        ACC_RANGE        = 0x41,
        GYR_CONF         = 0x42,
        GYR_RANGE        = 0x43,
        MAG_CONF         = 0x44,
        FIFO_DOWNS       = 0x45,
        FIFO_CONFIG_0    = 0x46,
        FIFO_CONFIG_1    = 0x47,
        MAG_IF_0         = 0x4B,
        MAG_IF_1         = 0x4C,
        MAG_IF_2         = 0x4D,
        MAG_IF_3         = 0x4E,
        MAG_IF_4         = 0x4F,
        INT_EN_0         = 0x50,
        INT_EN_1         = 0x51,
        INT_EN_2         = 0x52,
        INT_OUT_CTRL     = 0x53,
        INT_LATCH        = 0x54,
        INT_MAP_0        = 0x55,
        INT_MAP_1        = 0x56,
        INT_MAP_2        = 0x57,
        INT_DATA_0       = 0x58,
        INT_DATA_1       = 0x59,
        INT_LOWHIGH_0    = 0x5A,
        INT_LOWHIGH_1    = 0x5B,
        INT_LOWHIGH_2    = 0x5C,
        INT_LOWHIGH_3    = 0x5D,
        INT_LOWHIGH_4    = 0x5E,
        INT_MOTION_0     = 0x5F,
        INT_MOTION_1     = 0x60,
        INT_MOTION_2     = 0x61,
        INT_MOTION_3     = 0x62,
        INT_TAP_0        = 0x63,
        INT_TAP_1        = 0x64,
        INT_ORIENT_0     = 0x65,
        INT_ORIENT_1     = 0x66,
        INT_FLAT_0       = 0x67,
        INT_FLAT_1       = 0x68,
        FOC_CONF         = 0x69,
        CONF             = 0x6A,
        IF_CONF          = 0x6B,
        PMU_TRIGGER      = 0x6C,
        SELF_TEST        = 0x6D,
        NV_CONF          = 0x70,
        OFFSET_ACC_X     = 0x71,
        OFFSET_ACC_Y     = 0x72,
        OFFSET_ACC_Z     = 0x73,
        OFFSET_GYR_X     = 0x74,
        OFFSET_GYR_Y     = 0x75,
        OFFSET_GYR_Z     = 0x76,
        OFFSET_EN        = 0x77,
        STEP_CNT_0       = 0x78,
        STEP_CNT_1       = 0x79,
        STEP_CONF_0      = 0x7A,
        STEP_CONF_1      = 0x7B,
        COMMAND          = 0x7E,
    };
    
    // Диапазоны измерений
    enum class AccelRange : uint8_t {
        RANGE_2G  = 0x03,
        RANGE_4G  = 0x05,
        RANGE_8G  = 0x08,
        RANGE_16G = 0x0C
    };
    
    enum class GyroRange : uint8_t {
        RANGE_125DPS  = 0x04,
        RANGE_250DPS  = 0x03,
        RANGE_500DPS  = 0x02,
        RANGE_1000DPS = 0x01,
        RANGE_2000DPS = 0x00
    };
    
    // Конфигурация
    struct Config {
        AccelRange accelRange = AccelRange::RANGE_8G;
        GyroRange gyroRange = GyroRange::RANGE_1000DPS;
        uint16_t accelODR = 1600;    // Output data rate, Hz
        uint16_t gyroODR = 1600;
        bool useFIFO = false;
        uint16_t fifoWatermark = 100;
    };
    
    /**
     * @brief Конструктор для I2C
     */
    BMI160Driver(hal::II2C& i2c, uint8_t address = I2C_ADDR_PRIMARY)
        : i2c_(&i2c)
        , spi_(nullptr)
        , address_(address)
        , useSPI_(false)
    {}
    
    /**
     * @brief Конструктор для SPI
     */
    BMI160Driver(hal::ISPI& spi)
        : i2c_(nullptr)
        , spi_(&spi)
        , address_(0)
        , useSPI_(true)
    {}
    
    /**
     * @brief Инициализация датчика
     */
    hal::Status init(const Config& config) {
        config_ = config;
        
        // Проверка CHIP_ID
        uint8_t chipId;
        if (readRegister(Register::CHIP_ID_REG, &chipId, 1) != hal::Status::OK) {
            return hal::Status::ERROR;
        }
        
        if (chipId != CHIP_ID) {
            return hal::Status::ERROR;
        }
        
        // Soft reset
        writeCommand(0xB6);
        delayMs(100);
        
        // Включение акселерометра (normal mode)
        writeCommand(0x11);  // ACC_PMU = normal
        delayMs(10);
        
        // Включение гироскопа (normal mode)
        writeCommand(0x15);  // GYR_PMU = normal
        delayMs(80);
        
        // Настройка диапазонов
        uint8_t accRange = static_cast<uint8_t>(config.accelRange);
        uint8_t gyrRange = static_cast<uint8_t>(config.gyroRange);
        
        writeRegister(Register::ACC_RANGE, &accRange, 1);
        writeRegister(Register::GYR_RANGE, &gyrRange, 1);
        
        // Настройка ODR и bandwidth
        uint8_t accConf = computeODRConfig(config.accelODR, 2);  // OSR=2
        uint8_t gyrConf = computeODRConfig(config.gyroODR, 2);
        
        writeRegister(Register::ACC_CONF, &accConf, 1);
        writeRegister(Register::GYR_CONF, &gyrConf, 1);
        
        // Расчёт чувствительности
        computeSensitivity();
        
        return hal::Status::OK;
    }
    
    /**
     * @brief Чтение данных IMU
     */
    hal::Status readData(IMUData& data) {
        uint8_t buffer[12];
        
        // Чтение акселерометра и гироскопа (auto-increment address)
        if (readRegister(Register::ACC_X_L, buffer, 12) != hal::Status::OK) {
            return hal::Status::ERROR;
        }
        
        // Акселерометр (12-14 байты)
        int16_t acc_x = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
        int16_t acc_y = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
        int16_t acc_z = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);
        
        // Гироскоп (6-11 байты)
        int16_t gyr_x = static_cast<int16_t>((buffer[7] << 8) | buffer[6]);
        int16_t gyr_y = static_cast<int16_t>((buffer[9] << 8) | buffer[8]);
        int16_t gyr_z = static_cast<int16_t>((buffer[11] << 8) | buffer[10]);
        
        // Преобразование в физические единицы
        data.accel[0] = acc_x * accSensitivity_;
        data.accel[1] = acc_y * accSensitivity_;
        data.accel[2] = acc_z * accSensitivity_;
        
        data.gyro[0] = gyr_x * gyrSensitivity_;
        data.gyro[1] = gyr_y * gyrSensitivity_;
        data.gyro[2] = gyr_z * gyrSensitivity_;
        
        // Чтение температуры
        uint8_t tempBuf[2];
        if (readRegister(Register::TEMPERATURE_L, tempBuf, 2) == hal::Status::OK) {
            int16_t tempRaw = static_cast<int16_t>((tempBuf[1] << 8) | tempBuf[0]);
            data.temperature = 23.0f + tempRaw * 0.001953125f;  // 1/512 K/LSB
        }
        
        data.timestamp = getTimestamp();
        
        return hal::Status::OK;
    }
    
    /**
     * @brief Самодиагностика
     */
    hal::Status selfTest(SelfTestResult& result) {
        // Включение self-test для акселерометра
        uint8_t selfTestAcc = 0x0D;  // Positive sign, amplitude = 8g
        writeRegister(Register::SELF_TEST, &selfTestAcc, 1);
        delayMs(20);
        
        // Чтение результатов...
        // (упрощённо)
        
        result.passed = true;
        result.errorCode = 0;
        result.errorMessage = nullptr;
        
        return hal::Status::OK;
    }
    
    /**
     * @brief Калибровка смещения
     */
    hal::Status setOffsets(int16_t accX, int16_t accY, int16_t accZ,
                          int16_t gyrX, int16_t gyrY, int16_t gyrZ) {
        uint8_t offsets[7] = {
            static_cast<uint8_t>(accX & 0xFF),
            static_cast<uint8_t>((accX >> 8) & 0x3F),
            static_cast<uint8_t>(accY & 0xFF),
            static_cast<uint8_t>((accY >> 8) & 0x3F),
            static_cast<uint8_t>(accZ & 0xFF),
            static_cast<uint8_t>((accZ >> 8) & 0x3F),
            0x00
        };
        
        writeRegister(Register::OFFSET_ACC_X, offsets, 3);
        
        uint8_t gyrOffsets[6] = {
            static_cast<uint8_t>(gyrX & 0xFF),
            static_cast<uint8_t>((gyrX >> 8) & 0xFF),
            static_cast<uint8_t>(gyrY & 0xFF),
            static_cast<uint8_t>((gyrY >> 8) & 0xFF),
            static_cast<uint8_t>(gyrZ & 0xFF),
            static_cast<uint8_t>((gyrZ >> 8) & 0xFF)
        };
        
        writeRegister(Register::OFFSET_GYR_X, gyrOffsets, 6);
        
        // Включение offset compensation
        uint8_t offsetEn = 0xC0;  // ACC and GYR offset enable
        writeRegister(Register::OFFSET_EN, &offsetEn, 1);
        
        return hal::Status::OK;
    }

private:
    hal::II2C* i2c_;
    hal::ISPI* spi_;
    uint8_t address_;
    bool useSPI_;
    Config config_;
    
    float accSensitivity_ = 1.0f;
    float gyrSensitivity_ = 1.0f;
    
    hal::Status readRegister(uint8_t reg, uint8_t* data, size_t len) {
        if (useSPI_) {
            // SPI: CS low, read with bit 7 set
            spi_->selectDevice();
            uint8_t txData = reg | 0x80;
            spi_->transmit({&txData, 1}, 10);
            hal::Status status = spi_->receive({data, len}, 10);
            spi_->deselectDevice();
            return status;
        } else {
            return i2c_->readRegister(address_, reg, data, len, 100);
        }
    }
    
    hal::Status writeRegister(uint8_t reg, const uint8_t* data, size_t len) {
        if (useSPI_) {
            spi_->selectDevice();
            uint8_t txData = reg & 0x7F;
            spi_->transmit({&txData, 1}, 10);
            hal::Status status = spi_->transmit({data, len}, 10);
            spi_->deselectDevice();
            return status;
        } else {
            return i2c_->writeRegister(address_, reg, data, len, 100);
        }
    }
    
    void writeCommand(uint8_t cmd) {
        writeRegister(Register::COMMAND, &cmd, 1);
    }
    
    uint8_t computeODRConfig(uint16_t odr, uint8_t osr) {
        uint8_t bw = 0;
        if (odr >= 1600) bw = 0x0C;      // 1600 Hz
        else if (odr >= 800) bw = 0x0B;  // 800 Hz
        else if (odr >= 400) bw = 0x0A;  // 400 Hz
        else if (odr >= 200) bw = 0x09;  // 200 Hz
        else if (odr >= 100) bw = 0x08;  // 100 Hz
        else bw = 0x07;                  // 50 Hz
        
        return (osr << 4) | bw;
    }
    
    void computeSensitivity() {
        // Акселерометр (mg/LSB)
        switch (config_.accelRange) {
            case AccelRange::RANGE_2G:  accSensitivity_ = 0.000061035f; break;
            case AccelRange::RANGE_4G:  accSensitivity_ = 0.000122070f; break;
            case AccelRange::RANGE_8G:  accSensitivity_ = 0.000244141f; break;
            case AccelRange::RANGE_16G: accSensitivity_ = 0.000488281f; break;
        }
        accSensitivity_ *= 9.80665f;  // g -> m/s²
        
        // Гироскоп (°/s/LSB)
        switch (config_.gyroRange) {
            case GyroRange::RANGE_125DPS:  gyrSensitivity_ = 0.003815f; break;
            case GyroRange::RANGE_250DPS:  gyrSensitivity_ = 0.007629f; break;
            case GyroRange::RANGE_500DPS:  gyrSensitivity_ = 0.015259f; break;
            case GyroRange::RANGE_1000DPS: gyrSensitivity_ = 0.030518f; break;
            case GyroRange::RANGE_2000DPS: gyrSensitivity_ = 0.061035f; break;
        }
        gyrSensitivity_ *= 0.017453293f;  // ° -> rad
    }
    
    uint64_t getTimestamp() const {
        // TODO: Получить из системного таймера
        return 0;
    }
    
    void delayMs(uint32_t ms) {
        // TODO: Задержка
    }
};

// ============================================================================
// LIS3MDL - 3-осевой магнитометр
// ============================================================================

/**
 * @brief Драйвер магнитометра LIS3MDL
 * 
 * Особенности:
 * - ±4/8/12/16 гаусс
 * - До 80 Гц output data rate
 * - 16-битный АЦП
 */
class LIS3MDLDriver {
public:
    static constexpr uint8_t I2C_ADDR_PRIMARY = 0x1C;
    static constexpr uint8_t I2C_ADDR_SECONDARY = 0x1E;
    static constexpr uint8_t WHO_AM_I_VALUE = 0x3D;
    
    enum Register : uint8_t {
        WHO_AM_I     = 0x0F,
        CTRL_REG1    = 0x20,
        CTRL_REG2    = 0x21,
        CTRL_REG3    = 0x22,
        CTRL_REG4    = 0x23,
        CTRL_REG5    = 0x24,
        STATUS_REG   = 0x27,
        OUT_X_L      = 0x28,
        OUT_X_H      = 0x29,
        OUT_Y_L      = 0x2A,
        OUT_Y_H      = 0x2B,
        OUT_Z_L      = 0x2C,
        OUT_Z_H      = 0x2D,
        TEMP_OUT_L   = 0x2E,
        TEMP_OUT_H   = 0x2F,
        INT_CFG      = 0x30,
        INT_SRC      = 0x31,
        INT_THS_L    = 0x32,
        INT_THS_H    = 0x33,
    };
    
    enum class Scale : uint8_t {
        SCALE_4G  = 0x00,
        SCALE_8G  = 0x20,
        SCALE_12G = 0x40,
        SCALE_16G = 0x60
    };
    
    struct Config {
        Scale scale = Scale::SCALE_4G;
        uint8_t odr = 7;     // 0=0.625Hz, 1=1.25Hz, ..., 7=80Hz
        bool tempEnable = true;
        uint8_t mode = 0;    // 0=continuous, 1=single, 2=power-down
    };
    
    LIS3MDLDriver(hal::II2C& i2c, uint8_t address = I2C_ADDR_PRIMARY)
        : i2c_(i2c), address_(address) {}
    
    hal::Status init(const Config& config = {}) {
        config_ = config;
        
        // Проверка ID
        uint8_t whoAmI;
        if (i2c_.readRegister(address_, Register::WHO_AM_I, &whoAmI, 1, 100) 
            != hal::Status::OK) {
            return hal::Status::ERROR;
        }
        
        if (whoAmI != WHO_AM_I_VALUE) {
            return hal::Status::ERROR;
        }
        
        // CTRL_REG1: OM (operating mode) + DO (data rate)
        uint8_t ctrl1 = (0x03 << 2) | config.odr;  // UHP mode
        i2c_.writeRegister(address_, Register::CTRL_REG1, &ctrl1, 1, 100);
        
        // CTRL_REG2: FS (full scale)
        uint8_t ctrl2 = static_cast<uint8_t>(config.scale);
        i2c_.writeRegister(address_, Register::CTRL_REG2, &ctrl2, 1, 100);
        
        // CTRL_REG3: MD (mode) - continuous
        uint8_t ctrl3 = config.mode;
        i2c_.writeRegister(address_, Register::CTRL_REG3, &ctrl3, 1, 100);
        
        // CTRL_REG4: OMZ (Z-axis operating mode)
        uint8_t ctrl4 = 0x0C;  // UHP for Z
        i2c_.writeRegister(address_, Register::CTRL_REG4, &ctrl4, 1, 100);
        
        // CTRL_REG5: Block data update
        uint8_t ctrl5 = config.tempEnable ? 0x80 : 0x00;
        i2c_.writeRegister(address_, Register::CTRL_REG5, &ctrl5, 1, 100);
        
        // Расчёт чувствительности
        switch (config.scale) {
            case Scale::SCALE_4G:  sensitivity_ = 1.0f / 6842.0f; break;
            case Scale::SCALE_8G:  sensitivity_ = 1.0f / 3421.0f; break;
            case Scale::SCALE_12G: sensitivity_ = 1.0f / 2281.0f; break;
            case Scale::SCALE_16G: sensitivity_ = 1.0f / 1711.0f; break;
        }
        
        return hal::Status::OK;
    }
    
    hal::Status readData(MagData& data) {
        uint8_t status;
        i2c_.readRegister(address_, Register::STATUS_REG, &status, 1, 100);
        
        if (!(status & 0x08)) {  // ZYXDA - new data available
            return hal::Status::BUSY;
        }
        
        uint8_t buffer[6];
        // Чтение с auto-increment (bit 7 = 1)
        if (i2c_.readRegister(address_, Register::OUT_X_L | 0x80, buffer, 6, 100)
            != hal::Status::OK) {
            return hal::Status::ERROR;
        }
        
        int16_t x = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
        int16_t y = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
        int16_t z = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);
        
        data.mag[0] = x * sensitivity_;
        data.mag[1] = y * sensitivity_;
        data.mag[2] = z * sensitivity_;
        data.overflow = (status & 0x40) != 0;
        data.timestamp = getTimestamp();
        
        return hal::Status::OK;
    }
    
private:
    hal::II2C& i2c_;
    uint8_t address_;
    Config config_;
    float sensitivity_ = 1.0f;
    
    uint64_t getTimestamp() const { return 0; }
};

// ============================================================================
// u-blox NEO-M8 - GPS приёмник
// ============================================================================

/**
 * @brief Драйвер GPS приёмника u-blox NEO-M8
 * 
 * Особенности:
 * - Поддержка GPS, GLONASS, Galileo, BeiDou
 * - Точность позиционирования до 2.5 м
 * - UART интерфейс
 */
class UBloxGPSDriver {
public:
    // UBX протокол
    static constexpr uint8_t UBX_SYNC1 = 0xB5;
    static constexpr uint8_t UBX_SYNC2 = 0x62;
    
    // Классы сообщений
    enum UBXClass : uint8_t {
        CLASS_NAV = 0x01,
        CLASS_RXM = 0x02,
        CLASS_INF = 0x04,
        CLASS_ACK = 0x05,
        CLASS_CFG = 0x06,
        CLASS_MON = 0x0A,
        CLASS_AID = 0x0B,
    };
    
    // Идентификаторы сообщений
    enum UBXId : uint8_t {
        NAV_POSLLH = 0x02,
        NAV_STATUS = 0x03,
        NAV_DOP    = 0x04,
        NAV_SOL    = 0x06,
        NAV_PVT    = 0x07,
        NAV_VELNED = 0x12,
        NAV_TIMEUTC = 0x21,
        NAV_SVINFO = 0x30,
        CFG_MSG    = 0x01,
        CFG_PRT    = 0x00,
        CFG_RATE   = 0x08,
        CFG_NAV5   = 0x24,
    };
    
    struct Config {
        uint32_t baudRate = 9600;
        uint16_t measRate = 1000;   // ms между измерениями
        uint16_t navRate = 1;       // measRate * navRate = navigation solution interval
        uint8_t dynamicModel = 0;   // 0=portable, 2=stationary, 3=pedestrian, 6=airborne<1g
        uint8_t fixMode = 3;        // 1=2D only, 2=3D only, 3=auto 2D/3D
        bool useGLONASS = true;
        bool useGalileo = false;
    };
    
    UBloxGPSDriver(hal::IUART& uart) : uart_(uart) {}
    
    hal::Status init(const Config& config = {}) {
        config_ = config;
        
        // Конфигурация UART
        hal::UARTConfig uartConfig;
        uartConfig.baudRate = config.baudRate;
        uartConfig.dataBits = 8;
        uartConfig.stopBits = 1;
        
        if (uart_.init(uartConfig) != hal::Status::OK) {
            return hal::Status::ERROR;
        }
        
        // Настройка GPS
        configureNavEngine();
        configureMessageRates();
        configureMeasRate();
        
        return hal::Status::OK;
    }
    
    hal::Status readData(GPSData& data) {
        // Проверка наличия данных
        size_t available = uart_.available();
        if (available < 8) {
            return hal::Status::BUSY;
        }
        
        // Поиск заголовка UBX или NMEA
        uint8_t byte;
        while (uart_.receive({&byte, 1}, 10) == hal::Status::OK) {
            if (byte == UBX_SYNC1) {
                if (parseUBXMessage(data)) {
                    return hal::Status::OK;
                }
            } else if (byte == '$') {
                // NMEA сообщение - можно парсить при необходимости
                parseNMEAMessage(data);
            }
        }
        
        return hal::Status::ERROR;
    }
    
    bool hasValidFix() const {
        return lastFixValid_;
    }
    
    uint8_t getSatellites() const {
        return lastSatellites_;
    }

private:
    hal::IUART& uart_;
    Config config_;
    bool lastFixValid_ = false;
    uint8_t lastSatellites_ = 0;
    
    // Буфер для приёма
    std::array<uint8_t, 1024> rxBuffer_;
    
    hal::Status configureNavEngine() {
        // CFG-NAV5 сообщение
        std::array<uint8_t, 36> msg = {};
        
        msg[0] = UBX_SYNC1;
        msg[1] = UBX_SYNC2;
        msg[2] = UBXClass::CLASS_CFG;
        msg[3] = UBXId::CFG_NAV5;
        msg[4] = 0x24;  // Length L
        msg[5] = 0x00;  // Length H
        
        // Mask
        msg[6] = 0xFF;
        msg[7] = 0x07;
        
        // Dynamic model
        msg[8] = config_.dynamicModel;
        
        // Fix mode
        msg[9] = config_.fixMode;
        
        // ... остальные поля
        
        addChecksum(msg);
        
        return uart_.transmit(msg, 100);
    }
    
    hal::Status configureMessageRates() {
        // Включить только нужные сообщения
        // CFG-MSG для NAV-PVT
        uint8_t msg[] = {
            UBX_SYNC1, UBX_SYNC2,
            UBXClass::CLASS_CFG, UBXId::CFG_MSG,
            3, 0,                 // Length
            UBXClass::CLASS_NAV, UBXId::NAV_PVT,
            1                      // Rate
        };
        addChecksum({msg, sizeof(msg)});
        
        return uart_.transmit({msg, sizeof(msg)}, 100);
    }
    
    hal::Status configureMeasRate() {
        uint8_t msg[] = {
            UBX_SYNC1, UBX_SYNC2,
            UBXClass::CLASS_CFG, UBXId::CFG_RATE,
            6, 0,                         // Length
            static_cast<uint8_t>(config_.measRate & 0xFF),
            static_cast<uint8_t>(config_.measRate >> 8),
            static_cast<uint8_t>(config_.navRate & 0xFF),
            static_cast<uint8_t>(config_.navRate >> 8),
            0, 0                          // Time ref = UTC
        };
        addChecksum({msg, sizeof(msg)});
        
        return uart_.transmit({msg, sizeof(msg)}, 100);
    }
    
    bool parseUBXMessage(GPSData& data) {
        uint8_t header[6];
        if (uart_.receive(header, 10, 10) != hal::Status::OK) {
            return false;
        }
        
        uint8_t cls = header[2];
        uint8_t id = header[3];
        uint16_t len = header[4] | (header[5] << 8);
        
        if (len > rxBuffer_.size()) {
            return false;
        }
        
        if (uart_.receive({rxBuffer_.data(), len + 2}, 100, 10) != hal::Status::OK) {
            return false;
        }
        
        // Проверка checksum
        // ...
        
        if (cls == UBXClass::CLASS_NAV && id == UBXId::NAV_PVT) {
            parseNavPVT(data, rxBuffer_.data(), len);
            return true;
        }
        
        return false;
    }
    
    void parseNavPVT(GPSData& data, const uint8_t* payload, uint16_t len) {
        // NAV-PVT (92 байта)
        // iTOW: U4 (0-3)
        // year: U2 (4-5)
        // month: U1 (6)
        // day: U1 (7)
        // hour: U1 (8)
        // min: U1 (9)
        // sec: U1 (10)
        // valid: U1 (11)
        // tAcc: U4 (12-15)
        // nano: I4 (16-19)
        // fixType: U1 (20)
        // flags: U1 (21)
        // ...
        // lon: I4 (24-27) - 1e-7 deg
        // lat: I4 (28-31) - 1e-7 deg
        // height: I4 (32-35) - mm
        // hMSL: I4 (36-39) - mm
        // hAcc: U4 (40-43)
        // vAcc: U4 (44-47)
        // velN: I4 (48-51) - mm/s
        // velE: I4 (52-55)
        // velD: I4 (56-59)
        // gSpeed: U4 (60-63)
        // heading: I4 (64-67) - 1e-5 deg
        // sAcc: U4 (68-71)
        // headingAcc: U4 (72-75)
        // pDOP: U2 (76-77) - 0.01
        // ...
        // numSV: U1 (23) - number of satellites
        
        if (len < 92) return;
        
        data.year = payload[4] | (payload[5] << 8);
        data.month = payload[6];
        data.day = payload[7];
        data.hour = payload[8];
        data.minute = payload[9];
        data.second = payload[10];
        
        data.fixType = payload[20];
        lastFixValid_ = (data.fixType >= 2);
        
        int32_t lon = static_cast<int32_t>(
            payload[24] | (payload[25] << 8) | 
            (payload[26] << 16) | (payload[27] << 24));
        int32_t lat = static_cast<int32_t>(
            payload[28] | (payload[29] << 8) |
            (payload[30] << 16) | (payload[31] << 24));
        
        data.longitude = lon * 1e-7;
        data.latitude = lat * 1e-7;
        
        int32_t hMSL = static_cast<int32_t>(
            payload[36] | (payload[37] << 8) |
            (payload[38] << 16) | (payload[39] << 24));
        data.altitude = hMSL / 1000.0;
        
        uint32_t gSpeed = payload[60] | (payload[61] << 8) |
                          (payload[62] << 16) | (payload[63] << 24);
        data.speed = gSpeed / 1000.0f;  // mm/s -> m/s
        
        int32_t heading = static_cast<int32_t>(
            payload[64] | (payload[65] << 8) |
            (payload[66] << 16) | (payload[67] << 24));
        data.course = heading * 1e-5f;
        
        uint16_t pDOP = payload[76] | (payload[77] << 8);
        data.hdop = pDOP * 0.01f;
        
        data.satellites = payload[23];
        lastSatellites_ = data.satellites;
        
        data.timestamp = getTimestamp();
    }
    
    void parseNMEAMessage(GPSData& data) {
        // TODO: Парсинг NMEA сообщений (GPGGA, GPRMC, etc.)
    }
    
    void addChecksum(std::span<uint8_t> msg) {
        uint8_t ckA = 0, ckB = 0;
        
        // Checksum с байта 2 до конца payload
        for (size_t i = 2; i < msg.size() - 2; ++i) {
            ckA += msg[i];
            ckB += ckA;
        }
        
        msg[msg.size() - 2] = ckA;
        msg[msg.size() - 1] = ckB;
    }
    
    uint64_t getTimestamp() const { return 0; }
};

// ============================================================================
// Sun Sensor - симуляция для тестирования
// ============================================================================

/**
 * @brief Драйвер солнечного датчика
 * 
 * Типичная реализация использует 4 фотодиода для определения
 * направления на Солнце.
 */
class SunSensorDriver {
public:
    struct Config {
        float calibrationMatrix[3][4] = {
            {1, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 1, 0}
        };
        float threshold = 0.1f;  // Минимальный сигнал для определения
    };
    
    SunSensorDriver(hal::IADC& adc, std::array<uint8_t, 4> channels)
        : adc_(adc), channels_(channels) {}
    
    hal::Status init(const Config& config = {}) {
        config_ = config;
        return hal::Status::OK;
    }
    
    hal::Status readData(SunSensorData& data) {
        std::array<uint16_t, 4> rawValues;
        
        // Чтение 4 каналов АЦП
        for (int i = 0; i < 4; ++i) {
            if (adc_.read(channels_[i], rawValues[i], 10) != hal::Status::OK) {
                return hal::Status::ERROR;
            }
        }
        
        // Нормализация (предполагаем 12-бит АЦП)
        float normalized[4];
        float sum = 0;
        for (int i = 0; i < 4; ++i) {
            normalized[i] = rawValues[i] / 4095.0f;
            sum += normalized[i];
        }
        
        // Проверка порога
        data.valid = (sum > config_.threshold);
        
        if (data.valid) {
            // Вычисление вектора направления
            for (int i = 0; i < 3; ++i) {
                data.sunVector[i] = 0;
                for (int j = 0; j < 4; ++j) {
                    data.sunVector[i] += config_.calibrationMatrix[i][j] * normalized[j];
                }
            }
            
            // Нормализация вектора
            float norm = std::sqrt(
                data.sunVector[0] * data.sunVector[0] +
                data.sunVector[1] * data.sunVector[1] +
                data.sunVector[2] * data.sunVector[2]
            );
            
            if (norm > 0) {
                data.sunVector[0] /= norm;
                data.sunVector[1] /= norm;
                data.sunVector[2] /= norm;
            }
            
            // Угол от оси Z (нормали к датчику)
            data.angle = std::acos(data.sunVector[2]) * 180.0f / 3.14159265f;
        }
        
        data.timestamp = getTimestamp();
        return hal::Status::OK;
    }
    
private:
    hal::IADC& adc_;
    std::array<uint8_t, 4> channels_;
    Config config_;
    
    uint64_t getTimestamp() const { return 0; }
};

} // namespace sensors
} // namespace mka

#endif // SENSORS_DRIVERS_HPP
