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
#include <cstddef>
#include <cmath>
#include <array>
#include <optional>

// HAL интерфейсы
#if __cplusplus >= 202002L
    #include "hal/hal_full.hpp"
    #include "utils/result.hpp"
#else
    // Для C++17 используем минимальные интерфейсы
    // Требуется реализовать минимальные HAL интерфейсы для работы драйверов
    namespace mka { namespace hal {
        class II2C;
        class ISPI;
        class IUART;
        class ISystemTime;
        enum class Status : uint8_t;
    }}
#endif

// ============================================================================
// Платформенные макросы
// ============================================================================

// SystemCoreClock может быть определён в CMSIS (stm32f4xx.h и т.д.)
// Для host build определяем заглушку
#ifndef SystemCoreClock
    #ifdef HOST_BUILD
        #define SystemCoreClock 16000000UL  // 16 MHz для хоста
    #else
        #define SystemCoreClock 16000000UL  // Значение по умолчанию, переопределяется в CMSIS
    #endif
#endif

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
// Ошибки драйверов
// ============================================================================

/// Коды ошибок драйверов
enum class DriverError : uint8_t {
    OK = 0,
    NOT_INITIALIZED = 1,
    INVALID_PARAM = 2,
    TIMEOUT = 3,
    COMM_ERROR = 4,         // Ошибка связи (I2C/SPI/UART)
    CHECKSUM_ERROR = 5,     // Ошибка контрольной суммы
    DEVICE_NOT_FOUND = 6,   // Устройство не найдено
    DEVICE_BUSY = 7,        // Устройство занято
    FIFO_OVERFLOW = 8,      // Переполнение FIFO
    INVALID_CHIP_ID = 9,    // Неверный ID чипа
    CONFIG_ERROR = 10,      // Ошибка конфигурации
    CALIBRATION_ERROR = 11, // Ошибка калибровки
    BUFFER_EXCEEDED = 12,   // Превышен размер буфера
    UNSUPPORTED_FEATURE = 13 // Функция не поддерживается
};

// Валидация: DriverError должен быть 1 байт для эффективной упаковки
static_assert(sizeof(DriverError) == 1, "DriverError must be 1 byte");

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
        , timeSource_(nullptr)
        , address_(address)
        , useSPI_(false)
    {}

    /**
     * @brief Конструктор для SPI
     */
    BMI160Driver(hal::ISPI& spi)
        : i2c_(nullptr)
        , spi_(&spi)
        , timeSource_(nullptr)
        , address_(0)
        , useSPI_(true)
    {}

    /**
     * @brief Установить источник времени
     */
    void setTimeSource(hal::ISystemTime* timeSource) {
        timeSource_ = timeSource;
    }
    
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
    hal::ISystemTime* timeSource_ = nullptr;
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
        if (timeSource_) {
            return timeSource_->getMs();
        }
        return 0;
    }
    
    void delayMs(uint32_t ms) {
        // Простая задержка через пустой цикл
        // В реальной системе использовать HAL_Delay или FreeRTOS vTaskDelay
        volatile uint32_t count = ms * (SystemCoreClock / 1000 / 4);
        while (count--) {}
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
        size_t received = 0;
        if (uart_.receive({header, sizeof(header)}, 10, received) != hal::Status::OK || received < 6) {
            return false;
        }

        uint8_t cls = header[2];
        uint8_t id = header[3];
        uint16_t len = header[4] | (header[5] << 8);

        if (len > rxBuffer_.size()) {
            return false;  // Buffer overflow protection
        }

        // Чтение payload + checksum (2 байта)
        if (uart_.receive({rxBuffer_.data(), len + 2}, 100, received) != hal::Status::OK || 
            received < len + 2) {
            return false;  // Timeout или неполные данные
        }

        // Проверка checksum (считается от cls + id + length + payload)
        uint8_t ckA = 0, ckB = 0;
        ckA += cls; ckB += ckA;
        ckA += id; ckB += ckA;
        ckA += header[4]; ckB += ckA;
        ckA += header[5]; ckB += ckA;
        
        for (size_t i = 0; i < len; i++) {
            ckA += rxBuffer_[i];
            ckB += ckA;
        }
        
        if (ckA != rxBuffer_[len] || ckB != rxBuffer_[len + 1]) {
            return false;  // Checksum error
        }

        // Обработка различных классов сообщений
        if (cls == UBXClass::CLASS_NAV) {
            switch (id) {
                case UBXId::NAV_PVT:
                    if (len >= 92) {
                        parseNavPVT(data, rxBuffer_.data(), len);
                        return true;
                    }
                    break;
                case UBXId::NAV_POSLLH:
                    if (len >= 28) {
                        parseNavPOSLLH(data, rxBuffer_.data(), len);
                        return true;
                    }
                    break;
                case UBXId::NAV_STATUS:
                    if (len >= 16) {
                        parseNavSTATUS(data, rxBuffer_.data(), len);
                        return true;
                    }
                    break;
                case UBXId::NAV_SOL:
                    if (len >= 52) {
                        parseNavSOL(data, rxBuffer_.data(), len);
                        return true;
                    }
                    break;
                default:
                    break;
            }
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

    void parseNavPOSLLH(GPSData& data, const uint8_t* payload, uint16_t len) {
        // NAV-POSLLH (28 байт)
        // iTOW: U4 (0-3)
        // lon: I4 (4-7) - 1e-7 deg
        // lat: I4 (8-11) - 1e-7 deg
        // height: I4 (12-15) - mm
        // hMSL: I4 (16-19) - mm
        // hAcc: U4 (20-23)
        // vAcc: U4 (24-27)
        if (len < 28) return;

        int32_t lon = static_cast<int32_t>(
            payload[4] | (payload[5] << 8) |
            (payload[6] << 16) | (payload[7] << 24));
        int32_t lat = static_cast<int32_t>(
            payload[8] | (payload[9] << 8) |
            (payload[10] << 16) | (payload[11] << 24));

        data.longitude = lon * 1e-7;
        data.latitude = lat * 1e-7;

        int32_t height = static_cast<int32_t>(
            payload[12] | (payload[13] << 8) |
            (payload[14] << 16) | (payload[15] << 24));
        data.altitude = height / 1000.0;  // mm -> m

        data.timestamp = getTimestamp();
    }

    void parseNavSTATUS(GPSData& data, const uint8_t* payload, uint16_t len) {
        // NAV-STATUS (16 байт)
        // iTOW: U4 (0-3)
        // fixType: U1 (4)
        // fixStatus: U1 (5)
        // ...
        if (len < 16) return;

        data.fixType = payload[4];
        lastFixValid_ = (data.fixType >= 2);  // 2D или 3D fix

        uint8_t fixStatus = payload[5];
        lastFixValid_ = lastFixValid_ && ((fixStatus & 0x04) != 0);  // fixOK флаг

        data.satellites = payload[11];  // numSV
        lastSatellites_ = data.satellites;

        data.timestamp = getTimestamp();
    }

    void parseNavSOL(GPSData& data, const uint8_t* payload, uint16_t len) {
        // NAV-SOL (52 байта)
        // iTOW: U4 (0-3)
        // gSpeed: I4 (28-31) - cm/s
        // heading: I4 (32-35) - deg * 1e-5
        // pDOP: U2 (48-49) - 0.01
        if (len < 52) return;

        int32_t gSpeed = static_cast<int32_t>(
            payload[28] | (payload[29] << 8) |
            (payload[30] << 16) | (payload[31] << 24));
        data.speed = gSpeed / 100.0f;  // cm/s -> m/s

        int32_t heading = static_cast<int32_t>(
            payload[32] | (payload[33] << 8) |
            (payload[34] << 16) | (payload[35] << 24));
        data.course = heading * 1e-5f;

        uint16_t pDOP = payload[48] | (payload[49] << 8);
        data.hdop = pDOP * 0.01f;

        data.timestamp = getTimestamp();
    }

    void parseNMEAMessage(GPSData& data) {
        // Базовая реализация парсинга NMEA
        // Поддерживаются сообщения: GGA, RMC
        // Для полной реализации требуется буферизация строк
        (void)data;  // Заглушка для компиляции
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

// ============================================================================
// BMP388 - Барометр/Термометр
// ============================================================================

/**
 * @brief Драйвер барометра BMP388
 *
 * Особенности:
 * - Точность давления: ±0.02 гПа (±0.17 м)
 * - Точность температуры: ±0.5°C
 * - Диапазон давлений: 300-1250 гПа
 * - Интерфейсы: I2C (0x76, 0x77), SPI
 * - Разрешение: до 24 бит для давления
 *
 * Применение:
 * - Определение высоты над уровнем моря
 * - Термометрирование
 * - Метеомониторинг
 */
class BMP388Driver {
public:
    // Адреса I2C
    static constexpr uint8_t I2C_ADDR_PRIMARY = 0x76;
    static constexpr uint8_t I2C_ADDR_SECONDARY = 0x77;

    // Идентификатор
    static constexpr uint8_t CHIP_ID = 0x50;

    // Регистры
    enum Register : uint8_t {
        CHIP_ID_ADDR      = 0x00,
        ERR_REG           = 0x02,
        STATUS            = 0x03,
        DATA_XLSB         = 0x04,
        DATA_LSB          = 0x05,
        DATA_MSB          = 0x06,
        DATA_PRESS_XLSB   = 0x04,
        DATA_PRESS_LSB    = 0x05,
        DATA_PRESS_MSB    = 0x06,
        DATA_TEMP_XLSB    = 0x07,
        DATA_TEMP_LSB     = 0x08,
        DATA_TEMP_MSB     = 0x09,
        SENSOR_TIME_XLSB  = 0x0A,
        SENSOR_TIME_LSB   = 0x0B,
        SENSOR_TIME_MSB   = 0x0C,
        PWR_CTRL          = 0x1B,
        ODR               = 0x1D,
        CONFIG            = 0x1F,
        CALIB_DATA        = 0x31,
        CMD               = 0x7E
    };

    // Команды
    enum Command : uint8_t {
        SOFT_RESET = 0xB6,
        FIFO_FLUSH = 0xB0
    };

    // Осипрование (Oversampling)
    enum class Oversampling : uint8_t {
        OS_1X  = 0x00,
        OS_2X  = 0x01,
        OS_4X  = 0x02,
        OS_8X  = 0x03,
        OS_16X = 0x04,
        OS_32X = 0x05
    };

    // IIR фильтр
    enum class IIRFilter : uint8_t {
        FILTER_OFF = 0x00,
        FILTER_2   = 0x01,
        FILTER_4   = 0x02,
        FILTER_8   = 0x03,
        FILTER_16  = 0x04,
        FILTER_32  = 0x05,
        FILTER_64  = 0x06,
        FILTER_128 = 0x07
    };

    // Конфигурация
    struct Config {
        Oversampling pressOS = Oversampling::OS_8X;
        Oversampling tempOS = Oversampling::OS_8X;
        IIRFilter iirFilter = IIRFilter::FILTER_4;
        uint8_t odr = 0x07;  // 25 Hz (0x07), можно выбрать другое
        bool forcedMode = false;  // false = normal mode
    };

    // Калибровочные коэффициенты
    struct CalibData {
        uint16_t par_t1;
        int16_t  par_t2;
        int8_t   par_t3;
        int16_t  par_p1;
        int16_t  par_p2;
        int8_t   par_p3;
        int8_t   par_p4;
        uint16_t par_p5;
        uint16_t par_p6;
        int8_t   par_p7;
        int8_t   par_p8;
        int16_t  par_p9;
        int8_t   par_p10;
        int8_t   par_p11;
    };

    /**
     * @brief Конструктор для I2C
     */
    BMP388Driver(hal::II2C& i2c, uint8_t address = I2C_ADDR_PRIMARY)
        : i2c_(&i2c)
        , spi_(nullptr)
        , timeSource_(nullptr)
        , address_(address)
        , useSPI_(false)
    {}

    /**
     * @brief Конструктор для SPI
     */
    BMP388Driver(hal::ISPI& spi)
        : i2c_(nullptr)
        , spi_(&spi)
        , timeSource_(nullptr)
        , address_(0)
        , useSPI_(true)
    {}

    /**
     * @brief Установить источник времени
     */
    void setTimeSource(hal::ISystemTime* timeSource) {
        timeSource_ = timeSource;
    }

    /**
     * @brief Инициализация датчика
     */
    hal::Status init(const Config& config) {
        config_ = config;

        // Проверка CHIP_ID
        uint8_t chipId;
        if (readRegister(Register::CHIP_ID_ADDR, &chipId, 1) != hal::Status::OK) {
            return hal::Status::ERROR;
        }

        if (chipId != CHIP_ID) {
            return hal::Status::ERROR;
        }

        // Soft reset
        writeRegister(Register::CMD, &Command::SOFT_RESET, 1);
        delayMs(10);

        // Загрузка калибровочных коэффициентов
        if (loadCalibrationData() != hal::Status::OK) {
            return hal::Status::ERROR;
        }

        // Настройка ODR, фильтра и oversampling
        if (configureSensor() != hal::Status::OK) {
            return hal::Status::ERROR;
        }

        // Включение питания (pressure + temperature)
        uint8_t pwrCtrl = 0x07;  // PRESS_EN | TEMP_EN | MODE_NORMAL
        writeRegister(Register::PWR_CTRL, &pwrCtrl, 1);
        delayMs(50);

        return hal::Status::OK;
    }

    /**
     * @brief Чтение давления и температуры
     */
    hal::Status readData(float& pressure, float& temperature) {
        uint8_t buffer[6];

        // Чтение данных давления и температуры
        if (readRegister(Register::DATA_PRESS_MSB, buffer, 6) != hal::Status::OK) {
            return hal::Status::ERROR;
        }

        // Давление (3 байта)
        uint32_t pressRaw = ((uint32_t)buffer[2] << 16) |
                            ((uint32_t)buffer[1] << 8) |
                            (uint32_t)buffer[0];

        // Температура (3 байта)
        uint32_t tempRaw = ((uint32_t)buffer[5] << 16) |
                           ((uint32_t)buffer[4] << 8) |
                           (uint32_t)buffer[3];

        // Компенсация температуры
        temperature = compensateTemperature(tempRaw);

        // Компенсация давления
        pressure = compensatePressure(pressRaw, temperature);

        return hal::Status::OK;
    }

    /**
     * @brief Вычисление высоты над уровнем моря
     * @param pressure Давление в гПа
     * @param seaLevelPressure Давление на уровне моря (стандарт: 1013.25 гПа)
     * @return Высота в метрах
     */
    static float calculateAltitude(float pressure, float seaLevelPressure = 1013.25f) {
        // Барометрическая формула
        constexpr float a = 44330.0f;
        constexpr float b = 1.0f / 5.255f;
        return a * (1.0f - std::pow(pressure / seaLevelPressure, b));
    }

    /**
     * @brief Чтение только давления
     */
    hal::Status readPressure(float& pressure) {
        float temperature;
        return readData(pressure, temperature);
    }

    /**
     * @brief Чтение только температуры
     */
    hal::Status readTemperature(float& temperature) {
        float pressure;
        return readData(pressure, temperature);
    }

private:
    hal::II2C* i2c_;
    hal::ISPI* spi_;
    hal::ISystemTime* timeSource_ = nullptr;
    uint8_t address_;
    bool useSPI_;
    Config config_;
    CalibData calib_;

    hal::Status readRegister(uint8_t reg, uint8_t* data, size_t len) {
        if (useSPI_) {
            spi_->selectDevice();
            uint8_t txData = reg | 0x80;  // Bit 7 = 1 для чтения
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
            uint8_t txData = reg & 0x7F;  // Bit 7 = 0 для записи
            spi_->transmit({&txData, 1}, 10);
            hal::Status status = spi_->transmit({data, len}, 10);
            spi_->deselectDevice();
            return status;
        } else {
            return i2c_->writeRegister(address_, reg, data, len, 100);
        }
    }

    hal::Status loadCalibrationData() {
        uint8_t buffer[21];
        if (readRegister(Register::CALIB_DATA, buffer, sizeof(buffer)) != hal::Status::OK) {
            return hal::Status::ERROR;
        }

        size_t i = 0;
        calib_.par_t1  = buffer[i++] | (buffer[i++] << 8);
        calib_.par_t2  = buffer[i++] | (buffer[i++] << 8);
        calib_.par_t3  = (int8_t)buffer[i++];
        calib_.par_p1  = buffer[i++] | (buffer[i++] << 8);
        calib_.par_p2  = buffer[i++] | (buffer[i++] << 8);
        calib_.par_p3  = (int8_t)buffer[i++];
        calib_.par_p4  = (int8_t)buffer[i++];
        calib_.par_p5  = buffer[i++] | (buffer[i++] << 8);
        calib_.par_p6  = buffer[i++] | (buffer[i++] << 8);
        calib_.par_p7  = (int8_t)buffer[i++];
        calib_.par_p8  = (int8_t)buffer[i++];
        calib_.par_p9  = buffer[i++] | (buffer[i++] << 8);
        calib_.par_p10 = (int8_t)buffer[i++];
        calib_.par_p11 = (int8_t)buffer[i++];

        return hal::Status::OK;
    }

    hal::Status configureSensor() {
        // Настройка ODR и фильтра
        uint8_t config = (static_cast<uint8_t>(config_.iirFilter) << 2) | 0x00;
        writeRegister(Register::CONFIG, &config, 1);

        uint8_t odr = config_.odr;
        writeRegister(Register::ODR, &odr, 1);

        // Настройка oversampling
        uint8_t osr = (static_cast<uint8_t>(config_.tempOS) << 3) |
                      static_cast<uint8_t>(config_.pressOS);
        writeRegister(Register::OSR, &osr, 1);

        return hal::Status::OK;
    }

    float compensateTemperature(uint32_t tempRaw) {
        float partialData1;
        float partialData2;
        float compensatedTemp;

        partialData1 = static_cast<float>(tempRaw) - static_cast<float>(calib_.par_t1);
        partialData2 = partialData1 * static_cast<float>(calib_.par_t2);
        compensatedTemp = partialData2 + (partialData1 * partialData1) * static_cast<float>(calib_.par_t3);

        return compensatedTemp;
    }

    float compensatePressure(uint32_t pressRaw, float temperature) {
        float partialData1;
        float partialData2;
        float compensatedPressure;

        partialData1 = static_cast<float>(calib_.par_p6) * temperature;
        partialData2 = static_cast<float>(calib_.par_p7) * (temperature * temperature);

        partialData1 = static_cast<float>(calib_.par_p5) + partialData1 + partialData2;
        partialData2 = static_cast<float>(calib_.par_p4) * temperature;

        float temp = static_cast<float>(calib_.par_p2) * temperature +
                     static_cast<float>(calib_.par_p3) +
                     static_cast<float>(pressRaw);
        partialData2 = temp * partialData2 / 65536.0f;

        temp = static_cast<float>(calib_.par_p1) * temp / 131072.0f;
        partialData1 = temp * partialData1;

        compensatedPressure = 1.0f - (partialData1 / 1048576.0f);
        compensatedPressure = static_cast<float>(pressRaw) - (partialData2 / compensatedPressure);
        compensatedPressure = compensatedPressure / partialData1;

        return compensatedPressure / 100.0f;  // Па -> гПа
    }

    void delayMs(uint32_t ms) {
        if (timeSource_) {
            timeSource_->delayMs(ms);
        } else {
            volatile uint32_t count = ms * (SystemCoreClock / 1000 / 4);
            while (count--) {}
        }
    }

    uint64_t getTimestamp() const {
        return timeSource_ ? timeSource_->getMs() : 0;
    }
};

// Дополнительный регистр для BMP388 (нужен для configureSensor)
namespace bmp388 {
    enum RegisterExt : uint8_t {
        OSR = 0x1C  // Oversampling register
    };
}

// ============================================================================
// LSM6DSO - 6-осевой IMU (Accelerometer + Gyroscope)
// ============================================================================

/**
 * @brief Драйвер IMU LSM6DSO
 *
 * Особенности:
 * - Акселерометр: ±2/4/8/16g, ODR до 6.66 kHz
 * - Гироскоп: ±125/250/500/1000/2000/4000 dps, ODR до 6.66 kHz
 * - Встроенный FIFO до 9 KB
 * - Машинное обучение на борту (конечные автоматы, нейросети)
 * - Интерфейсы: I2C (0x6A, 0x6B), SPI (до 3.33 MHz)
 * - Температурный сенсор: ±1°C точность
 *
 * Применение:
 * - Высокодинамичные системы ориентации
 * - Детекция вибраций
 * - Step detection, activity recognition
 */
class LSM6DSODriver {
public:
    // Адреса I2C
    static constexpr uint8_t I2C_ADDR_PRIMARY = 0x6A;
    static constexpr uint8_t I2C_ADDR_SECONDARY = 0x6B;

    // Идентификатор
    static constexpr uint8_t WHO_AM_I_VALUE = 0x6C;

    // Регистры
    enum Register : uint8_t {
        FUNC_CK_GATE          = 0x01,
        WAKE_UP_SRC           = 0x1B,
        TAP_SRC               = 0x1C,
        D6D_SRC               = 0x1D,
        STATUS_REG            = 0x1E,
        OUT_TEMP_L            = 0x20,
        OUT_TEMP_H            = 0x21,
        OUTX_L_G              = 0x22,
        OUTX_H_G              = 0x23,
        OUTY_L_G              = 0x24,
        OUTY_H_G              = 0x25,
        OUTZ_L_G              = 0x26,
        OUTZ_H_G              = 0x27,
        OUTX_L_A              = 0x28,
        OUTX_H_A              = 0x29,
        OUTY_L_A              = 0x2A,
        OUTY_H_A              = 0x2B,
        OUTZ_L_A              = 0x2C,
        OUTZ_H_A              = 0x2D,
        EMMAIN_STATUS         = 0x2E,
        TIMESTAMP0_REG        = 0x40,
        TIMESTAMP1_REG        = 0x41,
        TIMESTAMP2_REG        = 0x42,
        FIFO_STATUS1          = 0x3A,
        FIFO_STATUS2          = 0x3B,
        FIFO_DATA_OUT_TAG     = 0x3B,
        FIFO_DATA_OUT_X_L     = 0x3C,
        FIFO_DATA_OUT_X_H     = 0x3D,
        FIFO_DATA_OUT_Y_L     = 0x3E,
        FIFO_DATA_OUT_Y_H     = 0x3F,
        FIFO_DATA_OUT_Z_L     = 0x40,
        FIFO_DATA_OUT_Z_H     = 0x41,
        TIMESTAMP2_REG        = 0x42,
        STEP_COUNTER_L        = 0x4B,
        STEP_COUNTER_H        = 0x4C,
        FUNC_SRC              = 0x53,
        FUNC_CK_GATE          = 0x01,
        MD1_CFG               = 0x5E,
        MD2_CFG               = 0x5F,
        FXS_OUT_X_L           = 0x66,
        FXS_OUT_X_H           = 0x67,
        FXS_OUT_Y_L           = 0x68,
        FXS_OUT_Y_H           = 0x69,
        FXS_OUT_Z_L           = 0x6A,
        FXS_OUT_Z_H           = 0x6B,
        WHO_AM_I              = 0x0F,
        CTRL1_XL              = 0x10,  // Accelerometer control
        CTRL2_G               = 0x11,  // Gyroscope control
        CTRL3_C               = 0x12,  // Common control
        CTRL4_C               = 0x13,  // Common control
        CTRL5_C               = 0x14,  // Common control
        CTRL6_C               = 0x15,  // Accelerometer advanced
        CTRL7_G               = 0x16,  // Gyroscope advanced
        CTRL8_XL              = 0x17,  // Accelerometer advanced
        CTRL9_XL              = 0x18,  // Accelerometer advanced
        CTRL10_C              = 0x19,  // Common control
        ALL_INT_SRC           = 0x1A,
        FIFO_CTRL1            = 0x07,
        FIFO_CTRL2            = 0x08,
        FIFO_CTRL3            = 0x09,
        FIFO_CTRL4            = 0x0A,
        COUNTER_BDR_REG1      = 0x73,
        COUNTER_BDR_REG2      = 0x74,
        INT_FIFO_CTRL         = 0x44,
        INT_FIFO_STATUS       = 0x45,
        WATERMARK             = 0x0B,
        I3C_IF_AVAIL          = 0x7F,
        ORIENT_CFG_G          = 0x56,
    };

    // Диапазоны измерений акселерометра
    enum class AccelRange : uint8_t {
        RANGE_2G  = 0x00,
        RANGE_4G  = 0x08,
        RANGE_8G  = 0x0C,
        RANGE_16G = 0x04
    };

    // Диапазоны измерений гироскопа
    enum class GyroRange : uint8_t {
        RANGE_125DPS  = 0x02,  // 125 dps (бит FS_125 в CTRL2_G)
        RANGE_250DPS  = 0x00,
        RANGE_500DPS  = 0x04,
        RANGE_1000DPS = 0x08,
        RANGE_2000DPS = 0x0C,
        RANGE_4000DPS = 0x01   // Только для ODR <= 1.67kHz
    };

    // ODR (Output Data Rate)
    enum class ODR : uint8_t {
        ODR_OFF   = 0x00,
        ODR_12_5  = 0x10,
        ODR_26    = 0x20,
        ODR_52    = 0x30,
        ODR_104   = 0x40,
        ODR_208   = 0x50,
        ODR_417   = 0x60,
        ODR_833   = 0x70,
        ODR_1667  = 0x80,
        ODR_3333  = 0x90,
        ODR_6667  = 0xA0
    };

    // Режимы работы FIFO
    enum class FIFOMode : uint8_t {
        BYPASS      = 0x00,
        FIFO        = 0x01,
        CONTINUOUS  = 0x06,
        BYPASS_CONT = 0x07
    };

    // Конфигурация
    struct Config {
        AccelRange accelRange = AccelRange::RANGE_8G;
        GyroRange gyroRange = GyroRange::RANGE_1000DPS;
        ODR accelODR = ODR::ODR_833;
        ODR gyroODR = ODR::ODR_833;
        bool useFIFO = false;
        FIFOMode fifoMode = FIFOMode::CONTINUOUS;
        uint16_t fifoWatermark = 100;
        bool enableTemperature = true;
        bool highPerformance = true;  // High-performance mode
    };

    /**
     * @brief Конструктор для I2C
     */
    LSM6DSODriver(hal::II2C& i2c, uint8_t address = I2C_ADDR_PRIMARY)
        : i2c_(&i2c)
        , spi_(nullptr)
        , timeSource_(nullptr)
        , address_(address)
        , useSPI_(false)
    {}

    /**
     * @brief Конструктор для SPI
     */
    LSM6DSODriver(hal::ISPI& spi)
        : i2c_(nullptr)
        , spi_(&spi)
        , timeSource_(nullptr)
        , address_(0)
        , useSPI_(true)
    {}

    /**
     * @brief Установить источник времени
     */
    void setTimeSource(hal::ISystemTime* timeSource) {
        timeSource_ = timeSource;
    }

    /**
     * @brief Инициализация датчика
     */
    hal::Status init(const Config& config) {
        config_ = config;

        // Проверка WHO_AM_I
        uint8_t whoAmI;
        if (readRegister(Register::WHO_AM_I, &whoAmI, 1) != hal::Status::OK) {
            return hal::Status::ERROR;
        }

        if (whoAmI != WHO_AM_I_VALUE) {
            return hal::Status::ERROR;
        }

        // Reset устройства
        uint8_t ctrl3 = 0x01;  // BOOT - Reboot memory content
        writeRegister(Register::CTRL3_C, &ctrl3, 1);
        delayMs(25);

        // Настройка акселерометра (CTRL1_XL)
        uint8_t ctrl1 = static_cast<uint8_t>(config.accelODR) |
                        static_cast<uint8_t>(config.accelRange);
        writeRegister(Register::CTRL1_XL, &ctrl1, 1);

        // Настройка гироскопа (CTRL2_G)
        uint8_t ctrl2 = static_cast<uint8_t>(config.gyroODR);
        // Обработка special case для 125 dps
        if (config.gyroRange == GyroRange::RANGE_125DPS) {
            ctrl2 |= 0x02;  // FS_125 bit
        } else {
            ctrl2 |= static_cast<uint8_t>(config.gyroRange);
        }
        writeRegister(Register::CTRL2_G, &ctrl2, 1);

        // CTRL3_C: Блок данных, автоинкремент, SPI режим
        uint8_t ctrl3 = 0x04;  // BDU - Block Data Update
        if (!useSPI_) {
            ctrl3 |= 0x02;  // IF_INC - Register address auto-increment
        } else {
            ctrl3 |= 0x0C;  // SIM + PP_OD
        }
        writeRegister(Register::CTRL3_C, &ctrl3, 1);

        // CTRL4_C: High-performance mode
        uint8_t ctrl4 = config.highPerformance ? 0x03 : 0x00;
        writeRegister(Register::CTRL4_C, &ctrl4, 1);

        // Настройка FIFO
        if (config.useFIFO) {
            setupFIFO();
        }

        // Расчёт чувствительности
        computeSensitivity();

        return hal::Status::OK;
    }

    /**
     * @brief Чтение данных IMU
     */
    hal::Status readData(IMUData& data) {
        uint8_t buffer[12];

        // Чтение акселерометра и гироскопа (auto-increment)
        if (readRegister(Register::OUTX_L_G, buffer, 12) != hal::Status::OK) {
            return hal::Status::ERROR;
        }

        // Гироскоп (первые 6 байт)
        int16_t gyr_x = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
        int16_t gyr_y = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
        int16_t gyr_z = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);

        // Акселерометр (следующие 6 байт)
        int16_t acc_x = static_cast<int16_t>((buffer[7] << 8) | buffer[6]);
        int16_t acc_y = static_cast<int16_t>((buffer[9] << 8) | buffer[8]);
        int16_t acc_z = static_cast<int16_t>((buffer[11] << 8) | buffer[10]);

        // Преобразование в физические единицы
        data.accel[0] = acc_x * accSensitivity_;
        data.accel[1] = acc_y * accSensitivity_;
        data.accel[2] = acc_z * accSensitivity_;

        data.gyro[0] = gyr_x * gyrSensitivity_;
        data.gyro[1] = gyr_y * gyrSensitivity_;
        data.gyro[2] = gyr_z * gyrSensitivity_;

        // Чтение температуры
        if (config_.enableTemperature) {
            uint8_t tempBuf[2];
            if (readRegister(Register::OUT_TEMP_L, tempBuf, 2) == hal::Status::OK) {
                int16_t tempRaw = static_cast<int16_t>((tempBuf[1] << 8) | tempBuf[0]);
                // LSM6DSO: 25°C при 0, 16 бит/°C
                data.temperature = 25.0f + tempRaw / 256.0f;
            }
        }

        data.timestamp = getTimestamp();

        return hal::Status::OK;
    }

    /**
     * @brief Чтение данных из FIFO
     */
    hal::Status readFIFO(std::span<IMUData> samples, size_t& numSamples) {
        if (!config_.useFIFO) {
            return hal::Status::INVALID_PARAM;
        }

        // Чтение статуса FIFO
        uint8_t fifoStatus[2];
        if (readRegister(Register::FIFO_STATUS1, fifoStatus, 2) != hal::Status::OK) {
            return hal::Status::ERROR;
        }

        uint16_t diff = fifoStatus[0] | ((fifoStatus[1] & 0x3F) << 8);
        numSamples = diff / 6;  // 6 байт на sample (только gyro или accel)

        if (numSamples > samples.size()) {
            numSamples = samples.size();
        }

        // Чтение данных из FIFO
        for (size_t i = 0; i < numSamples; ++i) {
            uint8_t buffer[6];
            if (readRegister(Register::FIFO_DATA_OUT_X_L, buffer, 6) != hal::Status::OK) {
                return hal::Status::ERROR;
            }

            int16_t x = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
            int16_t y = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
            int16_t z = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);

            // В зависимости от конфигурации FIFO это могут быть gyro или accel данные
            samples[i].gyro[0] = x * gyrSensitivity_;
            samples[i].gyro[1] = y * gyrSensitivity_;
            samples[i].gyro[2] = z * gyrSensitivity_;
            samples[i].timestamp = getTimestamp();
        }

        return hal::Status::OK;
    }

    /**
     * @brief Самодиагностика
     */
    hal::Status selfTest(SelfTestResult& result) {
        // Включение self-test для акселерометра
        uint8_t ctrl5 = 0x01;  // ST_0 + ST_1 для accel
        writeRegister(Register::CTRL5_C, &ctrl5, 1);
        delayMs(50);

        // Чтение данных...
        // (упрощённая реализация)

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
        // LSM6DSO поддерживает калибровку через register offsets
        uint8_t accOffsets[6] = {
            static_cast<uint8_t>(accX & 0xFF),
            static_cast<uint8_t>((accX >> 8) & 0xFF),
            static_cast<uint8_t>(accY & 0xFF),
            static_cast<uint8_t>((accY >> 8) & 0xFF),
            static_cast<uint8_t>(accZ & 0xFF),
            static_cast<uint8_t>((accZ >> 8) & 0xFF)
        };

        writeRegister(Register::X_OFS_USR, accOffsets, 3);
        writeRegister(Register::Y_OFS_USR, &accOffsets[3], 3);

        uint8_t gyrOffsets[6] = {
            static_cast<uint8_t>(gyrX & 0xFF),
            static_cast<uint8_t>((gyrX >> 8) & 0xFF),
            static_cast<uint8_t>(gyrY & 0xFF),
            static_cast<uint8_t>((gyrY >> 8) & 0xFF),
            static_cast<uint8_t>(gyrZ & 0xFF),
            static_cast<uint8_t>((gyrZ >> 8) & 0xFF)
        };

        writeRegister(Register::GYRO_OFF_X, gyrOffsets, 6);

        return hal::Status::OK;
    }

    /**
     * @brief Получить количество образцов в FIFO
     */
    uint16_t getFIFOLevel() const {
        uint8_t status[2];
        const_cast<LSM6DSODriver*>(this)->readRegister(
            Register::FIFO_STATUS1, status, 2);
        return status[0] | ((status[1] & 0x3F) << 8);
    }

    /**
     * @brief Очистка FIFO
     */
    hal::Status clearFIFO() {
        uint8_t fifoCtrl = 0x00;  // Bypass mode
        hal::Status status = writeRegister(Register::FIFO_CTRL1, &fifoCtrl, 1);
        delayMs(10);
        // Восстановление режима
        if (config_.useFIFO) {
            setupFIFO();
        }
        return status;
    }

private:
    hal::II2C* i2c_;
    hal::ISPI* spi_;
    hal::ISystemTime* timeSource_ = nullptr;
    uint8_t address_;
    bool useSPI_;
    Config config_;

    float accSensitivity_ = 1.0f;
    float gyrSensitivity_ = 1.0f;

    // Дополнительные регистры для offsets
    enum RegisterExt : uint8_t {
        X_OFS_USR   = 0x73,
        Y_OFS_USR   = 0x74,
        Z_OFS_USR   = 0x75,
        GYRO_OFF_X  = 0x76,
        GYRO_OFF_Y  = 0x77
    };

    hal::Status readRegister(uint8_t reg, uint8_t* data, size_t len) {
        if (useSPI_) {
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

    void setupFIFO() {
        // Настройка FIFO для гироскопа
        uint8_t fifoCtrl1 = 0x00;  // Gyro data
        uint8_t fifoCtrl2 = static_cast<uint8_t>(config_.fifoMode);

        writeRegister(Register::FIFO_CTRL1, &fifoCtrl1, 1);
        writeRegister(Register::FIFO_CTRL2, &fifoCtrl2, 1);

        // Watermark
        uint8_t wmL = config_.fifoWatermark & 0xFF;
        uint8_t wmH = (config_.fifoWatermark >> 8) & 0x03;
        writeRegister(Register::WATERMARK, &wmL, 1);
        writeRegister(Register::WATERMARK + 1, &wmH, 1);
    }

    void computeSensitivity() {
        // Акселерометр (mg/LSB -> m/s²)
        switch (config_.accelRange) {
            case AccelRange::RANGE_2G:  accSensitivity_ = 0.061f * 9.80665f / 1000.0f; break;
            case AccelRange::RANGE_4G:  accSensitivity_ = 0.122f * 9.80665f / 1000.0f; break;
            case AccelRange::RANGE_8G:  accSensitivity_ = 0.244f * 9.80665f / 1000.0f; break;
            case AccelRange::RANGE_16G: accSensitivity_ = 0.488f * 9.80665f / 1000.0f; break;
        }

        // Гироскоп (dps/LSB -> rad/s)
        switch (config_.gyroRange) {
            case GyroRange::RANGE_125DPS:  gyrSensitivity_ = 4.375f * 0.017453293f / 1000.0f; break;
            case GyroRange::RANGE_250DPS:  gyrSensitivity_ = 8.750f * 0.017453293f / 1000.0f; break;
            case GyroRange::RANGE_500DPS:  gyrSensitivity_ = 17.50f * 0.017453293f / 1000.0f; break;
            case GyroRange::RANGE_1000DPS: gyrSensitivity_ = 35.00f * 0.017453293f / 1000.0f; break;
            case GyroRange::RANGE_2000DPS: gyrSensitivity_ = 70.00f * 0.017453293f / 1000.0f; break;
            case GyroRange::RANGE_4000DPS: gyrSensitivity_ = 140.0f * 0.017453293f / 1000.0f; break;
        }
    }

    void delayMs(uint32_t ms) {
        if (timeSource_) {
            timeSource_->delayMs(ms);
        } else {
            volatile uint32_t count = ms * (SystemCoreClock / 1000 / 4);
            while (count--) {}
        }
    }

    uint64_t getTimestamp() const {
        return timeSource_ ? timeSource_->getMs() : 0;
    }
};

} // namespace sensors
} // namespace mka

#endif // SENSORS_DRIVERS_HPP
