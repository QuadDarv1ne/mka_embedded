/**
 * @file sun_sensor.hpp
 * @brief Sun Sensor Driver for MKA
 * 
 * Драйверы солнечных датчиков:
 * - Аналоговый солнечный датчик (4 фотодиода + ADC)
 * - Цифровой солнечный датчик (I2C/SPI)
 */

#ifndef SUN_SENSOR_HPP
#define SUN_SENSOR_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>
#include <array>

#include "../utils/span.hpp"

namespace mka {
namespace sensors {

// ============================================================================
// Результат измерения солнечного датчика
// ============================================================================

struct SunVector {
    float x;            // Компонента X
    float y;            // Компонента Y
    float z;            // Компонента Z (обычно 1.0 к солнцу)
    float intensity;    // Интенсивность (0-1)
    bool valid;         // Валидность измерения
    float angleX;       // Угол от оси X (радианы)
    float angleY;       // Угол от оси Y (радианы)
};

// ============================================================================
// Интерфейсы
// ============================================================================

class IADC {
public:
    virtual ~IADC() = default;
    virtual bool readChannel(uint8_t channel, uint16_t& value) = 0;
};

class II2C {
public:
    virtual ~II2C() = default;
    virtual bool writeRegister(uint8_t devAddr, uint8_t reg, 
                               std::span<const uint8_t> data) = 0;
    virtual bool readRegister(uint8_t devAddr, uint8_t reg,
                              std::span<uint8_t> data) = 0;
};

// ============================================================================
// Аналоговый солнечный датчик
// ============================================================================

/**
 * @brief Аналоговый солнечный датчик на 4 фотодиодах
 * 
 * Конструкция:
 * - 4 фотодиода расположены по 4 направлениям (+X, -X, +Y, -Y)
 * - Каждый фотодиод наклонён под углом к плоскости
 * - Выходы подключены к ADC
 * 
 * Преимущества:
 * - Простота конструкции
 * - Низкая стоимость
 * - Высокая надёжность
 * 
 * Недостатки:
 * - Ограниченная точность
 * - Зависимость от температуры
 */
class AnalogSunSensor {
public:
    // Каналы ADC
    enum Channel : uint8_t {
        CH_POS_X = 0,   // +X фотодиод
        CH_NEG_X = 1,   // -X фотодиод
        CH_POS_Y = 2,   // +Y фотодиод
        CH_NEG_Y = 3,   // -Y фотодиод
        CH_COUNT = 4
    };
    
    // Калибровочные коэффициенты
    struct Calibration {
        float scale[CH_COUNT];          // Масштабные коэффициенты
        float offset[CH_COUNT];         // Смещения
        float crossCoupling[2][2];      // Матрица перекрёстных связей
        float tanTheta;                 // tan(угол наклона фотодиодов)
        float threshold;                // Порог детектирования
    };
    
    AnalogSunSensor(IADC& adc, const Calibration& cal)
        : adc_(adc), calibration_(cal) {}
    
    /**
     * @brief Чтение данных сенсора
     */
    bool read(SunVector& result) {
        // Чтение всех каналов ADC
        std::array<uint16_t, CH_COUNT> rawValues{};
        
        for (int i = 0; i < CH_COUNT; i++) {
            if (!adc_.readChannel(i, rawValues[i])) {
                result.valid = false;
                return false;
            }
        }
        
        // Применение калибровки
        float calValues[CH_COUNT];
        for (int i = 0; i < CH_COUNT; i++) {
            calValues[i] = (rawValues[i] - calibration_.offset[i]) * 
                           calibration_.scale[i];
        }
        
        // Расчёт вектора направления
        // dX = (Pos_X - Neg_X) / (Pos_X + Neg_X)
        // dY = (Pos_Y - Neg_Y) / (Pos_Y + Neg_Y)
        
        float sumX = calValues[CH_POS_X] + calValues[CH_NEG_X];
        float sumY = calValues[CH_POS_Y] + calValues[CH_NEG_Y];
        
        // Проверка наличия солнца
        float totalIntensity = sumX + sumY;
        if (totalIntensity < calibration_.threshold * 4.0f) {
            result.valid = false;
            result.intensity = 0.0f;
            return true;  // Солнце не в поле зрения
        }
        
        // Нормализованные разности
        float dX = (calValues[CH_POS_X] - calValues[CH_NEG_X]) / sumX;
        float dY = (calValues[CH_POS_Y] - calValues[CH_NEG_Y]) / sumY;
        
        // Коррекция перекрёстных связей
        float dX_corr = calibration_.crossCoupling[0][0] * dX + 
                        calibration_.crossCoupling[0][1] * dY;
        float dY_corr = calibration_.crossCoupling[1][0] * dX + 
                        calibration_.crossCoupling[1][1] * dY;
        
        // Расчёт углов
        result.angleX = std::atan(dX_corr / calibration_.tanTheta);
        result.angleY = std::atan(dY_corr / calibration_.tanTheta);
        
        // Расчёт вектора направления
        float tanX = std::tan(result.angleX);
        float tanY = std::tan(result.angleY);
        
        // Нормализация вектора
        float norm = std::sqrt(1.0f + tanX * tanX + tanY * tanY);
        
        result.x = tanX / norm;
        result.y = tanY / norm;
        result.z = 1.0f / norm;
        
        // Интенсивность
        result.intensity = totalIntensity / (4.0f * 4095.0f);  // Normalize to 0-1
        result.intensity = std::min(1.0f, result.intensity);
        
        result.valid = true;
        return true;
    }
    
    /**
     * @brief Получение "сырых" значений
     */
    bool readRaw(std::array<uint16_t, CH_COUNT>& values) {
        for (int i = 0; i < CH_COUNT; i++) {
            if (!adc_.readChannel(i, values[i])) {
                return false;
            }
        }
        return true;
    }
    
    /**
     * @brief Калибровка по эталонному вектору
     */
    void calibrate(const SunVector& reference, 
                   const std::array<uint16_t, CH_COUNT>& rawValues) {
        // Вычисление новых коэффициентов по эталону
        // Упрощённый алгоритм
        float refAngleX = std::atan2(reference.x, reference.z);
        float refAngleY = std::atan2(reference.y, reference.z);
        
        // Обновление калибровки (итеративно)
        // В реальной системе требуется множественные измерения
    }
    
    /**
     * @brief Установка калибровочных данных
     */
    void setCalibration(const Calibration& cal) {
        calibration_ = cal;
    }
    
    /**
     * @brief Создание калибровки по умолчанию
     */
    static Calibration defaultCalibration() {
        Calibration cal{};
        for (int i = 0; i < CH_COUNT; i++) {
            cal.scale[i] = 1.0f / 4095.0f;
            cal.offset[i] = 0.0f;
        }
        cal.crossCoupling[0][0] = 1.0f;
        cal.crossCoupling[0][1] = 0.0f;
        cal.crossCoupling[1][0] = 0.0f;
        cal.crossCoupling[1][1] = 1.0f;
        cal.tanTheta = 1.0f;  // 45° angle
        cal.threshold = 0.1f;
        return cal;
    }
    
private:
    IADC& adc_;
    Calibration calibration_;
};

// ============================================================================
// Цифровой солнечный датчик (CSS811-like)
// ============================================================================

/**
 * @brief Цифровой солнечный датчик с I2C интерфейсом
 * 
 * Характеристики:
 * - Разрешение: 14 бит
 * - Поле зрения: ±60°
 * - Точность: <0.5°
 * - Интерфейс: I2C (up to 400 kHz)
 */
class DigitalSunSensor {
public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x30;
    
    // Регистры
    enum Register : uint8_t {
        REG_WHO_AM_I    = 0x00,
        REG_STATUS      = 0x01,
        REG_X_MSB       = 0x02,
        REG_X_LSB       = 0x03,
        REG_Y_MSB       = 0x04,
        REG_Y_LSB       = 0x05,
        REG_INTENSITY   = 0x06,
        REG_TEMPERATURE = 0x07,
        REG_CONTROL     = 0x10,
        REG_CONFIG      = 0x11
    };
    
    // Статус
    struct Status {
        bool dataReady;
        bool overrange;
        bool underrange;
        bool error;
    };
    
    DigitalSunSensor(II2C& i2c, uint8_t address = DEFAULT_ADDRESS)
        : i2c_(i2c), address_(address) {}
    
    /**
     * @brief Инициализация датчика
     */
    bool init() {
        // Проверка WHO_AM_I
        uint8_t whoAmI;
        if (!i2c_.readRegister(address_, REG_WHO_AM_I, {&whoAmI, 1})) {
            return false;
        }
        
        if (whoAmI != 0x55) {  // Ожидаемое значение
            return false;
        }
        
        // Конфигурация
        uint8_t config = 0x01;  // Continuous mode
        if (!i2c_.writeRegister(address_, REG_CONFIG, {&config, 1})) {
            return false;
        }
        
        initialized_ = true;
        return true;
    }
    
    /**
     * @brief Проверка статуса
     */
    Status getStatus() {
        uint8_t status;
        Status s{};
        
        if (i2c_.readRegister(address_, REG_STATUS, {&status, 1})) {
            s.dataReady = (status & 0x01) != 0;
            s.overrange = (status & 0x02) != 0;
            s.underrange = (status & 0x04) != 0;
            s.error = (status & 0x80) != 0;
        }
        
        return s;
    }
    
    /**
     * @brief Чтение данных
     */
    bool read(SunVector& result) {
        if (!initialized_) {
            result.valid = false;
            return false;
        }
        
        // Проверка готовности данных
        Status status = getStatus();
        if (!status.dataReady) {
            result.valid = false;
            return true;
        }
        
        // Чтение данных
        uint8_t data[6];
        if (!i2c_.readRegister(address_, REG_X_MSB, data)) {
            result.valid = false;
            return false;
        }
        
        // Парсинг
        int16_t rawX = (data[0] << 8) | data[1];
        int16_t rawY = (data[2] << 8) | data[3];
        uint16_t rawIntensity = (data[4] << 8) | data[5];
        
        // Конвертация
        result.angleX = rawX * (3.14159f / 32768.0f);  // ±π range
        result.angleY = rawY * (3.14159f / 32768.0f);
        result.intensity = rawIntensity / 65535.0f;
        
        // Вычисление вектора
        float tanX = std::tan(result.angleX);
        float tanY = std::tan(result.angleY);
        float norm = std::sqrt(1.0f + tanX * tanX + tanY * tanY);
        
        result.x = tanX / norm;
        result.y = tanY / norm;
        result.z = 1.0f / norm;
        
        result.valid = !status.overrange && !status.error;
        
        return result.valid;
    }
    
    /**
     * @brief Чтение температуры датчика
     */
    bool readTemperature(float& temp) {
        uint8_t data[2];
        if (!i2c_.readRegister(address_, REG_TEMPERATURE, data)) {
            return false;
        }
        
        int16_t rawTemp = (data[0] << 8) | data[1];
        temp = rawTemp * 0.1f;  // 0.1°C per LSB
        return true;
    }
    
private:
    II2C& i2c_;
    uint8_t address_;
    bool initialized_ = false;
};

// ============================================================================
// Sun Sensor Array (несколько датчиков)
// ============================================================================

/**
 * @brief Массив солнечных датчиков для полного покрытия
 * 
 * Типичная конфигурация CubeSat:
 * - 6 датчиков (по одному на грань)
 * - Поле зрения каждого: ±60°
 * - Перекрытие между датчиками
 */
template<size_t NumSensors>
class SunSensorArray {
public:
    struct SensorReading {
        SunVector vector;
        uint8_t sensorId;
        float weight;  // Вес измерения (по интенсивности)
    };
    
    SunSensorArray(std::array<DigitalSunSensor*, NumSensors> sensors)
        : sensors_(sensors) {}
    
    /**
     * @brief Чтение всех датчиков
     */
    size_t readAll(std::array<SensorReading, NumSensors>& readings) {
        size_t validCount = 0;
        
        for (size_t i = 0; i < NumSensors; i++) {
            readings[validCount].sensorId = i;
            
            if (sensors_[i] && sensors_[i]->read(readings[validCount].vector)) {
                if (readings[validCount].vector.valid) {
                    // Вес пропорционален интенсивности
                    readings[validCount].weight = readings[validCount].vector.intensity;
                    validCount++;
                }
            }
        }
        
        return validCount;
    }
    
    /**
     * @brief Получение взвешенного вектора солнца
     */
    bool getWeightedVector(SunVector& result) {
        std::array<SensorReading, NumSensors> readings;
        size_t count = readAll(readings);
        
        if (count == 0) {
            result.valid = false;
            return false;
        }
        
        // Взвешенное усреднение
        float sumX = 0, sumY = 0, sumZ = 0, sumW = 0;
        
        for (size_t i = 0; i < count; i++) {
            float w = readings[i].weight;
            sumX += readings[i].vector.x * w;
            sumY += readings[i].vector.y * w;
            sumZ += readings[i].vector.z * w;
            sumW += w;
        }
        
        // Нормализация
        float norm = std::sqrt(sumX * sumX + sumY * sumY + sumZ * sumZ);
        
        result.x = sumX / norm;
        result.y = sumY / norm;
        result.z = sumZ / norm;
        result.intensity = sumW / count;
        result.valid = true;
        
        return true;
    }
    
private:
    std::array<DigitalSunSensor*, NumSensors> sensors_;
};

} // namespace sensors
} // namespace mka

#endif // SUN_SENSOR_HPP
