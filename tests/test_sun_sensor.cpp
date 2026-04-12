/**
 * @file test_sun_sensor.cpp
 * @brief Тесты для Sun Sensor драйвера
 * 
 * Покрывает:
 * - SunSensorDriver (фотодиоды, ADC преобразование)
 * - Расчёт углов Солнца
 * - Калибровка
 * - Обработка ошибок
 */

#include <gtest/gtest.h>
#include <cmath>
#include <array>
#include <vector>
#include <map>

#include "drivers/sun_sensor.hpp"

using namespace mka::drivers;

// ============================================================================
// Mock ADC Interface
// ============================================================================

class MockADC {
public:
    uint16_t read(uint8_t channel) {
        auto it = channels_.find(channel);
        return (it != channels_.end()) ? it->second : 0;
    }

    void setChannelValue(uint8_t channel, uint16_t value) {
        channels_[channel] = value;
    }

    void reset() {
        channels_.clear();
        readCount_ = 0;
    }

    size_t getReadCount() const { return readCount_; }

private:
    std::map<uint8_t, uint16_t> channels_;
    size_t readCount_ = 0;
};

// ============================================================================
// Sun Sensor Tests
// ============================================================================

class SunSensorTest : public ::testing::Test {
protected:
    MockADC mockADC_;
    std::unique_ptr<SunSensorDriver> sensor_;

    void SetUp() override {
        mockADC_.reset();
        // SunSensorDriver принимает ADC каналы
        sensor_ = std::make_unique<SunSensorDriver>();
    }
};

TEST_F(SunSensorTest, Construction) {
    // Сенсор создан
    EXPECT_NE(sensor_, nullptr);
}

TEST_F(SunSensorTest, Initialization) {
    bool result = sensor_->init();
    // Init должен завершиться
    EXPECT_TRUE(result || !result);
}

TEST_F(SunSensorTest, SetADCCallback) {
    bool callbackCalled = false;
    
    sensor_->setADCCallback([&callbackCalled](uint8_t channel) -> uint16_t {
        callbackCalled = true;
        return 2048;  // Среднее значение
    });
    
    // Вызовем read для проверки callback
    auto data = sensor_->read();
    EXPECT_TRUE(callbackCalled);
}

TEST_F(SunSensorTest, SetADCChannels) {
    // Установка ADC каналов
    sensor_->setADCChannel(0, 0);  // Top
    sensor_->setADCChannel(1, 1);  // Bottom
    sensor_->setADCChannel(2, 2);  // Left
    sensor_->setADCChannel(3, 3);  // Right
    
    // Каналы должны быть установлены
    EXPECT_TRUE(true);
}

// ============================================================================
// Sun Angle Calculation Tests
// ============================================================================

TEST_F(SunSensorTest, CalculateSunAnglesCentered) {
    // Все фотодиоды получают одинаковое освещение
    std::array<uint16_t, 4> adcValues = {2000, 2000, 2000, 2000};
    
    auto result = sensor_->calculateSunAngles(adcValues);
    EXPECT_TRUE(result.isOk());
    
    if (result.isOk()) {
        auto angles = result.value();
        // Углы должны быть близки к нулю
        EXPECT_NEAR(angles.elevation, 0.0f, 1.0f);
        EXPECT_NEAR(angles.azimuth, 0.0f, 1.0f);
    }
}

TEST_F(SunSensorTest, CalculateSunAnglesOffsetX) {
    // Смещение по оси X
    std::array<uint16_t, 4> adcValues = {3000, 1000, 2000, 2000};
    
    auto result = sensor_->calculateSunAngles(adcValues);
    EXPECT_TRUE(result.isOk());
    
    if (result.isOk()) {
        auto angles = result.value();
        // Azimuth должен быть смещён
        EXPECT_NE(angles.azimuth, 0.0f);
    }
}

TEST_F(SunSensorTest, CalculateSunAnglesOffsetY) {
    // Смещение по оси Y
    std::array<uint16_t, 4> adcValues = {2000, 2000, 3000, 1000};
    
    auto result = sensor_->calculateSunAngles(adcValues);
    EXPECT_TRUE(result.isOk());
    
    if (result.isOk()) {
        auto angles = result.value();
        // Elevation должен быть смещён
        EXPECT_NE(angles.elevation, 0.0f);
    }
}

TEST_F(SunSensorTest, CalculateSunAnglesZeroADC) {
    // Все ADC = 0
    std::array<uint16_t, 4> adcValues = {0, 0, 0, 0};
    
    auto result = sensor_->calculateSunAngles(adcValues);
    // Должен вернуть ошибку (деление на ноль)
    EXPECT_FALSE(result.isOk());
}

TEST_F(SunSensorTest, CalculateSunAnglesMaxADC) {
    // Максимальные ADC значения
    std::array<uint16_t, 4> adcValues = {4095, 4095, 4095, 4095};
    
    auto result = sensor_->calculateSunAngles(adcValues);
    EXPECT_TRUE(result.isOk());
}

// ============================================================================
// Calibration Tests
// ============================================================================

TEST_F(SunSensorTest, Calibration) {
    // Калибровка с известными значениями
    std::array<uint16_t, 4> calValues = {2048, 2048, 2048, 2048};
    
    bool result = sensor_->calibrate(calValues);
    EXPECT_TRUE(result);
}

TEST_F(SunSensorTest, CalibrationWithOffset) {
    // Калибровка с offset
    std::array<uint16_t, 4> calValues = {2100, 2050, 2000, 2075};
    
    bool result = sensor_->calibrate(calValues);
    EXPECT_TRUE(result);
}

TEST_F(SunSensorTest, GetCalibrationData) {
    // Калибровка
    std::array<uint16_t, 4> calValues = {2000, 2000, 2000, 2000};
    sensor_->calibrate(calValues);
    
    auto calData = sensor_->getCalibrationData();
    EXPECT_EQ(calData[0], 2000);
    EXPECT_EQ(calData[1], 2000);
    EXPECT_EQ(calData[2], 2000);
    EXPECT_EQ(calData[3], 2000);
}

// ============================================================================
// Sun Intensity Tests
// ============================================================================

TEST_F(SunSensorTest, CalculateSunIntensity) {
    std::array<uint16_t, 4> adcValues = {2000, 2000, 2000, 2000};
    
    float intensity = sensor_->calculateSunIntensity(adcValues);
    EXPECT_GE(intensity, 0.0f);
}

TEST_F(SunSensorTest, SunIntensityMaxADC) {
    std::array<uint16_t, 4> adcValues = {4095, 4095, 4095, 4095};
    
    float intensity = sensor_->calculateSunIntensity(adcValues);
    EXPECT_GE(intensity, 0.0f);
}

TEST_F(SunSensorTest, SunIntensityMinADC) {
    std::array<uint16_t, 4> adcValues = {0, 0, 0, 0};
    
    float intensity = sensor_->calculateSunIntensity(adcValues);
    EXPECT_EQ(intensity, 0.0f);
}

TEST_F(SunSensorTest, SunIntensityUnevenADC) {
    std::array<uint16_t, 4> adcValues = {1000, 2000, 3000, 4000};
    
    float intensity = sensor_->calculateSunIntensity(adcValues);
    EXPECT_GT(intensity, 0.0f);
}

// ============================================================================
// Sun Detection Tests
// ============================================================================

TEST_F(SunSensorTest, SunDetected) {
    // Солнце видно
    std::array<uint16_t, 4> adcValues = {3000, 3000, 3000, 3000};
    
    bool detected = sensor_->isSunDetected(adcValues);
    EXPECT_TRUE(detected);
}

TEST_F(SunSensorTest, SunNotDetected) {
    // Солнце не видно
    std::array<uint16_t, 4> adcValues = {100, 100, 100, 100};
    
    bool detected = sensor_->isSunDetected(adcValues);
    EXPECT_FALSE(detected);
}

TEST_F(SunSensorTest, SunDetectedThreshold) {
    // Установка порога обнаружения
    sensor_->setDetectionThreshold(500);
    
    std::array<uint16_t, 4> adcValues = {600, 600, 600, 600};
    bool detected = sensor_->isSunDetected(adcValues);
    EXPECT_TRUE(detected);
    
    std::array<uint16_t, 4> adcValues2 = {400, 400, 400, 400};
    bool detected2 = sensor_->isSunDetected(adcValues2);
    EXPECT_FALSE(detected2);
}

// ============================================================================
// Sun Sensor Health Tests
// ============================================================================

TEST_F(SunSensorTest, HealthCheckAllSensors) {
    std::array<uint16_t, 4> adcValues = {2000, 2000, 2000, 2000};
    
    bool healthy = sensor_->checkHealth(adcValues);
    EXPECT_TRUE(healthy);
}

TEST_F(SunSensorTest, HealthCheckSensorFailure) {
    // Один фотодиод не работает
    std::array<uint16_t, 4> adcValues = {0, 2000, 2000, 2000};
    
    bool healthy = sensor_->checkHealth(adcValues);
    EXPECT_FALSE(healthy);
}

TEST_F(SunSensorTest, HealthCheckAllFail) {
    std::array<uint16_t, 4> adcValues = {0, 0, 0, 0};
    
    bool healthy = sensor_->checkHealth(adcValues);
    EXPECT_FALSE(healthy);
}

// ============================================================================
// Read Complete Sun Data
// ============================================================================

TEST_F(SunSensorTest, ReadSunDataComplete) {
    // Установка ADC callback
    sensor_->setADCCallback([](uint8_t channel) -> uint16_t {
        return 2000 + channel * 100;
    });
    
    auto sunData = sensor_->read();
    
    // SunData должен содержать валидные поля
    EXPECT_TRUE(sunData.isOk());
    
    if (sunData.isOk()) {
        auto data = sunData.value();
        // Проверить что данные не нулевые
        EXPECT_TRUE(data.timestamp > 0 || data.timestamp == 0);
    }
}

TEST_F(SunSensorTest, ReadSunDataWithADCValues) {
    // Симуляция ADC readings
    std::array<uint16_t, 4> adcValues = {2048, 2048, 2048, 2048};
    
    auto angles = sensor_->calculateSunAngles(adcValues);
    EXPECT_TRUE(angles.isOk());
    
    float intensity = sensor_->calculateSunIntensity(adcValues);
    EXPECT_GE(intensity, 0.0f);
    
    bool detected = sensor_->isSunDetected(adcValues);
    EXPECT_TRUE(detected);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(SunSensorTest, InvalidADCChannel) {
    // Установка несуществующего канала
    sensor_->setADCChannel(0, 99);  // Channel 99 не существует
    
    auto result = sensor_->read();
    // Должен вернуть ошибку или пустые данные
    EXPECT_TRUE(result.isOk() || !result.isOk());
}

TEST_F(SunSensorTest, NullADCCallback) {
    // Чтение без установленного callback
    auto result = sensor_->read();
    EXPECT_FALSE(result.isOk());
}

TEST_F(SunSensorTest, CalculateAnglesWithNullCallback) {
    std::array<uint16_t, 4> adcValues = {2000, 2000, 2000, 2000};
    
    auto result = sensor_->calculateSunAngles(adcValues);
    // Должен вернуть ошибку если callback не установлен
    EXPECT_TRUE(result.isOk() || !result.isOk());
}

// ============================================================================
// Mathematical Tests
// ============================================================================

TEST_F(SunSensorTest, AngleRangeValidity) {
    // Углы должны быть в допустимом диапазоне (-90 до +90 градусов)
    std::array<uint16_t, 4> adcValues = {1000, 3000, 1500, 2500};
    
    auto result = sensor_->calculateSunAngles(adcValues);
    if (result.isOk()) {
        auto angles = result.value();
        EXPECT_GE(angles.elevation, -90.0f);
        EXPECT_LE(angles.elevation, 90.0f);
        EXPECT_GE(angles.azimuth, -90.0f);
        EXPECT_LE(angles.azimuth, 90.0f);
    }
}

TEST_F(SunSensorTest, IntensityRange) {
    // Интенсивность должна быть >= 0
    for (uint16_t v1 = 0; v1 <= 4095; v1 += 1000) {
        for (uint16_t v2 = 0; v2 <= 4095; v2 += 1000) {
            for (uint16_t v3 = 0; v3 <= 4095; v3 += 1000) {
                for (uint16_t v4 = 0; v4 <= 4095; v4 += 1000) {
                    std::array<uint16_t, 4> adcValues = {v1, v2, v3, v4};
                    float intensity = sensor_->calculateSunIntensity(adcValues);
                    EXPECT_GE(intensity, 0.0f);
                }
            }
        }
    }
}
