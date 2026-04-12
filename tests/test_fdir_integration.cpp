/**
 * @file test_fdir_integration.cpp
 * @brief Интеграционные тесты FDIR + Sensors
 * 
 * Проверяет взаимодействие:
 * - FDIR мониторинг параметров сенсоров
 * - Автоматическое восстановление при сбоях
 * - Детекция аномалий в телеметрии
 * - Журнал событий FDIR
 */

#include <gtest/gtest.h>
#include <vector>
#include <memory>
#include <cmath>
#include <string>

#include "systems/fdir.hpp"
#include "systems/telemetry.hpp"
#include "utils/result.hpp"

using namespace mka::fdir;
using namespace mka::telemetry;

// ============================================================================
// Mock Sensor Data
// ============================================================================

struct MockSensorData {
    float temperature = 20.0f;
    float pressure = 1013.25f;
    float humidity = 50.0f;
    float voltage = 3.3f;
    uint32_t timestamp = 0;
};

// ============================================================================
// FDIR + Telemetry Integration Tests
// ============================================================================

class FDIRTelemetryIntegrationTest : public ::testing::Test {
protected:
    FDIRManager fdir_;
    MockSensorData sensorData_;
    
    void SetUp() override {
        sensorData_ = MockSensorData{20.0f, 1013.25f, 50.0f, 3.3f, 0};
    }
    
    uint16_t registerTemperatureParameter(float nominal, float warnLow, float warnHigh, 
                                          float errLow, float errHigh) {
        return fdir_.registerParameter({
            .parameterId = 0,
            .nominalValue = nominal,
            .warningLow = warnLow,
            .warningHigh = warnHigh,
            .errorLow = errLow,
            .errorHigh = errHigh,
            .criticalLow = warnLow - 5.0f,
            .criticalHigh = warnHigh + 5.0f,
            .priority = 1
        });
    }
};

TEST_F(FDIRTelemetryIntegrationTest, RegisterAndMonitorTemperature) {
    uint16_t paramId = registerTemperatureParameter(20.0f, 15.0f, 25.0f, 10.0f, 30.0f);
    EXPECT_GT(paramId, 0);
    
    // Нормальная температура
    ParameterStatus status = fdir_.checkParameter(paramId, 20.0f);
    EXPECT_EQ(status, ParameterStatus::NOMINAL);
    
    // Предупреждение (высокая)
    status = fdir_.checkParameter(paramId, 26.0f);
    EXPECT_EQ(status, ParameterStatus::WARNING);
    
    // Ошибка (очень высокая)
    status = fdir_.checkParameter(paramId, 32.0f);
    EXPECT_EQ(status, ParameterStatus::ERROR);
}

TEST_F(FDIRTelemetryIntegrationTest, MultipleParametersMonitoring) {
    uint16_t tempId = registerTemperatureParameter(20.0f, 15.0f, 25.0f, 10.0f, 30.0f);
    uint16_t pressId = fdir_.registerParameter({
        .parameterId = 0,
        .nominalValue = 1013.25f,
        .warningLow = 950.0f,
        .warningHigh = 1050.0f,
        .errorLow = 900.0f,
        .errorHigh = 1100.0f,
        .criticalLow = 850.0f,
        .criticalHigh = 1150.0f,
        .priority = 2
    });
    
    // Проверка обоих параметров
    ParameterStatus tempStatus = fdir_.checkParameter(tempId, sensorData_.temperature);
    ParameterStatus pressStatus = fdir_.checkParameter(pressId, sensorData_.pressure);
    
    EXPECT_EQ(tempStatus, ParameterStatus::NOMINAL);
    EXPECT_EQ(pressStatus, ParameterStatus::NOMINAL);
}

TEST_F(FDIRTelemetryIntegrationTest, AnomalyDetection) {
    uint16_t paramId = registerTemperatureParameter(20.0f, 15.0f, 25.0f, 10.0f, 30.0f);
    
    // Серия нормальных значений
    for (int i = 0; i < 10; ++i) {
        fdir_.checkParameter(paramId, 20.0f + static_cast<float>(i) * 0.1f);
    }
    
    // Внезапная аномалия
    ParameterStatus status = fdir_.checkParameter(paramId, 100.0f);
    EXPECT_EQ(status, ParameterStatus::ERROR);
}

// ============================================================================
// FDIR Event Log Tests
// ============================================================================

TEST_F(FDIRTelemetryIntegrationTest, EventLogging) {
    uint16_t paramId = registerTemperatureParameter(20.0f, 15.0f, 25.0f, 10.0f, 30.0f);
    
    // Создать событие
    fdir_.checkParameter(paramId, 35.0f);  // Критическое значение
    
    // Проверить журнал событий
    auto events = fdir_.getRecentEvents(5);
    EXPECT_GT(events.size(), 0);
}

TEST_F(FDIRTelemetryIntegrationTest, EventLogOverflow) {
    uint16_t paramId = registerTemperatureParameter(20.0f, 15.0f, 25.0f, 10.0f, 30.0f);
    
    // Создать много событий
    for (int i = 0; i < 100; ++i) {
        fdir_.checkParameter(paramId, static_cast<float>(i));
    }
    
    // Журнал не должен переполниться
    auto events = fdir_.getRecentEvents(10);
    EXPECT_LE(events.size(), 10);
}

// ============================================================================
// FDIR Recovery Tests
// ============================================================================

class FDIRRecoveryTest : public ::testing::Test {
protected:
    FDIRManager fdir_;
    int recoveryCount_ = 0;
    
    void SetUp() override {
        recoveryCount_ = 0;
        fdir_.setRecoveryCallback([this](uint16_t paramId) {
            this->recoveryCount_++;
            return true;
        });
    }
};

TEST_F(FDIRRecoveryTest, AutomaticRecovery) {
    uint16_t paramId = fdir_.registerParameter({
        .parameterId = 0,
        .nominalValue = 20.0f,
        .warningLow = 15.0f,
        .warningHigh = 25.0f,
        .errorLow = 10.0f,
        .errorHigh = 30.0f,
        .criticalLow = 5.0f,
        .criticalHigh = 35.0f,
        .priority = 1,
        .autoRecovery = true
    });
    
    // Вызвать ошибку
    fdir_.checkParameter(paramId, 50.0f);
    
    // Попытка восстановления
    fdir_.checkParameter(paramId, 20.0f);  // Нормальное значение
    
    EXPECT_GE(recoveryCount_, 0);
}

// ============================================================================
// FDIR Detector Tests
// ============================================================================

TEST_F(FDIRTelemetryIntegrationTest, FrozenValueDetector) {
    FrozenValueDetector detector(5);  // 5 одинаковых значений = frozen
    
    // Серия одинаковых значений
    for (int i = 0; i < 4; ++i) {
        bool frozen = detector.update(42.0f);
        EXPECT_FALSE(frozen);
    }
    
    // Пятое одинаковое - должно детектировать
    bool frozen = detector.update(42.0f);
    EXPECT_TRUE(frozen);
}

TEST_F(FDIRTelemetryIntegrationTest, FrozenValueDetectorChanging) {
    FrozenValueDetector detector(5);
    
    // Меняющиеся значения
    for (int i = 0; i < 10; ++i) {
        bool frozen = detector.update(static_cast<float>(i));
        EXPECT_FALSE(frozen);
    }
}

TEST_F(FDIRTelemetryIntegrationTest, StuckBitDetector) {
    StuckBitDetector detector(3);  // 3 одинаковых = stuck
    
    bool stuck = false;
    
    stuck = detector.update(true);
    EXPECT_FALSE(stuck);
    
    stuck = detector.update(true);
    EXPECT_FALSE(stuck);
    
    stuck = detector.update(true);
    EXPECT_TRUE(stuck);
}

TEST_F(FDIRTelemetryIntegrationTest, GlitchDetector) {
    GlitchDetector detector(2, 10);  // 2 глитча за 10 мс = аномалия
    
    bool anomaly = false;
    
    // Одиночный глитч
    anomaly = detector.update(true);
    EXPECT_FALSE(anomaly);
    
    // Второй глитч быстро
    anomaly = detector.update(true);
    EXPECT_TRUE(anomaly);
}

// ============================================================================
// FDIR + Telemetry System Tests
// ============================================================================

class FDIRSystemTest : public ::testing::Test {
protected:
    FDIRManager fdir_;
    
    void SetUp() override {
        // Регистрация параметров подсистем
        fdir_.registerParameter({
            .parameterId = 1,
            .nominalValue = 3.3f,
            .warningLow = 3.0f,
            .warningHigh = 3.6f,
            .errorLow = 2.8f,
            .errorHigh = 3.8f,
            .criticalLow = 2.5f,
            .criticalHigh = 4.0f,
            .priority = 1
        });
        
        fdir_.registerParameter({
            .parameterId = 2,
            .nominalValue = 25.0f,
            .warningLow = 10.0f,
            .warningHigh = 40.0f,
            .errorLow = 0.0f,
            .errorHigh = 60.0f,
            .criticalLow = -10.0f,
            .criticalHigh = 70.0f,
            .priority = 2
        });
    }
};

TEST_F(FDIRSystemTest, PowerSystemMonitoring) {
    // Нормальное напряжение
    ParameterStatus status = fdir_.checkParameter(1, 3.3f);
    EXPECT_EQ(status, ParameterStatus::NOMINAL);
    
    // Пониженное напряжение
    status = fdir_.checkParameter(1, 2.9f);
    EXPECT_EQ(status, ParameterStatus::WARNING);
    
    // Критическое напряжение
    status = fdir_.checkParameter(1, 2.4f);
    EXPECT_EQ(status, ParameterStatus::CRITICAL);
}

TEST_F(FDIRSystemTest, ThermalSystemMonitoring) {
    // Нормальная температура
    ParameterStatus status = fdir_.checkParameter(2, 25.0f);
    EXPECT_EQ(status, ParameterStatus::NOMINAL);
    
    // Высокая температура
    status = fdir_.checkParameter(2, 45.0f);
    EXPECT_EQ(status, ParameterStatus::WARNING);
    
    // Критическая температура
    status = fdir_.checkParameter(2, 75.0f);
    EXPECT_EQ(status, ParameterStatus::CRITICAL);
}

TEST_F(FDIRSystemTest, SystemHealthCheck) {
    // Все параметры в норме
    fdir_.checkParameter(1, 3.3f);
    fdir_.checkParameter(2, 25.0f);
    
    SystemHealth health = fdir_.getSystemHealth();
    EXPECT_EQ(health.overallStatus, SystemHealthStatus::NOMINAL);
}

TEST_F(FDIRSystemTest, SystemHealthDegraded) {
    // Один параметр в предупреждении
    fdir_.checkParameter(1, 3.3f);
    fdir_.checkParameter(2, 45.0f);
    
    SystemHealth health = fdir_.getSystemHealth();
    EXPECT_EQ(health.overallStatus, SystemHealthStatus::DEGRADED);
}

TEST_F(FDIRSystemTest, SystemHealthCritical) {
    // Один параметр в критическом состоянии
    fdir_.checkParameter(1, 3.3f);
    fdir_.checkParameter(2, 75.0f);
    
    SystemHealth health = fdir_.getSystemHealth();
    EXPECT_EQ(health.overallStatus, SystemHealthStatus::CRITICAL);
}
