/**
 * @file test_health_monitor.cpp
 * @brief Unit tests для health monitoring + FDIR integration
 */

#include <gtest/gtest.h>
#include <cstdint>
#include <string>

#include "systems/health_monitor.hpp"
#include "systems/fdir.hpp"

using namespace mka::systems;
using namespace mka::fdir;

// ============================================================================
// Mock драйвер для тестирования
// ============================================================================

class MockDriver : public IHealthMonitorDriver {
public:
    MockDriver(const std::string& name) : name_(name) {}
    
    uint32_t getErrorCount() const override { return errorCount_; }
    uint32_t getTimeoutCount() const override { return timeoutCount_; }
    void resetErrorCounters() override { 
        errorCount_ = 0; 
        timeoutCount_ = 0; 
    }
    const char* getDriverName() const override { return name_.c_str(); }
    bool isHealthy() const override { return errorCount_ < criticalThreshold_; }
    bool recover() override { 
        resetErrorCounters(); 
        recovered_ = true;
        return true; 
    }
    
    void addErrors(uint32_t count) { errorCount_ += count; }
    void addTimeouts(uint32_t count) { timeoutCount_ += count; }
    bool wasRecovered() const { return recovered_; }
    
    void setHealthy(bool healthy) { 
        healthy_ = healthy; 
        criticalThreshold_ = healthy ? 1000 : 0;
    }
    
private:
    std::string name_;
    mutable uint32_t errorCount_ = 0;
    mutable uint32_t timeoutCount_ = 0;
    mutable bool recovered_ = false;
    mutable bool healthy_ = true;
    mutable uint32_t criticalThreshold_ = 1000;
};

// ============================================================================
// Тесты инициализации
// ============================================================================

TEST(HealthMonitorTest, Initialization) {
    HealthMonitorManager healthMgr;
    FDIRManager fdirMgr;
    
    // Без FDIR менеджера
    EXPECT_FALSE(healthMgr.init(nullptr));
    
    // С FDIR менеджером
    EXPECT_TRUE(healthMgr.init(&fdirMgr));
    EXPECT_EQ(healthMgr.getDriverCount(), 0);
}

TEST(HealthMonitorTest, RegisterDriver) {
    HealthMonitorManager healthMgr;
    FDIRManager fdirMgr;
    MockDriver driver("TestDriver");
    
    healthMgr.init(&fdirMgr);
    
    DriverHealthConfig config = {
        .driverName = "TestDriver",
        .subsystemId = static_cast<uint8_t>(Subsystem::ADCS),
        .driverId = 0,
        .warningThreshold = 5,
        .errorThreshold = 10,
        .criticalThreshold = 20,
        .warningAction = RecoveryAction::LOG_ONLY,
        .errorAction = RecoveryAction::RESET_SENSOR,
        .criticalAction = RecoveryAction::SAFE_MODE,
        .checkIntervalMs = 1000
    };
    
    EXPECT_TRUE(healthMgr.registerDriver(&driver, config));
    EXPECT_EQ(healthMgr.getDriverCount(), 1);
}

TEST(HealthMonitorTest, RegisterMaxDrivers) {
    HealthMonitorManager healthMgr;
    FDIRManager fdirMgr;
    
    healthMgr.init(&fdirMgr);
    
    // Регистрируем MAX_DRIVERS
    for (size_t i = 0; i < HealthMonitorManager::MAX_DRIVERS; i++) {
        auto* driver = new MockDriver("Driver" + std::to_string(i));
        DriverHealthConfig config = {
            .driverName = "Driver",
            .subsystemId = 0,
            .driverId = static_cast<uint8_t>(i),
            .warningThreshold = 5,
            .errorThreshold = 10,
            .criticalThreshold = 20,
            .warningAction = RecoveryAction::LOG_ONLY,
            .errorAction = RecoveryAction::RESET_SENSOR,
            .criticalAction = RecoveryAction::SAFE_MODE,
            .checkIntervalMs = 1000
        };
        EXPECT_TRUE(healthMgr.registerDriver(driver, config));
    }
    
    // Попытка зарегистрировать ещё один
    MockDriver extraDriver("ExtraDriver");
    DriverHealthConfig config = {
        .driverName = "ExtraDriver",
        .subsystemId = 0,
        .driverId = 99,
        .warningThreshold = 5,
        .errorThreshold = 10,
        .criticalThreshold = 20,
        .warningAction = RecoveryAction::LOG_ONLY,
        .errorAction = RecoveryAction::RESET_SENSOR,
        .criticalAction = RecoveryAction::SAFE_MODE,
        .checkIntervalMs = 1000
    };
    EXPECT_FALSE(healthMgr.registerDriver(&extraDriver, config));
}

// ============================================================================
// Тесты мониторинга здоровья
// ============================================================================

TEST(HealthMonitorTest, NoErrorsNoEvent) {
    HealthMonitorManager healthMgr;
    FDIRManager fdirMgr;
    MockDriver driver("TestDriver");
    
    healthMgr.init(&fdirMgr);
    
    DriverHealthConfig config = {
        .driverName = "TestDriver",
        .subsystemId = static_cast<uint8_t>(Subsystem::ADCS),
        .driverId = 0,
        .warningThreshold = 5,
        .errorThreshold = 10,
        .criticalThreshold = 20,
        .warningAction = RecoveryAction::LOG_ONLY,
        .errorAction = RecoveryAction::RESET_SENSOR,
        .criticalAction = RecoveryAction::SAFE_MODE,
        .checkIntervalMs = 1000
    };
    
    healthMgr.registerDriver(&driver, config);
    
    // Проверяем без ошибок - severity должен быть INFO
    healthMgr.checkDriver(0, 1000);
    EXPECT_EQ(healthMgr.getDriverSeverity(0), Severity::INFO);
}

TEST(HealthMonitorTest, WarningThreshold) {
    HealthMonitorManager healthMgr;
    FDIRManager fdirMgr;
    MockDriver driver("TestDriver");
    
    healthMgr.init(&fdirMgr);
    
    DriverHealthConfig config = {
        .driverName = "TestDriver",
        .subsystemId = static_cast<uint8_t>(Subsystem::ADCS),
        .driverId = 0,
        .warningThreshold = 5,
        .errorThreshold = 10,
        .criticalThreshold = 20,
        .warningAction = RecoveryAction::LOG_ONLY,
        .errorAction = RecoveryAction::RESET_SENSOR,
        .criticalAction = RecoveryAction::SAFE_MODE,
        .checkIntervalMs = 1000
    };
    
    healthMgr.registerDriver(&driver, config);
    
    // Добавляем 5 ошибок (warning threshold)
    driver.addErrors(5);
    
    // Проверяем - должно создать WARNING событие
    healthMgr.checkDriver(0, 1000);
    EXPECT_EQ(healthMgr.getDriverSeverity(0), Severity::WARNING);
}

TEST(HealthMonitorTest, ErrorThreshold) {
    HealthMonitorManager healthMgr;
    FDIRManager fdirMgr;
    MockDriver driver("TestDriver");
    
    healthMgr.init(&fdirMgr);
    
    DriverHealthConfig config = {
        .driverName = "TestDriver",
        .subsystemId = static_cast<uint8_t>(Subsystem::ADCS),
        .driverId = 0,
        .warningThreshold = 5,
        .errorThreshold = 10,
        .criticalThreshold = 20,
        .warningAction = RecoveryAction::LOG_ONLY,
        .errorAction = RecoveryAction::RESET_SENSOR,
        .criticalAction = RecoveryAction::SAFE_MODE,
        .checkIntervalMs = 1000
    };
    
    healthMgr.registerDriver(&driver, config);
    
    // Добавляем 10 ошибок (error threshold)
    driver.addErrors(10);
    
    healthMgr.checkDriver(0, 1000);
    EXPECT_EQ(healthMgr.getDriverSeverity(0), Severity::ERROR);
    EXPECT_TRUE(driver.wasRecovered());  // errorAction = RESET_SENSOR
}

TEST(HealthMonitorTest, CriticalThreshold) {
    HealthMonitorManager healthMgr;
    FDIRManager fdirMgr;
    MockDriver driver("TestDriver");
    
    healthMgr.init(&fdirMgr);
    
    DriverHealthConfig config = {
        .driverName = "TestDriver",
        .subsystemId = static_cast<uint8_t>(Subsystem::ADCS),
        .driverId = 0,
        .warningThreshold = 5,
        .errorThreshold = 10,
        .criticalThreshold = 20,
        .warningAction = RecoveryAction::LOG_ONLY,
        .errorAction = RecoveryAction::RESET_SENSOR,
        .criticalAction = RecoveryAction::SAFE_MODE,
        .checkIntervalMs = 1000
    };
    
    healthMgr.registerDriver(&driver, config);
    
    // Добавляем 20 ошибок (critical threshold)
    driver.addErrors(20);
    
    healthMgr.checkDriver(0, 1000);
    EXPECT_EQ(healthMgr.getDriverSeverity(0), Severity::CRITICAL);
    EXPECT_TRUE(driver.wasRecovered());  // criticalAction = SAFE_MODE
}

// ============================================================================
// Тесты статистики
// ============================================================================

TEST(HealthMonitorTest, HealthStatsAllHealthy) {
    HealthMonitorManager healthMgr;
    FDIRManager fdirMgr;
    MockDriver driver1("Driver1");
    MockDriver driver2("Driver2");
    
    healthMgr.init(&fdirMgr);
    
    DriverHealthConfig config = {
        .driverName = "Driver",
        .subsystemId = 0,
        .driverId = 0,
        .warningThreshold = 5,
        .errorThreshold = 10,
        .criticalThreshold = 20,
        .warningAction = RecoveryAction::LOG_ONLY,
        .errorAction = RecoveryAction::RESET_SENSOR,
        .criticalAction = RecoveryAction::SAFE_MODE,
        .checkIntervalMs = 1000
    };
    
    healthMgr.registerDriver(&driver1, config);
    healthMgr.registerDriver(&driver2, config);
    
    uint8_t healthy, warning, error, critical;
    healthMgr.getHealthStats(healthy, warning, error, critical);
    
    EXPECT_EQ(healthy, 2);
    EXPECT_EQ(warning, 0);
    EXPECT_EQ(error, 0);
    EXPECT_EQ(critical, 0);
}

TEST(HealthMonitorTest, HealthStatsMixed) {
    HealthMonitorManager healthMgr;
    FDIRManager fdirMgr;
    MockDriver driver1("Driver1");
    MockDriver driver2("Driver2");
    MockDriver driver3("Driver3");
    
    healthMgr.init(&fdirMgr);
    
    DriverHealthConfig config = {
        .driverName = "Driver",
        .subsystemId = 0,
        .driverId = 0,
        .warningThreshold = 5,
        .errorThreshold = 10,
        .criticalThreshold = 20,
        .warningAction = RecoveryAction::LOG_ONLY,
        .errorAction = RecoveryAction::RESET_SENSOR,
        .criticalAction = RecoveryAction::SAFE_MODE,
        .checkIntervalMs = 1000
    };
    
    healthMgr.registerDriver(&driver1, config);
    healthMgr.registerDriver(&driver2, config);
    healthMgr.registerDriver(&driver3, config);
    
    // Driver2 - warning
    driver2.addErrors(5);
    healthMgr.checkDriver(1, 1000);
    
    // Driver3 - error
    driver3.addErrors(10);
    healthMgr.checkDriver(2, 2000);
    
    uint8_t healthy, warning, error, critical;
    healthMgr.getHealthStats(healthy, warning, error, critical);
    
    EXPECT_EQ(healthy, 1);   // Driver1
    EXPECT_EQ(warning, 1);   // Driver2
    EXPECT_EQ(error, 1);     // Driver3
    EXPECT_EQ(critical, 0);
}

// ============================================================================
// Тесты check interval
// ============================================================================

TEST(HealthMonitorTest, CheckInterval) {
    HealthMonitorManager healthMgr;
    FDIRManager fdirMgr;
    MockDriver driver("TestDriver");
    
    healthMgr.init(&fdirMgr);
    
    DriverHealthConfig config = {
        .driverName = "TestDriver",
        .subsystemId = static_cast<uint8_t>(Subsystem::ADCS),
        .driverId = 0,
        .warningThreshold = 5,
        .errorThreshold = 10,
        .criticalThreshold = 20,
        .warningAction = RecoveryAction::LOG_ONLY,
        .errorAction = RecoveryAction::RESET_SENSOR,
        .criticalAction = RecoveryAction::SAFE_MODE,
        .checkIntervalMs = 1000
    };
    
    healthMgr.registerDriver(&driver, config);
    
    // Добавляем 5 ошибок
    driver.addErrors(5);
    
    // Первая проверка - должен сработать warning
    healthMgr.checkDriver(0, 1000);
    EXPECT_EQ(healthMgr.getDriverSeverity(0), Severity::WARNING);
    
    // Сбрасываем recovered
    driver.addErrors(10);  // Теперь 15 ошибок
    
    // Вторая проверка слишком рано (интервал 1000мс, прошло 500мс)
    healthMgr.checkDriver(0, 1500);
    // Severity не должен измениться, т.к. проверка не выполнена
    EXPECT_EQ(healthMgr.getDriverSeverity(0), Severity::WARNING);
}

// ============================================================================
// Тесты сброса
// ============================================================================

TEST(HealthMonitorTest, ResetAllDrivers) {
    HealthMonitorManager healthMgr;
    FDIRManager fdirMgr;
    MockDriver driver("TestDriver");
    
    healthMgr.init(&fdirMgr);
    
    DriverHealthConfig config = {
        .driverName = "TestDriver",
        .subsystemId = static_cast<uint8_t>(Subsystem::ADCS),
        .driverId = 0,
        .warningThreshold = 5,
        .errorThreshold = 10,
        .criticalThreshold = 20,
        .warningAction = RecoveryAction::LOG_ONLY,
        .errorAction = RecoveryAction::RESET_SENSOR,
        .criticalAction = RecoveryAction::SAFE_MODE,
        .checkIntervalMs = 1000
    };
    
    healthMgr.registerDriver(&driver, config);
    
    // Добавляем ошибки
    driver.addErrors(10);
    healthMgr.checkDriver(0, 1000);
    EXPECT_EQ(healthMgr.getDriverSeverity(0), Severity::ERROR);
    
    // Сбрасываем
    healthMgr.resetAllDrivers();
    
    // Проверяем что severity вернулся к INFO
    EXPECT_EQ(healthMgr.getDriverSeverity(0), Severity::INFO);
    EXPECT_EQ(driver.getErrorCount(), 0);
}
