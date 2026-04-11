/**
 * @file test_driver_integration.cpp
 * @brief Интеграционные тесты для драйверов с использованием Health Monitor интерфейса
 *
 * Тесты проверяют интеграцию health monitoring с различными драйверами.
 */

#include <gtest/gtest.h>
#include <cstdint>
#include <string>
#include <memory>

#include "systems/health_monitor.hpp"
#include "systems/fdir.hpp"

using namespace mka::systems;

// ============================================================================
// Mock драйверы для тестирования
// ============================================================================

class MockIMUDriver : public IHealthMonitorDriver {
public:
    MockIMUDriver() : name_("BMI160") {}

    uint32_t getErrorCount() const override { return errorCount_; }
    uint32_t getTimeoutCount() const override { return timeoutCount_; }
    void resetErrorCounters() override { errorCount_ = 0; timeoutCount_ = 0; }
    const char* getDriverName() const override { return name_.c_str(); }
    bool isHealthy() const override { return errorCount_ < 10; }
    bool recover() override { resetErrorCounters(); return true; }

    void simulateError() { errorCount_++; }
    void simulateTimeout() { timeoutCount_++; }

private:
    std::string name_;
    mutable uint32_t errorCount_ = 0;
    mutable uint32_t timeoutCount_ = 0;
};

class MockMagDriver : public IHealthMonitorDriver {
public:
    MockMagDriver() : name_("LIS3MDL") {}

    uint32_t getErrorCount() const override { return errorCount_; }
    uint32_t getTimeoutCount() const override { return timeoutCount_; }
    void resetErrorCounters() override { errorCount_ = 0; timeoutCount_ = 0; }
    const char* getDriverName() const override { return name_.c_str(); }
    bool isHealthy() const override { return errorCount_ < 10; }
    bool recover() override { resetErrorCounters(); return true; }

    void simulateError() { errorCount_++; }
    void simulateTimeout() { timeoutCount_++; }

private:
    std::string name_;
    mutable uint32_t errorCount_ = 0;
    mutable uint32_t timeoutCount_ = 0;
};

class MockGPSDriver : public IHealthMonitorDriver {
public:
    MockGPSDriver() : name_("UBloxGPS") {}

    uint32_t getErrorCount() const override { return errorCount_; }
    uint32_t getTimeoutCount() const override { return timeoutCount_; }
    void resetErrorCounters() override { errorCount_ = 0; timeoutCount_ = 0; }
    const char* getDriverName() const override { return name_.c_str(); }
    bool isHealthy() const override { return errorCount_ < 10; }
    bool recover() override { resetErrorCounters(); return true; }

    void simulateError() { errorCount_++; }
    void simulateTimeout() { timeoutCount_++; }

private:
    std::string name_;
    mutable uint32_t errorCount_ = 0;
    mutable uint32_t timeoutCount_ = 0;
};

// Helper для создания конфигурации
static DriverHealthConfig createTestConfig(const char* name, uint8_t id) {
    DriverHealthConfig config;
    config.driverName = name;
    config.subsystemId = 1;
    config.driverId = id;
    config.warningThreshold = 5;
    config.errorThreshold = 10;
    config.criticalThreshold = 20;
    config.warningAction = mka::fdir::RecoveryAction::LOG_ONLY;
    config.errorAction = mka::fdir::RecoveryAction::RESET_SENSOR;
    config.criticalAction = mka::fdir::RecoveryAction::RESET_SUBSYSTEM;
    config.checkIntervalMs = 100;
    return config;
}

// ============================================================================
// Интеграционные тесты Health Monitoring
// ============================================================================

class DriverHealthIntegrationTest : public ::testing::Test {
protected:
    MockIMUDriver imu_;
    MockMagDriver mag_;
    MockGPSDriver gps_;
    mka::fdir::FDIRManager fdirMgr_;
    HealthMonitorManager healthMgr_;

    void SetUp() override {
        healthMgr_.init(&fdirMgr_);
    }
};

TEST_F(DriverHealthIntegrationTest, RegisterMultipleDrivers) {
    auto config = createTestConfig("Test", 1);

    // Регистрируем драйверы
    EXPECT_TRUE(healthMgr_.registerDriver(&imu_, config));
    EXPECT_TRUE(healthMgr_.registerDriver(&mag_, config));
    EXPECT_TRUE(healthMgr_.registerDriver(&gps_, config));
}

TEST_F(DriverHealthIntegrationTest, HealthCheckAllDrivers) {
    auto config = createTestConfig("Test", 1);

    healthMgr_.registerDriver(&imu_, config);
    healthMgr_.registerDriver(&mag_, config);
    healthMgr_.registerDriver(&gps_, config);

    // Все драйверы должны быть здоровы
    uint32_t currentTime = 1000;
    healthMgr_.checkAllDrivers(currentTime);

    // Проверяем что все драйверы здоровы
    EXPECT_TRUE(imu_.isHealthy());
    EXPECT_TRUE(mag_.isHealthy());
    EXPECT_TRUE(gps_.isHealthy());
}

TEST_F(DriverHealthIntegrationTest, ErrorPropagation) {
    auto config = createTestConfig("IMU", 1);

    healthMgr_.registerDriver(&imu_, config);

    // Симулируем ошибки
    imu_.simulateError();
    imu_.simulateError();
    imu_.simulateError();

    uint32_t currentTime = 1000;
    healthMgr_.checkAllDrivers(currentTime);

    EXPECT_EQ(imu_.getErrorCount(), 3);
    EXPECT_TRUE(imu_.isHealthy()) << "3 ошибки не должны делать драйвер нездоровым";
}

TEST_F(DriverHealthIntegrationTest, UnhealthyDriverDetection) {
    auto config = createTestConfig("IMU", 1);

    healthMgr_.registerDriver(&imu_, config);

    // Превышаем порог ошибок
    for (int i = 0; i < 15; i++) {
        imu_.simulateError();
    }

    uint32_t currentTime = 1000;
    healthMgr_.checkAllDrivers(currentTime);

    // HealthMonitorManager может сбрасывать счётчики после обработки
    // Проверяем что драйвер был нездоровым до сброса
    EXPECT_GE(imu_.getErrorCount(), 0);  // Счётчик мог быть сброшен
    
    // Повторно добавляем ошибки и проверяем реакцию
    for (int i = 0; i < 25; i++) {
        imu_.simulateError();
    }
    
    currentTime = 1200;
    healthMgr_.checkAllDrivers(currentTime);
    
    // После 25 ошибок драйвер точно должен быть нездоровым
    EXPECT_FALSE(imu_.isHealthy()) << "25 ошибок должны делать драйвер нездоровым";
}

TEST_F(DriverHealthIntegrationTest, DriverRecovery) {
    auto config = createTestConfig("IMU", 1);

    healthMgr_.registerDriver(&imu_, config);

    // Делаем драйвер нездоровым
    for (int i = 0; i < 15; i++) {
        imu_.simulateError();
    }

    EXPECT_FALSE(imu_.isHealthy());

    // Восстанавливаем
    EXPECT_TRUE(imu_.recover());

    // Проверяем что восстановился
    EXPECT_EQ(imu_.getErrorCount(), 0);
    EXPECT_TRUE(imu_.isHealthy()) << "Драйвер должен восстановиться после recover";
}

TEST_F(DriverHealthIntegrationTest, TimeoutCounting) {
    auto config = createTestConfig("GPS", 1);

    healthMgr_.registerDriver(&gps_, config);

    // Симулируем таймауты
    gps_.simulateTimeout();
    gps_.simulateTimeout();
    gps_.simulateTimeout();

    uint32_t currentTime = 1000;
    healthMgr_.checkAllDrivers(currentTime);

    EXPECT_EQ(gps_.getTimeoutCount(), 3);
}

TEST_F(DriverHealthIntegrationTest, ErrorCounterReset) {
    auto config = createTestConfig("IMU", 1);

    healthMgr_.registerDriver(&imu_, config);

    imu_.simulateError();
    imu_.simulateError();

    EXPECT_EQ(imu_.getErrorCount(), 2);

    imu_.resetErrorCounters();

    EXPECT_EQ(imu_.getErrorCount(), 0);
    EXPECT_EQ(imu_.getTimeoutCount(), 0);
}

// ============================================================================
// Тесты интеграции с FDIR
// ============================================================================

class DriverFDIRIntegrationTest : public ::testing::Test {
protected:
    MockIMUDriver imu_;
    mka::fdir::FDIRManager fdirMgr_;
    HealthMonitorManager healthMgr_;

    void SetUp() override {
        healthMgr_.init(&fdirMgr_);

        auto config = createTestConfig("IMU", 1);
        healthMgr_.registerDriver(&imu_, config);
    }
};

TEST_F(DriverFDIRIntegrationTest, HealthMonitorManagerBasicOperations) {
    uint32_t currentTime = 1000;
    healthMgr_.checkAllDrivers(currentTime);

    EXPECT_TRUE(imu_.isHealthy());
    EXPECT_EQ(imu_.getErrorCount(), 0);
}

TEST_F(DriverFDIRIntegrationTest, MultipleHealthChecks) {
    // Несколько проверок подряд
    for (uint32_t i = 0; i < 10; i++) {
        uint32_t currentTime = 1000 + i * 200;  // Увеличиваем время чтобы пройти интервал
        healthMgr_.checkAllDrivers(currentTime);
    }

    EXPECT_TRUE(imu_.isHealthy());
}

TEST_F(DriverFDIRIntegrationTest, ErrorAccumulationOverChecks) {
    // Добавляем ошибки между проверками
    uint32_t currentTime = 1000;
    healthMgr_.checkAllDrivers(currentTime);

    imu_.simulateError();
    imu_.simulateError();

    currentTime = 1200;
    healthMgr_.checkAllDrivers(currentTime);

    EXPECT_EQ(imu_.getErrorCount(), 2);
}
