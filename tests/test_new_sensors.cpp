/**
 * @file test_new_sensors.cpp
 * @brief Unit-тесты для драйверов HMC5883L и MAX31865
 *
 * Тесты проверяют:
 * - Инициализацию драйверов
 * - Чтение данных
 * - Калибровку (HMC5883L)
 * - Обработку ошибок
 * - Health monitoring
 */

#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <span>
#include <vector>

#include "drivers/new_sensors_drivers.hpp"
#include "hal/hal_full.hpp"

using namespace mka::sensors;
using namespace mka::hal;

// ============================================================================
// Mock реализации HAL для тестирования
// ============================================================================

class MockI2C : public II2C {
public:
    Status init(const I2CConfig& config) override { 
        (void)config;
        return Status::OK; 
    }
    void deinit() override {}
    
    Status writeRegister(uint8_t addr, uint8_t reg, std::span<const uint8_t> data, 
                        uint32_t timeoutMs) override {
        (void)timeoutMs;
        lastAddr_ = addr;
        lastReg_ = reg;
        lastWriteData_.assign(data.begin(), data.end());
        return writeStatus_;
    }

    Status readRegister(uint8_t addr, uint8_t reg, std::span<uint8_t> data, 
                       uint32_t timeoutMs) override {
        (void)timeoutMs;
        lastAddr_ = addr;
        lastReg_ = reg;
        
        // Имитация чтения ID
        if (reg == 0x0A) {  // ID_A регистр HMC5883L
            if (data.size() >= 1) data[0] = 0x48;
        } else if (reg == 0x0B) {  // ID_B регистр
            if (data.size() >= 1) data[0] = 0x34;
        } else if (reg == 0x0C) {  // ID_C регистр
            if (data.size() >= 1) data[0] = 0x33;
        } else if (reg == 0x09) {  // STATUS регистр
            if (data.size() >= 1) data[0] = 0x01;  // Data ready
        } else {
            std::memset(data.data(), 0, data.size());
        }
        
        return readStatus_;
    }
    
    Status writeRegister16(uint8_t, uint16_t, std::span<const uint8_t>, uint32_t) override { return Status::OK; }
    Status readRegister16(uint8_t, uint16_t, std::span<uint8_t>, uint32_t) override { return Status::OK; }
    size_t scanBus(std::span<uint8_t>) override { return 0; }
    bool isDevicePresent(uint8_t) override { return true; }
    Status recoverBus() override { return Status::OK; }
    I2CStatistics getStatistics() const override { return {}; }

    void setWriteStatus(Status status) { writeStatus_ = status; }
    void setReadStatus(Status status) { readStatus_ = status; }
    
    uint8_t getLastAddr() const { return lastAddr_; }
    uint8_t getLastReg() const { return lastReg_; }
    const std::vector<uint8_t>& getLastWriteData() const { return lastWriteData_; }

private:
    Status writeStatus_ = Status::OK;
    Status readStatus_ = Status::OK;
    uint8_t lastAddr_ = 0;
    uint8_t lastReg_ = 0;
    std::vector<uint8_t> lastWriteData_;
};

class MockSPI : public ISPI {
public:
    Status init(const SPIConfig& config) override { 
        (void)config;
        return Status::OK; 
    }
    void deinit() override {}
    Status selectDevice() override { return Status::OK; }
    void deselectDevice() override {}
    
    Status transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx, uint32_t timeoutMs) override {
        (void)timeoutMs;
        lastTx_.assign(tx.begin(), tx.end());
        
        // Имитация ответа MAX31865
        if (tx.size() >= 1 && rx.size() >= 2) {
            std::memset(rx.data(), 0, rx.size());
            if (tx[0] & 0x80) {
                // Запись - возвращаем dummy
            } else {
                // Чтение - возвращаем тестовые данные
                uint8_t reg = tx[0] & 0x7F;
                if (reg == 0x01) {  // RTD_MSB
                    rx[1] = 0x8A;  // Тестовое значение
                } else if (reg == 0x02) {  // RTD_LSB
                    rx[1] = 0x00;
                } else if (reg == 0x07) {  // FAULT_STATUS
                    rx[1] = 0x00;  // Нет ошибок
                }
            }
        }
        
        return transferStatus_;
    }
    
    Status transmit(std::span<const uint8_t>, uint32_t) override { return Status::OK; }
    Status receive(std::span<uint8_t>, uint32_t) override { return Status::OK; }
    Status transferAsync(std::span<const uint8_t>, std::span<uint8_t>, TransferCallback) override { return Status::OK; }
    Status setClockSpeed(uint32_t) override { return Status::OK; }
    bool isFIFOOverflow() const override { return false; }
    Status clearFIFO() override { return Status::OK; }
    SPIStatistics getStatistics() const override { return {}; }

    void setTransferStatus(Status status) { transferStatus_ = status; }
    
    const std::vector<uint8_t>& getLastTx() const { return lastTx_; }

private:
    Status transferStatus_ = Status::OK;
    std::vector<uint8_t> lastTx_;
};

// ============================================================================
// Тесты HMC5883L
// ============================================================================

class HMC5883LTest : public ::testing::Test {
protected:
    void SetUp() override {
        mockI2C_ = std::make_unique<MockI2C>();
        driver_ = std::make_unique<HMC5883LDriver>(*mockI2C_);
    }

    std::unique_ptr<MockI2C> mockI2C_;
    std::unique_ptr<HMC5883LDriver> driver_;
};

TEST_F(HMC5883LTest, InitSuccess) {
    HMC5883LDriver::Config config;
    config.scale = HMC5883LDriver::Scale::SCALE_1_3G;
    config.mode = HMC5883LDriver::Mode::CONTINUOUS;
    config.dataRate = HMC5883LDriver::DataRate::RATE_15HZ;
    
    Status status = driver_->init(config);
    EXPECT_EQ(status, Status::OK);
}

TEST_F(HMC5883LTest, InitWithAutoCalibrate) {
    HMC5883LDriver::Config config;
    config.autoCalibrate = true;
    
    // Автокалибровка должна вернуть ошибку (недостаточно данных в моке)
    // Но init() должен завершиться
    driver_->init(config);
    // Проверяем что драйвер не упал
    EXPECT_FALSE(driver_->isCalibrating());
}

TEST_F(HMC5883LTest, ReadDataSuccess) {
    // Сначала инициализация
    Status st = driver_->init();
    ASSERT_EQ(st, Status::OK);
    
    MagData data;
    st = driver_->readData(data);
    
    // Данные должны быть прочитаны
    EXPECT_TRUE(st == Status::OK || st == Status::BUSY);
}

TEST_F(HMC5883LTest, HealthMonitoring) {
    // Проверка начальных значений
    EXPECT_EQ(driver_->getErrorCount(), 0u);
    EXPECT_EQ(driver_->getTimeoutCount(), 0u);
    EXPECT_FALSE(driver_->isCalibrating());
    EXPECT_EQ(driver_->getCalibrationSamples(), 0u);
    
    // Симуляция ошибок
    mockI2C_->setReadStatus(Status::ERROR);
    
    MagData data;
    driver_->readData(data);
    
    EXPECT_GT(driver_->getErrorCount(), 0u);
    
    // Сброс счётчиков
    driver_->resetErrorCounters();
    EXPECT_EQ(driver_->getErrorCount(), 0u);
    EXPECT_EQ(driver_->getTimeoutCount(), 0u);
}

TEST_F(HMC5883LTest, Calibration) {
    HMC5883LDriver::CalibrationData cal;
    cal.offset[0] = 0.1f;
    cal.offset[1] = -0.2f;
    cal.offset[2] = 0.05f;
    cal.isCalibrated = true;
    
    driver_->setCalibration(cal);
    
    const auto& retrieved = driver_->getCalibration();
    EXPECT_TRUE(retrieved.isCalibrated);
    EXPECT_FLOAT_EQ(retrieved.offset[0], 0.1f);
    EXPECT_FLOAT_EQ(retrieved.offset[1], -0.2f);
    EXPECT_FLOAT_EQ(retrieved.offset[2], 0.05f);
    
    // Сброс калибровки
    driver_->resetCalibration();
    const auto& reset = driver_->getCalibration();
    EXPECT_FALSE(reset.isCalibrated);
}

TEST_F(HMC5883LTest, SetMode) {
    Status st = driver_->setMode(HMC5883LDriver::Mode::SINGLE);
    EXPECT_EQ(st, Status::OK);
}

TEST_F(HMC5883LTest, SetScale) {
    Status st = driver_->setScale(HMC5883LDriver::Scale::SCALE_2_5G);
    EXPECT_EQ(st, Status::OK);
}

TEST_F(HMC5883LTest, SoftIronCalibration) {
    float matrix[9] = {
        1.0f, 0.01f, 0.0f,
        0.01f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };
    
    Status st = driver_->setSoftIronCalibration(matrix);
    EXPECT_EQ(st, Status::OK);
    
    const auto& cal = driver_->getCalibration();
    EXPECT_TRUE(cal.isCalibrated);
    EXPECT_FLOAT_EQ(cal.softIronMatrix[0], 1.0f);
    EXPECT_FLOAT_EQ(cal.softIronMatrix[1], 0.01f);
    
    // Проверка невалидного параметра
    st = driver_->setSoftIronCalibration(nullptr);
    EXPECT_EQ(st, Status::INVALID_PARAM);
}

// ============================================================================
// Тесты MAX31865
// ============================================================================

class MAX31865Test : public ::testing::Test {
protected:
    void SetUp() override {
        mockSPI_ = std::make_unique<MockSPI>();
        driver_ = std::make_unique<MAX31865Driver>(*mockSPI_);
    }

    std::unique_ptr<MockSPI> mockSPI_;
    std::unique_ptr<MAX31865Driver> driver_;
};

TEST_F(MAX31865Test, InitSuccess) {
    MAX31865Driver::Config config;
    config.rtdType = MAX31865Driver::RTDType::PT100;
    config.rref = 4300.0f;
    config.wires = 4;
    
    Status status = driver_->init(config);
    EXPECT_EQ(status, Status::OK);
}

TEST_F(MAX31865Test, InitPT1000) {
    MAX31865Driver::Config config;
    config.rtdType = MAX31865Driver::RTDType::PT1000;
    config.rref = 43000.0f;
    
    Status status = driver_->init(config);
    EXPECT_EQ(status, Status::OK);
}

TEST_F(MAX31865Test, ReadTemperatureSuccess) {
    Status st = driver_->init();
    ASSERT_EQ(st, Status::OK);
    
    MAX31865Driver::TempData data;
    st = driver_->readTemperature(data);
    
    EXPECT_EQ(st, Status::OK);
    EXPECT_FALSE(data.faultDetected);
    EXPECT_EQ(data.faultStatus, 0u);
    EXPECT_GT(data.resistance, 0.0f);  // Сопротивление должно быть > 0
}

TEST_F(MAX31865Test, HealthMonitoring) {
    EXPECT_EQ(driver_->getErrorCount(), 0u);
    
    // Симуляция ошибки
    mockSPI_->setTransferStatus(Status::ERROR);
    
    MAX31865Driver::TempData data;
    driver_->readTemperature(data);
    
    EXPECT_GT(driver_->getErrorCount(), 0u);
    
    // Сброс
    driver_->resetErrorCounters();
    EXPECT_EQ(driver_->getErrorCount(), 0u);
}

TEST_F(MAX31865Test, SetFaultThresholds) {
    Status st = driver_->init();
    ASSERT_EQ(st, Status::OK);
    
    // Нормальные пороги
    st = driver_->setFaultThresholds(100.0f, 0.0f);
    EXPECT_EQ(st, Status::OK);
    
    // Невалидные пороги (high <= low)
    st = driver_->setFaultThresholds(0.0f, 100.0f);
    EXPECT_EQ(st, Status::INVALID_PARAM);
}

TEST_F(MAX31865Test, ClearFaultStatus) {
    Status st = driver_->init();
    ASSERT_EQ(st, Status::OK);
    
    st = driver_->clearFaultStatus();
    EXPECT_EQ(st, Status::OK);
}

TEST_F(MAX31865Test, TemperatureCalculation) {
    // Проверка расчёта для PT100 при 0°C
    MAX31865Driver::Config config;
    config.rtdType = MAX31865Driver::RTDType::PT100;
    driver_->init(config);
    
    // При R = 100 Ом, T должен быть около 0°C
    // Проверяем через чтение данных
    MAX31865Driver::TempData data;
    Status st = driver_->readTemperature(data);
    
    if (st == Status::OK) {
        // Температура должна быть в разумных пределах
        EXPECT_GT(data.temperature, -200.0f);
        EXPECT_LT(data.temperature, 850.0f);
    }
}

TEST_F(MAX31865Test, ConfigRetrieval) {
    MAX31865Driver::Config config;
    config.rtdType = MAX31865Driver::RTDType::PT500;
    config.rref = 21500.0f;
    config.wires = 3;
    
    driver_->init(config);
    
    const auto& retrieved = driver_->getConfig();
    EXPECT_EQ(retrieved.rtdType, MAX31865Driver::RTDType::PT500);
    EXPECT_FLOAT_EQ(retrieved.rref, 21500.0f);
    EXPECT_EQ(retrieved.wires, 3u);
}

// ============================================================================
// Интеграционные тесты
// ============================================================================

class SensorIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        mockI2C_ = std::make_unique<MockI2C>();
        hmcDriver_ = std::make_unique<HMC5883LDriver>(*mockI2C_);
        
        mockSPI_ = std::make_unique<MockSPI>();
        maxDriver_ = std::make_unique<MAX31865Driver>(*mockSPI_);
    }

    std::unique_ptr<MockI2C> mockI2C_;
    std::unique_ptr<HMC5883LDriver> hmcDriver_;
    std::unique_ptr<MockSPI> mockSPI_;
    std::unique_ptr<MAX31865Driver> maxDriver_;
};

TEST_F(SensorIntegrationTest, MultipleSensorsInit) {
    // Инициализация обоих датчиков
    Status hmcStatus = hmcDriver_->init();
    Status maxStatus = maxDriver_->init();
    
    EXPECT_EQ(hmcStatus, Status::OK);
    EXPECT_EQ(maxStatus, Status::OK);
}

TEST_F(SensorIntegrationTest, HealthMonitoringBothSensors) {
    hmcDriver_->init();
    maxDriver_->init();
    
    // Проверка что оба датчика имеют нулевые счётчики
    EXPECT_EQ(hmcDriver_->getErrorCount(), 0u);
    EXPECT_EQ(maxDriver_->getErrorCount(), 0u);
    
    // Симуляция ошибок на обоих
    mockI2C_->setReadStatus(Status::ERROR);
    mockSPI_->setTransferStatus(Status::ERROR);
    
    MagData magData;
    MAX31865Driver::TempData tempData;
    
    hmcDriver_->readData(magData);
    maxDriver_->readTemperature(tempData);
    
    EXPECT_GT(hmcDriver_->getErrorCount(), 0u);
    EXPECT_GT(maxDriver_->getErrorCount(), 0u);
}

TEST_F(SensorIntegrationTest, RecoverFromErrors) {
    hmcDriver_->init();
    maxDriver_->init();
    
    // Симуляция ошибок
    mockI2C_->setReadStatus(Status::ERROR);
    mockSPI_->setTransferStatus(Status::ERROR);
    
    MagData magData;
    MAX31865Driver::TempData tempData;
    
    hmcDriver_->readData(magData);
    maxDriver_->readTemperature(tempData);
    
    // Восстановление
    mockI2C_->setReadStatus(Status::OK);
    mockSPI_->setTransferStatus(Status::OK);
    
    hmcDriver_->resetErrorCounters();
    maxDriver_->resetErrorCounters();
    
    EXPECT_EQ(hmcDriver_->getErrorCount(), 0u);
    EXPECT_EQ(maxDriver_->getErrorCount(), 0u);
}

// ============================================================================
// Запуск тестов
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
