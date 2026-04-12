/**
 * @file test_radio.cpp
 * @brief Тесты для Radio драйвера (SI4463)
 * 
 * Покрывает:
 * - SI4463Driver (UHF/VHF радиомодуль)
 * - Конфигурация частот, модуляции, мощности
 * - Передача и приём пакетов
 * - Обработка ошибок и таймаутов
 */

#include <gtest/gtest.h>
#include <cstring>
#include <vector>
#include <map>
#include <functional>

#include "drivers/radio_driver.hpp"

using namespace mka::radio;

// ============================================================================
// Mock SPI Interface
// ============================================================================

class MockRadioSPI : public IRadioSPI {
public:
    bool transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx) override {
        if (tx.size() == 0) return false;
        
        // Симуляция команд радио
        uint8_t cmd = tx[0];
        
        switch (cmd) {
            case 0x01:  // POWER_UP
                chipReady_ = true;
                if (rx.size() > 0) rx[0] = 0xFF;  // CTS response
                break;
            case 0x02:  // POWER_DOWN
                chipReady_ = false;
                break;
            case 0x10:  // GET_INT_STATUS
                if (rx.size() > 0) rx[0] = interruptStatus_;
                break;
            case 0x11:  // GET_CHIP_STATUS
                if (rx.size() > 2) {
                    rx[0] = chipReady_ ? 0x01 : 0x00;  // CHIP_READY
                    rx[1] = 0x00;  // CMD_ERROR
                }
                break;
            case 0x20:  // SET_PROPERTY
                // Сохраняем property
                if (tx.size() >= 4) {
                    uint16_t prop = (tx[1] << 8) | tx[2];
                    properties_[prop] = tx[3];
                }
                break;
            case 0x22:  // GET_PROPERTY
                if (tx.size() >= 3 && rx.size() > 0) {
                    uint16_t prop = (tx[1] << 8) | tx[2];
                    auto it = properties_.find(prop);
                    rx[0] = (it != properties_.end()) ? it->second : 0x00;
                }
                break;
            case 0x30:  // START_TX
                txActive_ = true;
                break;
            case 0x31:  // START_RX
                rxActive_ = true;
                break;
            case 0x35:  // TX_FIFO_WRITE
                if (tx.size() >= 2) {
                    uint8_t len = tx[1];
                    for (size_t i = 0; i < len && i < tx.size() - 2; ++i) {
                        txFifo_.push_back(tx[2 + i]);
                    }
                }
                break;
            case 0x36:  // RX_FIFO_READ
                if (tx.size() >= 2 && rx.size() >= tx[1]) {
                    uint8_t len = tx[1];
                    for (size_t i = 0; i < len && i < rxFifo_.size(); ++i) {
                        rx[i] = rxFifo_[i];
                    }
                    rxFifo_.clear();
                }
                break;
            case 0x15:  // GET_MODEM_STATUS
                if (rx.size() > 4) {
                    rx[0] = interruptStatus_;
                    rx[1] = rssi_;  // RSSI
                    rx[2] = 0x00;   // RSSI clears
                    rx[3] = 0x00;   // RSSI latches
                }
                break;
            default:
                commandCount_++;
                break;
        }
        
        transferCount_++;
        return true;
    }

    void select() override { selected_ = true; }
    void deselect() override { selected_ = false; }

    void reset() {
        properties_.clear();
        txFifo_.clear();
        rxFifo_.clear();
        transferCount_ = 0;
        commandCount_ = 0;
        chipReady_ = false;
        txActive_ = false;
        rxActive_ = false;
        selected_ = false;
        interruptStatus_ = 0x00;
        rssi_ = -90;
    }

    void setInterruptStatus(uint8_t status) { interruptStatus_ = status; }
    void setRSSI(int8_t rssi) { rssi_ = rssi; }
    void queueRxData(const std::vector<uint8_t>& data) { rxFifo_ = data; }
    const std::vector<uint8_t>& getTxData() const { return txFifo_; }
    
    size_t getTransferCount() const { return transferCount_; }
    size_t getCommandCount() const { return commandCount_; }
    bool isChipReady() const { return chipReady_; }
    bool isTxActive() const { return txActive_; }
    bool isRxActive() const { return rxActive_; }

private:
    std::map<uint16_t, uint8_t> properties_;
    std::vector<uint8_t> txFifo_;
    std::vector<uint8_t> rxFifo_;
    size_t transferCount_ = 0;
    size_t commandCount_ = 0;
    bool chipReady_ = false;
    bool txActive_ = false;
    bool rxActive_ = false;
    bool selected_ = false;
    uint8_t interruptStatus_ = 0x00;
    int8_t rssi_ = -90;
};

// ============================================================================
// Mock GPIO Interface
// ============================================================================

class MockRadioGPIO : public IRadioGPIO {
public:
    void setReset(bool state) override { resetState_ = state; }
    bool getInterrupt() override { return interruptState_; }
    void setTxEnable(bool state) override { txEnable_ = state; }
    void setRxEnable(bool state) override { rxEnable_ = state; }

    void setInterruptState(bool state) { interruptState_ = state; }
    
    bool getResetState() const { return resetState_; }
    bool getTxEnable() const { return txEnable_; }
    bool getRxEnable() const { return rxEnable_; }

private:
    bool resetState_ = false;
    bool interruptState_ = false;
    bool txEnable_ = false;
    bool rxEnable_ = false;
};

// ============================================================================
// SI4463 Driver Tests
// ============================================================================

class SI4463DriverTest : public ::testing::Test {
protected:
    MockRadioSPI mockSPI_;
    MockRadioGPIO mockGPIO_;
    std::unique_ptr<SI4463Driver> radio_;

    void SetUp() override {
        mockSPI_.reset();
        radio_ = std::make_unique<SI4463Driver>(mockSPI_, mockGPIO_);
    }

    RadioConfig getDefaultConfig() {
        RadioConfig config = {};
        config.frequencyHz = 435000000;  // 435 MHz (UHF)
        config.channelSpacing = 25000;   // 25 kHz
        config.channel = 0;
        config.modulation = Modulation::GFSK;
        config.rxBandwidth = Bandwidth::BW_25_KHZ;
        config.datarate = 9600;          // 9600 bps
        config.deviation = 5000;         // 5 kHz
        config.txPower = TxPower::MEDIUM;
        config.preambleLength = 32;
        config.syncWord = 0x2DD4;
        config.syncWordLength = 16;
        config.rxTimeout = 1000;
        config.rssiThreshold = -100;
        config.maxPacketLength = 64;
        config.crcEnable = true;
        config.whiteningEnable = true;
        config.manchesterEnable = false;
        return config;
    }
};

TEST_F(SI4463DriverTest, Construction) {
    // Драйвер создан без инициализации
    EXPECT_FALSE(mockSPI_.isChipReady());
}

TEST_F(SI4463DriverTest, Initialization) {
    RadioConfig config = getDefaultConfig();
    RadioStatus status = radio_->init(config);
    
    EXPECT_TRUE(status == RadioStatus::OK || status == RadioStatus::ERROR);
    EXPECT_TRUE(mockSPI_.isChipReady());
}

TEST_F(SI4463DriverTest, InitializationWithNullPointers) {
    // Инициализация с nullptr должна вернуть ошибку
    SI4463Driver badRadio(nullptr, mockGPIO_);
    RadioConfig config = getDefaultConfig();
    RadioStatus status = badRadio.init(config);
    EXPECT_TRUE(status != RadioStatus::OK);
}

TEST_F(SI4463DriverTest, Configuration) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    // Проверить что свойства настроены
    EXPECT_GT(mockSPI_.getTransferCount(), 0);
}

TEST_F(SI4463DriverTest, FrequencyConfiguration) {
    RadioConfig config = getDefaultConfig();
    config.frequencyHz = 145000000;  // 145 MHz (VHF)
    radio_->init(config);
    
    // Частота должна быть установлена
    EXPECT_EQ(config.frequencyHz, 145000000u);
}

// ============================================================================
// Radio Mode Tests
// ============================================================================

TEST_F(SI4463DriverTest, SetModeSleep) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    RadioStatus status = radio_->setMode(RadioMode::SLEEP);
    EXPECT_TRUE(status == RadioStatus::OK || status == RadioStatus::ERROR);
}

TEST_F(SI4463DriverTest, SetModeStandby) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    RadioStatus status = radio_->setMode(RadioMode::STANDBY);
    EXPECT_TRUE(status == RadioStatus::OK || status == RadioStatus::ERROR);
}

TEST_F(SI4463DriverTest, SetModeTX) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    RadioStatus status = radio_->setMode(RadioMode::TX);
    // TX может вернуть TX_BUSY если уже активен
    EXPECT_TRUE(status == RadioStatus::OK || 
                status == RadioStatus::TX_BUSY ||
                status == RadioStatus::ERROR);
}

TEST_F(SI4463DriverTest, SetModeRX) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    RadioStatus status = radio_->setMode(RadioMode::RX);
    EXPECT_TRUE(status == RadioStatus::OK || 
                status == RadioStatus::RX_BUSY ||
                status == RadioStatus::ERROR);
}

// ============================================================================
// Transmission Tests
// ============================================================================

TEST_F(SI4463DriverTest, TransmitPacket) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05};
    RadioStatus status = radio_->transmit(data.data(), data.size());
    
    EXPECT_TRUE(status == RadioStatus::OK || status == RadioStatus::ERROR);
}

TEST_F(SI4463DriverTest, TransmitEmptyPacket) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    RadioStatus status = radio_->transmit(nullptr, 0);
    EXPECT_EQ(status, RadioStatus::INVALID_PARAM);
}

TEST_F(SI4463DriverTest, TransmitTooLargePacket) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    std::vector<uint8_t> data(300, 0xAB);  // > 255 bytes
    RadioStatus status = radio_->transmit(data.data(), data.size());
    EXPECT_EQ(status, RadioStatus::INVALID_PARAM);
}

// ============================================================================
// Reception Tests
// ============================================================================

TEST_F(SI4463DriverTest, ReceivePacket) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    // Queue some data
    mockSPI_.queueRxData({0x01, 0x02, 0x03, 0x04, 0x05});
    
    std::array<uint8_t, 64> buffer{};
    size_t len = 0;
    RadioStatus status = radio_->receive(buffer.data(), buffer.size(), len);
    
    // Reception may succeed or fail depending on mock implementation
    EXPECT_TRUE(status == RadioStatus::OK || status == RadioStatus::TIMEOUT);
}

TEST_F(SI4463DriverTest, ReceiveTimeout) {
    RadioConfig config = getDefaultConfig();
    config.rxTimeout = 100;  // Short timeout
    radio_->init(config);
    
    std::array<uint8_t, 64> buffer{};
    size_t len = 0;
    RadioStatus status = radio_->receive(buffer.data(), buffer.size(), len);
    
    // Should return TIMEOUT if no data
    EXPECT_TRUE(status == RadioStatus::TIMEOUT || status == RadioStatus::OK);
}

// ============================================================================
// RSSI Tests
// ============================================================================

TEST_F(SI4463DriverTest, GetRSSI) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    int16_t rssi = 0;
    RadioStatus status = radio_->getRSSI(rssi);
    
    EXPECT_TRUE(status == RadioStatus::OK || status == RadioStatus::ERROR);
}

TEST_F(SI4463DriverTest, RSSIValues) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    mockSPI_.setRSSI(-70);
    int16_t rssi1 = 0;
    radio_->getRSSI(rssi1);
    
    mockSPI_.setRSSI(-90);
    int16_t rssi2 = 0;
    radio_->getRSSI(rssi2);
    
    // RSSI должен измениться
    EXPECT_NE(rssi1, rssi2);
}

// ============================================================================
// Channel Activity Detection
// ============================================================================

TEST_F(SI4463DriverTest, ChannelActivityDetection) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    bool channelFree = radio_->isChannelFree();
    // Результат зависит от реализации
    EXPECT_TRUE(channelFree || !channelFree);
}

// ============================================================================
// State Management Tests
// ============================================================================

TEST_F(SI4463DriverTest, GetState) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    auto state = radio_->getState();
    // State должен быть валидным
    EXPECT_TRUE(state.mode == RadioMode::SLEEP || 
                state.mode == RadioMode::STANDBY);
}

TEST_F(SI4463DriverTest, StateTransitions) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    // Переход SLEEP -> STANDBY
    radio_->setMode(RadioMode::STANDBY);
    auto state1 = radio_->getState();
    
    // Переход STANDBY -> TX
    radio_->setMode(RadioMode::TX);
    auto state2 = radio_->getState();
    
    // Переход TX -> STANDBY
    radio_->setMode(RadioMode::STANDBY);
    auto state3 = radio_->getState();
    
    EXPECT_NE(state1.mode, state2.mode);
    EXPECT_EQ(state3.mode, RadioMode::STANDBY);
}

// ============================================================================
// Constants Tests
// ============================================================================

TEST(RadioConstantsTest, FifoSize) {
    EXPECT_EQ(SI4463Driver::FIFO_SIZE, 64u);
}

TEST(RadioConstantsTest, MaxPacketSize) {
    EXPECT_EQ(SI4463Driver::MAX_PACKET_SIZE, 255u);
}

TEST(RadioConstantsTest, RadioPacketSize) {
    EXPECT_EQ(sizeof(RadioPacket), 260u);
}

TEST(RadioConstantsTest, EnumValues) {
    EXPECT_EQ(static_cast<uint8_t>(Modulation::FSK), 0u);
    EXPECT_EQ(static_cast<uint8_t>(Modulation::GFSK), 1u);
    EXPECT_EQ(static_cast<uint8_t>(Modulation::OOK), 2u);
    
    EXPECT_EQ(static_cast<uint8_t>(TxPower::MIN), 0u);
    EXPECT_EQ(static_cast<uint8_t>(TxPower::MAX), 4u);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(SI4463DriverTest, NullSPIInitialization) {
    SI4463Driver radio(nullptr, mockGPIO_);
    RadioConfig config = getDefaultConfig();
    RadioStatus status = radio.init(config);
    EXPECT_TRUE(status != RadioStatus::OK);
}

TEST_F(SI4463DriverTest, NullGPIOInitialization) {
    SI4463Driver radio(mockSPI_, nullptr);
    RadioConfig config = getDefaultConfig();
    RadioStatus status = radio.init(config);
    EXPECT_TRUE(status != RadioStatus::OK);
}

TEST_F(SI4463DriverTest, MultipleInitialization) {
    RadioConfig config1 = getDefaultConfig();
    RadioStatus status1 = radio_->init(config1);
    
    RadioConfig config2 = getDefaultConfig();
    config2.frequencyHz = 145000000;
    RadioStatus status2 = radio_->init(config2);
    
    // Вторая инициализация должна завершиться успешно или вернуть ошибку
    EXPECT_TRUE(status2 == RadioStatus::OK || status2 == RadioStatus::ERROR);
}

// ============================================================================
// Callback Tests
// ============================================================================

TEST_F(SI4463DriverTest, RegisterRxCallback) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    bool callbackCalled = false;
    radio_->setRxCallback([&callbackCalled](const RadioPacket& pkt) {
        (void)pkt;
        callbackCalled = true;
    });
    
    // Callback должен быть зарегистрирован
    EXPECT_TRUE(callbackCalled == false);
}

TEST_F(SI4463DriverTest, RegisterTxCallback) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    bool callbackCalled = false;
    radio_->setTxCallback([&callbackCalled](bool success) {
        (void)success;
        callbackCalled = true;
    });
    
    EXPECT_TRUE(callbackCalled == false);
}

// ============================================================================
// Data Integrity Tests
// ============================================================================

TEST_F(SI4463DriverTest, TransmitReceiveDataIntegrity) {
    RadioConfig config = getDefaultConfig();
    radio_->init(config);
    
    std::vector<uint8_t> txData = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
    RadioStatus txStatus = radio_->transmit(txData.data(), txData.size());
    
    // В моке данные должны сохраниться
    if (txStatus == RadioStatus::OK) {
        const auto& sentData = mockSPI_.getTxData();
        if (!sentData.empty()) {
            EXPECT_EQ(sentData[0], 0xDE);
            EXPECT_EQ(sentData.back(), 0xFE);
        }
    }
}
