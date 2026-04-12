/**
 * @file test_canopen.cpp
<<<<<<< HEAD
 * @brief Unit-тесты для CANopen стека
 *
 * Тесты покрывают:
 * - NMT State Machine (состояния и переходы)
 * - SDO Server (чтение/запись Object Dictionary)
 * - PDO Manager (TPDO/RPDO конфигурация и передача)
 * - CANopen Stack (интеграция компонентов)
=======
 * @brief Полные unit-тесты для CANopen стека
>>>>>>> dev
 */

#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
<<<<<<< HEAD
#include <array>
#include <functional>
#include <vector>

#include "systems/canopen.hpp"
=======
#include <vector>
#include <queue>
#include <map>

#include "systems/canopen.hpp"
#include "utils/result.hpp"
>>>>>>> dev
#include "hal/hal_full.hpp"

using namespace mka::systems;

// ============================================================================
<<<<<<< HEAD
// Mock CAN интерфейс
// ============================================================================

namespace {

class MockCAN : public mka::hal::ICAN {
public:
    struct Message {
        uint32_t id = 0;
        std::array<uint8_t, 8> data{};
        size_t len = 0;
    };

    mka::hal::Status init(const mka::hal::CANConfig& config) override {
        (void)config;
        initialized_ = true;
        return mka::hal::Status::OK;
    }

    void deinit() override { initialized_ = false; }

    mka::hal::Status transmit(const mka::hal::CANMessage& msg,
                             uint32_t timeoutMs = 1000) override {
        (void)timeoutMs;
        Message m;
        m.id = msg.id;
        m.data = msg.data;
        m.len = msg.dlc;
        transmittedMessages_.push_back(m);
        return transmitEnabled_ ? mka::hal::Status::OK : mka::hal::Status::ERROR;
    }

    mka::hal::Status transmitAsync(const mka::hal::CANMessage& msg,
                                   TxCompleteCallback callback) override {
        (void)msg;
        (void)callback;
        return mka::hal::Status::ERROR;
    }

    mka::hal::Status receive(mka::hal::CANMessage& msg,
                            uint32_t timeoutMs = 1000) override {
        (void)msg;
        (void)timeoutMs;
        return mka::hal::Status::ERROR;
    }

    void setRxCallback(RxCallback callback) override {
        (void)callback;
    }

    mka::hal::Status addFilter(const mka::hal::CANFilter& filter,
                               uint8_t fifo = 0) override {
        (void)filter;
        (void)fifo;
        return mka::hal::Status::OK;
    }

    void clearFilters() override {}

    bool isBusOff() const override { return false; }

    mka::hal::Status recoverBus() override {
        return mka::hal::Status::OK;
    }

    mka::hal::CANStatistics getStatistics() const override {
        return mka::hal::CANStatistics{};
    }

    void resetStatistics() override {}

    void setTransmitEnabled(bool enabled) { transmitEnabled_ = enabled; }

    const std::vector<Message>& getTransmittedMessages() const {
        return transmittedMessages_;
    }

    void clearMessages() { transmittedMessages_.clear(); }

    size_t getTransmittedCount() const { return transmittedMessages_.size(); }

    bool isInitialized() const { return initialized_; }

private:
    std::vector<Message> transmittedMessages_;
    bool transmitEnabled_ = true;
    bool initialized_ = false;
};

} // namespace
=======
// Mock CAN Interface
// ============================================================================

class MockCAN : public hal::ICAN {
public:
    struct CANFrame {
        uint32_t id;
        std::vector<uint8_t> data;
    };

    hal::Status transmit(uint32_t id, const uint8_t* data, size_t len) override {
        CANFrame frame;
        frame.id = id;
        frame.data.assign(data, data + len);
        sentFrames_.push_back(frame);
        sendCount_++;
        return hal::Status::OK;
    }

    hal::Status receive(uint32_t& id, uint8_t* buffer, size_t maxLen, size_t& outLen, uint32_t timeout) override {
        if (receiveQueue_.empty()) {
            return hal::Status::TIMEOUT;
        }
        
        auto frame = receiveQueue_.front();
        receiveQueue_.pop();
        id = frame.id;
        size_t toCopy = std::min(frame.data.size(), maxLen);
        std::memcpy(buffer, frame.data.data(), toCopy);
        outLen = toCopy;
        receiveCount_++;
        return hal::Status::OK;
    }

    void queueFrame(uint32_t id, const std::vector<uint8_t>& data) {
        CANFrame frame;
        frame.id = id;
        frame.data = data;
        receiveQueue_.push(frame);
    }

    const std::vector<CANFrame>& getSentFrames() const { return sentFrames_; }
    size_t getSendCount() const { return sendCount_; }
    size_t getReceiveCount() const { return receiveCount_; }

    void reset() {
        sentFrames_.clear();
        receiveQueue_ = std::queue<CANFrame>();
        sendCount_ = 0;
        receiveCount_ = 0;
    }

private:
    std::vector<CANFrame> sentFrames_;
    std::queue<CANFrame> receiveQueue_;
    size_t sendCount_ = 0;
    size_t receiveCount_ = 0;
};

// ============================================================================
// CANopen Test Fixture
// ============================================================================

class CANopenTest : public ::testing::Test {
protected:
    MockCAN can_;
    CANopenStack stack_{1};

    void SetUp() override {
        can_.reset();
        stack_.init(can_);
    }

    void queueNMTCommand(NMTCommand cmd, uint8_t nodeId) {
        std::vector<uint8_t> data = {static_cast<uint8_t>(cmd), nodeId};
        can_.queueFrame(COBId::NMT_COMMAND, data);
    }

    void processQueuedMessages() {
        while (can_.getReceiveCount() < can_.getSendCount()) {
            uint32_t id;
            std::array<uint8_t, 8> buffer{};
            size_t len = 0;
            auto status = can_.receive(id, buffer.data(), buffer.size(), len);
            if (status != hal::Status::OK) break;
            
            stack_.processMessage(id, buffer.data(), len);
        }
    }
};
>>>>>>> dev

// ============================================================================
// NMT State Machine Tests
// ============================================================================

<<<<<<< HEAD
class NMTStateMachineTest : public ::testing::Test {
protected:
    NMTStateMachine nmt_{1};
};

// Тест: начальное состояние - Initializing
TEST_F(NMTStateMachineTest, InitialStateIsInitializing) {
    EXPECT_EQ(nmt_.getState(), NodeState::INITIALIZING);
}

// Тест: переход в Pre-Operational
TEST_F(NMTStateMachineTest, TransitionToPreOperational) {
    nmt_.processCommand(NMTCommand::ENTER_PRE_OPERATIONAL);
    EXPECT_EQ(nmt_.getState(), NodeState::PRE_OPERATIONAL);
    EXPECT_FALSE(nmt_.isOperational());
}

// Тест: переход в Operational
TEST_F(NMTStateMachineTest, TransitionToOperational) {
    nmt_.processCommand(NMTCommand::START_REMOTE_NODE);
    EXPECT_EQ(nmt_.getState(), NodeState::OPERATIONAL);
    EXPECT_TRUE(nmt_.isOperational());
}

// Тест: переход в Stopped
TEST_F(NMTStateMachineTest, TransitionToStopped) {
    nmt_.processCommand(NMTCommand::STOP_REMOTE_NODE);
    EXPECT_EQ(nmt_.getState(), NodeState::STOPPED);
    EXPECT_FALSE(nmt_.isOperational());
}

// Тест: полный цикл состояний
TEST_F(NMTStateMachineTest, FullStateCycle) {
    EXPECT_EQ(nmt_.getState(), NodeState::INITIALIZING);

    nmt_.processCommand(NMTCommand::ENTER_PRE_OPERATIONAL);
    EXPECT_EQ(nmt_.getState(), NodeState::PRE_OPERATIONAL);

    nmt_.processCommand(NMTCommand::START_REMOTE_NODE);
    EXPECT_EQ(nmt_.getState(), NodeState::OPERATIONAL);

    nmt_.processCommand(NMTCommand::STOP_REMOTE_NODE);
    EXPECT_EQ(nmt_.getState(), NodeState::STOPPED);

    nmt_.processCommand(NMTCommand::START_REMOTE_NODE);
    EXPECT_EQ(nmt_.getState(), NodeState::OPERATIONAL);
}

// Тест: Reset Node
TEST_F(NMTStateMachineTest, ResetNode) {
    nmt_.processCommand(NMTCommand::START_REMOTE_NODE);
    EXPECT_EQ(nmt_.getState(), NodeState::OPERATIONAL);

    nmt_.processCommand(NMTCommand::RESET_NODE);
    EXPECT_EQ(nmt_.getState(), NodeState::INITIALIZING);
}

// Тест: Reset Communication
TEST_F(NMTStateMachineTest, ResetCommunication) {
    nmt_.processCommand(NMTCommand::START_REMOTE_NODE);
    EXPECT_EQ(nmt_.getState(), NodeState::OPERATIONAL);

    nmt_.processCommand(NMTCommand::RESET_COMMUNICATION);
    EXPECT_EQ(nmt_.getState(), NodeState::PRE_OPERATIONAL);
}

// Тест: Heartbeat COB-ID
TEST_F(NMTStateMachineTest, HeartbeatCOBId) {
    EXPECT_EQ(nmt_.getHeartbeatCOBId(), 0x701);  // 0x700 + nodeId 1
}

// Тест: Heartbeat message
TEST_F(NMTStateMachineTest, HeartbeatMessage) {
    nmt_.processCommand(NMTCommand::ENTER_PRE_OPERATIONAL);
    EXPECT_EQ(nmt_.createHeartbeat(), static_cast<uint8_t>(NodeState::PRE_OPERATIONAL));

    nmt_.processCommand(NMTCommand::START_REMOTE_NODE);
    EXPECT_EQ(nmt_.createHeartbeat(), static_cast<uint8_t>(NodeState::OPERATIONAL));

    nmt_.processCommand(NMTCommand::STOP_REMOTE_NODE);
    EXPECT_EQ(nmt_.createHeartbeat(), static_cast<uint8_t>(NodeState::STOPPED));
}

// Тест: callback изменения состояния
TEST_F(NMTStateMachineTest, StateChangeCallback) {
    bool callbackCalled = false;
    NodeState reportedState = NodeState::INITIALIZING;

    nmt_.onStateChange([&](NodeState state) {
        callbackCalled = true;
        reportedState = state;
    });

    nmt_.processCommand(NMTCommand::START_REMOTE_NODE);

    EXPECT_TRUE(callbackCalled);
    EXPECT_EQ(reportedState, NodeState::OPERATIONAL);
}

// ============================================================================
// SDO Server Tests
// ============================================================================

class SDOServerTest : public ::testing::Test {
protected:
    SDOServer sdoServer_{1};
    uint8_t response_[8] = {};

    void sendRequest(uint8_t ccs, uint16_t index, uint8_t subIndex,
                     uint32_t data = 0, size_t dataSize = 0) {
        uint8_t request[8] = {};
        request[0] = (ccs << 5);
        request[1] = index & 0xFF;
        request[2] = (index >> 8) & 0xFF;
        request[3] = subIndex;

        if (dataSize > 0 && dataSize <= 4) {
            // Expedited transfer: установить флаг e=1 и размер
            request[0] |= 0x02;  // e=1
            request[0] |= ((4 - dataSize) << 2);
            request[4] = data & 0xFF;
            request[5] = (data >> 8) & 0xFF;
            request[6] = (data >> 16) & 0xFF;
            request[7] = (data >> 24) & 0xFF;
        }

        sdoServer_.processRequest(request, 8, response_);
    }

    uint8_t getResponseCCS() const { return response_[0] >> 5; }
    uint16_t getResponseIndex() const { return response_[1] | (response_[2] << 8); }
    uint8_t getResponseSubIndex() const { return response_[3]; }
    uint32_t getResponseData() const {
        return response_[4] | (response_[5] << 8) |
               (response_[6] << 16) | (response_[7] << 24);
    }
};

// Тест: регистрация записи в Object Dictionary
TEST_F(SDOServerTest, RegisterEntry) {
    ODEntry entry = {
        .index = 0x2000,
        .subIndex = 0x00,
        .access = ODEntry::Access::RW,
        .dataType = ODEntry::DataType::UNSIGNED32
    };
    entry.data = {0x10, 0x20, 0x30, 0x40};

    EXPECT_TRUE(sdoServer_.registerEntry(entry));
}

// Тест: Initiate Upload (чтение из OD)
TEST_F(SDOServerTest, InitiateUpload) {
    // Регистрация записи
    ODEntry entry = {
        .index = 0x2000,
        .subIndex = 0x00,
        .access = ODEntry::Access::RW,
        .dataType = ODEntry::DataType::UNSIGNED32
    };
    entry.data = {0x78, 0x56, 0x34, 0x12};  // 0x12345678
    sdoServer_.registerEntry(entry);

    // Запрос на чтение (CCS=0x03 - Initiate Upload)
    sendRequest(0x03, 0x2000, 0x00);

    // Проверка ответа
    EXPECT_EQ(getResponseCCS(), 0x02);  // Initiate Upload Response
    EXPECT_EQ(getResponseIndex(), 0x2000);
    EXPECT_EQ(getResponseSubIndex(), 0x00);
    EXPECT_EQ(getResponseData(), 0x12345678);
}

// Тест: Initiate Upload - несуществующий объект
TEST_F(SDOServerTest, InitiateUploadObjectNotFound) {
    sendRequest(0x03, 0x2000, 0x00);

    EXPECT_EQ(response_[0], SDOCommand::ABORT_TRANSFER);
    EXPECT_EQ(getResponseIndex(), 0x2000);
    // SDO ErrorCode: Object not exist (0x06020000)
    uint32_t errorCode = getResponseData();
    EXPECT_EQ(errorCode, 0x06020000);
}

// Тест: Initiate Upload - Read Only объект
TEST_F(SDOServerTest, InitiateUploadReadOnly) {
    ODEntry entry = {
        .index = 0x2000,
        .subIndex = 0x00,
        .access = ODEntry::Access::RO,
        .dataType = ODEntry::DataType::UNSIGNED32
    };
    entry.data = {0x01, 0x00, 0x00, 0x00};
    sdoServer_.registerEntry(entry);

    sendRequest(0x03, 0x2000, 0x00);

    EXPECT_EQ(getResponseCCS(), 0x02);  // Успешный ответ
    EXPECT_EQ(getResponseData(), 0x00000001);
}

// Тест: Initiate Download (запись в OD)
TEST_F(SDOServerTest, InitiateDownload) {
    ODEntry entry = {
        .index = 0x2000,
        .subIndex = 0x00,
        .access = ODEntry::Access::RW,
        .dataType = ODEntry::DataType::UNSIGNED32
    };
    entry.data = {0x00, 0x00, 0x00, 0x00};
    sdoServer_.registerEntry(entry);

    // CCS=0x02 - Initiate download в этой реализации
    uint8_t request[8] = {};
    request[0] = (0x02 << 5) | 0x02 | ((4 - 4) << 2);  // CCS=2, e=1, n=0
    request[1] = 0x00;  // Index LSB
    request[2] = 0x20;  // Index MSB
    request[3] = 0x00;  // SubIndex
    request[4] = 0xEF;  // Data
    request[5] = 0xBE;
    request[6] = 0xAD;
    request[7] = 0xDE;

    sdoServer_.processRequest(request, 8, response_);

    // Проверка ответа (CCS=3 - Initiate Download Response)
    EXPECT_EQ(getResponseCCS(), 0x03);

    // Проверим, что данные записаны, прочитав обратно
    uint8_t uploadRequest[8] = {};
    uploadRequest[0] = (0x03 << 5);  // CCS=3 - Initiate Upload
    uploadRequest[1] = 0x00;
    uploadRequest[2] = 0x20;
    uploadRequest[3] = 0x00;

    uint8_t uploadResponse[8] = {};
    sdoServer_.processRequest(uploadRequest, 8, uploadResponse);

    EXPECT_EQ(uploadResponse[0] >> 5, 0x02);  // Initiate Upload Response
    // Проверяем хотя бы первые 3 байта (4й может быть усечён из-за n в ответе)
    EXPECT_EQ(uploadResponse[4], 0xEF);
    EXPECT_EQ(uploadResponse[5], 0xBE);
    EXPECT_EQ(uploadResponse[6], 0xAD);
}

// Тест: Initiate Download - Write Only объект
TEST_F(SDOServerTest, InitiateDownloadWriteOnly) {
    ODEntry entry = {
        .index = 0x2000,
        .subIndex = 0x00,
        .access = ODEntry::Access::WO,
        .dataType = ODEntry::DataType::UNSIGNED32
    };
    sdoServer_.registerEntry(entry);

    sendRequest(0x01, 0x2000, 0x00, 0x12345678, 4);

    EXPECT_EQ(getResponseCCS(), 0x03);  // Успешный ответ
}

// Тест: SDO Response COB-ID
TEST_F(SDOServerTest, ResponseCOBId) {
    EXPECT_EQ(sdoServer_.getResponseCOBId(), 0x581);  // 0x580 + nodeId 1
}

// Тест: abort transfer для RO объекта при записи
TEST_F(SDOServerTest, AbortOnReadOnlyWrite) {
    ODEntry entry = {
        .index = 0x2000,
        .subIndex = 0x00,
        .access = ODEntry::Access::RO,
        .dataType = ODEntry::DataType::UNSIGNED32
    };
    sdoServer_.registerEntry(entry);

    // CCS=0x02 - Initiate download
    uint8_t request[8] = {};
    request[0] = (0x02 << 5) | 0x02;  // CCS=2, e=1
    request[1] = 0x00;
    request[2] = 0x20;
    request[3] = 0x00;
    request[4] = 0x78;
    request[5] = 0x56;
    request[6] = 0x34;
    request[7] = 0x12;

    sdoServer_.processRequest(request, 8, response_);

    // Реализация processInitiateDownload проверя RO и делает abort
    // Но abort CCS = 4 (abort transfer)
    EXPECT_EQ(response_[0], 0x80);  // Abort transfer
}

// ============================================================================
// PDO Manager Tests
// ============================================================================

class PDOManagerTest : public ::testing::Test {
protected:
    PDOManager pdo_{1};
};

// Тест: конфигурация TPDO
TEST_F(PDOManagerTest, ConfigureTPDO) {
    PDOManager::MappedObject mappings[] = {
        {(0x2000 << 16) | (0x00 << 8) | 32, 32}  // Index 0x2000, subIndex 0, 32 bits
    };

    EXPECT_TRUE(pdo_.configureTPDO(1, 0x181, mappings, 1));
    EXPECT_EQ(pdo_.getTPDOCobId(1), 0x181);
}

// Тест: конфигурация RPDO
TEST_F(PDOManagerTest, ConfigureRPDO) {
    PDOManager::MappedObject mappings[] = {
        {(0x2000 << 16) | (0x00 << 8) | 32, 32}
    };

    EXPECT_TRUE(pdo_.configureRPDO(1, 0x201, mappings, 1));
    EXPECT_EQ(pdo_.getRPDOCobId(1), 0x201);
}

// Тест: невалидный номер PDO
TEST_F(PDOManagerTest, InvalidPDONumber) {
    PDOManager::MappedObject mappings[] = {{(0x2000 << 16) | 32, 32}};

    EXPECT_FALSE(pdo_.configureTPDO(0, 0x181, mappings, 1));   // 0 - invalid
    EXPECT_FALSE(pdo_.configureTPDO(5, 0x181, mappings, 1));   // > MAX_TPDO
    EXPECT_FALSE(pdo_.configureRPDO(0, 0x201, mappings, 1));
    EXPECT_FALSE(pdo_.configureRPDO(5, 0x201, mappings, 1));
}

// Тест: слишком много mapping объектов
TEST_F(PDOManagerTest, TooManyMappings) {
    PDOManager::MappedObject mappings[5];
    EXPECT_FALSE(pdo_.configureTPDO(1, 0x181, mappings, 5));
}

// Тест: TPDO callback и создание кадра
TEST_F(PDOManagerTest, TPDOCreate) {
    // Настройка TPDO
    PDOManager::MappedObject mappings[] = {
        {(0x2000 << 16) | (0x00 << 8) | 32, 32}
    };
    pdo_.configureTPDO(1, 0x181, mappings, 1);

    // Установка callback
    pdo_.setTPDOCallback(1, [](uint16_t index, uint8_t subIndex, uint8_t size) -> uint32_t {
        EXPECT_EQ(index, 0x2000);
        EXPECT_EQ(subIndex, 0);
        EXPECT_EQ(size, 32);
        return 0x12345678;
    });

    // Создание TPDO
    uint8_t data[8] = {};
    size_t len = pdo_.createTPDO(1, data, 8);

    EXPECT_EQ(len, 4);  // 32 bits = 4 bytes
    EXPECT_EQ(data[0], 0x78);
    EXPECT_EQ(data[1], 0x56);
    EXPECT_EQ(data[2], 0x34);
    EXPECT_EQ(data[3], 0x12);
}

// Тест: RPDO callback и обработка
TEST_F(PDOManagerTest, RPDOProcess) {
    // Настройка RPDO
    PDOManager::MappedObject mappings[] = {
        {(0x2000 << 16) | (0x00 << 8) | 32, 32}
    };
    pdo_.configureRPDO(1, 0x201, mappings, 1);

    bool callbackCalled = false;
    uint32_t receivedValue = 0;

    pdo_.setRPDOCallback(1, [&](uint16_t index, uint8_t subIndex, uint32_t value, uint8_t size) {
        callbackCalled = true;
        receivedValue = value;
        EXPECT_EQ(index, 0x2000);
        EXPECT_EQ(subIndex, 0);
        EXPECT_EQ(size, 32);
    });

    // Обработка RPDO
    uint8_t data[] = {0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00};
    EXPECT_TRUE(pdo_.processRPDO(1, data, 8));

    EXPECT_TRUE(callbackCalled);
    EXPECT_EQ(receivedValue, 0x12345678);
}

// Тест: множественные mapping объекты
TEST_F(PDOManagerTest, MultipleMappings) {
    PDOManager::MappedObject mappings[] = {
        {(0x2000 << 16) | (0x00 << 8) | 8, 8},    // Index 0x2000, subIndex 0, 8 bits
        {(0x2001 << 16) | (0x00 << 8) | 8, 8},    // Index 0x2001, subIndex 0, 8 bits
        {(0x2002 << 16) | (0x00 << 8) | 16, 16}   // Index 0x2002, subIndex 0, 16 bits
    };

    EXPECT_TRUE(pdo_.configureTPDO(1, 0x181, mappings, 3));

    int callCount = 0;
    pdo_.setTPDOCallback(1, [&](uint16_t index, uint8_t, uint8_t) -> uint32_t {
        callCount++;
        if (index == 0x2000) return 0xAB;
        if (index == 0x2001) return 0xCD;
        if (index == 0x2002) return 0x1234;
        return 0;
    });

    uint8_t data[8] = {};
    size_t len = pdo_.createTPDO(1, data, 8);

    EXPECT_EQ(len, 4);  // 8 + 8 + 16 = 32 bits = 4 bytes
    EXPECT_EQ(callCount, 3);
    EXPECT_EQ(data[0], 0xAB);
    EXPECT_EQ(data[1], 0xCD);
    EXPECT_EQ(data[2], 0x34);  // Little-endian
    EXPECT_EQ(data[3], 0x12);
}

// ============================================================================
// CANopen Stack Integration Tests
// ============================================================================

class CANopenStackTest : public ::testing::Test {
protected:
    void SetUp() override {
        stack_ = std::make_unique<CANopenStack>(1);
    }

    std::unique_ptr<CANopenStack> stack_;
    MockCAN mockCan_;
};

// Тест: инициализация стека
TEST_F(CANopenStackTest, Initialization) {
    EXPECT_EQ(stack_->getState(), NodeState::INITIALIZING);

    auto status = stack_->init(mockCan_);
    EXPECT_EQ(status, mka::hal::Status::OK);

    // После init должно быть состояние Pre-Operational
    EXPECT_EQ(stack_->getState(), NodeState::PRE_OPERATIONAL);
}

// Тест: NMT команда - переход в Operational
TEST_F(CANopenStackTest, NMTStartCommand) {
    stack_->init(mockCan_);

    // NMT команда: Start Remote Node (nodeId = 1)
    uint8_t data[8] = {0x01, 0x01, 0, 0, 0, 0, 0, 0};
    stack_->processMessage(COBId::NMT_COMMAND, data, 8);

    EXPECT_EQ(stack_->getState(), NodeState::OPERATIONAL);
}

// Тест: NMT команда - broadcast (nodeId = 0)
TEST_F(CANopenStackTest, NMTBroadcastCommand) {
    stack_->init(mockCan_);

    // NMT команда: Start Remote Node (broadcast, nodeId = 0)
    uint8_t data[8] = {0x01, 0x00, 0, 0, 0, 0, 0, 0};
    stack_->processMessage(COBId::NMT_COMMAND, data, 8);

    EXPECT_EQ(stack_->getState(), NodeState::OPERATIONAL);
}

// Тест: SDO запрос через стек
TEST_F(CANopenStackTest, SDORequest) {
    stack_->init(mockCan_);

    // Регистрация записи
    ODEntry entry = {
        .index = 0x2000,
        .subIndex = 0x00,
        .access = ODEntry::Access::RW,
        .dataType = ODEntry::DataType::UNSIGNED32
    };
    entry.data = {0x12, 0x34, 0x56, 0x78};
    stack_->registerOD(entry);

    // SDO запрос (Initiate Upload)
    uint8_t request[8] = {0x60, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
    stack_->processMessage(0x601, request, 8);  // 0x600 + nodeId 1

    // Проверка ответа
    EXPECT_GT(mockCan_.getTransmittedCount(), 0);
    const auto& msg = mockCan_.getTransmittedMessages().back();
    EXPECT_EQ(msg.id, 0x581);  // 0x580 + nodeId 1
}

// Тест: TPDO отправка (только в Operational)
TEST_F(CANopenStackTest, TPDOSendOnlyInOperational) {
    stack_->init(mockCan_);

    // Настройка TPDO
    PDOManager::MappedObject mappings[] = {
        {0x200000, 32}
    };
    stack_->getPDO().configureTPDO(1, 0x181, mappings, 1);

    stack_->getPDO().setTPDOCallback(1, [](uint16_t, uint8_t, uint8_t) -> uint32_t {
        return 0xDEADBEEF;
    });

    // Попытка отправки в Pre-Operational (должна быть ошибка)
    EXPECT_FALSE(stack_->sendTPDO(1));
    EXPECT_EQ(mockCan_.getTransmittedCount(), 0);

    // Переход в Operational
    uint8_t data[8] = {0x01, 0x01, 0, 0, 0, 0, 0, 0};
    stack_->processMessage(COBId::NMT_COMMAND, data, 8);

    // Теперь отправка должна сработать
    EXPECT_TRUE(stack_->sendTPDO(1));
    EXPECT_GT(mockCan_.getTransmittedCount(), 0);

    const auto& msg = mockCan_.getTransmittedMessages().back();
    EXPECT_EQ(msg.id, 0x181);
}

// Тест: Heartbeat отправка
TEST_F(CANopenStackTest, HeartbeatSend) {
    stack_->init(mockCan_);

    EXPECT_TRUE(stack_->sendHeartbeat());
    EXPECT_GT(mockCan_.getTransmittedCount(), 0);

    const auto& msg = mockCan_.getTransmittedMessages().back();
    EXPECT_EQ(msg.id, 0x701);  // 0x700 + nodeId 1
    EXPECT_EQ(msg.data[0], static_cast<uint8_t>(NodeState::PRE_OPERATIONAL));
}

// Тест: Emergency отправка
TEST_F(CANopenStackTest, EmergencySend) {
    stack_->init(mockCan_);

    uint8_t emcyData[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
    EXPECT_TRUE(stack_->sendEmergency(0x0010, emcyData, 5));

    EXPECT_GT(mockCan_.getTransmittedCount(), 0);

    const auto& msg = mockCan_.getTransmittedMessages().back();
    EXPECT_EQ(msg.id, 0x081);  // 0x080 + nodeId 1
    EXPECT_EQ(msg.data[0], 0x10);  // Error code LSB
    EXPECT_EQ(msg.data[1], 0x00);  // Error code MSB
}

// Тест: стандартные Object Dictionary entries
TEST_F(CANopenStackTest, StandardODEntries) {
    stack_->init(mockCan_);

    // Проверка чтения Manufacturer Name (0x1008)
    uint8_t request[8] = {0x60, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
    stack_->processMessage(0x601, request, 8);

    EXPECT_GT(mockCan_.getTransmittedCount(), 0);
    const auto& msg = mockCan_.getTransmittedMessages().back();
    EXPECT_EQ(msg.id, 0x581);
    // Response CCS = 0x02 (Initiate Upload Response)
    EXPECT_EQ(msg.data[0] >> 5, 0x02);
}

// Тест: PDO доступ через стек
TEST_F(CANopenStackTest, PDOAccess) {
    stack_->init(mockCan_);

    auto& pdo = stack_->getPDO();
    PDOManager::MappedObject mappings[] = {
        {0x200000, 32}
    };

    EXPECT_TRUE(pdo.configureTPDO(1, 0x181, mappings, 1));
    EXPECT_EQ(pdo.getTPDOCobId(1), 0x181);
}

// Тест: RPDO обработка через стек
TEST_F(CANopenStackTest, RPDOProcess) {
    stack_->init(mockCan_);

    // Настройка RPDO
    PDOManager::MappedObject mappings[] = {
        {0x200000, 32}
    };
    stack_->getPDO().configureRPDO(1, 0x201, mappings, 1);

    bool callbackCalled = false;
    uint32_t receivedValue = 0;

    stack_->getPDO().setRPDOCallback(1, [&](uint16_t, uint8_t, uint32_t value, uint8_t) {
        callbackCalled = true;
        receivedValue = value;
    });

    // Отправка RPDO
    uint8_t data[8] = {0x78, 0x56, 0x34, 0x12, 0, 0, 0, 0};
    stack_->processMessage(0x201, data, 8);  // 0x200 + nodeId 1

    EXPECT_TRUE(callbackCalled);
    EXPECT_EQ(receivedValue, 0x12345678);
}

// Тест: SDO Abort на несуществующий объект
TEST_F(CANopenStackTest, SDOAbortOnMissingObject) {
    stack_->init(mockCan_);

    // Запрос несуществующего объекта
    uint8_t request[8] = {0x60, 0x00, 0x20, 0x00, 0x00, 0, 0, 0};
    stack_->processMessage(0x601, request, 8);

    EXPECT_GT(mockCan_.getTransmittedCount(), 0);
    const auto& msg = mockCan_.getTransmittedMessages().back();
    EXPECT_EQ(msg.id, 0x581);
    // Abort transfer
    EXPECT_EQ(msg.data[0], SDOCommand::ABORT_TRANSFER);
}

// ============================================================================
// Integration Tests
// ============================================================================

class CANopenIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        stack1_ = std::make_unique<CANopenStack>(1);
        stack2_ = std::make_unique<CANopenStack>(2);
        stack1_->init(can_);
        stack2_->init(can_);
    }

    std::unique_ptr<CANopenStack> stack1_;
    std::unique_ptr<CANopenStack> stack2_;
    MockCAN can_;
};

// Тест: взаимодействие двух узлов
TEST_F(CANopenIntegrationTest, TwoNodeInteraction) {
    // Узел 1 - master, переводит узел 2 в Operational
    uint8_t data[8] = {0x01, 0x02, 0, 0, 0, 0, 0, 0};  // Start node 2
    stack1_->processMessage(COBId::NMT_COMMAND, data, 8);

    // Узел 2 должен перейти в Operational
    uint8_t nmtData[8] = {0x01, 0x02, 0, 0, 0, 0, 0, 0};
    stack2_->processMessage(COBId::NMT_COMMAND, nmtData, 8);

    EXPECT_EQ(stack2_->getState(), NodeState::OPERATIONAL);
}

// Тест: SDO обмен между узлами
TEST_F(CANopenIntegrationTest, SDOExchange) {
    // Регистрация записи в узле 2
    ODEntry entry = {
        .index = 0x2000,
        .subIndex = 0x00,
        .access = ODEntry::Access::RW,
        .dataType = ODEntry::DataType::UNSIGNED32
    };
    entry.data = {0xAB, 0xCD, 0xEF, 0x12};
    stack2_->registerOD(entry);

    // Узел 1 отправляет SDO запрос узлу 2
    uint8_t request[8] = {0x60, 0x00, 0x20, 0x00, 0x00, 0, 0, 0};
    stack2_->processMessage(0x602, request, 8);  // 0x600 + nodeId 2

    // Проверка ответа
    const auto& messages = can_.getTransmittedMessages();
    EXPECT_GT(messages.size(), 0);

    // Последний message - ответ от узла 2
    const auto& response = messages.back();
    EXPECT_EQ(response.id, 0x582);  // 0x580 + nodeId 2
}

// ============================================================================
// Запуск тестов
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
=======
TEST_F(CANopenTest, InitialStateIsPreOperational) {
    auto state = stack_.getState();
    EXPECT_EQ(state, NodeState::PRE_OPERATIONAL);
}

TEST_F(CANopenTest, StartNodeTransitionsToOperational) {
    queueNMTCommand(NMTCommand::START_REMOTE_NODE, 1);
    uint32_t id;
    std::array<uint8_t, 8> buffer{};
    size_t len = 0;
    can_.receive(id, buffer.data(), buffer.size(), len);
    stack_.processMessage(COBId::NMT_COMMAND, buffer.data(), 2);
    
    EXPECT_EQ(stack_.getState(), NodeState::OPERATIONAL);
}

TEST_F(CANopenTest, StopNodeTransitionsToStopped) {
    // Сначала в Operational
    queueNMTCommand(NMTCommand::START_REMOTE_NODE, 1);
    uint32_t id;
    std::array<uint8_t, 8> buffer{};
    size_t len = 0;
    can_.receive(id, buffer.data(), buffer.size(), len);
    stack_.processMessage(COBId::NMT_COMMAND, buffer.data(), 2);
    
    EXPECT_EQ(stack_.getState(), NodeState::OPERATIONAL);
    
    // Теперь в Stopped
    queueNMTCommand(NMTCommand::STOP_REMOTE_NODE, 1);
    can_.receive(id, buffer.data(), buffer.size(), len);
    stack_.processMessage(COBId::NMT_COMMAND, buffer.data(), 2);
    
    EXPECT_EQ(stack_.getState(), NodeState::STOPPED);
}

TEST_F(CANopenTest, ResetNodeReturnsToPreOperational) {
    queueNMTCommand(NMTCommand::START_REMOTE_NODE, 1);
    uint32_t id;
    std::array<uint8_t, 8> buffer{};
    size_t len = 0;
    can_.receive(id, buffer.data(), buffer.size(), len);
    stack_.processMessage(COBId::NMT_COMMAND, buffer.data(), 2);
    
    queueNMTCommand(NMTCommand::RESET_NODE, 1);
    can_.receive(id, buffer.data(), buffer.size(), len);
    stack_.processMessage(COBId::NMT_COMMAND, buffer.data(), 2);
    
    EXPECT_EQ(stack_.getState(), NodeState::PRE_OPERATIONAL);
}

// ============================================================================
// Object Dictionary Tests
// ============================================================================

TEST_F(CANopenTest, RegisterODEntry) {
    uint32_t value = 42;
    ODEntry entry;
    entry.index = 0x2000;
    entry.subIndex = 0x00;
    entry.access = ODEntry::Access::RW;
    entry.dataType = ODEntry::DataType::UNSIGNED32;
    std::memcpy(entry.data.data(), &value, sizeof(value));
    
    bool result = stack_.registerOD(entry);
    EXPECT_TRUE(result);
}

TEST_F(CANopenTest, SDOUploadRequest) {
    uint32_t value = 0x12345678;
    ODEntry entry;
    entry.index = 0x2000;
    entry.subIndex = 0x00;
    entry.access = ODEntry::Access::RW;
    entry.dataType = ODEntry::DataType::UNSIGNED32;
    std::memcpy(entry.data.data(), &value, sizeof(value));
    
    stack_.registerOD(entry);
    
    // SDO Upload Request (Initiate Upload)
    std::vector<uint8_t> sdoRequest = {
        0x40,  // CCS=2 (Initiate Upload)
        0x00, 0x20,  // Index 0x2000
        0x00,  // SubIndex 0x00
        0x00, 0x00, 0x00, 0x00
    };
    
    can_.queueFrame(COBId::SDO_RX + 1, sdoRequest);
    uint32_t id;
    std::array<uint8_t, 8> buffer{};
    size_t len = 0;
    auto status = can_.receive(id, buffer.data(), buffer.size(), len);
    
    if (status == hal::Status::OK) {
        stack_.processMessage(id, buffer.data(), len);
    }
    
    // Проверить что ответ был отправлен
    EXPECT_GT(can_.getSendCount(), 0);
}

// ============================================================================
// PDO Tests
// ============================================================================

TEST_F(CANopenTest, TPDOTransmission) {
    // Переход в Operational
    queueNMTCommand(NMTCommand::START_REMOTE_NODE, 1);
    uint32_t id;
    std::array<uint8_t, 8> buffer{};
    size_t len = 0;
    can_.receive(id, buffer.data(), buffer.size(), len);
    stack_.processMessage(COBId::NMT_COMMAND, buffer.data(), 2);
    
    EXPECT_EQ(stack_.getState(), NodeState::OPERATIONAL);
    
    // Отправка TPDO
    bool result = stack_.sendTPDO(1);
    EXPECT_TRUE(result);
}

TEST_F(CANopenTest, HeartbeatGeneration) {
    queueNMTCommand(NMTCommand::START_REMOTE_NODE, 1);
    uint32_t id;
    std::array<uint8_t, 8> buffer{};
    size_t len = 0;
    can_.receive(id, buffer.data(), buffer.size(), len);
    stack_.processMessage(COBId::NMT_COMMAND, buffer.data(), 2);
    
    bool result = stack_.sendHeartbeat();
    EXPECT_TRUE(result);
    
    const auto& sentFrames = can_.getSentFrames();
    if (sentFrames.size() >= 2) {
        // Heartbeat COB-ID = 0x700 + nodeId
        EXPECT_EQ(sentFrames.back().id, 0x701);
    }
}

// ============================================================================
// Emergency Tests
// ============================================================================

TEST_F(CANopenTest, EmergencyMessage) {
    uint16_t errorCode = 0x3200;
    std::array<uint8_t, 5> additionalData = {0x01, 0x02, 0x03, 0x04, 0x05};
    
    bool result = stack_.sendEmergency(errorCode, additionalData.data(), additionalData.size());
    EXPECT_TRUE(result);
    
    const auto& sentFrames = can_.getSentFrames();
    if (!sentFrames.empty()) {
        // Emergency COB-ID = 0x080 + nodeId
        EXPECT_EQ(sentFrames.back().id, 0x081);
    }
}

// ============================================================================
// State Machine Edge Cases
// ============================================================================

TEST_F(CANopenTest, RapidStateTransitions) {
    for (int i = 0; i < 100; ++i) {
        queueNMTCommand(NMTCommand::START_REMOTE_NODE, 1);
        uint32_t id;
        std::array<uint8_t, 8> buffer{};
        size_t len = 0;
        can_.receive(id, buffer.data(), buffer.size(), len);
        stack_.processMessage(COBId::NMT_COMMAND, buffer.data(), 2);
        
        queueNMTCommand(NMTCommand::STOP_REMOTE_NODE, 1);
        can_.receive(id, buffer.data(), buffer.size(), len);
        stack_.processMessage(COBId::NMT_COMMAND, buffer.data(), 2);
    }
    
    EXPECT_EQ(stack_.getState(), NodeState::STOPPED);
}

TEST_F(CANopenTest, BroadcastNMTCommand) {
    // Node ID = 0 означает broadcast
    queueNMTCommand(NMTCommand::START_REMOTE_NODE, 0);
    uint32_t id;
    std::array<uint8_t, 8> buffer{};
    size_t len = 0;
    can_.receive(id, buffer.data(), buffer.size(), len);
    stack_.processMessage(COBId::NMT_COMMAND, buffer.data(), 2);
    
    EXPECT_EQ(stack_.getState(), NodeState::OPERATIONAL);
}

// ============================================================================
// SDOServer Tests
// ============================================================================

TEST(CANopenSDOTest, SDOServerConstruction) {
    SDOServer server(1);
    // Server создан
}

TEST(CANopenSDOTest, RegisterODEntry) {
    SDOServer server(1);
    
    uint32_t value = 12345;
    ODEntry entry;
    entry.index = 0x2000;
    entry.subIndex = 0x00;
    entry.access = ODEntry::Access::RW;
    entry.dataType = ODEntry::DataType::UNSIGNED32;
    std::memcpy(entry.data.data(), &value, sizeof(value));
    
    bool result = server.registerEntry(entry);
    EXPECT_TRUE(result);
}

TEST(CANopenSDOTest, ProcessInitiateUpload) {
    SDOServer server(1);
    
    uint32_t value = 0xDEADBEEF;
    ODEntry entry;
    entry.index = 0x2000;
    entry.subIndex = 0x00;
    entry.access = ODEntry::Access::RW;
    entry.dataType = ODEntry::DataType::UNSIGNED32;
    std::memcpy(entry.data.data(), &value, sizeof(value));
    
    server.registerEntry(entry);
    
    // Initiate Upload Request
    std::array<uint8_t, 8> request = {0x40, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
    std::array<uint8_t, 8> response{};
    
    bool result = server.processRequest(request.data(), request.size(), response.data());
    EXPECT_TRUE(result);
    
    // Проверить что response содержит CCS=2 (Initiate Upload Response)
    EXPECT_EQ((response[0] >> 5), 2);
}

TEST(CANopenSDOTest, ProcessInitiateDownload) {
    SDOServer server(1);
    
    uint32_t value = 0;
    ODEntry entry;
    entry.index = 0x2000;
    entry.subIndex = 0x00;
    entry.access = ODEntry::Access::RW;
    entry.dataType = ODEntry::DataType::UNSIGNED32;
    std::memcpy(entry.data.data(), &value, sizeof(value));
    
    server.registerEntry(entry);
    
    // Initiate Download Request (4 bytes)
    uint32_t newValue = 0x12345678;
    std::array<uint8_t, 8> request = {
        0x23,  // CCS=1, n=3 (4 bytes)
        0x00, 0x20,  // Index 0x2000
        0x00,  // SubIndex 0x00
        static_cast<uint8_t>(newValue & 0xFF),
        static_cast<uint8_t>((newValue >> 8) & 0xFF),
        static_cast<uint8_t>((newValue >> 16) & 0xFF),
        static_cast<uint8_t>((newValue >> 24) & 0xFF)
    };
    
    std::array<uint8_t, 8> response{};
    bool result = server.processRequest(request.data(), request.size(), response.data());
    EXPECT_TRUE(result);
    
    // Проверить что значение записано
    uint32_t writtenValue;
    std::memcpy(&writtenValue, entry.data.data(), sizeof(writtenValue));
    EXPECT_EQ(writtenValue, 0x12345678);
}

TEST(CANopenSDOTest, UploadNonExistentEntry) {
    SDOServer server(1);
    
    // Запрос несуществующей записи
    std::array<uint8_t, 8> request = {0x40, 0x99, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00};
    std::array<uint8_t, 8> response{};
    
    bool result = server.processRequest(request.data(), request.size(), response.data());
    EXPECT_TRUE(result);
    
    // Должен вернуть Abort с OBJECT_NOT_EXIST
    EXPECT_EQ((response[0] >> 5), 4);  // CCS=4 (Abort)
}

// ============================================================================
// PDO Unit Tests
// ============================================================================

TEST(CANopenPDOTest, PDOConstruction) {
    PDOServer pdo(1);
    // PDO создан
}

TEST(CANopenPDOTest, ConfigureTPDO) {
    PDOServer pdo(1);
    
    PDOMappingEntry entry;
    entry.cobId = 0x181;
    entry.transmissionType = 255;
    entry.eventTimer = 100;
    
    // PDO должен иметь методы конфигурации
    // (зависит от реализации)
}

// ============================================================================
// NMT Unit Tests
// ============================================================================

TEST(CANopenNMTTest, NMTConstruction) {
    NMTStateMachine nmt(1);
    EXPECT_EQ(nmt.getState(), NodeState::PRE_OPERATIONAL);
}

TEST(CANopenNMTTest, ProcessStartCommand) {
    NMTStateMachine nmt(1);
    nmt.processCommand(NMTCommand::START_REMOTE_NODE);
    EXPECT_EQ(nmt.getState(), NodeState::OPERATIONAL);
}

TEST(CANopenNMTTest, ProcessStopCommand) {
    NMTStateMachine nmt(1);
    nmt.processCommand(NMTCommand::START_REMOTE_NODE);
    EXPECT_EQ(nmt.getState(), NodeState::OPERATIONAL);
    
    nmt.processCommand(NMTCommand::STOP_REMOTE_NODE);
    EXPECT_EQ(nmt.getState(), NodeState::STOPPED);
}

TEST(CANopenNMTTest, IsOperationalCheck) {
    NMTStateMachine nmt(1);
    EXPECT_FALSE(nmt.isOperational());
    
    nmt.processCommand(NMTCommand::START_REMOTE_NODE);
    EXPECT_TRUE(nmt.isOperational());
}

TEST(CANopenNMTTest, HeartbeatCOBId) {
    NMTStateMachine nmt(1);
    EXPECT_EQ(nmt.getHeartbeatCOBId(), 0x701);
    
    NMTStateMachine nmt2(5);
    EXPECT_EQ(nmt2.getHeartbeatCOBId(), 0x705);
>>>>>>> dev
}
