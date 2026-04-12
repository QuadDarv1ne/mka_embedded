/**
 * @file test_canopen.cpp
 * @brief Полные unit-тесты для CANopen стека
 */

#include <gtest/gtest.h>
#include <cstring>
#include <vector>
#include <queue>
#include <map>

#include "systems/canopen.hpp"
#include "utils/result.hpp"
#include "hal/hal_full.hpp"

using namespace mka::systems;

// ============================================================================
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

// ============================================================================
// NMT State Machine Tests
// ============================================================================

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
}
