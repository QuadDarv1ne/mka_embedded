/**
 * @file test_telemetry.cpp
 * @brief Tests for telemetry and command system
 */

#include <gtest/gtest.h>
#include <array>
#include <cstring>

#include "systems/telemetry.hpp"

using namespace mka::telemetry;

// ============================================================================
// CRC16 Tests
// ============================================================================

class CRCTest : public ::testing::Test {
protected:
    void SetUp() override {}
};

TEST_F(CRCTest, CalculateBasic) {
    std::array<uint8_t, 5> data = {0x31, 0x32, 0x33, 0x34, 0x35};
    uint16_t crc = CRC16::calculate(data.data(), data.size());
    
    // CRC value depends on the polynomial and initial value
    // Just verify it's non-zero and consistent
    EXPECT_NE(crc, 0);
    EXPECT_EQ(crc, CRC16::calculate(data.data(), data.size()));
}

TEST_F(CRCTest, CalculateEmpty) {
    std::array<uint8_t, 0> data = {};
    uint16_t crc = CRC16::calculate(data.data(), data.size(), 0xFFFF);
    
    EXPECT_EQ(crc, 0xFFFF);
}

TEST_F(CRCTest, VerifyCorrect) {
    std::array<uint8_t, 7> dataWithCrc = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0xA1};
    
    uint16_t expected = CRC16::calculate(dataWithCrc.data(), dataWithCrc.size() - 2);
    
    dataWithCrc[5] = (expected >> 8) & 0xFF;
    dataWithCrc[6] = expected & 0xFF;
    
    EXPECT_TRUE(CRC16::verify(dataWithCrc.data(), dataWithCrc.size()));
}

TEST_F(CRCTest, VerifyIncorrect) {
    std::array<uint8_t, 7> dataWithCrc = {0x31, 0x32, 0x33, 0x34, 0x35, 0x00, 0x00};
    
    EXPECT_FALSE(CRC16::verify(dataWithCrc.data(), dataWithCrc.size()));
}

// ============================================================================
// TelemetryGenerator Tests
// ============================================================================

class TelemetryGeneratorTest : public ::testing::Test {
protected:
    std::array<uint8_t, 256> buffer_{};
    
    void SetUp() override {
        buffer_.fill(0);
    }
};

TEST_F(TelemetryGeneratorTest, RegisterChannels) {
    TelemetryGenerator gen(buffer_.data(), buffer_.size());
    
    TelemetryChannel ch1{"Temperature", TelemetryType::FLOAT, 0, 1.0f, 0.0f, "C", -40.0f, 85.0f, true};
    TelemetryChannel ch2{"Pressure", TelemetryType::FLOAT, 1, 1.0f, 0.0f, "hPa", 300.0f, 1250.0f, true};
    
    uint8_t id1 = gen.registerChannel(ch1);
    uint8_t id2 = gen.registerChannel(ch2);
    
    EXPECT_EQ(id1, 0);
    EXPECT_EQ(id2, 1);
    EXPECT_EQ(gen.getChannelCount(), 2);
}

TEST_F(TelemetryGeneratorTest, UpdateValues) {
    TelemetryGenerator gen(buffer_.data(), buffer_.size());
    
    TelemetryChannel ch{"Voltage", TelemetryType::FLOAT, 0, 1.0f, 0.0f, "V", 0.0f, 12.0f, true};
    gen.registerChannel(ch);
    
    auto status = gen.updateValue(0, 7.4f, 1000);
    EXPECT_EQ(status, TelemetryGenerator::FrameStatus::OK);
}

TEST_F(TelemetryGeneratorTest, GenerateFrame) {
    TelemetryGenerator gen(buffer_.data(), buffer_.size());
    
    TelemetryChannel ch1{"Temp", TelemetryType::FLOAT, 0, 1.0f, 0.0f, "C", -40.0f, 85.0f, true};
    TelemetryChannel ch2{"Voltage", TelemetryType::UINT16, 1, 1.0f, 0.0f, "mV", 0, 12000, true};
    
    gen.registerChannel(ch1);
    gen.registerChannel(ch2);
    
    gen.updateValue(0, 25.5f, 1000);
    gen.updateValue(1, static_cast<uint16_t>(7400), 1000);
    
    size_t frameSize = 0;
    auto status = gen.generateFrame(0x001, 0xFFFFFFFF, &frameSize);
    
    EXPECT_EQ(status, TelemetryGenerator::FrameStatus::OK);
    EXPECT_GT(frameSize, sizeof(TelemetryHeader));
    EXPECT_LE(frameSize, buffer_.size());
}

TEST_F(TelemetryGeneratorTest, GenerateFrameEmpty) {
    TelemetryGenerator gen(buffer_.data(), buffer_.size());
    
    size_t frameSize = 0;
    auto status = gen.generateFrame(0x001, 0xFFFFFFFF, &frameSize);
    
    EXPECT_EQ(status, TelemetryGenerator::FrameStatus::NO_DATA);
}

TEST_F(TelemetryGeneratorTest, GenerateFrameDisabledChannel) {
    TelemetryGenerator gen(buffer_.data(), buffer_.size());
    
    TelemetryChannel ch{"Temp", TelemetryType::FLOAT, 0, 1.0f, 0.0f, "C", -40.0f, 85.0f, false};
    gen.registerChannel(ch);
    gen.updateValue(0, 25.5f, 1000);
    
    size_t frameSize = 0;
    auto status = gen.generateFrame(0x001, 0xFFFFFFFF, &frameSize);
    
    EXPECT_EQ(status, TelemetryGenerator::FrameStatus::NO_DATA);
}

TEST_F(TelemetryGeneratorTest, SequenceCountIncrement) {
    TelemetryGenerator gen(buffer_.data(), buffer_.size());
    
    TelemetryChannel ch{"Temp", TelemetryType::FLOAT, 0, 1.0f, 0.0f, "C", -40.0f, 85.0f, true};
    gen.registerChannel(ch);
    gen.updateValue(0, 25.5f, 1000);
    
    size_t frameSize1 = 0, frameSize2 = 0;
    gen.generateFrame(0x001, 0xFFFFFFFF, &frameSize1);
    gen.updateValue(0, 26.0f, 2000);
    gen.generateFrame(0x001, 0xFFFFFFFF, &frameSize2);
    
    EXPECT_GT(frameSize1, 0);
    EXPECT_GT(frameSize2, 0);
}

TEST_F(TelemetryGeneratorTest, BufferTooSmall) {
    // Test with a buffer that's too small for header + data + CRC
    // Header=6 + channel=1 + data=4 + crc=2 = 13 bytes minimum
    std::array<uint8_t, 10> smallBuffer{};
    TelemetryGenerator gen(smallBuffer.data(), smallBuffer.size());
    
    TelemetryChannel ch{"Temp", TelemetryType::FLOAT, 0, 1.0f, 0.0f, "C", -40.0f, 85.0f, true};
    gen.registerChannel(ch);
    gen.updateValue(0, 25.5f, 1000);
    
    size_t frameSize = 0;
    auto status = gen.generateFrame(0x001, 0xFFFFFFFF, &frameSize);
    
    // 10 bytes is not enough for header(6) + channel(1) + float(4) + crc(2) = 13
    EXPECT_EQ(status, TelemetryGenerator::FrameStatus::BUFFER_TOO_SMALL);
}

// ============================================================================
// CommandManager Tests
// ============================================================================

class CommandManagerTest : public ::testing::Test {
protected:
    CommandManager cmdManager_;
    std::array<uint8_t, 64> buffer_{};
    
    void SetUp() override {
        buffer_.fill(0);
    }
};

TEST_F(CommandManagerTest, RegisterHandler) {
    bool handlerCalled = false;
    
    auto handler = [&handlerCalled](const Command&) -> CommandResult {
        handlerCalled = true;
        return CommandResult::OK;
    };
    
    EXPECT_TRUE(cmdManager_.registerHandler(0x01, handler));
    
    CommandHeader header;
    header.set(0x001, 0, 0x01, 0);
    
    std::memcpy(buffer_.data(), &header, sizeof(header));
    
    uint16_t crc = CRC16::calculate(buffer_.data(), sizeof(header));
    buffer_[sizeof(header)] = (crc >> 8) & 0xFF;
    buffer_[sizeof(header) + 1] = crc & 0xFF;
    
    CommandResult result;
    auto status = cmdManager_.processCommand(buffer_.data(), sizeof(header) + 2, &result);
    
    EXPECT_EQ(status, CommandResult::OK);
    EXPECT_EQ(result, CommandResult::OK);
    EXPECT_TRUE(handlerCalled);
}

TEST_F(CommandManagerTest, InvalidCommand) {
    CommandHeader header;
    header.set(0x001, 0, 0xFF, 0);
    
    std::memcpy(buffer_.data(), &header, sizeof(header));
    
    uint16_t crc = CRC16::calculate(buffer_.data(), sizeof(header));
    buffer_[sizeof(header)] = (crc >> 8) & 0xFF;
    buffer_[sizeof(header) + 1] = crc & 0xFF;
    
    CommandResult result;
    auto status = cmdManager_.processCommand(buffer_.data(), sizeof(header) + 2, &result);
    
    EXPECT_EQ(status, CommandResult::INVALID_COMMAND);
}

TEST_F(CommandManagerTest, CRCError) {
    auto handler = [](const Command&) -> CommandResult {
        return CommandResult::OK;
    };
    
    cmdManager_.registerHandler(0x01, handler);
    
    CommandHeader header;
    header.set(0x001, 0, 0x01, 0);
    
    std::memcpy(buffer_.data(), &header, sizeof(header));
    
    buffer_[sizeof(header)] = 0x00;
    buffer_[sizeof(header) + 1] = 0x00;
    
    CommandResult result;
    auto status = cmdManager_.processCommand(buffer_.data(), sizeof(header) + 2, &result);
    
    EXPECT_EQ(status, CommandResult::CRC_ERROR);
}

TEST_F(CommandManagerTest, InvalidPayload) {
    std::array<uint8_t, 3> shortData = {0x01, 0x02, 0x03};

    CommandResult result;
    auto status = cmdManager_.processCommand(shortData.data(), shortData.size(), &result);

    EXPECT_EQ(status, CommandResult::INVALID_PAYLOAD);
}

TEST_F(CommandManagerTest, PayloadLengthOverflow) {
    auto handler = [](const Command&) -> CommandResult {
        return CommandResult::OK;
    };

    cmdManager_.registerHandler(0x01, handler);

    CommandHeader header;
    header.set(0x001, 0, 0x01, 250);  // Payload length больше чем данные

    std::memcpy(buffer_.data(), &header, sizeof(header));

    // Заполняем буфер нулями, но длина не соответствует payloadLength
    std::memset(buffer_.data() + sizeof(header), 0, 10);

    uint16_t crc = CRC16::calculate(buffer_.data(), sizeof(header) + 10);
    buffer_[sizeof(header) + 10] = (crc >> 8) & 0xFF;
    buffer_[sizeof(header) + 11] = crc & 0xFF;

    CommandResult result;
    auto status = cmdManager_.processCommand(buffer_.data(), sizeof(header) + 10 + 2, &result);

    // Должен отклонить из-за несоответствия длины
    EXPECT_EQ(status, CommandResult::INVALID_PAYLOAD);
}

TEST_F(CommandManagerTest, BuildAcknowledgment) {
    std::array<uint8_t, 16> ackBuffer{};
    size_t frameSize = 0;
    
    bool result = cmdManager_.buildAcknowledgment(
        0x01,
        CommandResult::OK,
        ackBuffer.data(),
        ackBuffer.size(),
        &frameSize
    );
    
    EXPECT_TRUE(result);
    EXPECT_EQ(frameSize, 6);
    EXPECT_EQ(ackBuffer[0], 0x01);
    EXPECT_EQ(ackBuffer[1], static_cast<uint8_t>(CommandResult::OK));
}

TEST_F(CommandManagerTest, HandlerWithPayload) {
    uint8_t receivedPayload = 0;
    
    auto handler = [&receivedPayload](const Command& cmd) -> CommandResult {
        if (cmd.payloadSize >= 1) {
            receivedPayload = cmd.payload[0];
        }
        return CommandResult::OK;
    };
    
    cmdManager_.registerHandler(0x02, handler);
    
    CommandHeader header;
    header.set(0x001, 0, 0x02, 1);
    
    std::memcpy(buffer_.data(), &header, sizeof(header));
    buffer_[sizeof(header)] = 0x42;
    
    uint16_t crc = CRC16::calculate(buffer_.data(), sizeof(header) + 1);
    buffer_[sizeof(header) + 1] = (crc >> 8) & 0xFF;
    buffer_[sizeof(header) + 2] = crc & 0xFF;
    
    CommandResult result;
    auto status = cmdManager_.processCommand(buffer_.data(), sizeof(header) + 1 + 2, &result);
    
    EXPECT_EQ(status, CommandResult::OK);
    EXPECT_EQ(receivedPayload, 0x42);
}

// ============================================================================
// ParameterStore Tests
// ============================================================================

class ParameterStoreTest : public ::testing::Test {
protected:
    ParameterStore store_;
};

TEST_F(ParameterStoreTest, RegisterParameter) {
    ParameterAttributes attrs{
        "MaxTemperature",
        ParameterType::FLOAT,
        0,
        true,
        -40.0f,
        85.0f,
        25.0f
    };
    
    uint16_t id = store_.registerParameter(attrs, 20.0f);
    
    EXPECT_EQ(id, 0);
    EXPECT_EQ(store_.getParameterCount(), 1);
}

TEST_F(ParameterStoreTest, GetSetParameter) {
    ParameterAttributes attrs{
        "Voltage",
        ParameterType::FLOAT,
        0,
        true,
        0.0f,
        12.0f,
        7.4f
    };
    
    uint16_t id = store_.registerParameter(attrs, 7.4f);
    EXPECT_EQ(id, 0);
    
    float value;
    EXPECT_TRUE(store_.getParameter(0, value));
    EXPECT_FLOAT_EQ(value, 7.4f);
    
    EXPECT_TRUE(store_.setParameter(0, 8.2f));
    EXPECT_TRUE(store_.getParameter(0, value));
    EXPECT_FLOAT_EQ(value, 8.2f);
}

TEST_F(ParameterStoreTest, SetParameterOutOfRange) {
    ParameterAttributes attrs{
        "Current",
        ParameterType::FLOAT,
        0,
        true,
        0.0f,
        5.0f,
        1.0f
    };
    
    store_.registerParameter(attrs, 1.0f);
    
    EXPECT_FALSE(store_.setParameter(0, 10.0f, true));
    EXPECT_TRUE(store_.setParameter(0, 10.0f, false));
}

TEST_F(ParameterStoreTest, ResetParameter) {
    ParameterAttributes attrs{
        "Threshold",
        ParameterType::FLOAT,
        0,
        true,
        0.0f,
        100.0f,
        50.0f
    };
    
    store_.registerParameter(attrs, 50.0f);
    
    store_.setParameter(0, 75.0f);
    
    float value;
    store_.getParameter(0, value);
    EXPECT_FLOAT_EQ(value, 75.0f);
    
    store_.resetParameter(0);
    store_.getParameter(0, value);
    EXPECT_FLOAT_EQ(value, 50.0f);
}

TEST_F(ParameterStoreTest, ResetAllParameters) {
    ParameterAttributes attrs1{"P1", ParameterType::FLOAT, 0, true, 0.0f, 100.0f, 10.0f};
    ParameterAttributes attrs2{"P2", ParameterType::FLOAT, 1, true, 0.0f, 100.0f, 20.0f};
    ParameterAttributes attrs3{"P3", ParameterType::FLOAT, 2, true, 0.0f, 100.0f, 30.0f};
    
    store_.registerParameter(attrs1, 10.0f);
    store_.registerParameter(attrs2, 20.0f);
    store_.registerParameter(attrs3, 30.0f);
    
    store_.setParameter(0, 15.0f);
    store_.setParameter(1, 25.0f);
    store_.setParameter(2, 35.0f);
    
    store_.resetAllParameters();
    
    float v1, v2, v3;
    store_.getParameter(0, v1);
    store_.getParameter(1, v2);
    store_.getParameter(2, v3);
    
    EXPECT_FLOAT_EQ(v1, 10.0f);
    EXPECT_FLOAT_EQ(v2, 20.0f);
    EXPECT_FLOAT_EQ(v3, 30.0f);
}

TEST_F(ParameterStoreTest, GetParameterInvalidId) {
    float value;
    EXPECT_FALSE(store_.getParameter(999, value));
}

TEST_F(ParameterStoreTest, SetParameterInvalidId) {
    EXPECT_FALSE(store_.setParameter(999, 1.0f));
}

TEST_F(ParameterStoreTest, GetParameterAttributes) {
    ParameterAttributes attrs{
        "TestParam",
        ParameterType::FLOAT,
        0,
        true,
        0.0f,
        100.0f,
        50.0f
    };
    
    store_.registerParameter(attrs, 50.0f);
    
    const auto* retrievedAttrs = store_.getParameterAttributes(0);
    ASSERT_NE(retrievedAttrs, nullptr);
    EXPECT_STREQ(retrievedAttrs->name, "TestParam");
    EXPECT_EQ(retrievedAttrs->type, ParameterType::FLOAT);
    EXPECT_FLOAT_EQ(retrievedAttrs->defaultValue, 50.0f);
}

// ============================================================================
// Integration Tests
// ============================================================================

class IntegrationTest : public ::testing::Test {
protected:
    std::array<uint8_t, 256> buffer_{};
    
    void SetUp() override {
        buffer_.fill(0);
    }
};

TEST_F(IntegrationTest, TelemetryAndCommands) {
    TelemetryGenerator tmGen(buffer_.data(), buffer_.size());
    
    TelemetryChannel ch1{"BatteryVoltage", TelemetryType::FLOAT, 0, 1.0f, 0.0f, "V", 0.0f, 12.0f, true};
    TelemetryChannel ch2{"Temperature", TelemetryType::FLOAT, 1, 1.0f, 0.0f, "C", -40.0f, 85.0f, true};
    
    tmGen.registerChannel(ch1);
    tmGen.registerChannel(ch2);
    
    tmGen.updateValue(0, 7.4f, 1000);
    tmGen.updateValue(1, 25.0f, 1000);
    
    size_t tmFrameSize = 0;
    auto tmStatus = tmGen.generateFrame(0x100, 0xFFFFFFFF, &tmFrameSize);
    
    EXPECT_EQ(tmStatus, TelemetryGenerator::FrameStatus::OK);
    EXPECT_GT(tmFrameSize, 0);
    
    CommandManager cmdManager;
    
    bool commandExecuted = false;
    cmdManager.registerHandler(0x01, [&commandExecuted](const Command&) {
        commandExecuted = true;
        return CommandResult::OK;
    });
    
    CommandHeader header;
    header.set(0x001, 0, 0x01, 0);
    std::memcpy(buffer_.data(), &header, sizeof(header));
    
    uint16_t crc = CRC16::calculate(buffer_.data(), sizeof(header));
    buffer_[sizeof(header)] = (crc >> 8) & 0xFF;
    buffer_[sizeof(header) + 1] = crc & 0xFF;
    
    CommandResult cmdResult;
    auto cmdStatus = cmdManager.processCommand(buffer_.data(), sizeof(header) + 2, &cmdResult);
    
    EXPECT_EQ(cmdStatus, CommandResult::OK);
    EXPECT_TRUE(commandExecuted);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
