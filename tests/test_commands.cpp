/**
 * @file test_commands.cpp
 * @brief Unit tests for Command Handler system
 */

#include <gtest/gtest.h>
#include <cstring>

#include "systems/command_handler.hpp"

using namespace mka::cmd;

// ============================================================================
// Вспомогательные функции
// ============================================================================

namespace {

Command createTestCommand(uint16_t cmdId, uint16_t seqNum, 
                          const uint8_t* data = nullptr, size_t dataLen = 0) {
    Command cmd{};
    cmd.header.commandId = cmdId;
    cmd.header.sequenceNumber = seqNum;
    cmd.header.priority = static_cast<uint8_t>(CommandPriority::NORMAL);
    cmd.header.flags = 0;
    cmd.header.dataLength = static_cast<uint16_t>(dataLen);
    cmd.valid = true;
    cmd.timestamp = 0;
    cmd.source = 0;
    
    if (data && dataLen > 0) {
        std::memcpy(cmd.data.data(), data, std::min(dataLen, cmd.data.size()));
    }
    
    return cmd;
}

} // namespace

// ============================================================================
// Тесты ResultCode
// ============================================================================

TEST(ResultCodeTest, SuccessValues) {
    EXPECT_EQ(static_cast<uint8_t>(ResultCode::OK), 0u);
    EXPECT_EQ(static_cast<uint8_t>(ResultCode::EXECUTED), 1u);
    EXPECT_EQ(static_cast<uint8_t>(ResultCode::QUEUED), 2u);
    EXPECT_EQ(static_cast<uint8_t>(ResultCode::BUSY), 3u);
}

TEST(ResultCodeTest, ErrorValues) {
    EXPECT_EQ(static_cast<uint8_t>(ResultCode::UNKNOWN_COMMAND), 0x80u);
    EXPECT_EQ(static_cast<uint8_t>(ResultCode::INVALID_PARAMS), 0x81u);
    EXPECT_EQ(static_cast<uint8_t>(ResultCode::INVALID_LENGTH), 0x82u);
    EXPECT_EQ(static_cast<uint8_t>(ResultCode::CHECKSUM_ERROR), 0x83u);
}

// ============================================================================
// Тесты CommandHeader
// ============================================================================

TEST(CommandHeaderTest, DefaultInitialization) {
    CommandHeader header{};
    EXPECT_EQ(header.commandId, 0u);
    EXPECT_EQ(header.sequenceNumber, 0u);
    EXPECT_EQ(header.priority, 0u);
    EXPECT_EQ(header.flags, 0u);
    EXPECT_EQ(header.dataLength, 0u);
}

TEST(CommandHeaderTest, Assignment) {
    CommandHeader header{};
    header.commandId = 0x0100;
    header.sequenceNumber = 42;
    header.priority = 2;
    header.flags = 0x01;
    header.dataLength = 10;
    
    EXPECT_EQ(header.commandId, 0x0100u);
    EXPECT_EQ(header.sequenceNumber, 42u);
    EXPECT_EQ(header.priority, 2u);
    EXPECT_EQ(header.dataLength, 10u);
}

// ============================================================================
// Тесты Command
// ============================================================================

TEST(CommandTest, DefaultInitialization) {
    Command cmd{};
    EXPECT_EQ(cmd.header.commandId, 0u);
    EXPECT_FALSE(cmd.valid);
    EXPECT_EQ(cmd.timestamp, 0u);
}

TEST(CommandTest, DataCopy) {
    Command cmd{};
    uint8_t testData[] = {0x01, 0x02, 0x03, 0x04};
    cmd.header.dataLength = 4;
    std::memcpy(cmd.data.data(), testData, 4);
    
    EXPECT_EQ(cmd.data[0], 0x01);
    EXPECT_EQ(cmd.data[3], 0x04);
}

// ============================================================================
// Тесты CommandResponse
// ============================================================================

TEST(CommandResponseTest, DefaultInitialization) {
    CommandResponse resp{};
    EXPECT_EQ(resp.commandId, 0u);
    EXPECT_EQ(resp.sequenceNumber, 0u);
    // resultCode по умолчанию 0 (OK), а не UNKNOWN_COMMAND
    EXPECT_EQ(resp.responseLength, 0u);
    EXPECT_EQ(resp.executionTimeMs, 0u);
}

// ============================================================================
// Тесты CommandManager - базовые
// ============================================================================

TEST(CommandManagerTest, InitialState) {
    CommandManager manager;
    EXPECT_FALSE(manager.hasCommand(commands::NOOP));
    EXPECT_STREQ(manager.getCommandName(0xFFFF), "UNKNOWN");
}

TEST(CommandManagerTest, RegisterCommand) {
    CommandManager manager;
    
    CommandRegistration reg{};
    reg.commandId = commands::NOOP;
    reg.name = "NOOP";
    reg.handler = createNoopHandler();
    reg.timeoutMs = 1000;
    reg.requiresAuth = false;
    reg.minParams = 0;
    reg.maxParams = 0;
    
    EXPECT_TRUE(manager.registerCommand(reg));
    EXPECT_TRUE(manager.hasCommand(commands::NOOP));
    EXPECT_STREQ(manager.getCommandName(commands::NOOP), "NOOP");
}

TEST(CommandManagerTest, RegisterDuplicateCommand) {
    CommandManager manager;
    
    CommandRegistration reg{};
    reg.commandId = commands::NOOP;
    reg.name = "NOOP";
    reg.handler = createNoopHandler();
    reg.timeoutMs = 1000;
    reg.requiresAuth = false;
    reg.minParams = 0;
    reg.maxParams = 0;
    
    EXPECT_TRUE(manager.registerCommand(reg));
    
    // Обновление существующей команды
    reg.name = "NOOP_UPDATED";
    EXPECT_TRUE(manager.registerCommand(reg));
    EXPECT_STREQ(manager.getCommandName(commands::NOOP), "NOOP_UPDATED");
}

// ============================================================================
// Тесты CommandManager - обработка команд
// ============================================================================

TEST(CommandManagerTest, ProcessNoopCommand) {
    CommandManager manager;
    
    CommandRegistration reg{};
    reg.commandId = commands::NOOP;
    reg.name = "NOOP";
    reg.handler = createNoopHandler();
    reg.timeoutMs = 1000;
    reg.requiresAuth = false;
    reg.minParams = 0;
    reg.maxParams = 0;
    
    manager.registerCommand(reg);
    
    Command cmd = createTestCommand(commands::NOOP, 1);
    CommandResponse resp = manager.processCommand(cmd);
    
    EXPECT_EQ(resp.commandId, commands::NOOP);
    EXPECT_EQ(resp.sequenceNumber, 1u);
    EXPECT_EQ(resp.resultCode, ResultCode::OK);
    EXPECT_EQ(resp.responseLength, 2u);
    EXPECT_EQ(resp.data[0], 'O');
    EXPECT_EQ(resp.data[1], 'K');
}

TEST(CommandManagerTest, ProcessUnknownCommand) {
    CommandManager manager;
    
    Command cmd = createTestCommand(0xFFFF, 1);
    CommandResponse resp = manager.processCommand(cmd);
    
    EXPECT_EQ(resp.commandId, 0xFFFFu);
    EXPECT_EQ(resp.resultCode, ResultCode::UNKNOWN_COMMAND);
}

TEST(CommandManagerTest, ProcessCommandWithInvalidLength) {
    CommandManager manager;
    
    auto handler = +[](const Command&, CommandResponse&) -> ResultCode {
        return ResultCode::OK;
    };
    
    CommandRegistration reg{};
    reg.commandId = 0x0100;
    reg.name = "TEST";
    reg.handler = handler;
    reg.timeoutMs = 1000;
    reg.requiresAuth = false;
    reg.minParams = 5;  // Минимум 5 байт
    reg.maxParams = 10;
    
    manager.registerCommand(reg);
    
    // Слишком мало данных
    Command cmd = createTestCommand(0x0100, 1, nullptr, 2);
    CommandResponse resp = manager.processCommand(cmd);
    
    EXPECT_EQ(resp.resultCode, ResultCode::INVALID_LENGTH);
}

// ============================================================================
// Тесты CommandManager - валидация параметров
// ============================================================================

TEST(CommandManagerTest, ProcessCommandWithValidator) {
    CommandManager manager;
    
    auto validator = +[](const uint8_t* params, size_t size) -> bool {
        return size >= 2 && params[0] > 0;
    };
    
    auto handler = +[](const Command&, CommandResponse&) -> ResultCode {
        return ResultCode::OK;
    };
    
    CommandRegistration reg{};
    reg.commandId = 0x0100;
    reg.name = "TEST";
    reg.handler = handler;
    reg.validator = validator;
    reg.timeoutMs = 1000;
    reg.requiresAuth = false;
    reg.minParams = 1;
    reg.maxParams = 10;
    
    manager.registerCommand(reg);
    
    // Валидные параметры
    uint8_t validData[] = {0x01, 0x02};
    Command validCmd = createTestCommand(0x0100, 1, validData, 2);
    CommandResponse validResp = manager.processCommand(validCmd);
    EXPECT_EQ(validResp.resultCode, ResultCode::OK);
    
    // Невалидные параметры (первый байт = 0)
    uint8_t invalidData[] = {0x00, 0x02};
    Command invalidCmd = createTestCommand(0x0100, 2, invalidData, 2);
    CommandResponse invalidResp = manager.processCommand(invalidCmd);
    EXPECT_EQ(invalidResp.resultCode, ResultCode::INVALID_PARAMS);
}

// ============================================================================
// Тесты CommandManager - обработчик исключений
// ============================================================================

TEST(CommandManagerTest, ProcessCommandWithException) {
    CommandManager manager;
    
    auto throwingHandler = +[](const Command&, CommandResponse&) -> ResultCode {
        throw 42;  // Бросаем исключение
    };
    
    CommandRegistration reg{};
    reg.commandId = 0x0100;
    reg.name = "THROW";
    reg.handler = throwingHandler;
    reg.timeoutMs = 1000;
    reg.requiresAuth = false;
    reg.minParams = 0;
    reg.maxParams = 0;
    
    manager.registerCommand(reg);
    
    Command cmd = createTestCommand(0x0100, 1);
    CommandResponse resp = manager.processCommand(cmd);
    
    EXPECT_EQ(resp.resultCode, ResultCode::EXECUTION_ERROR);
}

// ============================================================================
// Тесты CommandManager - timestamp
// ============================================================================

TEST(CommandManagerTest, ExecutionTimeMeasurement) {
    CommandManager manager;
    
    // Используем статическую переменную для времени
    static uint32_t s_currentTime = 0;
    s_currentTime = 1000;
    
    manager.setTimestampSource(+[]() -> uint32_t {
        return s_currentTime;
    });
    
    auto slowHandler = +[](const Command&, CommandResponse&) -> ResultCode {
        s_currentTime = 1050;  // Имитация задержки 50ms
        return ResultCode::OK;
    };
    
    CommandRegistration reg{};
    reg.commandId = 0x0100;
    reg.name = "SLOW";
    reg.handler = slowHandler;
    reg.timeoutMs = 1000;
    reg.requiresAuth = false;
    reg.minParams = 0;
    reg.maxParams = 0;
    
    manager.registerCommand(reg);
    
    s_currentTime = 1000;
    Command cmd = createTestCommand(0x0100, 1);
    CommandResponse resp = manager.processCommand(cmd);
    
    EXPECT_EQ(resp.executionTimeMs, 50u);
}

// ============================================================================
// Тесты стандартных обработчиков
// ============================================================================

TEST(StandardHandlersTest, NoopHandler) {
    auto handler = createNoopHandler();
    
    Command cmd{};
    CommandResponse resp{};
    
    ResultCode result = handler(cmd, resp);
    
    EXPECT_EQ(result, ResultCode::OK);
    EXPECT_EQ(resp.responseLength, 2u);
    EXPECT_EQ(resp.data[0], 'O');
    EXPECT_EQ(resp.data[1], 'K');
}

TEST(StandardHandlersTest, GetVersionHandler) {
    auto handler = createGetVersionHandler(1, 2, 3);
    
    Command cmd{};
    CommandResponse resp{};
    
    ResultCode result = handler(cmd, resp);
    
    EXPECT_EQ(result, ResultCode::OK);
    EXPECT_EQ(resp.responseLength, 3u);
    EXPECT_EQ(resp.data[0], 1u);
    EXPECT_EQ(resp.data[1], 2u);
    EXPECT_EQ(resp.data[2], 3u);
}

// ============================================================================
// Тесты parseCommand
// ============================================================================

TEST(ParseCommandTest, ParseValidCommand) {
    // Little-endian формат
    uint8_t data[] = {
        0x00, 0x01,  // commandId = 0x0100 (little-endian)
        0x01, 0x00,  // sequenceNumber = 1 (little-endian)
        0x01,        // priority = 1
        0x00,        // flags = 0
        0x02, 0x00,  // dataLength = 2 (little-endian)
        0xAA, 0xBB   // data
    };
    
    auto result = parseCommand(data, sizeof(data));
    
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result->header.commandId, 0x0100u);
    EXPECT_EQ(result->header.sequenceNumber, 1u);
    EXPECT_EQ(result->header.dataLength, 2u);
    EXPECT_EQ(result->data[0], 0xAA);
    EXPECT_EQ(result->data[1], 0xBB);
    EXPECT_TRUE(result->valid);
}

TEST(ParseCommandTest, ParseTooShortData) {
    uint8_t data[] = {0x01, 0x00, 0x01};  // Меньше размера заголовка
    
    auto result = parseCommand(data, sizeof(data));
    
    EXPECT_FALSE(result.has_value());
}

TEST(ParseCommandTest, ParseInsufficientDataLength) {
    uint8_t data[] = {
        0x01, 0x00,  // commandId = 0x0100
        0x00, 0x01,  // sequenceNumber = 1
        0x01,        // priority = 1
        0x00,        // flags = 0
        0x05, 0x00,  // dataLength = 5 (но данных только 2)
        0xAA, 0xBB   // data (только 2 байта вместо 5)
    };
    
    auto result = parseCommand(data, sizeof(data));
    
    EXPECT_FALSE(result.has_value());
}

// ============================================================================
// Тесты serializeResponse
// ============================================================================

TEST(SerializeResponseTest, SerializeValidResponse) {
    CommandResponse resp{};
    resp.commandId = 0x0100;
    resp.sequenceNumber = 42;
    resp.resultCode = ResultCode::OK;
    resp.responseLength = 2;
    resp.data[0] = 0xAA;
    resp.data[1] = 0xBB;
    
    std::array<uint8_t, 64> buffer{};
    size_t written = serializeResponse(resp, buffer.data(), buffer.size());
    
    EXPECT_EQ(written, 9u);  // 2 + 2 + 1 + 2 + 2 = 9 байт
    EXPECT_EQ(buffer[0], 0x01);  // commandId high
    EXPECT_EQ(buffer[1], 0x00);  // commandId low
    EXPECT_EQ(buffer[2], 0x00);  // sequenceNumber high
    EXPECT_EQ(buffer[3], 0x2A);  // sequenceNumber low (42)
    EXPECT_EQ(buffer[4], static_cast<uint8_t>(ResultCode::OK));
    EXPECT_EQ(buffer[5], 0x00);  // responseLength high
    EXPECT_EQ(buffer[6], 0x02);  // responseLength low
    EXPECT_EQ(buffer[7], 0xAA);  // data[0]
    EXPECT_EQ(buffer[8], 0xBB);  // data[1]
}

TEST(SerializeResponseTest, SerializeBufferTooSmall) {
    CommandResponse resp{};
    resp.responseLength = 10;
    
    std::array<uint8_t, 5> buffer{};  // Слишком маленький буфер
    size_t written = serializeResponse(resp, buffer.data(), buffer.size());
    
    EXPECT_EQ(written, 0u);
}

TEST(SerializeResponseTest, SerializeEmptyResponse) {
    CommandResponse resp{};
    resp.commandId = 0x0100;
    resp.sequenceNumber = 1;
    resp.resultCode = ResultCode::OK;
    resp.responseLength = 0;
    
    std::array<uint8_t, 16> buffer{};
    size_t written = serializeResponse(resp, buffer.data(), buffer.size());
    
    EXPECT_EQ(written, 7u);  // Только заголовок
}

// ============================================================================
// Тесты CommandPriority
// ============================================================================

TEST(CommandPriorityTest, PriorityValues) {
    EXPECT_EQ(static_cast<uint8_t>(CommandPriority::LOW), 0u);
    EXPECT_EQ(static_cast<uint8_t>(CommandPriority::NORMAL), 1u);
    EXPECT_EQ(static_cast<uint8_t>(CommandPriority::HIGH), 2u);
    EXPECT_EQ(static_cast<uint8_t>(CommandPriority::CRITICAL), 3u);
}

// ============================================================================
// Тесты CommandRegistration
// ============================================================================

TEST(CommandRegistrationTest, DefaultInitialization) {
    CommandRegistration reg{};
    EXPECT_EQ(reg.commandId, 0u);
    EXPECT_EQ(reg.name, nullptr);
    EXPECT_FALSE(reg.validator);
    EXPECT_EQ(reg.timeoutMs, 0u);
    EXPECT_FALSE(reg.requiresAuth);
    EXPECT_EQ(reg.minParams, 0u);
    EXPECT_EQ(reg.maxParams, 0u);
}
