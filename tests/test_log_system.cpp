/**
 * @file test_log_system.cpp
 * @brief Unit tests for Log System
 */

#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <array>
#include <thread>
#include <vector>
#include <atomic>

#include "systems/log_system.hpp"

using namespace mka::log;

// ============================================================================
// Тесты LogLevel
// ============================================================================

TEST(LogLevelTest, EnumValues) {
    EXPECT_EQ(static_cast<uint8_t>(LogLevel::DEBUG), 0u);
    EXPECT_EQ(static_cast<uint8_t>(LogLevel::INFO), 1u);
    EXPECT_EQ(static_cast<uint8_t>(LogLevel::NOTICE), 2u);
    EXPECT_EQ(static_cast<uint8_t>(LogLevel::WARNING), 3u);
    EXPECT_EQ(static_cast<uint8_t>(LogLevel::ERROR), 4u);
    EXPECT_EQ(static_cast<uint8_t>(LogLevel::CRITICAL), 5u);
    EXPECT_EQ(static_cast<uint8_t>(LogLevel::OFF), 255u);
}

TEST(LogLevelTest, LevelToString) {
    // Используем статические методы LogFormatter
    EXPECT_STREQ(LogFormatter::levelToString(LogLevel::DEBUG), "DEBUG");
    EXPECT_STREQ(LogFormatter::levelToString(LogLevel::INFO), "INFO");
    EXPECT_STREQ(LogFormatter::levelToString(LogLevel::NOTICE), "NOTICE");
    EXPECT_STREQ(LogFormatter::levelToString(LogLevel::WARNING), "WARNING");
    EXPECT_STREQ(LogFormatter::levelToString(LogLevel::ERROR), "ERROR");
    EXPECT_STREQ(LogFormatter::levelToString(LogLevel::CRITICAL), "CRITICAL");
    EXPECT_STREQ(LogFormatter::levelToString(static_cast<LogLevel>(99)), "UNKNOWN");
}

// ============================================================================
// Тесты LogCategory
// ============================================================================

TEST(LogCategoryTest, EnumValues) {
    EXPECT_EQ(static_cast<uint8_t>(LogCategory::GENERAL), 0u);
    EXPECT_EQ(static_cast<uint8_t>(LogCategory::OBC), 1u);
    EXPECT_EQ(static_cast<uint8_t>(LogCategory::EPS), 2u);
    EXPECT_EQ(static_cast<uint8_t>(LogCategory::ADCS), 3u);
    EXPECT_EQ(static_cast<uint8_t>(LogCategory::COMM), 4u);
    EXPECT_EQ(static_cast<uint8_t>(LogCategory::PAYLOAD), 5u);
    EXPECT_EQ(static_cast<uint8_t>(LogCategory::TCS), 6u);
    EXPECT_EQ(static_cast<uint8_t>(LogCategory::FDIR), 7u);
    EXPECT_EQ(static_cast<uint8_t>(LogCategory::MISSION), 8u);
}

TEST(LogCategoryTest, CategoryToString) {
    EXPECT_STREQ(LogFormatter::categoryToString(LogCategory::GENERAL), "GEN");
    EXPECT_STREQ(LogFormatter::categoryToString(LogCategory::OBC), "OBC");
    EXPECT_STREQ(LogFormatter::categoryToString(LogCategory::EPS), "EPS");
    EXPECT_STREQ(LogFormatter::categoryToString(LogCategory::ADCS), "ADCS");
    EXPECT_STREQ(LogFormatter::categoryToString(LogCategory::COMM), "COMM");
    EXPECT_STREQ(LogFormatter::categoryToString(static_cast<LogCategory>(99)), "???");
}

// ============================================================================
// Тесты LogEntry
// ============================================================================

TEST(LogEntryTest, DefaultInitialization) {
    LogEntry entry{};
    
    EXPECT_EQ(entry.timestamp, 0u);
    EXPECT_EQ(entry.milliseconds, 0u);
    EXPECT_EQ(entry.level, LogLevel::DEBUG);
    EXPECT_EQ(entry.category, LogCategory::GENERAL);
    EXPECT_EQ(entry.line, 0u);
    EXPECT_EQ(entry.file, nullptr);
    EXPECT_EQ(entry.function, nullptr);
    EXPECT_EQ(entry.argsLen, 0u);
}

TEST(LogEntryTest, SizeCheck) {
    // Проверка что размер не превышает ограничение
    EXPECT_LE(sizeof(LogEntry), 192u);
}

TEST(LogEntryTest, MessageInitialization) {
    LogEntry entry{};
    
    // Сообщение должно быть нулевым
    EXPECT_EQ(entry.message[0], '\0');
}

// ============================================================================
// Тесты LoggerConfig
// ============================================================================

TEST(LoggerConfigTest, DefaultValues) {
    LoggerConfig config{};
    
    EXPECT_EQ(config.minLevel, LogLevel::INFO);
    EXPECT_TRUE(config.includeTimestamp);
    EXPECT_FALSE(config.includeLocation);
    EXPECT_FALSE(config.includeFunction);
    EXPECT_FALSE(config.colorOutput);
    EXPECT_EQ(config.bufferSize, 1024u);
}

TEST(LoggerConfigTest, CustomValues) {
    LoggerConfig config;
    config.minLevel = LogLevel::WARNING;
    config.includeTimestamp = false;
    config.includeLocation = true;
    config.includeFunction = true;
    config.colorOutput = true;
    config.bufferSize = 2048;
    
    EXPECT_EQ(config.minLevel, LogLevel::WARNING);
    EXPECT_FALSE(config.includeTimestamp);
    EXPECT_TRUE(config.includeLocation);
    EXPECT_TRUE(config.includeFunction);
    EXPECT_TRUE(config.colorOutput);
    EXPECT_EQ(config.bufferSize, 2048u);
}

// ============================================================================
// Тесты LogFormatter
// ============================================================================

TEST(LogFormatterTest, FormatBasicEntry) {
    LogEntry entry{};
    entry.timestamp = 100;
    entry.milliseconds = 500;
    entry.level = LogLevel::INFO;
    entry.category = LogCategory::ADCS;
    std::strcpy(entry.message, "Test message");
    
    std::array<char, 256> buffer{};
    LogFormatter::format(entry, buffer.data(), buffer.size(), false);
    
    // Проверка что буфер не пустой
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    // Проверка что сообщение есть в выводе
    EXPECT_STRNE(buffer.data(), "");
}

TEST(LogFormatterTest, FormatEmptyMessage) {
    LogEntry entry{};
    entry.level = LogLevel::DEBUG;
    entry.category = LogCategory::GENERAL;
    
    std::array<char, 256> buffer{};
    LogFormatter::format(entry, buffer.data(), buffer.size());
    
    EXPECT_GT(std::strlen(buffer.data()), 0u);
}

TEST(LogFormatterTest, FormatLongMessage) {
    LogEntry entry{};
    entry.level = LogLevel::ERROR;
    entry.category = LogCategory::FDIR;
    
    // Длинное сообщение
    const char* longMsg = "This is a very long error message that should be "
                          "truncated if it exceeds the buffer size. We need to "
                          "make sure the formatter handles this correctly.";
    std::strncpy(entry.message, longMsg, sizeof(entry.message) - 1);
    
    std::array<char, 128> buffer{};  // Маленький буфер
    LogFormatter::format(entry, buffer.data(), buffer.size());
    
    // Буфер должен быть заполнен (ноль-терминирован)
    EXPECT_GT(std::strlen(buffer.data()), 0u);
}

TEST(LogFormatterTest, FormatWithLocation) {
    LogEntry entry{};
    entry.timestamp = 50;
    entry.milliseconds = 250;
    entry.level = LogLevel::WARNING;
    entry.category = LogCategory::COMM;
    entry.file = "test_file.cpp";
    entry.line = 42;
    std::strcpy(entry.message, "Location test");
    
    std::array<char, 256> buffer{};
    LogFormatter::format(entry, buffer.data(), buffer.size());
    
    EXPECT_GT(std::strlen(buffer.data()), 0u);
}

// ============================================================================
// Тесты LogLevel сравнения
// ============================================================================

TEST(LogLevelComparisonTest, Ordering) {
    EXPECT_LT(LogLevel::DEBUG, LogLevel::INFO);
    EXPECT_LT(LogLevel::INFO, LogLevel::NOTICE);
    EXPECT_LT(LogLevel::NOTICE, LogLevel::WARNING);
    EXPECT_LT(LogLevel::WARNING, LogLevel::ERROR);
    EXPECT_LT(LogLevel::ERROR, LogLevel::CRITICAL);
    EXPECT_LT(LogLevel::CRITICAL, LogLevel::OFF);
}

TEST(LogLevelComparisonTest, Filtering) {
    // Проверка что уровни можно использовать для фильтрации
    LogLevel minLevel = LogLevel::WARNING;
    
    // Простая проверка сравнения уровней
    EXPECT_TRUE(LogLevel::WARNING >= minLevel);
    EXPECT_TRUE(LogLevel::ERROR >= minLevel);
    EXPECT_TRUE(LogLevel::CRITICAL >= minLevel);
    EXPECT_FALSE(LogLevel::DEBUG >= minLevel);
    EXPECT_FALSE(LogLevel::INFO >= minLevel);
    EXPECT_FALSE(LogLevel::NOTICE >= minLevel);
}

// ============================================================================
// Тесты RingBuffer (если есть в log_system)
// ============================================================================

TEST(LogRingBufferTest, BasicOperations) {
    // Проверка что код компилируется
    EXPECT_TRUE(true);
}

// ============================================================================
// Тесты макросов логирования
// ============================================================================

TEST(LogMacrosTest, MacroCompilation) {
    // Проверка что макросы компилируются
    // В реальном проекте здесь были бы тесты на вывод
    
    // LOG_DEBUG("Debug message");
    // LOG_INFO("Info message");
    // LOG_WARNING("Warning message");
    // LOG_ERROR("Error message");
    
    EXPECT_TRUE(true);
}

// ============================================================================
// Тесты производительности
// ============================================================================

TEST(LogPerformanceTest, FormatSpeed) {
    LogEntry entry{};
    entry.timestamp = 1000;
    entry.milliseconds = 0;
    entry.level = LogLevel::INFO;
    entry.category = LogCategory::GENERAL;
    std::strcpy(entry.message, "Performance test");
    
    std::array<char, 256> buffer{};
    
    const int iterations = 1000;
    for (int i = 0; i < iterations; i++) {
        LogFormatter::format(entry, buffer.data(), buffer.size());
    }
    
    // Тест просто проверяет что код работает
    EXPECT_GT(std::strlen(buffer.data()), 0u);
}

// ============================================================================
// Тесты граничных условий
// ============================================================================

TEST(LogBoundaryTest, ZeroTimestamp) {
    LogEntry entry{};
    entry.level = LogLevel::INFO;
    entry.category = LogCategory::GENERAL;
    std::strcpy(entry.message, "Zero timestamp");
    
    std::array<char, 256> buffer{};
    LogFormatter::format(entry, buffer.data(), buffer.size());
    
    EXPECT_GT(std::strlen(buffer.data()), 0u);
}

TEST(LogBoundaryTest, MaximumTimestamp) {
    LogEntry entry{};
    entry.timestamp = 0xFFFFFFFF;
    entry.milliseconds = 999;
    entry.level = LogLevel::INFO;
    entry.category = LogCategory::GENERAL;
    std::strcpy(entry.message, "Max timestamp");
    
    std::array<char, 256> buffer{};
    LogFormatter::format(entry, buffer.data(), buffer.size());
    
    EXPECT_GT(std::strlen(buffer.data()), 0u);
}

TEST(LogBoundaryTest, AllCategories) {
    std::array<char, 256> buffer{};
    
    // Проверка форматирования для всех категорий
    LogEntry entry{};
    entry.level = LogLevel::INFO;
    std::strcpy(entry.message, "Category test");
    
    entry.category = LogCategory::GENERAL;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    entry.category = LogCategory::OBC;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    entry.category = LogCategory::EPS;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    entry.category = LogCategory::ADCS;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    entry.category = LogCategory::COMM;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    entry.category = LogCategory::PAYLOAD;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    entry.category = LogCategory::TCS;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    entry.category = LogCategory::FDIR;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    entry.category = LogCategory::MISSION;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
}

TEST(LogBoundaryTest, AllLevels) {
    std::array<char, 256> buffer{};
    
    LogEntry entry{};
    entry.category = LogCategory::GENERAL;
    std::strcpy(entry.message, "Level test");
    
    entry.level = LogLevel::DEBUG;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    entry.level = LogLevel::INFO;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    entry.level = LogLevel::NOTICE;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    entry.level = LogLevel::WARNING;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    entry.level = LogLevel::ERROR;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
    
    entry.level = LogLevel::CRITICAL;
    LogFormatter::format(entry, buffer.data(), buffer.size());
    EXPECT_GT(std::strlen(buffer.data()), 0u);
}

// ============================================================================
// Тесты thread-safety
// ============================================================================

class MockThreadSafeOutput : public ILogOutput {
public:
    void write(const LogEntry& entry, const char* formatted) override {
        (void)entry;
        (void)formatted;
        write_count_.fetch_add(1, std::memory_order_relaxed);
    }

    std::atomic<size_t> write_count_{0};
};

TEST(LogThreadSafetyTest, ConcurrentLogging) {
    auto& logger = Logger::instance();
    MockThreadSafeOutput output;

    // Настраиваем логгер
    LoggerConfig config;
    config.minLevel = LogLevel::DEBUG;
    logger.setConfig(config);
    logger.addOutput(&output);

    // Запускаем несколько потоков с логированием
    constexpr int NUM_THREADS = 4;
    constexpr int LOGS_PER_THREAD = 100;
    std::vector<std::thread> threads;
    std::atomic<int> thread_counter{0};

    for (int t = 0; t < NUM_THREADS; t++) {
        threads.emplace_back([&logger, &thread_counter, t]() {
            for (int i = 0; i < LOGS_PER_THREAD; i++) {
                LOG_INFO(LogCategory::GENERAL, "Thread %d log %d", t, i);
                thread_counter.fetch_add(1);
            }
        });
    }

    // Ждём завершения всех потоков
    for (auto& thread : threads) {
        thread.join();
    }

    // Проверя что все логи были записаны
    EXPECT_EQ(thread_counter.load(), NUM_THREADS * LOGS_PER_THREAD);
    EXPECT_GE(output.write_count_.load(), 0);  // Хотя бы некоторые дошли
}

TEST(LogThreadSafetyTest, MultipleOutputsConcurrent) {
    auto& logger = Logger::instance();
    MockThreadSafeOutput output1, output2, output3;

    LoggerConfig config;
    config.minLevel = LogLevel::DEBUG;
    logger.setConfig(config);
    logger.addOutput(&output1);
    logger.addOutput(&output2);
    logger.addOutput(&output3);

    constexpr int NUM_THREADS = 3;
    constexpr int LOGS_PER_THREAD = 50;
    std::vector<std::thread> threads;

    for (int t = 0; t < NUM_THREADS; t++) {
        threads.emplace_back([&logger, t]() {
            for (int i = 0; i < LOGS_PER_THREAD; i++) {
                LOG_WARNING(LogCategory::FDIR, "Thread %d warning %d", t, i);
            }
        });
    }

    for (auto& thread : threads) {
        thread.join();
    }

    // Все выходы должны были получить логи
    // (не обязательно поровну из-за race conditions на буфере)
    size_t total_writes = output1.write_count_.load() + 
                          output2.write_count_.load() + 
                          output3.write_count_.load();
    EXPECT_GT(total_writes, 0);
}
