/**
 * @file test_watchdog_manager.cpp
 * @brief Тесты для Watchdog Manager
 * 
 * Покрывает:
 * - Регистрация задач с лимитами
 * - Мониторинг watchdog
 * - Обработка просрочек
 * - Восстановление после сбоев
 */

#include <gtest/gtest.h>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>

#include "systems/watchdog_manager.hpp"

using namespace mka::watchdog;

// ============================================================================
// Basic Watchdog Tests
// ============================================================================

class WatchdogManagerTest : public ::testing::Test {
protected:
    WatchdogManager& watchdog_ = WatchdogManager::instance();
    
    void SetUp() override {
        watchdog_.reset();
    }
    
    void TearDown() override {
        watchdog_.reset();
    }
};

TEST_F(WatchdogManagerTest, SingletonInstance) {
    auto& instance1 = WatchdogManager::instance();
    auto& instance2 = WatchdogManager::instance();
    EXPECT_EQ(&instance1, &instance2);
}

TEST_F(WatchdogManagerTest, RegisterTask) {
    TaskId taskId = watchdog_.registerTask("TestTask", 1000);  // 1000 ms лимит
    EXPECT_GT(taskId, 0);
}

TEST_F(WatchdogManagerTest, RegisterMultipleTasks) {
    TaskId task1 = watchdog_.registerTask("Task1", 1000);
    TaskId task2 = watchdog_.registerTask("Task2", 2000);
    TaskId task3 = watchdog_.registerTask("Task3", 500);
    
    EXPECT_GT(task1, 0);
    EXPECT_GT(task2, 0);
    EXPECT_GT(task3, 0);
    EXPECT_NE(task1, task2);
    EXPECT_NE(task2, task3);
}

TEST_F(WatchdogManagerTest, UnregisterTask) {
    TaskId taskId = watchdog_.registerTask("TempTask", 1000);
    EXPECT_GT(taskId, 0);
    
    bool result = watchdog_.unregisterTask(taskId);
    EXPECT_TRUE(result);
}

// ============================================================================
// Watchdog Feeding Tests
// ============================================================================

TEST_F(WatchdogManagerTest, FeedWatchdog) {
    TaskId taskId = watchdog_.registerTask("TestTask", 1000);
    
    bool result = watchdog_.feedWatchdog(taskId);
    EXPECT_TRUE(result);
}

TEST_F(WatchdogManagerTest, FeedInvalidTask) {
    bool result = watchdog_.feedWatchdog(99999);
    EXPECT_FALSE(result);
}

TEST_F(WatchdogManagerTest, FeedUnregisteredTask) {
    TaskId taskId = watchdog_.registerTask("TempTask", 1000);
    watchdog_.unregisterTask(taskId);
    
    bool result = watchdog_.feedWatchdog(taskId);
    EXPECT_FALSE(result);
}

// ============================================================================
// Watchdog Timeout Tests
// ============================================================================

TEST_F(WatchdogManagerTest, CheckTimeouts) {
    TaskId taskId = watchdog_.registerTask("TestTask", 100);  // 100 ms лимит
    
    // Не кормим watchdog
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    
    auto timeouts = watchdog_.checkTimeouts();
    EXPECT_GT(timeouts.size(), 0);
    EXPECT_EQ(timeouts[0].taskId, taskId);
}

TEST_F(WatchdogManagerTest, NoTimeoutWhenFed) {
    TaskId taskId = watchdog_.registerTask("TestTask", 500);  // 500 ms лимит
    
    // Кормим watchdog вовремя
    watchdog_.feedWatchdog(taskId);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    watchdog_.feedWatchdog(taskId);
    
    auto timeouts = watchdog_.checkTimeouts();
    EXPECT_EQ(timeouts.size(), 0);
}

TEST_F(WatchdogManagerTest, MultipleTasksTimeout) {
    TaskId task1 = watchdog_.registerTask("Task1", 100);
    TaskId task2 = watchdog_.registerTask("Task2", 200);
    TaskId task3 = watchdog_.registerTask("Task3", 300);
    
    // Кормим только task1 и task3
    watchdog_.feedWatchdog(task1);
    watchdog_.feedWatchdog(task3);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    
    auto timeouts = watchdog_.checkTimeouts();
    EXPECT_EQ(timeouts.size(), 1);
    EXPECT_EQ(timeouts[0].taskId, task2);
}

// ============================================================================
// Watchdog Configuration Tests
// ============================================================================

TEST_F(WatchdogManagerTest, UpdateTaskLimit) {
    TaskId taskId = watchdog_.registerTask("TestTask", 1000);
    
    bool result = watchdog_.updateTaskLimit(taskId, 500);
    EXPECT_TRUE(result);
}

TEST_F(WatchdogManagerTest, UpdateInvalidTaskLimit) {
    bool result = watchdog_.updateTaskLimit(99999, 500);
    EXPECT_FALSE(result);
}

TEST_F(WatchdogManagerTest, GetTaskInfo) {
    TaskId taskId = watchdog_.registerTask("TestTask", 1000);
    
    auto info = watchdog_.getTaskInfo(taskId);
    EXPECT_TRUE(info.has_value());
    EXPECT_EQ(info.value().taskId, taskId);
    EXPECT_EQ(info.value().limitMs, 1000);
}

TEST_F(WatchdogManagerTest, GetInvalidTaskInfo) {
    auto info = watchdog_.getTaskInfo(99999);
    EXPECT_FALSE(info.has_value());
}

// ============================================================================
// Watchdog Statistics Tests
// ============================================================================

TEST_F(WatchdogManagerTest, GetStatistics) {
    TaskId taskId = watchdog_.registerTask("TestTask", 1000);
    
    watchdog_.feedWatchdog(taskId);
    watchdog_.feedWatchdog(taskId);
    watchdog_.feedWatchdog(taskId);
    
    auto stats = watchdog_.getTaskStatistics(taskId);
    EXPECT_TRUE(stats.has_value());
    EXPECT_GE(stats.value().feedCount, 3);
}

TEST_F(WatchdogManagerTest, GetInvalidTaskStatistics) {
    auto stats = watchdog_.getTaskStatistics(99999);
    EXPECT_FALSE(stats.has_value());
}

// ============================================================================
// Watchdog Callback Tests
// ============================================================================

class WatchdogCallbackTest : public ::testing::Test {
protected:
    WatchdogManager& watchdog_ = WatchdogManager::instance();
    int timeoutCallbackCount_ = 0;
    
    void SetUp() override {
        watchdog_.reset();
        timeoutCallbackCount_ = 0;
        watchdog_.setTimeoutCallback([this](TaskId taskId) {
            this->timeoutCallbackCount_++;
        });
    }
    
    void TearDown() override {
        watchdog_.reset();
    }
};

TEST_F(WatchdogCallbackTest, TimeoutCallback) {
    TaskId taskId = watchdog_.registerTask("TestTask", 50);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    watchdog_.checkTimeouts();
    
    EXPECT_GE(timeoutCallbackCount_, 1);
}

// ============================================================================
// Watchdog Edge Cases
// ============================================================================

TEST_F(WatchdogManagerTest, RegisterTaskWithZeroLimit) {
    TaskId taskId = watchdog_.registerTask("TestTask", 0);
    EXPECT_GT(taskId, 0);
}

TEST_F(WatchdogManagerTest, RegisterTaskWithLargeLimit) {
    TaskId taskId = watchdog_.registerTask("TestTask", 1000000);
    EXPECT_GT(taskId, 0);
}

TEST_F(WatchdogManagerTest, RegisterTaskWithEmptyName) {
    TaskId taskId = watchdog_.registerTask("", 1000);
    EXPECT_GT(taskId, 0);
}

TEST_F(WatchdogManagerTest, MultipleFeedsInShortTime) {
    TaskId taskId = watchdog_.registerTask("TestTask", 1000);
    
    // Много кормлений подряд
    for (int i = 0; i < 100; ++i) {
        watchdog_.feedWatchdog(taskId);
    }
    
    auto stats = watchdog_.getTaskStatistics(taskId);
    EXPECT_TRUE(stats.has_value());
    EXPECT_GE(stats.value().feedCount, 100);
}

// ============================================================================
// Watchdog Manager Reset Tests
// ============================================================================

TEST_F(WatchdogManagerTest, ResetClearsAllTasks) {
    watchdog_.registerTask("Task1", 1000);
    watchdog_.registerTask("Task2", 1000);
    watchdog_.registerTask("Task3", 1000);
    
    watchdog_.reset();
    
    auto info1 = watchdog_.getTaskInfo(1);
    auto info2 = watchdog_.getTaskInfo(2);
    auto info3 = watchdog_.getTaskInfo(3);
    
    EXPECT_FALSE(info1.has_value());
    EXPECT_FALSE(info2.has_value());
    EXPECT_FALSE(info3.has_value());
}

TEST_F(WatchdogManagerTest, ResetClearsStatistics) {
    TaskId taskId = watchdog_.registerTask("TestTask", 1000);
    
    for (int i = 0; i < 10; ++i) {
        watchdog_.feedWatchdog(taskId);
    }
    
    watchdog_.reset();
    
    // После ресета задача не должна существовать
    auto stats = watchdog_.getTaskStatistics(taskId);
    EXPECT_FALSE(stats.has_value());
}
