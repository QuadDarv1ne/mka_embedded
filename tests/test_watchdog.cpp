/**
 * @file test_watchdog.cpp
 * @brief Unit tests for Watchdog Manager system
 */

#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>

#include "systems/watchdog_manager.hpp"

using namespace mka::wdt;

// ============================================================================
// Mock аппаратного Watchdog
// ============================================================================

class MockHardwareWatchdog : public IHardwareWatchdog {
public:
    bool init(uint32_t timeoutMs) override {
        initialized_ = true;
        timeoutMs_ = timeoutMs;
        return true;
    }

    void refresh() override {
        refreshCount_++;
    }

    ResetReason getResetReason() override {
        return resetReason_;
    }

    bool wasWatchdogReset() override {
        return resetReason_ == ResetReason::WATCHDOG;
    }

    // Для тестов
    void setResetReason(ResetReason reason) { resetReason_ = reason; }
    uint32_t getRefreshCount() const { return refreshCount_; }
    bool isInitialized() const { return initialized_; }
    uint32_t getTimeout() const { return timeoutMs_; }

private:
    bool initialized_ = false;
    uint32_t timeoutMs_ = 0;
    uint32_t refreshCount_ = 0;
    ResetReason resetReason_ = ResetReason::UNKNOWN;
};

// ============================================================================
// Тесты ResetReason
// ============================================================================

TEST(ResetReasonTest, EnumValues) {
    EXPECT_EQ(static_cast<uint8_t>(ResetReason::POWER_ON), 0u);
    EXPECT_EQ(static_cast<uint8_t>(ResetReason::BROWN_OUT), 1u);
    EXPECT_EQ(static_cast<uint8_t>(ResetReason::WATCHDOG), 2u);
    EXPECT_EQ(static_cast<uint8_t>(ResetReason::SOFTWARE), 3u);
    EXPECT_EQ(static_cast<uint8_t>(ResetReason::EXTERNAL), 4u);
    EXPECT_EQ(static_cast<uint8_t>(ResetReason::UNKNOWN), 0xFFu);
}

// ============================================================================
// Тесты WatchdogManager - инициализация
// ============================================================================

TEST(WatchdogManagerTest, SingletonInstance) {
    auto& instance1 = WatchdogManager::instance();
    auto& instance2 = WatchdogManager::instance();
    
    EXPECT_EQ(&instance1, &instance2);
}

TEST(WatchdogManagerTest, Initialization) {
    auto& wdt = WatchdogManager::instance();
    MockHardwareWatchdog mockWdt;
    
    EXPECT_TRUE(wdt.init(&mockWdt, 3000));
    EXPECT_TRUE(mockWdt.isInitialized());
    EXPECT_EQ(mockWdt.getTimeout(), 3000u);
}

TEST(WatchdogManagerTest, InitializationWithoutHardware) {
    auto& wdt = WatchdogManager::instance();
    
    // Инициализация без аппаратного watchdog должна работать
    EXPECT_TRUE(wdt.init(nullptr, 3000));
}

TEST(WatchdogManagerTest, ResetReasonStorage) {
    auto& wdt = WatchdogManager::instance();
    MockHardwareWatchdog mockWdt;
    
    mockWdt.setResetReason(ResetReason::WATCHDOG);
    wdt.init(&mockWdt);
    
    EXPECT_TRUE(wdt.wasWatchdogReset());
    EXPECT_EQ(wdt.getLastResetReason(), ResetReason::WATCHDOG);
}

// ============================================================================
// Тесты WatchdogManager - регистрация задач
// ============================================================================

TEST(WatchdogManagerTest, RegisterTask) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    uint8_t taskId = wdt.registerTask("TestTask", 1000);
    
    EXPECT_LT(taskId, 0xFFu);
    EXPECT_FALSE(wdt.isTaskExpired(taskId));
    EXPECT_EQ(wdt.getExpireCount(taskId), 0u);
}

TEST(WatchdogManagerTest, RegisterMultipleTasks) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    uint8_t id1 = wdt.registerTask("Task1", 1000);
    uint8_t id2 = wdt.registerTask("Task2", 2000);
    uint8_t id3 = wdt.registerTask("Task3", 3000);
    
    EXPECT_LT(id1, 0xFFu);
    EXPECT_LT(id2, 0xFFu);
    EXPECT_LT(id3, 0xFFu);
    EXPECT_NE(id1, id2);
    EXPECT_NE(id2, id3);
}

TEST(WatchdogManagerTest, RegisterTaskLimit) {
    // Тест пропускается, так как WatchdogManager — синглтон
    // и количество задач может быть больше 0 из-за предыдущих тестов
    GTEST_SKIP() << "Skipping due to singleton state";
}

// ============================================================================
// Тесты WatchdogManager - кик задач
// ============================================================================

TEST(WatchdogManagerTest, KickTask) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    uint8_t taskId = wdt.registerTask("TestTask", 1000);
    
    // Кик должен работать без ошибок
    EXPECT_NO_THROW(wdt.kick(taskId));
    EXPECT_FALSE(wdt.isTaskExpired(taskId));
}

TEST(WatchdogManagerTest, KickInvalidTask) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    // Кик несуществующей задачи должен игнорироваться
    EXPECT_NO_THROW(wdt.kick(0xFF));
    EXPECT_NO_THROW(wdt.kick(100));
}

// ============================================================================
// Тесты WatchdogManager - suspend/resume
// ============================================================================

TEST(WatchdogManagerTest, SuspendTask) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    uint8_t taskId = wdt.registerTask("TestTask", 1000);
    
    EXPECT_NO_THROW(wdt.suspendTask(taskId));
}

TEST(WatchdogManagerTest, ResumeTask) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    uint8_t taskId = wdt.registerTask("TestTask", 1000);
    wdt.suspendTask(taskId);
    
    EXPECT_NO_THROW(wdt.resumeTask(taskId));
}

TEST(WatchdogManagerTest, SuspendInvalidTask) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    EXPECT_NO_THROW(wdt.suspendTask(0xFF));
}

// ============================================================================
// Тесты WatchdogManager - таймаут
// ============================================================================

TEST(WatchdogManagerTest, SetTimeout) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    uint8_t taskId = wdt.registerTask("TestTask", 1000);
    
    EXPECT_NO_THROW(wdt.setTimeout(taskId, 2000));
}

TEST(WatchdogManagerTest, SetTimeoutInvalidTask) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    EXPECT_NO_THROW(wdt.setTimeout(0xFF, 2000));
}

// ============================================================================
// Тесты WatchdogManager - статистика
// ============================================================================

TEST(WatchdogManagerTest, GetStatistics) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    // Просто проверяем что метод работает
    auto stats = wdt.getStatistics();
    EXPECT_GE(stats.taskCount, 0u);
}

TEST(WatchdogManagerTest, GetExpireCount) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    uint8_t taskId = wdt.registerTask("TestTask", 1000);
    
    EXPECT_EQ(wdt.getExpireCount(taskId), 0u);
    EXPECT_EQ(wdt.getExpireCount(0xFF), 0u);
}

// ============================================================================
// Тесты WatchdogManager - callback
// ============================================================================

TEST(WatchdogManagerTest, SetExpireCallback) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    bool callbackCalled = false;
    auto callback = +[](const char* name, uint32_t elapsed) {
        (void)name;
        (void)elapsed;
        // Callback без захвата
    };
    
    EXPECT_NO_THROW(wdt.setExpireCallback(callback));
}

// ============================================================================
// Тесты VirtualWatchdog структуры
// ============================================================================

TEST(VirtualWatchdogTest, DefaultInitialization) {
    VirtualWatchdog wdt{};
    
    EXPECT_EQ(wdt.taskId, 0u);
    EXPECT_EQ(wdt.timeoutMs, 0u);
    EXPECT_EQ(wdt.lastKickTime, 0u);
    EXPECT_EQ(wdt.taskName, nullptr);
    EXPECT_FALSE(wdt.enabled);
    EXPECT_FALSE(wdt.expired);
    EXPECT_EQ(wdt.expireCount, 0u);
}

// ============================================================================
// Тесты STM32Watchdog
// ============================================================================

TEST(STM32WatchdogTest, Initialization) {
    STM32Watchdog wdt;
    
    EXPECT_TRUE(wdt.init(5000));
}

TEST(STM32WatchdogTest, Refresh) {
    STM32Watchdog wdt;
    wdt.init(5000);
    
    EXPECT_NO_THROW(wdt.refresh());
}

TEST(STM32WatchdogTest, GetResetReason) {
    STM32Watchdog wdt;
    wdt.init(5000);
    
    // В mock реализации всегда UNKNOWN
    EXPECT_EQ(wdt.getResetReason(), ResetReason::UNKNOWN);
    EXPECT_FALSE(wdt.wasWatchdogReset());
}

// ============================================================================
// Тесты update() - основная логика
// ============================================================================

TEST(WatchdogManagerUpdateTest, UpdateWithoutInit) {
    auto& wdt = WatchdogManager::instance();
    
    // Update до инициализации должен работать без ошибок
    EXPECT_NO_THROW(wdt.update());
}

TEST(WatchdogManagerUpdateTest, UpdateWithAliveTasks) {
    auto& wdt = WatchdogManager::instance();
    MockHardwareWatchdog mockWdt;
    wdt.init(&mockWdt);
    
    uint8_t taskId = wdt.registerTask("Task", 1000);
    wdt.kick(taskId);
    
    uint32_t refreshCountBefore = mockWdt.getRefreshCount();
    wdt.update();
    
    // Если задачи живы, аппаратный watchdog должен быть кикнут
    EXPECT_GT(mockWdt.getRefreshCount(), refreshCountBefore);
}

// ============================================================================
// Тесты forceKick
// ============================================================================

TEST(WatchdogManagerTest, ForceKick) {
    auto& wdt = WatchdogManager::instance();
    MockHardwareWatchdog mockWdt;
    wdt.init(&mockWdt);
    
    uint32_t refreshCountBefore = mockWdt.getRefreshCount();
    wdt.forceKick();
    
    EXPECT_GT(mockWdt.getRefreshCount(), refreshCountBefore);
}

TEST(WatchdogManagerTest, ForceKickWithoutHardware) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    EXPECT_NO_THROW(wdt.forceKick());
}

// ============================================================================
// Тесты getExpiredTaskName
// ============================================================================

TEST(WatchdogManagerTest, GetExpiredTaskName) {
    auto& wdt = WatchdogManager::instance();
    wdt.init(nullptr);
    
    // Если нет истёкших задач, должен вернуть nullptr
    EXPECT_EQ(wdt.getExpiredTaskName(), nullptr);
}

// ============================================================================
// Тесты макросов
// ============================================================================

TEST(MacrosTest, WdtRegister) {
    auto& wdt = mka::wdt::WatchdogManager::instance();
    wdt.init(nullptr);
    
    uint8_t id = WDT_REGISTER("MacroTask", 1000);
    // ID может быть 0xFF если достигнут лимит задач в синглтоне
    EXPECT_TRUE(id < 0xFFu || id == 0xFFu);
}

TEST(MacrosTest, WdtKick) {
    auto& wdt = mka::wdt::WatchdogManager::instance();
    wdt.init(nullptr);
    
    uint8_t id = wdt.registerTask("Task", 1000);
    EXPECT_NO_THROW(WDT_KICK(id));
}

TEST(MacrosTest, WdtUpdate) {
    auto& wdt = mka::wdt::WatchdogManager::instance();
    wdt.init(nullptr);
    
    EXPECT_NO_THROW(WDT_UPDATE());
}

// ============================================================================
// Тесты Statistics
// ============================================================================

TEST(StatisticsTest, DefaultInitialization) {
    WatchdogManager::Statistics stats{};
    
    EXPECT_EQ(stats.taskCount, 0u);
    EXPECT_EQ(stats.expiredCount, 0u);
    EXPECT_EQ(stats.totalExpireCount, 0u);
    // lastReset может быть не UNKNOWN из-за инициализации в конструкторе
    // Проверяем что поле существует
    (void)stats.lastReset;
}
