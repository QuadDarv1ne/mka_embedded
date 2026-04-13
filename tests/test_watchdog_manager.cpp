/**
 * @file test_watchdog_manager.cpp
 * @brief Тесты для Watchdog Manager
 *
 * Покрывает:
 * - Регистрация задач
 * - Kick watchdog
 * - Проверка истечения таймаута
 * - Suspend/resume
 * - Callback при истечении
 */

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

#include "systems/watchdog_manager.hpp"

using namespace mka::wdt;

// ============================================================================
// Mock Hardware Watchdog
// ============================================================================

class MockHardwareWatchdog : public IHardwareWatchdog {
public:
    bool init(uint32_t timeoutMs) override {
        initCalled_ = true;
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
        return wasReset_;
    }

    void setResetReason(ResetReason reason) { resetReason_ = reason; }
    void setWasWatchdogReset(bool value) { wasReset_ = value; }

    bool initCalled_ = false;
    uint32_t timeoutMs_ = 0;
    int refreshCount_ = 0;
    ResetReason resetReason_ = ResetReason::POWER_ON;
    bool wasReset_ = false;
};

// ============================================================================
// Fixture
// ============================================================================

class WatchdogManagerTest : public ::testing::Test {
protected:
    MockHardwareWatchdog mockWdt_;

    void SetUp() override {
        auto& mgr = WatchdogManager::instance();
        mgr.reset();  // Сброс состояния между тестами
        mgr.init(&mockWdt_, 5000);
    }
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(WatchdogManagerTest, InitWithHardware) {
    EXPECT_TRUE(mockWdt_.initCalled_);
    EXPECT_EQ(mockWdt_.timeoutMs_, 5000);
}

TEST_F(WatchdogManagerTest, IsInitialized) {
    auto& mgr = WatchdogManager::instance();
    EXPECT_TRUE(mgr.isInitialized());
}

TEST_F(WatchdogManagerTest, GetResetReason) {
    auto& mgr = WatchdogManager::instance();
    EXPECT_EQ(mgr.getLastResetReason(), ResetReason::POWER_ON);
}

// ============================================================================
// Register Task Tests
// ============================================================================

TEST_F(WatchdogManagerTest, RegisterSingleTask) {
    auto& mgr = WatchdogManager::instance();
    uint8_t id = mgr.registerTask("Task1", 1000);
    EXPECT_NE(id, 0xFF);
}

TEST_F(WatchdogManagerTest, RegisterMultipleTasks) {
    auto& mgr = WatchdogManager::instance();
    uint8_t id1 = mgr.registerTask("Task1", 1000);
    uint8_t id2 = mgr.registerTask("Task2", 2000);
    uint8_t id3 = mgr.registerTask("Task3", 500);

    EXPECT_NE(id1, 0xFF);
    EXPECT_NE(id2, 0xFF);
    EXPECT_NE(id3, 0xFF);
    EXPECT_NE(id1, id2);
    EXPECT_NE(id2, id3);
}

TEST_F(WatchdogManagerTest, RegisterMaxTasks) {
    auto& mgr = WatchdogManager::instance();

    // Зарегистрируем MAX_WATCHDOG_TASKS задач
    for (size_t i = 0; i < MAX_WATCHDOG_TASKS; ++i) {
        std::string name = "Task" + std::to_string(i);
        uint8_t id = mgr.registerTask(name.c_str(), 1000);
        EXPECT_NE(id, 0xFF);
    }

    // Следующая задача должна вернуть ошибку
    uint8_t id = mgr.registerTask("Overflow", 1000);
    EXPECT_EQ(id, 0xFF);
}

// ============================================================================
// Kick Tests
// ============================================================================

TEST_F(WatchdogManagerTest, KickTask) {
    auto& mgr = WatchdogManager::instance();
    uint8_t id = mgr.registerTask("Task1", 1000);
    EXPECT_NE(id, 0xFF);

    // Кик не должен вызывать краш
    mgr.kick(id);
    EXPECT_FALSE(mgr.isTaskExpired(id));
}

TEST_F(WatchdogManagerTest, KickInvalidTask) {
    auto& mgr = WatchdogManager::instance();
    // Кик несуществующего task — не должен крашить
    mgr.kick(0xFF);
}

// ============================================================================
// Timeout Tests
// ============================================================================

TEST_F(WatchdogManagerTest, TaskDoesNotExpireImmediately) {
    auto& mgr = WatchdogManager::instance();
    uint8_t id = mgr.registerTask("Task1", 5000);
    EXPECT_NE(id, 0xFF);

    // Сразу после регистрации не должен истечь
    EXPECT_FALSE(mgr.isTaskExpired(id));
}

TEST_F(WatchdogManagerTest, TaskExpiresAfterTimeout) {
    auto& mgr = WatchdogManager::instance();
    uint8_t id = mgr.registerTask("FastTask", 50);  // 50ms таймаут
    EXPECT_NE(id, 0xFF);

    // Ждём больше таймаута
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Обновляем watchdog менеджер
    mgr.update();

    EXPECT_TRUE(mgr.isTaskExpired(id));
}

TEST_F(WatchdogManagerTest, KickPreventsExpiry) {
    auto& mgr = WatchdogManager::instance();
    uint8_t id = mgr.registerTask("KickedTask", 100);
    EXPECT_NE(id, 0xFF);

    // Кикаем несколько раз
    for (int i = 0; i < 5; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        mgr.kick(id);
        mgr.update();
    }

    // Задача не должна истечь благодаря регулярным кикам
    EXPECT_FALSE(mgr.isTaskExpired(id));
}

// ============================================================================
// Suspend/Resume Tests
// ============================================================================

TEST_F(WatchdogManagerTest, SuspendTask) {
    auto& mgr = WatchdogManager::instance();
    uint8_t id = mgr.registerTask("SuspendTask", 50);
    EXPECT_NE(id, 0xFF);

    mgr.suspendTask(id);

    // Ждём больше таймаута
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mgr.update();

    // Подвешенная задача не должна истекать
    EXPECT_FALSE(mgr.isTaskExpired(id));
}

TEST_F(WatchdogManagerTest, ResumeTask) {
    auto& mgr = WatchdogManager::instance();
    uint8_t id = mgr.registerTask("ResumeTask", 50);
    EXPECT_NE(id, 0xFF);

    mgr.suspendTask(id);
    mgr.resumeTask(id);

    // Ждём больше таймаута
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mgr.update();

    // Возобновлённая задача должна истечь
    EXPECT_TRUE(mgr.isTaskExpired(id));
}

// ============================================================================
// Set Timeout Tests
// ============================================================================

TEST_F(WatchdogManagerTest, SetTimeout) {
    auto& mgr = WatchdogManager::instance();
    uint8_t id = mgr.registerTask("TimeoutTask", 1000);
    EXPECT_NE(id, 0xFF);

    // Меняем таймаут на короткий
    mgr.setTimeout(id, 50);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mgr.update();

    EXPECT_TRUE(mgr.isTaskExpired(id));
}

// ============================================================================
// Callback Tests
// ============================================================================

class WatchdogCallbackTest : public ::testing::Test {
protected:
    MockHardwareWatchdog mockWdt_;
    int expireCount_ = 0;
    std::string lastExpiredTask_;
    uint32_t lastElapsedMs_ = 0;

    void SetUp() override {
        expireCount_ = 0;
        lastExpiredTask_.clear();
        lastElapsedMs_ = 0;
        callbackStorage_ = nullptr;

        auto& mgr = WatchdogManager::instance();
        mgr.reset();  // Сброс состояния между тестами
        mgr.init(&mockWdt_, 5000);
        // Callback без захвата — используем статический указатель на this
        callbackStorage_ = this;
        mgr.setExpireCallback(&expireCallbackStub);
    }

    static void expireCallbackStub(const char* name, uint32_t elapsed) {
        auto* test = static_cast<WatchdogCallbackTest*>(callbackStorage_);
        if (test) {
            test->expireCount_++;
            test->lastExpiredTask_ = name;
            test->lastElapsedMs_ = elapsed;
        }
    }

    static WatchdogCallbackTest* callbackStorage_;
};

WatchdogCallbackTest* WatchdogCallbackTest::callbackStorage_ = nullptr;

TEST_F(WatchdogCallbackTest, ExpireCallbackCalled) {
    auto& mgr = WatchdogManager::instance();
    uint8_t id = mgr.registerTask("CallbackTask", 50);
    EXPECT_NE(id, 0xFF);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mgr.update();

    EXPECT_GE(expireCount_, 1);
    EXPECT_EQ(lastExpiredTask_, "CallbackTask");
    EXPECT_GT(lastElapsedMs_, 0);
}

TEST_F(WatchdogCallbackTest, MultipleTasksExpire) {
    auto& mgr = WatchdogManager::instance();
    uint8_t id1 = mgr.registerTask("Task1", 50);
    uint8_t id2 = mgr.registerTask("Task2", 75);
    EXPECT_NE(id1, 0xFF);
    EXPECT_NE(id2, 0xFF);

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    mgr.update();

    EXPECT_GE(expireCount_, 2);
}

// ============================================================================
// Force Kick Tests
// ============================================================================

TEST_F(WatchdogManagerTest, ForceKick) {
    auto& mgr = WatchdogManager::instance();
    uint8_t id = mgr.registerTask("ForceTask", 50);
    EXPECT_NE(id, 0xFF);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mgr.update();

    EXPECT_TRUE(mgr.isTaskExpired(id));

    // Force kick должен кикнуть hardware watchdog
    mgr.forceKick();

    // Hardware watchdog должен быть кикнут
    EXPECT_GT(mockWdt_.refreshCount_, 0);
}

// ============================================================================
// Expired Task Name Tests
// ============================================================================

TEST_F(WatchdogManagerTest, GetExpiredTaskName) {
    auto& mgr = WatchdogManager::instance();
    uint8_t id = mgr.registerTask("NamedTask", 50);
    EXPECT_NE(id, 0xFF);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mgr.update();

    const char* name = mgr.getExpiredTaskName();
    EXPECT_NE(name, nullptr);
    EXPECT_STREQ(name, "NamedTask");
}

// ============================================================================
// Hardware Watchdog Refresh Tests
// ============================================================================

TEST_F(WatchdogManagerTest, RefreshHardwareWatchdog) {
    auto& mgr = WatchdogManager::instance();

    // Force kick должен кикать hardware watchdog
    mgr.forceKick();

    EXPECT_GT(mockWdt_.refreshCount_, 0);
}
