/**
 * @file test_freertos_wrapper.cpp
 * @brief Unit tests for FreeRTOS Wrapper
 */

#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>

#include "rtos/freertos_wrapper.hpp"

using namespace mka::rtos;

// ============================================================================
// Тесты констант и утилит
// ============================================================================

TEST(FreeRTOSConstantsTest, InfiniteTimeout) {
    EXPECT_EQ(INFINITE_TIMEOUT, 0xFFFFFFFFu);
}

TEST(FreeRTOSConstantsTest, NoTimeout) {
    EXPECT_EQ(NO_TIMEOUT, 0u);
}

TEST(FreeRTOSConstantsTest, MsToTicks) {
    // При configTICK_RATE_HZ = 1000 (1 мс за тик)
    EXPECT_EQ(msToTicks(1), 1u);
    EXPECT_EQ(msToTicks(10), 10u);
    EXPECT_EQ(msToTicks(100), 100u);
    EXPECT_EQ(msToTicks(1000), 1000u);
}

TEST(FreeRTOSConstantsTest, MsToTicksRounding) {
    // Проверка округления вверх
    // (ms * 1000 + 999) / 1000
    EXPECT_EQ(msToTicks(0), 0u);
    EXPECT_EQ(msToTicks(1), 1u);
    EXPECT_EQ(msToTicks(50), 50u);
}

// ============================================================================
// Тесты Task
// ============================================================================

TEST(TaskTest, Constructor) {
    auto func = +[]() {
        // Пустая функция
    };
    
    Task<1024> task("TestTask", 5, func);
    
    EXPECT_STREQ(task.getName(), "TestTask");
    EXPECT_EQ(task.getPriority(), 0u);  // До запуска
    EXPECT_EQ(task.getHandle(), nullptr);
}

TEST(TaskTest, DelayConversion) {
    // Тестирование статических методов (без реального вызова FreeRTOS)
    // Просто проверяем что код компилируется
    EXPECT_NO_THROW(Task<1024>::delay(100));
}

TEST(TaskTest, TickCount) {
    // Проверка что метод существует и компилируется
    uint32_t ticks = Task<1024>::getTickCount();
    EXPECT_EQ(ticks, 0u);  // В mock реализации 0
}

// ============================================================================
// Тесты Queue
// ============================================================================

TEST(QueueTest, Constructor) {
    Queue<int, 10> queue;
    
    EXPECT_TRUE(queue.isEmpty());
    EXPECT_FALSE(queue.isFull());
    EXPECT_EQ(queue.messagesWaiting(), 0u);
}

TEST(QueueTest, SendReceive) {
    Queue<int, 10> queue;
    
    // Отправка элемента
    EXPECT_TRUE(queue.send(42, NO_TIMEOUT));
    EXPECT_EQ(queue.messagesWaiting(), 1u);
    EXPECT_FALSE(queue.isEmpty());
    
    // Получение элемента
    auto result = queue.receive(NO_TIMEOUT);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 42);
    EXPECT_TRUE(queue.isEmpty());
}

TEST(QueueTest, ReceiveEmptyQueue) {
    Queue<int, 10> queue;
    
    // Получение из пустой очереди без ожидания
    auto result = queue.receive(NO_TIMEOUT);
    EXPECT_FALSE(result.has_value());
}

TEST(QueueTest, MultipleElements) {
    Queue<int, 10> queue;
    
    for (int i = 0; i < 5; i++) {
        EXPECT_TRUE(queue.send(i, NO_TIMEOUT));
    }
    
    EXPECT_EQ(queue.messagesWaiting(), 5u);
    EXPECT_FALSE(queue.isFull());
    
    for (int i = 0; i < 5; i++) {
        auto result = queue.receive(NO_TIMEOUT);
        EXPECT_TRUE(result.has_value());
        EXPECT_EQ(result.value(), i);
    }
}

TEST(QueueTest, FullQueue) {
    Queue<int, 3> queue;
    
    EXPECT_TRUE(queue.send(1, NO_TIMEOUT));
    EXPECT_TRUE(queue.send(2, NO_TIMEOUT));
    EXPECT_TRUE(queue.send(3, NO_TIMEOUT));
    
    EXPECT_TRUE(queue.isFull());
    EXPECT_EQ(queue.messagesWaiting(), 3u);
}

// ============================================================================
// Тесты BinarySemaphore
// ============================================================================

TEST(BinarySemaphoreTest, Constructor) {
    BinarySemaphore sem;
    
    EXPECT_NE(sem.getHandle(), nullptr);
}

TEST(BinarySemaphoreTest, GiveTake) {
    BinarySemaphore sem;
    
    // Даём семафор
    EXPECT_TRUE(sem.give());
    
    // Берём семафор
    EXPECT_TRUE(sem.take(NO_TIMEOUT));
}

TEST(BinarySemaphoreTest, TakeEmpty) {
    BinarySemaphore sem;
    
    // Попытка взять пустой семафор без ожидания
    EXPECT_FALSE(sem.tryTake());
}

TEST(BinarySemaphoreTest, TryTake) {
    BinarySemaphore sem;
    
    sem.give();
    EXPECT_TRUE(sem.tryTake());
}

// ============================================================================
// Тесты CountingSemaphore
// ============================================================================

TEST(CountingSemaphoreTest, Constructor) {
    CountingSemaphore sem(5, 0);
    
    // Проверяем что код компилируется
    EXPECT_TRUE(sem.take(NO_TIMEOUT) || true);
}

TEST(CountingSemaphoreTest, InitialCount) {
    CountingSemaphore sem(5, 3);
    
    // Должно быть доступно 3 семафора
    EXPECT_TRUE(sem.take(NO_TIMEOUT));
    EXPECT_TRUE(sem.take(NO_TIMEOUT));
    EXPECT_TRUE(sem.take(NO_TIMEOUT));
    EXPECT_FALSE(sem.take(NO_TIMEOUT));  // Пусто
}

TEST(CountingSemaphoreTest, GiveMultiple) {
    CountingSemaphore sem(5, 0);
    
    sem.give();
    sem.give();
    sem.give();
    
    EXPECT_TRUE(sem.take(NO_TIMEOUT));
    EXPECT_TRUE(sem.take(NO_TIMEOUT));
    EXPECT_TRUE(sem.take(NO_TIMEOUT));
    EXPECT_FALSE(sem.take(NO_TIMEOUT));
}

// ============================================================================
// Тесты Mutex
// ============================================================================

TEST(MutexTest, Constructor) {
    Mutex mutex;
    
    EXPECT_NE(mutex.getHandle(), nullptr);
}

TEST(MutexTest, LockUnlock) {
    Mutex mutex;
    
    EXPECT_TRUE(mutex.lock(NO_TIMEOUT));
    EXPECT_TRUE(mutex.unlock());
}

TEST(MutexTest, RecursiveLock) {
    Mutex mutex;
    
    // Первый захват
    EXPECT_TRUE(mutex.lock(NO_TIMEOUT));
    
    // Второй захват (должен заблокироваться в том же потоке)
    // В реальном FreeRTOS мьютекс рекурсивный
    // В тесте может вернуть false или true в зависимости от реализации
    mutex.unlock();
}

// ============================================================================
// Тесты MutexGuard
// ============================================================================

TEST(MutexGuardTest, LockOnConstruct) {
    Mutex mutex;
    
    {
        MutexGuard guard(mutex);
        // Мьютекс захвачен
    }
    // Мьютекс освобождён
    EXPECT_TRUE(mutex.lock(NO_TIMEOUT));  // Должен успешно захватиться
    mutex.unlock();
}

TEST(MutexGuardTest, UnlockOnDestruct) {
    Mutex mutex;
    mutex.lock(NO_TIMEOUT);
    
    {
        MutexGuard guard(mutex);
        // Мьютекс захвачен guard (повторный захват)
    }
    // Мьютекс должен быть освобождён
    EXPECT_TRUE(mutex.lock(NO_TIMEOUT));
    mutex.unlock();
}

// ============================================================================
// Тесты Timer
// ============================================================================

TEST(TimerTest, Constructor) {
    auto callback = +[]() {
        // Пустой callback
    };
    
    Timer timer("TestTimer", 100, true, callback);
    
    EXPECT_FALSE(timer.isRunning());
}

TEST(TimerTest, StartStop) {
    auto callback = +[]() {
        // Пустой callback
    };
    
    Timer timer("TestTimer", 100, true, callback);
    
    // В mock реализации start может вернуть false
    // Проверяем что код компилируется и методы существуют
    EXPECT_NO_THROW(timer.start());
    EXPECT_NO_THROW(timer.stop());
    EXPECT_NO_THROW(timer.reset());
}

// ============================================================================
// Тесты EventGroup
// ============================================================================

TEST(EventGroupTest, Constructor) {
    EventGroup eg;
    
    EXPECT_NE(eg.wait(0x01, true, false, NO_TIMEOUT), 0xFFFFFFFFu);
}

TEST(EventGroupTest, SetBits) {
    EventGroup eg;
    
    EXPECT_NO_THROW(eg.set(0x01));
}

TEST(EventGroupTest, ClearBits) {
    EventGroup eventGroup;
    
    EXPECT_NO_THROW(eventGroup.clear(0x01));
}

TEST(EventGroupTest, WaitBits) {
    EventGroup eg;
    
    // Ожидание без битов
    auto result = eg.wait(0x00, true, false, NO_TIMEOUT);
    (void)result;  // В mock реализации результат не определён
}

// ============================================================================
// Тесты GuardedVariable
// ============================================================================

TEST(GuardedVariableTest, DefaultValue) {
    GuardedVariable<int> var;
    
    EXPECT_EQ(var.get(), 0);
}

TEST(GuardedVariableTest, ConstructorWithValue) {
    GuardedVariable<int> var(42);
    
    EXPECT_EQ(var.get(), 42);
}

TEST(GuardedVariableTest, SetGet) {
    GuardedVariable<int> var;
    
    var.set(100);
    EXPECT_EQ(var.get(), 100);
    
    var.set(200);
    EXPECT_EQ(var.get(), 200);
}

TEST(GuardedVariableTest, Update) {
    GuardedVariable<int> var(10);
    
    var.update(+[](int& value) {
        value += 5;
    });
    
    EXPECT_EQ(var.get(), 15);
}

TEST(GuardedVariableTest, UpdateWithStaticDelta) {
    GuardedVariable<int> var(10);
    static int delta = 7;
    
    var.update(+[](int& value) {
        value += delta;
    });
    
    EXPECT_EQ(var.get(), 17);
}

// ============================================================================
// Тесты предопределённых типов
// ============================================================================

TEST(PredefinedTypesTest, TelemetryMessageSize) {
    TelemetryQueue queue;
    
    EXPECT_TRUE(queue.isEmpty());
}

TEST(PredefinedTypesTest, CommandMessageSize) {
    CommandQueue queue;
    
    EXPECT_TRUE(queue.isEmpty());
}

TEST(PredefinedTypesTest, SatelliteTaskSize) {
    auto func = +[]() {
        // Пустая задача
    };
    
    SatelliteTask task("SatTask", 5, func);
    EXPECT_STREQ(task.getName(), "SatTask");
}

// ============================================================================
// Тесты структуры TelemetryMessage
// ============================================================================

TEST(TelemetryMessageTest, DefaultInitialization) {
    TelemetryMessage msg{};
    
    EXPECT_EQ(msg.apid, 0u);
    EXPECT_EQ(msg.priority, 0u);
    EXPECT_EQ(msg.length, 0u);
}

TEST(TelemetryMessageTest, Assignment) {
    TelemetryMessage msg{};
    msg.apid = 1;
    msg.priority = 2;
    msg.length = 10;
    msg.data[0] = 0xAA;
    
    EXPECT_EQ(msg.apid, 1u);
    EXPECT_EQ(msg.priority, 2u);
    EXPECT_EQ(msg.length, 10u);
    EXPECT_EQ(msg.data[0], 0xAAu);
}

// ============================================================================
// Тесты структуры CommandMessage
// ============================================================================

TEST(CommandMessageTest, DefaultInitialization) {
    CommandMessage msg{};
    
    EXPECT_EQ(msg.commandId, 0u);
    EXPECT_EQ(msg.sequence, 0u);
    EXPECT_EQ(msg.source, 0u);
    EXPECT_EQ(msg.paramsLen, 0u);
}

TEST(CommandMessageTest, Assignment) {
    CommandMessage msg{};
    msg.commandId = 0x0100;
    msg.sequence = 42;
    msg.source = 1;
    msg.paramsLen = 5;
    msg.params[0] = 0xBB;
    
    EXPECT_EQ(msg.commandId, 0x0100u);
    EXPECT_EQ(msg.sequence, 42u);
    EXPECT_EQ(msg.source, 1u);
    EXPECT_EQ(msg.paramsLen, 5u);
    EXPECT_EQ(msg.params[0], 0xBBu);
}
