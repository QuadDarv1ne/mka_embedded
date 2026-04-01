/**
 * @file main.cpp
 * @brief Пример многозадачности с FreeRTOS wrapper
 *
 * Демонстрирует создание задач, очередей, семафоров и мьютексов
 * с использованием C++ wrapper для FreeRTOS.
 */

#include <cstdint>
#include <cstdio>
#include <cstring>

// FreeRTOS wrapper
#include "rtos/freertos_wrapper.hpp"

// ============================================================================
// Конфигурация
// ============================================================================

namespace config {
    constexpr uint32_t TASK1_PERIOD_MS = 500;    // Задача 1: 2 Гц
    constexpr uint32_t TASK2_PERIOD_MS = 1000;   // Задача 2: 1 Гц
    constexpr uint32_t LED_TASK_PERIOD_MS = 250; // Задача светодиода: 4 Гц
}

// ============================================================================
// Глобальные объекты
// ============================================================================

using namespace mka::rtos;

// Очередь для передачи данных между задачами
static Queue<int, 10> dataQueue;

// Семафор для синхронизации
static BinarySemaphore syncSemaphore;

// Мьютекс для защиты консоли
static Mutex consoleMutex;

// Счётчик задач
static volatile uint32_t task1Count = 0;
static volatile uint32_t task2Count = 0;

// ============================================================================
// Задача 1: Быстрая задача (производитель данных)
// ============================================================================

void vTask1(void* pvParameters) {
    uint32_t lastWakeTime = Task<1024>::getTickCount();
    int counter = 0;

    printf("[Task1] Started with priority %u\n", Task<1024>::getTickCount());

    for (;;) {
        counter++;
        task1Count++;

        // Отправка данных в очередь
        if (!dataQueue.send(counter, 100)) {
            {
                MutexGuard guard(consoleMutex);
                printf("[Task1] Queue full!\n");
            }
        }

        // Периодическое ожидание
        Task<1024>::delayUntil(lastWakeTime, config::TASK1_PERIOD_MS);
    }
}

// ============================================================================
// Задача 2: Медленная задача (потребитель данных)
// ============================================================================

void vTask2(void* pvParameters) {
    uint32_t lastWakeTime = Task<1024>::getTickCount();

    printf("[Task2] Started\n");

    for (;;) {
        // Получение данных из очереди
        auto data = dataQueue.receive(100);

        if (data.has_value()) {
            {
                MutexGuard guard(consoleMutex);
                printf("[Task2] Received: %d (queue: %zu)\n",
                       data.value(), dataQueue.messagesWaiting());
            }
        }

        task2Count++;

        Task<1024>::delayUntil(lastWakeTime, config::TASK2_PERIOD_MS);
    }
}

// ============================================================================
// Задача 3: Мигающий светодиод (с семафором)
// ============================================================================

void vLedTask(void* pvParameters) {
    // Имитация светодиода (GPIO)
    bool ledState = false;

    printf("[LedTask] Started\n");

    for (;;) {
        // Ожидание семафора (сигнал от другой задачи)
        if (syncSemaphore.take(100)) {
            ledState = !ledState;
            {
                MutexGuard guard(consoleMutex);
                printf("[LedTask] LED %s\n", ledState ? "ON" : "OFF");
            }
        }
    }
}

// ============================================================================
// Задача 4: Мониторинг (статистика)
// ============================================================================

void vMonitorTask(void* pvParameters) {
    uint32_t lastWakeTime = Task<1024>::getTickCount();

    printf("[Monitor] Started\n");

    for (;;) {
        {
            MutexGuard guard(consoleMutex);
            printf("\n=== System Monitor ===\n");
            printf("Task1 count: %lu\n", task1Count);
            printf("Task2 count: %lu\n", task2Count);
            printf("Queue size:  %zu\n", dataQueue.messagesWaiting());
            printf("Free heap:   implementation-specific\n");
            printf("======================\n\n");
        }

        Task<1024>::delayUntil(lastWakeTime, 2000);  // 2 секунды
    }
}

// ============================================================================
// Инициализация и запуск
// ============================================================================

void vStartAllTasks(void) {
    printf("=== FreeRTOS Tasks Example ===\n\n");

    // Статические задачи (память выделяется статически)
    static Task<2048> task1("Task1", 4, +[]() { vTask1(nullptr); });
    static Task<2048> task2("Task2", 3, +[]() { vTask2(nullptr); });
    static Task<1024> ledTask("LedTask", 2, +[]() { vLedTask(nullptr); });
    static Task<2048> monitor("Monitor", 3, +[]() { vMonitorTask(nullptr); });

    // Запуск всех задач
    printf("[INIT] Creating tasks...\n");

    if (!task1.start()) {
        printf("[ERROR] Failed to start Task1\n");
    } else {
        printf("[OK] Task1 created (stack: 2KB, priority: 4)\n");
    }

    if (!task2.start()) {
        printf("[ERROR] Failed to start Task2\n");
    } else {
        printf("[OK] Task2 created (stack: 2KB, priority: 3)\n");
    }

    if (!ledTask.start()) {
        printf("[ERROR] Failed to start LedTask\n");
    } else {
        printf("[OK] LedTask created (stack: 1KB, priority: 2)\n");
    }

    if (!monitor.start()) {
        printf("[ERROR] Failed to start Monitor\n");
    } else {
        printf("[OK] Monitor created (stack: 2KB, priority: 3)\n");
    }

    printf("\n[OK] All tasks started. Scheduler running.\n\n");
}

// ============================================================================
// Основная программа (для host build)
// ============================================================================

#ifndef USE_FREERTOS

int main() {
    printf("FreeRTOS Tasks Example (Host Build)\n");
    printf("Note: This example requires FreeRTOS to run properly.\n\n");

    // Для host build просто показываем код
    printf("To use this example:\n");
    printf("1. Enable USE_FREERTOS define\n");
    printf("2. Link with FreeRTOS library\n");
    printf("3. Call vStartAllTasks() from main()\n\n");

    vStartAllTasks();

    // В реальной системе здесь будет запуск планировщика
    // vTaskStartScheduler();

    printf("Example completed (simulation mode)\n");
    return 0;
}

#else

// Для FreeRTOS build
extern "C" void vTaskStartScheduler(void);

int main(void) {
    // Инициализация hardware (если нужно)
    // HAL_Init();
    // SystemClock_Config();

    // Запуск задач
    vStartAllTasks();

    // Запуск планировщика FreeRTOS
    vTaskStartScheduler();

    // Достижимо только если планировщик остановлен
    return 0;
}

#endif // USE_FREERTOS

// ============================================================================
// Дополнительные примеры использования
// ============================================================================

#ifdef EXAMPLES

// Пример: Guarded Variable
void exampleGuardedVariable() {
    GuardedVariable<float> temperature;

    // Установка значения
    temperature.set(25.5f);

    // Чтение значения
    float temp = temperature.get();
    printf("Temperature: %.1f\n", temp);

    // Атомарное обновление
    temperature.update(+[](float& value) {
        value += 1.0f;
    });
}

// Пример: Counting Semaphore для подсчёта событий
void exampleCountingSemaphore() {
    static CountingSemaphore eventCounter(10, 0);

    // В задаме-производителе
    eventCounter.give();  // Сигнал о событии

    // В задаме-потребителе
    if (eventCounter.take(100)) {
        // Обработка события
    }
}

// Пример: Event Group для синхронизации нескольких задач
void exampleEventGroup() {
    static EventGroup systemEvents;

    // Задача 1 ждёт бит 0 и бит 1
    uint32_t flags = systemEvents.wait(0x03, true, true, 1000);

    // Задача 2 устанавливает биты
    systemEvents.set(0x01);

    // Задача 3 очищает биты
    systemEvents.clear(0x01);
}

// Пример: Timer для периодических действий
void exampleTimer() {
    static Timer periodicTimer("PeriodicTimer", 1000, true, +[]() {
        printf("Timer callback!\n");
    });

    periodicTimer.start();
}

#endif // EXAMPLES
