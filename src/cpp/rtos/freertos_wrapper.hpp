/**
 * @file freertos_wrapper.hpp
 * @brief FreeRTOS C++ Wrapper for MKA
 * 
 * Удобные C++ обёртки для FreeRTOS API.
 * Предоставляет RAII-совместимые интерфейсы для задач,
 * очередей, семафоров, мьютексов и таймеров.
 */

#ifndef FREERTOS_WRAPPER_HPP
#define FREERTOS_WRAPPER_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <functional>
#include <array>
#include <span>
#include <optional>

// Forward declarations для FreeRTOS типов
typedef void* TaskHandle_t;
typedef void* QueueHandle_t;
typedef void* SemaphoreHandle_t;
typedef void* TimerHandle_t;
typedef void* EventGroupHandle_t;
typedef void* StreamBufferHandle_t;

// FreeRTOS API declarations (для компиляции без заголовков FreeRTOS)
extern "C" {
    // Task API
    TaskHandle_t xTaskCreateStatic(void(*pxTaskCode)(void*), const char* const pcName,
                                   uint32_t ulStackDepth, void* pvParameters,
                                   uint32_t uxPriority, uint8_t* puxStackBuffer,
                                   void* pxTaskBuffer);
    void vTaskDelete(TaskHandle_t xTask);
    void vTaskDelay(uint32_t xTicksToDelay);
    void vTaskDelayUntil(uint32_t* pxPreviousWakeTime, uint32_t xTimeIncrement);
    void vTaskSuspend(TaskHandle_t xTask);
    void vTaskResume(TaskHandle_t xTask);
    uint32_t xTaskGetTickCount();
    uint32_t uxTaskPriorityGet(TaskHandle_t xTask);
    void vTaskPrioritySet(TaskHandle_t xTask, uint32_t uxNewPriority);
    char* pcTaskGetName(TaskHandle_t xTask);
    
    // Queue API
    QueueHandle_t xQueueCreateStatic(uint32_t uxQueueLength, uint32_t uxItemSize,
                                     uint8_t* pucQueueStorage, void* pxStaticQueue);
    uint32_t xQueueSend(QueueHandle_t xQueue, const void* pvItemToQueue, uint32_t xTicksToWait);
    uint32_t xQueueReceive(QueueHandle_t xQueue, void* pvBuffer, uint32_t xTicksToWait);
    uint32_t uxQueueMessagesWaiting(QueueHandle_t xQueue);
    void vQueueDelete(QueueHandle_t xQueue);
    
    // Semaphore API
    SemaphoreHandle_t xSemaphoreCreateBinaryStatic(void* pxStaticSemaphore);
    SemaphoreHandle_t xSemaphoreCreateCountingStatic(uint32_t uxMaxCount, uint32_t uxInitialCount,
                                                      void* pxStaticSemaphore);
    SemaphoreHandle_t xSemaphoreCreateMutexStatic(void* pxStaticSemaphore);
    uint32_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, uint32_t xTicksToWait);
    uint32_t xSemaphoreGive(SemaphoreHandle_t xSemaphore);
    
    // Timer API
    TimerHandle_t xTimerCreateStatic(const char* pcTimerName, uint32_t xTimerPeriod,
                                     uint32_t uxAutoReload, void* pvTimerID,
                                     void(*pxCallbackFunction)(TimerHandle_t),
                                     void* pxTimerBuffer);
    uint32_t xTimerStart(TimerHandle_t xTimer, uint32_t xTicksToWait);
    uint32_t xTimerStop(TimerHandle_t xTimer, uint32_t xTicksToWait);
    uint32_t xTimerReset(TimerHandle_t xTimer, uint32_t xTicksToWait);
    
    // Event Group API
    EventGroupHandle_t xEventGroupCreateStatic(void* pxEventGroupBuffer);
    uint32_t xEventGroupWaitBits(EventGroupHandle_t xEventGroup, uint32_t uxBitsToWaitFor,
                                  uint32_t xClearOnExit, uint32_t xWaitForAllBits,
                                  uint32_t xTicksToWait);
    uint32_t xEventGroupSetBits(EventGroupHandle_t xEventGroup, uint32_t uxBitsToSet);
    uint32_t xEventGroupClearBits(EventGroupHandle_t xEventGroup, uint32_t uxBitsToClear);
}

namespace mka {
namespace rtos {

// ============================================================================
// Константы
// ============================================================================

#ifndef configTICK_RATE_HZ
constexpr uint32_t configTICK_RATE_HZ = 1000;  // По умолчанию 1 кГц (1 мс за тик)
#endif

constexpr uint32_t INFINITE_TIMEOUT = 0xFFFFFFFF;
constexpr uint32_t NO_TIMEOUT = 0;

// Преобразование миллисекунд в тики
constexpr uint32_t msToTicks(uint32_t ms) {
    return (ms * configTICK_RATE_HZ + 999) / 1000;
}

// ============================================================================
// Задача (Task)
// ============================================================================

/**
 * @brief Статическая задача FreeRTOS
 */
template<size_t StackSize>
class Task {
public:
    using TaskFunction = std::function<void()>;
    
    Task(const char* name, uint32_t priority, TaskFunction func)
        : name_(name), priority_(priority), func_(func) {}
    
    ~Task() {
        if (handle_) {
            vTaskDelete(handle_);
        }
    }
    
    bool start() {
        // Статическое создание задачи
        handle_ = xTaskCreateStatic(
            taskWrapper,
            name_,
            StackSize / 4,  // Размер в словах
            this,
            priority_,
            stack_.data(),
            &taskBuffer_
        );
        
        return handle_ != nullptr;
    }
    
    void suspend() {
        if (handle_) vTaskSuspend(handle_);
    }
    
    void resume() {
        if (handle_) vTaskResume(handle_);
    }
    
    uint32_t getPriority() const {
        return handle_ ? uxTaskPriorityGet(handle_) : 0;
    }
    
    void setPriority(uint32_t priority) {
        if (handle_) vTaskPrioritySet(handle_, priority);
    }
    
    const char* getName() const { return name_; }
    TaskHandle_t getHandle() const { return handle_; }
    
    static void delay(uint32_t ms) {
        vTaskDelay(msToTicks(ms));
    }
    
    static void delayUntil(uint32_t& lastWakeTime, uint32_t periodMs) {
        vTaskDelayUntil(&lastWakeTime, msToTicks(periodMs));
    }
    
    static uint32_t getTickCount() {
        return xTaskGetTickCount();
    }
    
private:
    const char* name_;
    uint32_t priority_;
    TaskFunction func_;
    TaskHandle_t handle_ = nullptr;
    
    alignas(8) std::array<uint8_t, StackSize> stack_{};
    // Static task buffer (platform-specific structure)
    uint8_t taskBuffer_[64]{};  // Placeholder для TCB
    
    static void taskWrapper(void* param) {
        Task* task = static_cast<Task*>(param);
        if (task && task->func_) {
            task->func_();
        }
        vTaskDelete(nullptr);
    }
};

// ============================================================================
// Очередь (Queue)
// ============================================================================

/**
 * @brief Статическая очередь FreeRTOS
 */
template<typename T, size_t Length>
class Queue {
public:
    Queue() {
        handle_ = xQueueCreateStatic(
            Length,
            sizeof(T),
            storage_.data(),
            &queueBuffer_
        );
    }
    
    ~Queue() {
        if (handle_) vQueueDelete(handle_);
    }
    
    bool send(const T& item, uint32_t timeoutMs = INFINITE_TIMEOUT) {
        uint32_t ticks = (timeoutMs == INFINITE_TIMEOUT) ? INFINITE_TIMEOUT : msToTicks(timeoutMs);
        return xQueueSend(handle_, &item, ticks) != 0;
    }
    
    bool receive(T& item, uint32_t timeoutMs = INFINITE_TIMEOUT) {
        uint32_t ticks = (timeoutMs == INFINITE_TIMEOUT) ? INFINITE_TIMEOUT : msToTicks(timeoutMs);
        return xQueueReceive(handle_, &item, ticks) != 0;
    }
    
    std::optional<T> receive(uint32_t timeoutMs = 0) {
        T item;
        if (receive(item, timeoutMs)) {
            return item;
        }
        return std::nullopt;
    }
    
    size_t messagesWaiting() const {
        return handle_ ? uxQueueMessagesWaiting(handle_) : 0;
    }
    
    bool isEmpty() const { return messagesWaiting() == 0; }
    bool isFull() const { return messagesWaiting() >= Length; }
    
    QueueHandle_t getHandle() const { return handle_; }
    
private:
    QueueHandle_t handle_ = nullptr;
    alignas(8) std::array<uint8_t, Length * sizeof(T)> storage_{};
    uint8_t queueBuffer_[80]{};  // Placeholder для StaticQueue_t
};

// ============================================================================
// Семафор (Semaphore)
// ============================================================================

/**
 * @brief Бинарный семафор
 */
class BinarySemaphore {
public:
    BinarySemaphore() {
        handle_ = xSemaphoreCreateBinaryStatic(&semBuffer_);
    }
    
    ~BinarySemaphore() {
        // Binary semaphores created statically don't need explicit deletion
    }
    
    bool take(uint32_t timeoutMs = INFINITE_TIMEOUT) {
        uint32_t ticks = (timeoutMs == INFINITE_TIMEOUT) ? INFINITE_TIMEOUT : msToTicks(timeoutMs);
        return xSemaphoreTake(handle_, ticks) != 0;
    }
    
    bool give() {
        return xSemaphoreGive(handle_) != 0;
    }
    
    bool tryTake() { return take(NO_TIMEOUT); }
    
    SemaphoreHandle_t getHandle() const { return handle_; }
    
private:
    SemaphoreHandle_t handle_ = nullptr;
    uint8_t semBuffer_[40]{};  // Placeholder
};

/**
 * @brief Считающий семафор
 */
class CountingSemaphore {
public:
    CountingSemaphore(uint32_t maxCount, uint32_t initialCount = 0) {
        handle_ = xSemaphoreCreateCountingStatic(maxCount, initialCount, &semBuffer_);
    }
    
    bool take(uint32_t timeoutMs = INFINITE_TIMEOUT) {
        uint32_t ticks = (timeoutMs == INFINITE_TIMEOUT) ? INFINITE_TIMEOUT : msToTicks(timeoutMs);
        return xSemaphoreTake(handle_, ticks) != 0;
    }
    
    bool give() {
        return xSemaphoreGive(handle_) != 0;
    }
    
private:
    SemaphoreHandle_t handle_ = nullptr;
    uint8_t semBuffer_[40]{};
};

/**
 * @brief Мьютекс
 */
class Mutex {
public:
    Mutex() {
        handle_ = xSemaphoreCreateMutexStatic(&mutexBuffer_);
    }
    
    bool lock(uint32_t timeoutMs = INFINITE_TIMEOUT) {
        uint32_t ticks = (timeoutMs == INFINITE_TIMEOUT) ? INFINITE_TIMEOUT : msToTicks(timeoutMs);
        return xSemaphoreTake(handle_, ticks) != 0;
    }
    
    bool unlock() {
        return xSemaphoreGive(handle_) != 0;
    }
    
    SemaphoreHandle_t getHandle() const { return handle_; }
    
private:
    SemaphoreHandle_t handle_ = nullptr;
    uint8_t mutexBuffer_[40]{};
};

/**
 * @brief RAII lock guard для мьютекса
 */
class MutexGuard {
public:
    explicit MutexGuard(Mutex& mutex) : mutex_(mutex) {
        mutex_.lock();
    }
    
    ~MutexGuard() {
        mutex_.unlock();
    }
    
    MutexGuard(const MutexGuard&) = delete;
    MutexGuard& operator=(const MutexGuard&) = delete;
    
private:
    Mutex& mutex_;
};

// ============================================================================
// Таймер (Timer)
// ============================================================================

/**
 * @brief Программный таймер
 */
class Timer {
public:
    using TimerCallback = std::function<void()>;
    
    Timer(const char* name, uint32_t periodMs, bool autoReload, TimerCallback callback)
        : name_(name), period_(periodMs), autoReload_(autoReload), callback_(callback) {}
    
    bool start() {
        if (!handle_) {
            handle_ = xTimerCreateStatic(
                name_,
                period_,
                autoReload_ ? 1 : 0,
                this,
                timerCallback,
                &timerBuffer_
            );
        }
        return handle_ ? xTimerStart(handle_, 100) != 0 : false;
    }
    
    bool stop() {
        return handle_ ? xTimerStop(handle_, 100) != 0 : false;
    }
    
    bool reset() {
        return handle_ ? xTimerReset(handle_, 100) != 0 : false;
    }
    
    bool isRunning() const { return running_; }
    
private:
    const char* name_;
    uint32_t period_;
    bool autoReload_;
    TimerCallback callback_;
    TimerHandle_t handle_ = nullptr;
    bool running_ = false;
    uint8_t timerBuffer_[40]{};
    
    static void timerCallback(TimerHandle_t timer) {
        // Extract Timer object and call callback
        // In real implementation, use pvTimerGetTimerID
    }
};

// ============================================================================
// Группа событий (Event Group)
// ============================================================================

/**
 * @brief Группа событий для синхронизации
 */
class EventGroup {
public:
    EventGroup() {
        handle_ = xEventGroupCreateStatic(&eventBuffer_);
    }
    
    uint32_t wait(uint32_t bits, bool clearOnExit = true, bool waitForAll = false,
                  uint32_t timeoutMs = INFINITE_TIMEOUT) {
        uint32_t ticks = (timeoutMs == INFINITE_TIMEOUT) ? INFINITE_TIMEOUT : msToTicks(timeoutMs);
        return xEventGroupWaitBits(handle_, bits, clearOnExit ? 1 : 0,
                                   waitForAll ? 1 : 0, ticks);
    }
    
    uint32_t set(uint32_t bits) {
        return xEventGroupSetBits(handle_, bits);
    }
    
    uint32_t clear(uint32_t bits) {
        return xEventGroupClearBits(handle_, bits);
    }
    
    bool isSet(uint32_t bit) const {
        // Check if specific bit is set
        return false;  // Simplified
    }
    
private:
    EventGroupHandle_t handle_ = nullptr;
    uint8_t eventBuffer_[40]{};
};

// ============================================================================
// Guarded Variable
// ============================================================================

/**
 * @brief Переменная с защитой мьютексом
 */
template<typename T>
class GuardedVariable {
public:
    GuardedVariable() = default;
    explicit GuardedVariable(const T& value) : value_(value) {}
    
    T get() {
        MutexGuard guard(mutex_);
        return value_;
    }
    
    void set(const T& value) {
        MutexGuard guard(mutex_);
        value_ = value;
    }
    
    void update(std::function<void(T&)> updater) {
        MutexGuard guard(mutex_);
        updater(value_);
    }
    
private:
    T value_{};
    Mutex mutex_;
};

// ============================================================================
// Предопределённые типы для МКА
// ============================================================================

// Телеметрическая очередь
struct TelemetryMessage {
    uint8_t apid;
    uint8_t priority;
    uint16_t length;
    std::array<uint8_t, 256> data;
};

using TelemetryQueue = Queue<TelemetryMessage, 32>;

// Очередь команд
struct CommandMessage {
    uint16_t commandId;
    uint16_t sequence;
    uint8_t source;
    std::array<uint8_t, 128> params;
    uint8_t paramsLen;
};

using CommandQueue = Queue<CommandMessage, 16>;

// Задачи МКА
using SatelliteTask = Task<2048>;  // 2KB stack

} // namespace rtos
} // namespace mka

#endif // FREERTOS_WRAPPER_HPP
