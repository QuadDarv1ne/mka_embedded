/**
 * @file watchdog_manager.hpp
 * @brief Watchdog Manager for MKA
 * 
 * Менеджер сторожевого таймера с поддержкой:
 * - Множественных "виртуальных" watchdog'ов для задач
 * - Контроля зависания задач
 * - Диагностики причин сброса
 */

#ifndef WATCHDOG_MANAGER_HPP
#define WATCHDOG_MANAGER_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>

#include "utils/callback.hpp"
#include "utils/span.hpp"

namespace mka {
namespace wdt {

// ============================================================================
// Константы
// ============================================================================

constexpr size_t MAX_WATCHDOG_TASKS = 16;
constexpr uint32_t DEFAULT_TIMEOUT_MS = 5000;

// ============================================================================
// Причины сброса
// ============================================================================

enum class ResetReason : uint8_t {
    POWER_ON = 0,          // Включение питания
    BROWN_OUT = 1,         // Снижение напряжения
    WATCHDOG = 2,          // Сторожевой таймер
    SOFTWARE = 3,          // Программный сброс
    EXTERNAL = 4,          // Внешний сброс
    UNKNOWN = 0xFF
};

// ============================================================================
// Интерфейс аппаратного Watchdog
// ============================================================================

class IHardwareWatchdog {
public:
    virtual ~IHardwareWatchdog() = default;
    
    /// Инициализация с периодом в миллисекундах
    virtual bool init(uint32_t timeoutMs) = 0;
    
    /// Обновить (кикнуть) watchdog
    virtual void refresh() = 0;
    
    /// Получить причину последнего сброса
    virtual ResetReason getResetReason() = 0;
    
    /// Проверить, был ли сброс от watchdog
    virtual bool wasWatchdogReset() = 0;
};

// ============================================================================
// Виртуальный Watchdog для задач
// ============================================================================

struct VirtualWatchdog {
    uint8_t taskId;              // Идентификатор задачи
    uint32_t timeoutMs;          // Таймаут в мс
    uint32_t lastKickTime;       // Время последнего кика
    const char* taskName;        // Имя задачи
    bool enabled;                // Включён ли мониторинг
    bool expired;                // Истёк ли таймаут
    uint32_t expireCount;        // Количество истечений
};

// Валидация размеров структур
static_assert(sizeof(VirtualWatchdog) <= 32, "VirtualWatchdog слишком велик");

// ============================================================================
// Watchdog Manager
// ============================================================================

/**
 * @brief Менеджер сторожевого таймера
 * 
 * Управляет множественными виртуальными watchdog'ами для задач RTOS.
 * Каждая задача должна периодически "кикать" свой watchdog.
 * Если задача не кикает в течение таймаута, считается зависшей.
 * 
 * Пример использования:
 * 
 * void adcTask(void* param) {
 *     auto wdtId = WatchdogManager::instance().registerTask("ADC", 1000);
 *     
 *     while (1) {
 *         // ... работа задачи ...
 *         
 *         WatchdogManager::instance().kick(wdtId);
 *         vTaskDelay(100);
 *     }
 * }
 */
class WatchdogManager {
public:
    static WatchdogManager& instance() {
        static WatchdogManager instance;
        return instance;
    }

    WatchdogManager() = default;

    /**
     * @brief Инициализация менеджера
     */
    bool init(IHardwareWatchdog* hwWdt, uint32_t hwTimeoutMs = DEFAULT_TIMEOUT_MS) {
        hwWdt_ = hwWdt;

        if (hwWdt_) {
            if (!hwWdt_->init(hwTimeoutMs)) {
                return false;
            }
        }

        // Сохранение причины сброса
        lastResetReason_ = hwWdt_ ? hwWdt_->getResetReason() : ResetReason::UNKNOWN;

        initialized_ = true;
        return true;
    }

    /**
     * @brief Регистрация задачи для мониторинга
     * @param name Имя задачи
     * @param timeoutMs Таймаут в миллисекундах
     * @return ID виртуального watchdog'а или 0xFF при ошибке
     */
    uint8_t registerTask(const char* name, uint32_t timeoutMs) {
        if (!initialized_ || taskCount_ >= MAX_WATCHDOG_TASKS) {
            return 0xFF;
        }

        uint8_t id = taskCount_++;

        virtualWdts_[id].taskId = id;
        virtualWdts_[id].timeoutMs = timeoutMs;
        virtualWdts_[id].lastKickTime = getTickMs();
        virtualWdts_[id].taskName = name;
        virtualWdts_[id].enabled = true;
        virtualWdts_[id].expired = false;
        virtualWdts_[id].expireCount = 0;

        return id;
    }
    
    /**
     * @brief "Кикнуть" виртуальный watchdog
     * @param taskId ID задачи, полученный при регистрации
     */
    void kick(uint8_t taskId) {
        if (taskId >= taskCount_) return;
        
        virtualWdts_[taskId].lastKickTime = getTickMs();
        virtualWdts_[taskId].expired = false;
    }
    
    /**
     * @brief Обновление менеджера (вызывать из основной задачи)
     * 
     * Проверяет все виртуальные watchdog'и и кикает аппаратный,
     * только если все задачи живы.
     */
    void update() {
        if (!initialized_) return;
        
        uint32_t now = getTickMs();
        bool allAlive = true;
        
        // Проверка всех виртуальных watchdog'ов
        for (size_t i = 0; i < taskCount_; i++) {
            auto& wdt = virtualWdts_[i];
            
            if (!wdt.enabled) continue;
            
            if (now - wdt.lastKickTime > wdt.timeoutMs) {
                // Задача зависла
                if (!wdt.expired) {
                    wdt.expired = true;
                    wdt.expireCount++;
                    
                    // Логирование
                    onTaskExpired(wdt.taskName, now - wdt.lastKickTime);
                }
                allAlive = false;
            }
        }
        
        // Кик аппаратного watchdog'а только если все живы
        if (allAlive && hwWdt_) {
            hwWdt_->refresh();
        }
    }
    
    /**
     * @brief Принудительный кик аппаратного watchdog'а
     */
    void forceKick() {
        if (hwWdt_) {
            hwWdt_->refresh();
        }
    }
    
    /**
     * @brief Приостановка мониторинга задачи
     */
    void suspendTask(uint8_t taskId) {
        if (taskId < taskCount_) {
            virtualWdts_[taskId].enabled = false;
        }
    }
    
    /**
     * @brief Возобновление мониторинга задачи
     */
    void resumeTask(uint8_t taskId) {
        if (taskId < taskCount_) {
            virtualWdts_[taskId].enabled = true;
            virtualWdts_[taskId].lastKickTime = getTickMs();
            virtualWdts_[taskId].expired = false;
        }
    }
    
    /**
     * @brief Установка нового таймаута для задачи
     */
    void setTimeout(uint8_t taskId, uint32_t timeoutMs) {
        if (taskId < taskCount_) {
            virtualWdts_[taskId].timeoutMs = timeoutMs;
        }
    }
    
    // ========================================================================
    // Диагностика
    // ========================================================================
    
    /**
     * @brief Проверка, зависла ли задача
     */
    bool isTaskExpired(uint8_t taskId) const {
        if (taskId >= taskCount_) return false;
        return virtualWdts_[taskId].expired;
    }
    
    /**
     * @brief Получение количества истечений таймаута
     */
    uint32_t getExpireCount(uint8_t taskId) const {
        if (taskId >= taskCount_) return 0;
        return virtualWdts_[taskId].expireCount;
    }
    
    /**
     * @brief Получение причины последнего сброса
     */
    ResetReason getLastResetReason() const {
        return lastResetReason_;
    }
    
    /**
     * @brief Проверка, был ли последний сброс от watchdog
     */
    bool wasWatchdogReset() const {
        return lastResetReason_ == ResetReason::WATCHDOG;
    }
    
    /**
     * @brief Получение имени зависшей задачи
     * @return Имя или nullptr если все живы
     */
    const char* getExpiredTaskName() const {
        for (size_t i = 0; i < taskCount_; i++) {
            if (virtualWdts_[i].expired) {
                return virtualWdts_[i].taskName;
            }
        }
        return nullptr;
    }
    
    /**
     * @brief Установка callback для события истечения таймаута
     */
    void setExpireCallback(Callback<void(const char*, uint32_t)> callback) {
        expireCallback_ = callback;
    }
    
    // ========================================================================
    // Статистика
    // ========================================================================
    
    struct Statistics {
        uint8_t taskCount;
        uint8_t expiredCount;
        uint32_t totalExpireCount;
        ResetReason lastReset;
    };
    
    Statistics getStatistics() const {
        Statistics stats{};
        stats.taskCount = taskCount_;
        stats.lastReset = lastResetReason_;
        
        for (size_t i = 0; i < taskCount_; i++) {
            if (virtualWdts_[i].expired) {
                stats.expiredCount++;
            }
            stats.totalExpireCount += virtualWdts_[i].expireCount;
        }
        
        return stats;
    }

    /**
     * @brief Проверка инициализации
     */
    bool isInitialized() const { return initialized_; }

private:
    IHardwareWatchdog* hwWdt_ = nullptr;
    bool initialized_ = false;

    std::array<VirtualWatchdog, MAX_WATCHDOG_TASKS> virtualWdts_{};
    size_t taskCount_ = 0;

    ResetReason lastResetReason_ = ResetReason::UNKNOWN;

    Callback<void(const char*, uint32_t)> expireCallback_;
    
    void onTaskExpired(const char* taskName, uint32_t elapsedMs) {
        // Логирование (можно заменить на реальный логгер)
        // LOG_ERROR("WDT", "Task %s expired after %u ms", taskName, elapsedMs);
        
        if (expireCallback_) {
            expireCallback_(taskName, elapsedMs);
        }
    }
    
    uint32_t getTickMs() {
        // Platform-specific implementation
        // В реальном проекте использовать RTOS tick или аппаратный таймер
        return 0;
    }
};

// ============================================================================
// STM32 Watchdog Implementation
// ============================================================================

/**
 * @brief Реализация IHardwareWatchdog для STM32
 */
class STM32Watchdog : public IHardwareWatchdog {
public:
    STM32Watchdog() = default;
    
    bool init(uint32_t timeoutMs) override {
        // Настройка IWDG
        // В реальном проекте используется HAL:
        // hiwdg.Instance = IWDG;
        // hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
        // hiwdg.Init.Reload = (timeoutMs * 32000) / (256 * 1000);
        // HAL_IWDG_Init(&hiwdg);
        
        timeoutMs_ = timeoutMs;
        initialized_ = true;
        return true;
    }
    
    void refresh() override {
        if (!initialized_) return;
        
        // HAL_IWDG_Refresh(&hiwdg);
    }
    
    ResetReason getResetReason() override {
        // Проверка флагов RCC
        // if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
        //     return ResetReason::WATCHDOG;
        // if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
        //     return ResetReason::POWER_ON;
        // и т.д.
        
        return ResetReason::UNKNOWN;
    }
    
    bool wasWatchdogReset() override {
        return getResetReason() == ResetReason::WATCHDOG;
    }
    
private:
    uint32_t timeoutMs_ = 0;
    bool initialized_ = false;
};

// ============================================================================
// Helper Macros
// ============================================================================

#define WDT_REGISTER(name, timeout_ms) \
    mka::wdt::WatchdogManager::instance().registerTask(name, timeout_ms)

#define WDT_KICK(task_id) \
    mka::wdt::WatchdogManager::instance().kick(task_id)

#define WDT_UPDATE() \
    mka::wdt::WatchdogManager::instance().update()

} // namespace wdt
} // namespace mka

#endif // WATCHDOG_MANAGER_HPP
