/**
 * @file health_monitor.hpp
 * @brief Интеграция health monitoring драйверов с FDIR системой
 *
 * Автоматический мониторинг здоровья драйверов и интеграция с FDIR
 * для обнаружения, изоляции и восстановления неисправностей.
 *
 * Особенности:
 * - Автоматический опрос счётчиков ошибок драйверов
 * - Пороговые значения для warning/error/critical
 * - Автоматические recovery действия
 * - Логирование в FDIR event log
 * - Поддержка любых драйверов с getErrorCount()/getTimeoutCount()
 */

#ifndef HEALTH_MONITOR_HPP
#define HEALTH_MONITOR_HPP

#include <cstdint>
#include <cstddef>
#include <array>
#include <functional>

#include "systems/fdir.hpp"

namespace mka {
namespace systems {

// ============================================================================
// Конфигурация мониторинга здоровья
// ============================================================================

/// Конфигурация монитора здоровья драйвера
struct DriverHealthConfig {
    const char* driverName;               // Имя драйвера (для логов)
    uint8_t subsystemId;                  // ID подсистемы (из FDIR::Subsystem)
    uint8_t driverId;                     // Уникальный ID драйвера в подсистеме
    
    // Пороги ошибок
    uint32_t warningThreshold;            // Порог warning
    uint32_t errorThreshold;              // Порог error
    uint32_t criticalThreshold;           // Порог critical
    
    // Recovery действия
    fdir::RecoveryAction warningAction;   // Действие при warning
    fdir::RecoveryAction errorAction;     // Действие при error
    fdir::RecoveryAction criticalAction;  // Действие при critical
    
    // Период проверки
    uint32_t checkIntervalMs;             // Интервал проверки (мс)
};

// ============================================================================
// Абстрактный интерфейс для драйверов
// ============================================================================

/**
 * @brief Интерфейс для драйверов с health monitoring
 * 
 * Любой драйвер может быть интегрирован с HealthMonitor,
 * если реализует этот интерфейс.
 */
class IHealthMonitorDriver {
public:
    virtual ~IHealthMonitorDriver() = default;
    
    /// Получить количество ошибок
    virtual uint32_t getErrorCount() const = 0;
    
    /// Получить количество таймаутов
    virtual uint32_t getTimeoutCount() const = 0;
    
    /// Сбросить счётчики ошибок
    virtual void resetErrorCounters() = 0;
    
    /// Получить имя драйвера
    virtual const char* getDriverName() const = 0;
    
    /// Проверка здоровья (может включать самопроверку)
    virtual bool isHealthy() const { return true; }
    
    /// Восстановление после ошибки
    virtual bool recover() { 
        resetErrorCounters(); 
        return true; 
    }
};

// ============================================================================
// Менеджер здоровья драйверов
// ============================================================================

/**
 * @brief Менеджер здоровья драйверов с интеграцией FDIR
 * 
 * Мониторит счётчики ошибок драйверов и автоматически
 * создаёт события в FDIR системе при превышении порогов.
 * 
 * Пример использования:
 * @code
 * HealthMonitorManager healthMgr;
 * 
 * // Регистрация драйвера
 * healthMgr.registerDriver(&imuDriver, {
 *     .driverName = "BMI160",
 *     .subsystemId = static_cast<uint8_t>(fdir::Subsystem::ADCS),
 *     .driverId = 0,
 *     .warningThreshold = 5,
 *     .errorThreshold = 10,
 *     .criticalThreshold = 20,
 *     .warningAction = fdir::RecoveryAction::LOG_ONLY,
 *     .errorAction = fdir::RecoveryAction::RESET_SENSOR,
 *     .criticalAction = fdir::RecoveryAction::SAFE_MODE,
 *     .checkIntervalMs = 1000
 * });
 * 
 * // Проверка здоровья
 * healthMgr.checkAllDrivers(currentTime);
 * @endcode
 */
class HealthMonitorManager {
public:
    static constexpr size_t MAX_DRIVERS = 16;
    
    HealthMonitorManager() = default;
    
    /**
     * @brief Инициализация менеджера
     * @param fdirManager Указатель на FDIR менеджер
     * @return true при успешной инициализации
     */
    bool init(fdir::FDIRManager* fdirManager) {
        if (!fdirManager) {
            return false;
        }
        fdirManager_ = fdirManager;
        return true;
    }
    
    /**
     * @brief Регистрация драйвера в системе здоровья
     * @param driver Указатель на драйвер
     * @param config Конфигурация мониторинга
     * @return true при успешной регистрации
     */
    bool registerDriver(IHealthMonitorDriver* driver, const DriverHealthConfig& config) {
        if (!driver || !fdirManager_) {
            return false;
        }
        
        if (driverCount_ >= MAX_DRIVERS) {
            return false;
        }
        
        drivers_[driverCount_] = driver;
        configs_[driverCount_] = config;
        lastErrorCounts_[driverCount_] = 0;
        lastCheckTime_[driverCount_] = 0;
        currentSeverity_[driverCount_] = fdir::Severity::INFO;
        
        driverCount_++;
        return true;
    }
    
    /**
     * @brief Проверка здоровья всех зарегистрированных драйверов
     * @param currentTimeMs Текущее время в миллисекундах
     */
    void checkAllDrivers(uint32_t currentTimeMs) {
        for (size_t i = 0; i < driverCount_; i++) {
            checkDriver(i, currentTimeMs);
        }
    }
    
    /**
     * @brief Проверка здоровья конкретного драйвера
     * @param index Индекс драйвера
     * @param currentTimeMs Текущее время в миллисекундах
     */
    void checkDriver(size_t index, uint32_t currentTimeMs) {
        if (index >= driverCount_) {
            return;
        }
        
        auto* driver = drivers_[index];
        const auto& config = configs_[index];
        
        // Проверяем интервал (с защитой от wrap uint32_t)
        uint32_t elapsed = (currentTimeMs >= lastCheckTime_[index]) ? 
                          (currentTimeMs - lastCheckTime_[index]) : 0;
        if (elapsed < config.checkIntervalMs) {
            return;
        }

        lastCheckTime_[index] = currentTimeMs;

        // Получаем количество ошибок
        uint32_t errorCount = driver->getErrorCount();
        // Защита от wrap: если счётчик сброшен между проверками
        uint32_t newErrors = (errorCount >= lastErrorCounts_[index]) ? 
                            (errorCount - lastErrorCounts_[index]) : errorCount;
        
        // Если ошибок нет, сбрасываем серьёзность
        if (newErrors == 0 && driver->isHealthy()) {
            if (currentSeverity_[index] != fdir::Severity::INFO) {
                currentSeverity_[index] = fdir::Severity::INFO;
            }
            return;
        }
        
        // Определяем серьёзность
        fdir::Severity newSeverity = fdir::Severity::INFO;
        fdir::RecoveryAction recoveryAction = fdir::RecoveryAction::NONE;
        
        if (newErrors >= config.criticalThreshold) {
            newSeverity = fdir::Severity::CRITICAL;
            recoveryAction = config.criticalAction;
        } else if (newErrors >= config.errorThreshold) {
            newSeverity = fdir::Severity::ERROR;
            recoveryAction = config.errorAction;
        } else if (newErrors >= config.warningThreshold) {
            newSeverity = fdir::Severity::WARNING;
            recoveryAction = config.warningAction;
        }
        
        // Если серьёзность повысилась, создаём событие
        if (static_cast<uint8_t>(newSeverity) > static_cast<uint8_t>(currentSeverity_[index])) {
            // Формируем код ошибки
            fdir::ErrorCode errorCode = generateErrorCode(config.subsystemId, config.driverId, newSeverity);
            
            // Создаём событие в FDIR
            fdirManager_->recordDriverEvent(
                errorCode,
                newSeverity,
                static_cast<fdir::Subsystem>(config.subsystemId),
                config.driverId,
                static_cast<int16_t>(errorCount & 0x7FFF),
                static_cast<int16_t>(newErrors & 0x7FFF)
            );
            
            // Выполняем recovery действие
            if (recoveryAction != fdir::RecoveryAction::NONE && 
                recoveryAction != fdir::RecoveryAction::LOG_ONLY) {
                driver->recover();
                lastErrorCounts_[index] = driver->getErrorCount();
            }
            
            currentSeverity_[index] = newSeverity;
        }
        
        lastErrorCounts_[index] = errorCount;
    }
    
    /**
     * @brief Получить текущую серьёзность для драйвера
     * @param index Индекс драйвера
     * @return Текущая серьёзность
     */
    fdir::Severity getDriverSeverity(size_t index) const {
        if (index >= driverCount_) {
            return fdir::Severity::INFO;
        }
        return currentSeverity_[index];
    }
    
    /**
     * @brief Получить статистику здоровья
     * @param healthyCount[out] Количество здоровых драйверов
     * @param warningCount[out] Количество драйверов с warning
     * @param errorCount[out] Количество драйверов с error
     * @param criticalCount[out] Количество драйверов с critical
     */
    void getHealthStats(uint8_t& healthyCount, uint8_t& warningCount, 
                        uint8_t& errorCount, uint8_t& criticalCount) const {
        healthyCount = 0;
        warningCount = 0;
        errorCount = 0;
        criticalCount = 0;
        
        for (size_t i = 0; i < driverCount_; i++) {
            switch (currentSeverity_[i]) {
                case fdir::Severity::INFO:
                    healthyCount++;
                    break;
                case fdir::Severity::WARNING:
                    warningCount++;
                    break;
                case fdir::Severity::ERROR:
                    errorCount++;
                    break;
                case fdir::Severity::CRITICAL:
                    criticalCount++;
                    break;
            }
        }
    }
    
    /**
     * @brief Сбросить счётчики всех драйверов
     */
    void resetAllDrivers() {
        for (size_t i = 0; i < driverCount_; i++) {
            drivers_[i]->resetErrorCounters();
            lastErrorCounts_[i] = 0;
            currentSeverity_[i] = fdir::Severity::INFO;
        }
    }
    
    /**
     * @brief Получить количество зарегистрированных драйверов
     */
    size_t getDriverCount() const { return driverCount_; }
    
private:
    fdir::FDIRManager* fdirManager_ = nullptr;
    
    IHealthMonitorDriver* drivers_[MAX_DRIVERS];
    DriverHealthConfig configs_[MAX_DRIVERS];
    uint32_t lastErrorCounts_[MAX_DRIVERS];
    uint32_t lastCheckTime_[MAX_DRIVERS];
    fdir::Severity currentSeverity_[MAX_DRIVERS];
    size_t driverCount_ = 0;
    
    /**
     * @brief Генерация кода ошибки для драйвера
     */
    fdir::ErrorCode generateErrorCode(uint8_t subsystemId, uint8_t driverId, 
                                       fdir::Severity severity) const {
        // Формат: 0xXXYY, где XX - подсистема, YY - драйвер + severity
        uint16_t code = (static_cast<uint16_t>(subsystemId) << 8) | 
                        (static_cast<uint16_t>(driverId) << 4) |
                        static_cast<uint8_t>(severity);
        
        return static_cast<fdir::ErrorCode>(code);
    }
};

} // namespace systems
} // namespace mka

#endif // HEALTH_MONITOR_HPP
