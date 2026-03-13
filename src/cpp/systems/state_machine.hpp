/**
 * @file state_machine.hpp
 * @brief Satellite Mode State Machine
 * 
 * Машина состояний для управления режимами работы спутника.
 * Реализует переходы между режимами с проверкой условий
 * и выполнением действий при переходах.
 */

#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>

#include "utils/callback.hpp"

namespace mka {
namespace statemachine {

// ============================================================================
// Определение режимов
// ============================================================================

/**
 * @brief Режимы работы спутника
 * 
 * Иерархия режимов (от низшего к высшему):
 * SAFE -> STANDBY -> NOMINAL -> MISSION
 */
enum class SatelliteMode : uint8_t {
    OFF = 0,            // Выключен
    INIT = 1,           // Инициализация
    SAFE = 2,           // Безопасный режим (минимальное потребление)
    STANDBY = 3,        // Дежурный режим (ожидание команд)
    NOMINAL = 4,        // Номинальный режим (штатная работа)
    MISSION = 5,        // Режим миссии (активная полезная нагрузка)
    MAINTENANCE = 6,    // Режим обслуживания
    EMERGENCY = 7       // Аварийный режим
};

/**
 * @brief Причины перехода между режимами
 */
enum class TransitionReason : uint8_t {
    COMMAND,            // Команда от НКУ
    AUTONOMOUS,         // Автономное решение бортового ПО
    POWER_LOW,          // Низкий заряд батареи
    POWER_CRITICAL,     // Критический заряд батареи
    THERMAL_WARNING,    // Температурное предупреждение
    THERMAL_CRITICAL,   // Критическая температура
    ADCS_FAILURE,       // Отказ системы ориентации
    COMM_FAILURE,       // Отказ связи
    PAYLOAD_FAILURE,    // Отказ полезной нагрузки
    WATCHDOG,           // Сработал watchdog
    TIMEOUT,            // Таймаут ожидания
    INIT_COMPLETE,      // Завершение инициализации
    ERROR_RECOVERY      // Восстановление после ошибки
};

// ============================================================================
// Структуры данных
// ============================================================================

/**
 * @brief Результат попытки перехода
 */
struct TransitionResult {
    bool success;
    SatelliteMode previousMode;
    SatelliteMode newMode;
    TransitionReason reason;
    const char* errorMessage;
};

/**
 * @brief Контекст машины состояний
 */
struct StateMachineContext {
    float batteryLevel;         // Уровень заряда батареи (%)
    float batteryVoltage;       // Напряжение батареи (В)
    float temperatureOBC;       // Температура OBC (°C)
    float temperatureExternal;  // Внешняя температура (°C)
    bool antennaDeployed;       // Антенна развернута
    bool solarPanelsDeployed;   // Солнечные панели развернуты
    bool adcsNominal;           // ADCS в норме
    bool commNominal;           // COMM в норме
    bool payloadNominal;        // Полезная нагрузка в норме
    uint32_t uptime;            // Время работы (секунды)
    uint32_t lastContactTime;   // Время последней связи с НКУ
    uint8_t errorFlags;         // Флаги ошибок
};

// ============================================================================
// Валидаторы переходов
// ============================================================================

/**
 * @brief Проверка возможности перехода
 */
using TransitionValidator = Callback<bool(const StateMachineContext&)>;

/**
 * @brief Действие при переходе
 */
using TransitionAction = Callback<void(SatelliteMode, SatelliteMode, TransitionReason)>;

// ============================================================================
// Конфигурация переходов
// ============================================================================

struct TransitionConfig {
    SatelliteMode fromMode;
    SatelliteMode toMode;
    TransitionValidator validator;
    TransitionAction preAction;    // Выполняется до перехода
    TransitionAction postAction;   // Выполняется после перехода
    uint32_t timeoutMs;            // Таймаут для перехода
};

// ============================================================================
// Машина состояний
// ============================================================================

/**
 * @brief Машина состояний спутника
 */
class SatelliteStateMachine {
public:
    static constexpr size_t MAX_TRANSITIONS = 32;
    static constexpr size_t MODE_HISTORY_SIZE = 16;

    using ModeChangeCallback = Callback<void(SatelliteMode, SatelliteMode, TransitionReason)>;

    SatelliteStateMachine() {
        initializeDefaultTransitions();
    }
    
    // ========================================================================
    // Управление переходами
    // ========================================================================
    
    /**
     * @brief Запрос перехода в новый режим
     */
    TransitionResult requestTransition(SatelliteMode targetMode, 
                                       TransitionReason reason = TransitionReason::COMMAND) {
        TransitionResult result{};
        result.previousMode = currentMode_;
        result.newMode = targetMode;
        result.reason = reason;
        
        // Проверка допустимости перехода
        if (!isTransitionAllowed(currentMode_, targetMode)) {
            result.success = false;
            result.errorMessage = "Transition not allowed";
            return result;
        }
        
        // Поиск конфигурации перехода
        const TransitionConfig* config = findTransition(currentMode_, targetMode);
        
        // Валидация условий
        if (config && config->validator) {
            if (!config->validator(context_)) {
                result.success = false;
                result.errorMessage = "Transition conditions not met";
                return result;
            }
        }
        
        // Выполнение перехода
        SatelliteMode oldMode = currentMode_;
        
        // Pre-action
        if (config && config->preAction) {
            config->preAction(oldMode, targetMode, reason);
        }
        
        // Переход
        currentMode_ = targetMode;
        lastTransitionReason_ = reason;
        transitionsCount_++;
        
        // Запись в историю
        addToHistory(oldMode, targetMode, reason);
        
        // Post-action
        if (config && config->postAction) {
            config->postAction(oldMode, targetMode, reason);
        }
        
        // Уведомление
        if (modeChangeCallback_) {
            modeChangeCallback_(oldMode, targetMode, reason);
        }
        
        result.success = true;
        result.errorMessage = nullptr;
        return result;
    }
    
    /**
     * @brief Принудительный переход (без проверки условий)
     */
    void forceTransition(SatelliteMode targetMode, TransitionReason reason) {
        SatelliteMode oldMode = currentMode_;
        currentMode_ = targetMode;
        lastTransitionReason_ = reason;
        addToHistory(oldMode, targetMode, reason);
        
        if (modeChangeCallback_) {
            modeChangeCallback_(oldMode, targetMode, reason);
        }
    }
    
    /**
     * @brief Переход в безопасный режим (аварийный)
     */
    void enterSafeMode(TransitionReason reason) {
        forceTransition(SatelliteMode::SAFE, reason);
    }
    
    // ========================================================================
    // Автоматическое управление режимами
    // ========================================================================
    
    /**
     * @brief Автоматическая проверка и смена режима
     * 
     * Вызывается периодически для проверки условий автоматических переходов.
     */
    void autonomousCheck() {
        // Проверка питания
        if (context_.batteryLevel < 10.0f && currentMode_ != SatelliteMode::SAFE) {
            enterSafeMode(TransitionReason::POWER_CRITICAL);
            return;
        }
        
        if (context_.batteryLevel < 20.0f && 
            currentMode_ == SatelliteMode::MISSION) {
            requestTransition(SatelliteMode::NOMINAL, TransitionReason::POWER_LOW);
            return;
        }
        
        // Проверка температуры
        if (context_.temperatureOBC > 60.0f || context_.temperatureOBC < -20.0f) {
            if (currentMode_ != SatelliteMode::SAFE) {
                enterSafeMode(TransitionReason::THERMAL_CRITICAL);
                return;
            }
        }
        
        // Проверка subsystem failures
        if (!context_.adcsNominal && currentMode_ == SatelliteMode::MISSION) {
            requestTransition(SatelliteMode::NOMINAL, TransitionReason::ADCS_FAILURE);
        }
        
        // Восстановление из SAFE при улучшении условий
        if (currentMode_ == SatelliteMode::SAFE) {
            if (context_.batteryLevel > 30.0f && 
                context_.temperatureOBC > 0.0f && 
                context_.temperatureOBC < 50.0f) {
                requestTransition(SatelliteMode::STANDBY, TransitionReason::AUTONOMOUS);
            }
        }
        
        // Автоматический переход в NOMINAL из STANDBY
        if (currentMode_ == SatelliteMode::STANDBY) {
            if (context_.batteryLevel > 50.0f && 
                context_.adcsNominal && 
                context_.commNominal) {
                requestTransition(SatelliteMode::NOMINAL, TransitionReason::AUTONOMOUS);
            }
        }
    }
    
    // ========================================================================
    // Доступ к состоянию
    // ========================================================================
    
    SatelliteMode getCurrentMode() const { return currentMode_; }
    TransitionReason getLastTransitionReason() const { return lastTransitionReason_; }
    uint32_t getTransitionsCount() const { return transitionsCount_; }
    uint32_t getTimeInCurrentMode() const { return timeInMode_; }
    
    void updateContext(const StateMachineContext& ctx) { context_ = ctx; }
    const StateMachineContext& getContext() const { return context_; }
    
    void setModeChangeCallback(ModeChangeCallback callback) {
        modeChangeCallback_ = callback;
    }
    
    /**
     * @brief Обновление времени в текущем режиме
     */
    void tick(uint32_t deltaSeconds) {
        timeInMode_ += deltaSeconds;
    }
    
    // ========================================================================
    // История переходов
    // ========================================================================
    
    struct HistoryEntry {
        uint32_t timestamp;
        SatelliteMode fromMode;
        SatelliteMode toMode;
        TransitionReason reason;
    };
    
    size_t getHistorySize() const { return historyCount_; }
    
    bool getHistoryEntry(size_t index, HistoryEntry& entry) const {
        if (index >= historyCount_) return false;
        size_t actualIndex = (historyHead_ + index) % MODE_HISTORY_SIZE;
        entry = history_[actualIndex];
        return true;
    }
    
    void clearHistory() {
        historyCount_ = 0;
        historyHead_ = 0;
    }
    
    // ========================================================================
    // Информация о режимах
    // ========================================================================
    
    static const char* getModeName(SatelliteMode mode) {
        switch (mode) {
            case SatelliteMode::OFF:         return "OFF";
            case SatelliteMode::INIT:        return "INIT";
            case SatelliteMode::SAFE:        return "SAFE";
            case SatelliteMode::STANDBY:     return "STANDBY";
            case SatelliteMode::NOMINAL:     return "NOMINAL";
            case SatelliteMode::MISSION:     return "MISSION";
            case SatelliteMode::MAINTENANCE: return "MAINTENANCE";
            case SatelliteMode::EMERGENCY:   return "EMERGENCY";
            default: return "UNKNOWN";
        }
    }
    
    static const char* getReasonName(TransitionReason reason) {
        switch (reason) {
            case TransitionReason::COMMAND:          return "COMMAND";
            case TransitionReason::AUTONOMOUS:       return "AUTONOMOUS";
            case TransitionReason::POWER_LOW:        return "POWER_LOW";
            case TransitionReason::POWER_CRITICAL:   return "POWER_CRITICAL";
            case TransitionReason::THERMAL_WARNING:  return "THERMAL_WARNING";
            case TransitionReason::THERMAL_CRITICAL: return "THERMAL_CRITICAL";
            case TransitionReason::ADCS_FAILURE:     return "ADCS_FAILURE";
            case TransitionReason::COMM_FAILURE:     return "COMM_FAILURE";
            case TransitionReason::PAYLOAD_FAILURE:  return "PAYLOAD_FAILURE";
            case TransitionReason::WATCHDOG:         return "WATCHDOG";
            case TransitionReason::TIMEOUT:          return "TIMEOUT";
            case TransitionReason::INIT_COMPLETE:    return "INIT_COMPLETE";
            case TransitionReason::ERROR_RECOVERY:   return "ERROR_RECOVERY";
            default: return "UNKNOWN";
        }
    }
    
private:
    SatelliteMode currentMode_ = SatelliteMode::OFF;
    TransitionReason lastTransitionReason_ = TransitionReason::COMMAND;
    uint32_t transitionsCount_ = 0;
    uint32_t timeInMode_ = 0;
    
    StateMachineContext context_{};
    
    ModeChangeCallback modeChangeCallback_;
    
    std::array<TransitionConfig, MAX_TRANSITIONS> transitions_{};
    size_t transitionCount_ = 0;
    
    std::array<HistoryEntry, MODE_HISTORY_SIZE> history_{};
    size_t historyHead_ = 0;
    size_t historyCount_ = 0;
    
    // ========================================================================
    // Внутренние методы
    // ========================================================================
    
    void initializeDefaultTransitions() {
        // OFF -> INIT
        addTransition(SatelliteMode::OFF, SatelliteMode::INIT, 
            [](const StateMachineContext& ctx) { return true; });
        
        // INIT -> SAFE
        addTransition(SatelliteMode::INIT, SatelliteMode::SAFE,
            [](const StateMachineContext& ctx) { return ctx.antennaDeployed; });
        
        // SAFE -> STANDBY
        addTransition(SatelliteMode::SAFE, SatelliteMode::STANDBY,
            [](const StateMachineContext& ctx) {
                return ctx.batteryLevel > 30.0f && 
                       ctx.temperatureOBC > -10.0f && 
                       ctx.temperatureOBC < 55.0f;
            });
        
        // STANDBY -> NOMINAL
        addTransition(SatelliteMode::STANDBY, SatelliteMode::NOMINAL,
            [](const StateMachineContext& ctx) {
                return ctx.batteryLevel > 50.0f && 
                       ctx.adcsNominal && 
                       ctx.commNominal;
            });
        
        // NOMINAL -> MISSION
        addTransition(SatelliteMode::NOMINAL, SatelliteMode::MISSION,
            [](const StateMachineContext& ctx) {
                return ctx.batteryLevel > 70.0f && 
                       ctx.adcsNominal && 
                       ctx.commNominal &&
                       ctx.payloadNominal;
            });
        
        // Обратные переходы (всегда разрешены для снижения режима)
        addTransition(SatelliteMode::MISSION, SatelliteMode::NOMINAL, nullptr);
        addTransition(SatelliteMode::NOMINAL, SatelliteMode::STANDBY, nullptr);
        addTransition(SatelliteMode::STANDBY, SatelliteMode::SAFE, nullptr);
        
        // SAFE можно войти из любого режима
        addTransition(SatelliteMode::NOMINAL, SatelliteMode::SAFE, nullptr);
        addTransition(SatelliteMode::MISSION, SatelliteMode::SAFE, nullptr);
        
        // MAINTENANCE
        addTransition(SatelliteMode::NOMINAL, SatelliteMode::MAINTENANCE, nullptr);
        addTransition(SatelliteMode::MAINTENANCE, SatelliteMode::NOMINAL, nullptr);
        
        // EMERGENCY
        addTransition(SatelliteMode::SAFE, SatelliteMode::EMERGENCY, nullptr);
        addTransition(SatelliteMode::EMERGENCY, SatelliteMode::SAFE, nullptr);
    }
    
    void addTransition(SatelliteMode from, SatelliteMode to, 
                       TransitionValidator validator) {
        if (transitionCount_ < MAX_TRANSITIONS) {
            transitions_[transitionCount_++] = {from, to, validator, nullptr, nullptr, 5000};
        }
    }
    
    bool isTransitionAllowed(SatelliteMode from, SatelliteMode to) {
        // Переход в тот же режим разрешён (no-op)
        if (from == to) return true;
        
        return findTransition(from, to) != nullptr;
    }
    
    const TransitionConfig* findTransition(SatelliteMode from, SatelliteMode to) {
        for (size_t i = 0; i < transitionCount_; i++) {
            if (transitions_[i].fromMode == from && transitions_[i].toMode == to) {
                return &transitions_[i];
            }
        }
        return nullptr;
    }
    
    void addToHistory(SatelliteMode from, SatelliteMode to, TransitionReason reason) {
        history_[historyHead_].timestamp = context_.uptime;
        history_[historyHead_].fromMode = from;
        history_[historyHead_].toMode = to;
        history_[historyHead_].reason = reason;
        
        historyHead_ = (historyHead_ + 1) % MODE_HISTORY_SIZE;
        if (historyCount_ < MODE_HISTORY_SIZE) historyCount_++;
        
        timeInMode_ = 0;
    }
};

} // namespace statemachine
} // namespace mka

#endif // STATE_MACHINE_HPP
