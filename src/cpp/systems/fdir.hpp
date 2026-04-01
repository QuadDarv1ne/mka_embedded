/**
 * @file fdir.hpp
 * @brief Fault Detection, Isolation and Recovery (FDIR) System
 *
 * Система обнаружения, изоляции и восстановления неисправностей
 * для бортового ПО малого космического аппарата.
 *
 * Ключевые компоненты:
 * - Мониторинг параметров с пороговыми значениями
 * - Детектор аномалий на основе статистики
 * - ML детектор аномалий (Isolation Forest)
 * - Автоматическое восстановление
 * - Журнал событий
 */

#ifndef FDIR_HPP
#define FDIR_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <cmath>

#include "utils/callback.hpp"
#include "utils/span.hpp"
#include "../algorithms/anomaly_detector.hpp"

namespace mka {
namespace fdir {

// ============================================================================
// Типы и константы
// ============================================================================

/// Уровень серьёзности события
enum class Severity : uint8_t {
    INFO = 0,       // Информационное сообщение
    WARNING = 1,    // Предупреждение
    ERROR = 2,      // Ошибка
    CRITICAL = 3    // Критическая ошибка
};

/// Категория подсистемы
enum class Subsystem : uint8_t {
    OBC = 0,        // Бортовой компьютер
    EPS = 1,        // Система электропитания
    ADCS = 2,       // Система ориентации
    COMM = 3,       // Система связи
    PAYLOAD = 4,    // Полезная нагрузка
    TCS = 5,        // Система терморегулирования
    PROP = 6,       // Двигательная установка
    CDH = 7         // Командно-измерительная система
};

/// Коды ошибок
enum class ErrorCode : uint16_t {
    // EPS ошибки (0x01xx)
    EPS_UNDERVOLTAGE = 0x0100,
    EPS_OVERVOLTAGE = 0x0101,
    EPS_OVERCURRENT = 0x0102,
    EPS_BATTERY_LOW = 0x0103,
    EPS_BATTERY_CRITICAL = 0x0104,
    EPS_SOLAR_PANEL_FAULT = 0x0105,
    EPS_POWER_RAIL_FAULT = 0x0106,
    
    // ADCS ошибки (0x02xx)
    ADCS_SENSOR_FAILURE = 0x0200,
    ADCS_ACTUATOR_FAILURE = 0x0201,
    ADCS_ATTITUDE_UNCONTROLLED = 0x0202,
    ADCS_WHEEL_OVERSPEED = 0x0203,
    ADCS_MAGNETORQUE_FAULT = 0x0204,
    
    // COMM ошибки (0x03xx)
    COMM_TX_FAILURE = 0x0300,
    COMM_RX_FAILURE = 0x0301,
    COMM_ANTENNA_FAULT = 0x0302,
    COMM_LOW_SIGNAL = 0x0303,

    // OBC ошибки (0x00xx)
    OBC_WATCHDOG_RESET = 0x0000,
    OBC_MEMORY_FAULT = 0x0001,
    OBC_CPU_OVERLOAD = 0x0002,
    OBC_CPU_OVERTEMP = 0x0003,
    OBC_RTC_FAULT = 0x0004,
    OBC_FLASH_FAULT = 0x0005,
    OBC_SD_CARD_FAULT = 0x0006,
    FDIR_ML_ANOMALY_DETECTED = 0x0010,  // ML детекция аномалии

    // Общие ошибки (0xFFxx)
    TIMEOUT = 0xFF00,
    INVALID_DATA = 0xFF01,
    COMMUNICATION_ERROR = 0xFF02,
    UNKNOWN = 0xFFFF
};

/// Действие восстановления
enum class RecoveryAction : uint8_t {
    NONE = 0,           // Нет действия
    LOG_ONLY,           // Только запись в журнал
    RESET_SENSOR,       // Сброс датчика
    RESET_SUBSYSTEM,    // Сброс подсистемы
    SAFE_MODE,          // Переход в безопасный режим
    POWER_CYCLE,        // Перезагрузка по питанию
    SYSTEM_RESET        // Полный сброс системы
};

// ============================================================================
// Структуры данных
// ============================================================================

/// Запись журнала событий
struct EventLogEntry {
    uint32_t timestamp;     // Время события (секунды от запуска)
    ErrorCode code;         // Код ошибки
    Severity severity;      // Серьёзность
    Subsystem subsystem;    // Подсистема
    uint8_t source;         // Источник внутри подсистемы
    int16_t value;          // Измеренное значение
    int16_t threshold;      // Пороговое значение
    uint8_t recoveryTaken;  // Принятое действие
    uint8_t reserved;       // Зарезервировано
};

static_assert(sizeof(EventLogEntry) == 16, "EventLogEntry must be 16 bytes");

/// Компактная запись журнала (для хранения в FRAM/EEPROM)
struct CompactEventLogEntry {
    uint32_t timestamp;     // Время (секунды от запуска)
    uint16_t code;          // Код ошибки
    uint8_t severity_subsystem;  // Severity (4 бита) + Subsystem (4 бита)
    uint8_t source;         // Источник внутри подсистемы
    int8_t value_scaled;    // Нормализованное значение (-128..127)
    int8_t threshold_scaled; // Нормализованный порог
    uint8_t recovery;       // Принятое действие

    CompactEventLogEntry() = default;

    // Конвертация из полной записи
    explicit CompactEventLogEntry(const EventLogEntry& entry)
        : timestamp(entry.timestamp)
        , code(static_cast<uint16_t>(entry.code))
        , severity_subsystem((static_cast<uint8_t>(entry.severity) << 4) |
                             static_cast<uint8_t>(entry.subsystem))
        , source(entry.source)
        , value_scaled(static_cast<int8_t>(entry.value / 10))
        , threshold_scaled(static_cast<int8_t>(entry.threshold / 10))
        , recovery(entry.recoveryTaken) {}

    // Конвертация в полную запись
    EventLogEntry toFull() const {
        EventLogEntry full;
        full.timestamp = timestamp;
        full.code = static_cast<ErrorCode>(code);
        full.severity = static_cast<Severity>((severity_subsystem >> 4) & 0x0F);
        full.subsystem = static_cast<Subsystem>(severity_subsystem & 0x0F);
        full.source = source;
        full.value = value_scaled * 10;
        full.threshold = threshold_scaled * 10;
        full.recoveryTaken = recovery;
        return full;
    }
};

// Валидация: 4 + 2 + 1 + 1 + 1 + 1 + 1 = 11 байт (экономия 31% по сравнению с 16)
static_assert(sizeof(CompactEventLogEntry) == 11, "CompactEventLogEntry must be 11 bytes");

/// Конфигурация монитора параметра
struct ParameterConfig {
    float nominalValue;         // Номинальное значение
    float warningLow;           // Нижний порог предупреждения
    float warningHigh;          // Верхний порог предупреждения
    float errorLow;             // Нижний порог ошибки
    float errorHigh;            // Верхний порог ошибки
    float criticalLow;          // Нижний критический порог
    float criticalHigh;         // Верхний критический порог
    uint16_t debounceMs;        // Время подавления дребезга
    uint8_t subsystem;          // Подсистема
    uint8_t parameterId;        // Идентификатор параметра
};

// Валидация: 8 x 4 + 2 + 1 + 1 = 36 байт
static_assert(sizeof(ParameterConfig) == 36, "ParameterConfig must be 36 bytes");
static_assert(offsetof(ParameterConfig, nominalValue) == 0, "nominalValue should be first");

/// Статистика параметра для детектора аномалий
struct ParameterStats {
    float mean = 0.0f;          // Среднее значение
    float variance = 0.0f;      // Дисперсия
    float min = 0.0f;           // Минимум
    float max = 0.0f;           // Максимум
    uint32_t count = 0;         // Количество измерений
    uint32_t anomalyCount = 0;  // Количество аномалий
    
    static constexpr float ANOMALY_SIGMA = 3.0f; // Порог аномалии (сигма)
    
    void update(float value) {
        // Онлайн алгоритм Welford для дисперсии
        count++;
        float delta = value - mean;
        mean += delta / count;
        float delta2 = value - mean;
        variance += delta * delta2;
        
        if (value < min || count == 1) min = value;
        if (value > max || count == 1) max = value;
    }
    
    float stddev() const {
        return count > 1 ? std::sqrt(variance / (count - 1)) : 0.0f;
    }
    
    bool isAnomaly(float value) const {
        if (count < 10) return false; // Недостаточно данных
        float sigma = stddev();
        if (sigma < 0.001f) return false;
        return std::abs(value - mean) > ANOMALY_SIGMA * sigma;
    }
    
    void reset() {
        *this = ParameterStats{};
    }
};

// ============================================================================
// Мониторинг параметров
// ============================================================================

/**
 * @brief Мониторинг параметра с пороговыми значениями
 */
class ParameterMonitor {
public:
    using Callback = Callback<void(Severity, float, float)>;

    ParameterMonitor() = default;

    ParameterMonitor(const ParameterConfig& config, Callback callback)
        : config_(config), callback_(callback) {}
    
    /**
     * @brief Проверка значения параметра
     * @param value Текущее значение
     * @param timestampMs Время в миллисекундах
     * @return Серьёзность нарушения порога
     */
    Severity check(float value, uint32_t timestampMs) {
        // Обновление статистики
        stats_.update(value);
        
        // Проверка порогов
        Severity severity = Severity::INFO;
        
        if (value < config_.criticalLow || value > config_.criticalHigh) {
            severity = Severity::CRITICAL;
        } else if (value < config_.errorLow || value > config_.errorHigh) {
            severity = Severity::ERROR;
        } else if (value < config_.warningLow || value > config_.warningHigh) {
            severity = Severity::WARNING;
        }
        
        // Debounce
        if (severity != lastSeverity_) {
            lastChangeTime_ = timestampMs;
            lastSeverity_ = severity;
            pendingSeverity_ = severity;
        } else if (timestampMs - lastChangeTime_ >= config_.debounceMs) {
            if (pendingSeverity_ != Severity::INFO && pendingSeverity_ != reportedSeverity_) {
                reportedSeverity_ = pendingSeverity_;
                if (callback_) {
                    float threshold = (value < config_.nominalValue) 
                        ? config_.warningLow : config_.warningHigh;
                    callback_(pendingSeverity_, value, threshold);
                }
            }
        }
        
        return reportedSeverity_;
    }
    
    /// Проверка на аномалию (статистический детектор)
    bool checkAnomaly(float value) const {
        return stats_.isAnomaly(value);
    }
    
    /// Получение статистики
    const ParameterStats& getStats() const { return stats_; }
    
    /// Сброс статистики
    void resetStats() { stats_.reset(); }
    
    /// Сброс состояния монитора
    void reset() {
        stats_.reset();
        lastSeverity_ = Severity::INFO;
        pendingSeverity_ = Severity::INFO;
        reportedSeverity_ = Severity::INFO;
        lastChangeTime_ = 0;
        violationCount_ = 0;
    }
    
private:
    ParameterConfig config_;
    Callback callback_;
    ParameterStats stats_;
    
    Severity lastSeverity_ = Severity::INFO;
    Severity pendingSeverity_ = Severity::INFO;
    Severity reportedSeverity_ = Severity::INFO;
    uint32_t lastChangeTime_ = 0;
    uint32_t violationCount_ = 0;
};

// ============================================================================
// FDIR Менеджер
// ============================================================================

/**
 * @brief Центральный менеджер FDIR
 *
 * Интегрирует:
 * - Пороговый мониторинг параметров
 * - Статистическую детекцию аномалий
 * - ML детекцию аномалий (Isolation Forest)
 * - Автоматическое восстановление
 */
class FDIRManager {
public:
    static constexpr size_t MAX_PARAMETERS = 64;
    static constexpr size_t EVENT_LOG_SIZE = 256;

    using EventCallback = Callback<void(const EventLogEntry&)>;
    using RecoveryHandler = Callback<bool(ErrorCode, RecoveryAction)>;
    using TimestampSource = Callback<uint32_t()>;

    FDIRManager() = default;

    /**
     * @brief Инициализация ML детектора аномалий
     * @param config Конфигурация детектора
     * @param featureNames Имена признаков для телеметрии
     */
    void initMLDetector(const ml::AnomalyDetectorConfig& config,
                        const std::vector<std::string>& featureNames) {
        mlDetector_.configure(config);
        featureNames_ = featureNames;
        mlEnabled_ = true;
    }

    /**
     * @brief Обучение ML детектора на исторических данных
     * @param data Данные телеметрии [n_samples x n_features]
     * @return true если успешно
     */
    bool trainMLDetector(const std::vector<std::vector<float>>& data) {
        if (!mlEnabled_) return false;
        return mlDetector_.fit(data);
    }

    /**
     * @brief Детекция ML аномалий для телеметрии
     * @param telemetry Вектор телеметрии
     * @return Результат детекции
     */
    ml::AnomalyResult detectMLAnomaly(const std::vector<float>& telemetry) {
        if (!mlEnabled_ || !mlDetector_.isReady()) {
            return {};
        }
        return mlDetector_.detect(telemetry);
    }

    /**
     * @brief Проверка ML аномалии и генерация события
     * @param telemetry Вектор телеметрии
     * @param timestamp Время события
     */
    void checkMLAnomaly(const std::vector<float>& telemetry, uint32_t timestamp) {
        if (!mlEnabled_ || !mlDetector_.isReady()) return;

        auto result = mlDetector_.detect(telemetry);
        if (result.isAnomaly) {
            // Генерация события об аномалии
            logEvent(ErrorCode::FDIR_ML_ANOMALY_DETECTED, Severity::WARNING,
                     Subsystem::OBC, 0,
                     static_cast<int16_t>(result.anomalyScore * 100),
                     static_cast<int16_t>(result.threshold * 100));
        }
    }

    /**
     * @brief Проверка включён ли ML детектор
     */
    bool isMLEnabled() const { return mlEnabled_; }

    /**
     * @brief Получить статистику ML детектора
     */
    ml::AnomalyDetector::Statistics getMLStatistics() const {
        return mlDetector_.getStatistics();
    }

    /// Регистрация параметра для мониторинга
    uint8_t registerParameter(const ParameterConfig& config,
                              ParameterMonitor::Callback callback = {}) {
        if (paramCount_ >= MAX_PARAMETERS) return 0xFF;

        uint8_t id = paramCount_++;
        monitors_[id] = ParameterMonitor(config, callback);
        return id;
    }
    
    /// Установка обработчика событий
    void setEventCallback(EventCallback callback) {
        eventCallback_ = callback;
    }
    
    /// Установка обработчика восстановления
    void setRecoveryHandler(RecoveryHandler handler) {
        recoveryHandler_ = handler;
    }

    /// Установка источника времени для меток событий
    void setTimestampSource(TimestampSource source) {
        timestampSource_ = source;
    }

    /// Обновление значения параметра
    Severity updateParameter(uint8_t id, float value, uint32_t timestampMs) {
        if (id >= paramCount_) return Severity::INFO;

        Severity severity = monitors_[id].check(value, timestampMs);

        if (severity >= Severity::WARNING) {
            logEvent(ErrorCode::UNKNOWN, severity,
                    Subsystem::OBC, id,
                    static_cast<int16_t>(value), 0);
        }

        return severity;
    }
    
    /// Ручной отчёт об ошибке
    void reportError(ErrorCode code, Severity severity, 
                    Subsystem subsystem, uint8_t source = 0,
                    int16_t value = 0, int16_t threshold = 0) {
        logEvent(code, severity, subsystem, source, value, threshold);
        
        // Автоматическое восстановление
        RecoveryAction action = determineRecoveryAction(code, severity);
        if (action != RecoveryAction::NONE) {
            attemptRecovery(code, action);
        }
    }
    
    /// Получение записи из журнала событий
    bool getEventLogEntry(size_t index, EventLogEntry& entry) const {
        if (index >= EVENT_LOG_SIZE) return false;
        
        size_t actualIndex = (logHead_ + index) % EVENT_LOG_SIZE;
        if (eventLog_[actualIndex].timestamp == 0 && index >= logCount_) {
            return false;
        }
        
        entry = eventLog_[actualIndex];
        return true;
    }
    
    /// Количество событий в журнале
    size_t getEventCount() const { return logCount_; }
    
    /// Очистка журнала
    void clearEventLog() {
        std::memset(eventLog_.data(), 0, sizeof(eventLog_));
        logHead_ = 0;
        logCount_ = 0;
    }
    
    /// Получение статистики аномалий
    void getAnomalyStats(uint8_t paramId, ParameterStats& stats) const {
        if (paramId < paramCount_) {
            stats = monitors_[paramId].getStats();
        }
    }
    
    /// Сброс всех мониторов
    void resetAllMonitors() {
        for (size_t i = 0; i < paramCount_; i++) {
            monitors_[i].reset();
        }
    }
    
private:
    std::array<ParameterMonitor, MAX_PARAMETERS> monitors_{};
    size_t paramCount_ = 0;

    std::array<EventLogEntry, EVENT_LOG_SIZE> eventLog_{};
    size_t logHead_ = 0;
    size_t logCount_ = 0;

    EventCallback eventCallback_;
    RecoveryHandler recoveryHandler_;
    TimestampSource timestampSource_;

    // ML детектор аномалий
    ml::AnomalyDetector mlDetector_;
    std::vector<std::string> featureNames_;
    bool mlEnabled_ = false;

    void logEvent(ErrorCode code, Severity severity,
                  Subsystem subsystem, uint8_t source,
                  int16_t value, int16_t threshold) {
        EventLogEntry entry{};
        entry.timestamp = timestampSource_ ? timestampSource_() : 0;
        entry.code = code;
        entry.severity = severity;
        entry.subsystem = subsystem;
        entry.source = source;
        entry.value = value;
        entry.threshold = threshold;
        entry.recoveryTaken = static_cast<uint8_t>(RecoveryAction::NONE);
        
        eventLog_[logHead_] = entry;
        logHead_ = (logHead_ + 1) % EVENT_LOG_SIZE;
        if (logCount_ < EVENT_LOG_SIZE) logCount_++;
        
        if (eventCallback_) {
            eventCallback_(entry);
        }
    }
    
    RecoveryAction determineRecoveryAction(ErrorCode code, Severity severity) {
        if (severity == Severity::CRITICAL) {
            switch (code) {
                case ErrorCode::EPS_BATTERY_CRITICAL:
                    return RecoveryAction::SAFE_MODE;
                case ErrorCode::OBC_WATCHDOG_RESET:
                    return RecoveryAction::SYSTEM_RESET;
                case ErrorCode::OBC_MEMORY_FAULT:
                    return RecoveryAction::SYSTEM_RESET;
                default:
                    return RecoveryAction::RESET_SUBSYSTEM;
            }
        } else if (severity == Severity::ERROR) {
            switch (code) {
                case ErrorCode::ADCS_SENSOR_FAILURE:
                case ErrorCode::ADCS_ACTUATOR_FAILURE:
                    return RecoveryAction::RESET_SENSOR;
                default:
                    return RecoveryAction::LOG_ONLY;
            }
        }
        return RecoveryAction::LOG_ONLY;
    }
    
    void attemptRecovery(ErrorCode code, RecoveryAction action) {
        if (recoveryHandler_) {
            bool success = recoveryHandler_(code, action);
            // Записать результат в последнее событие
            if (logCount_ > 0) {
                size_t lastIndex = (logHead_ + EVENT_LOG_SIZE - 1) % EVENT_LOG_SIZE;
                eventLog_[lastIndex].recoveryTaken = 
                    static_cast<uint8_t>(success ? action : RecoveryAction::NONE);
            }
        }
    }
};

// ============================================================================
// Встроенные детекторы
// ============================================================================

/**
 * @brief Детектор замороженного значения
 * 
 * Обнаруживает, когда значение не изменяется в течение длительного времени,
 * что может указывать на отказ датчика.
 */
class FrozenValueDetector {
public:
    FrozenValueDetector(uint32_t thresholdSamples = 10)
        : thresholdSamples_(thresholdSamples) {}
    
    bool check(float value) {
        if (std::abs(value - lastValue_) < 0.0001f) {
            unchangedCount_++;
            return unchangedCount_ >= thresholdSamples_;
        } else {
            lastValue_ = value;
            unchangedCount_ = 0;
            return false;
        }
    }
    
    void reset() {
        lastValue_ = 0.0f;
        unchangedCount_ = 0;
    }
    
private:
    float lastValue_ = 0.0f;
    uint32_t unchangedCount_ = 0;
    uint32_t thresholdSamples_;
};

/**
 * @brief Детектор застрявшего бита
 * 
 * Обнаруживает ситуации, когда цифровой сигнал не изменяется
 * или изменяется с аномальной частотой.
 */
class StuckBitDetector {
public:
    StuckBitDetector(uint32_t minTransitions = 2, uint32_t windowSamples = 100)
        : minTransitions_(minTransitions), windowSamples_(windowSamples) {}
    
    bool check(bool value) {
        if (value != lastValue_) {
            transitionCount_++;
            lastValue_ = value;
        }
        
        sampleCount_++;
        
        if (sampleCount_ >= windowSamples_) {
            bool stuck = transitionCount_ < minTransitions_;
            transitionCount_ = 0;
            sampleCount_ = 0;
            return stuck;
        }
        
        return false;
    }
    
    void reset() {
        lastValue_ = false;
        transitionCount_ = 0;
        sampleCount_ = 0;
    }
    
private:
    bool lastValue_ = false;
    uint32_t transitionCount_ = 0;
    uint32_t sampleCount_ = 0;
    uint32_t minTransitions_;
    uint32_t windowSamples_;
};

/**
 * @brief Детектор всплесков (glitch detector)
 * 
 * Обнаруживает кратковременные аномальные значения.
 */
class GlitchDetector {
public:
    GlitchDetector(float threshold, uint32_t glitchWindow = 3)
        : threshold_(threshold), glitchWindow_(glitchWindow) {}
    
    bool check(float value) {
        // Проверка на резкое изменение
        if (std::abs(value - filteredValue_) > threshold_) {
            glitchCount_++;
            if (glitchCount_ >= glitchWindow_) {
                // Подтверждённый глитч
                filteredValue_ = value; // Принять новое значение
                return true;
            }
        } else {
            glitchCount_ = 0;
            // Экспоненциальное сглаживание
            filteredValue_ = 0.9f * filteredValue_ + 0.1f * value;
        }
        return false;
    }
    
    void reset() {
        filteredValue_ = 0.0f;
        glitchCount_ = 0;
    }
    
private:
    float filteredValue_ = 0.0f;
    float threshold_;
    uint32_t glitchCount_ = 0;
    uint32_t glitchWindow_;
};

} // namespace fdir
} // namespace mka

#endif // FDIR_HPP
