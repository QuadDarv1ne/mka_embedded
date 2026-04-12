/**
 * @file auto_actualization.hpp
 * @brief Система автоматической актуализации всех расчётов по МСК времени
 *
 * Автоматически отслеживает и обновляет:
 * - SGP4 орбитальные расчёты (TLE данные)
 * - Телеметрические расчёты
 * - FDIR параметры
 * - Health Monitoring данные
 * - Навигационные расчёты
 * - Алгоритмы ориентации (ADCS)
 *
 * Все расчёты актуализируются автоматически при устаревании данных
 * с учётом Московского времени (МСК = UTC+3)
 *
 * Поддерживаемые режимы работы:
 * - Ручной: вызов actualizeAllCalculations() по требованию
 * - Автоматический: фоновый планировщик с периодической актуализацией
 * - FreeRTOS: задача для работы в реальном времени на борту МКА
 */

#ifndef AUTO_ACTUALIZATION_HPP
#define AUTO_ACTUALIZATION_HPP

#include <cstdint>
#include <functional>
#include <vector>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include "systems/moscow_time.hpp"

namespace mka {
namespace systems {

// ============================================================================
// Типы расчётных данных
// ============================================================================

/// Тип расчётных данных
enum class CalculationType : uint8_t {
    SGP4_ORBITAL = 0,       // Орбитальные расчёты (SGP4)
    TELEMETRY = 1,          // Телеметрические расчёты
    FDIR_PARAMETERS = 2,    // Параметры FDIR
    HEALTH_MONITORING = 3,  // Health Monitoring данные
    NAVIGATION = 4,         // Навигационные расчёты
    ADCS_ALGORITHMS = 5,    // Алгоритмы ориентации
    POWER_CALCULATIONS = 6, // Расчёты энергопитания
    THERMAL_CALCULATIONS = 7, // Термические расчёты
    COMMUNICATION = 8,      // Коммуникационные расчёты
    PAYLOAD_DATA = 9,       // Данные полезной нагрузки
    MAX_TYPES               // Максимальное количество типов
};

/// Приоритет актуализации
enum class ActualizationPriority : uint8_t {
    LOW = 0,        // Низкий приоритет (обновление раз в час)
    MEDIUM = 1,     // Средний приоритет (обновление раз в 15 минут)
    HIGH = 2,       // Высокий приоритет (обновление раз в 5 минут)
    CRITICAL = 3    // Критический приоритет (обновление раз в минуту)
};

/// Конфигурация актуализации расчётов
struct CalculationActualizationConfig {
    CalculationType type;                       // Тип расчёта
    const char* name;                           // Имя расчёта
    uint32_t freshThresholdSeconds;             // Порог свежести
    uint32_t staleThresholdSeconds;             // Порог устаревания
    uint32_t expireThresholdSeconds;            // Порог истечения
    ActualizationPriority priority;             // Приоритет
    bool autoActualize;                         // Автоматическая актуализация
    std::function<bool()> actualizationCallback; // Callback актуализации
};

// ============================================================================
// Менеджер автоматической актуализации расчётов
// ============================================================================

/**
 * @brief Менеджер автоматической актуализации всех расчётов по МСК
 * 
 * Отслеживает свежесть всех расчётных данных и автоматически
 * инициирует перерасчёт при устаревании.
 * 
 * Пример использования:
 * @code
 * AutoActualizationManager actualizationMgr;
 * actualizationMgr.init(utcTimeSource);
 * 
 * // Регистрация SGP4 расчётов
 * actualizationMgr.registerCalculation({
 *     .type = CalculationType::SGP4_ORBITAL,
 *     .name = "ISS_TLE_Propagation",
 *     .freshThresholdSeconds = 3600,
 *     .staleThresholdSeconds = 7200,
 *     .expireThresholdSeconds = 86400,
 *     .priority = ActualizationPriority::HIGH,
 *     .autoActualize = true,
 *     .actualizationCallback = []() { 
 *         // Загрузить новые TLE данные и пересчитать орбиту
 *         return updateSGP4Data(); 
 *     }
 * });
 * 
 * // Проверить и автоматически актуализировать все расчёты
 * actualizationMgr.actualizeAllCalculations();
 * @endcode
 */
class AutoActualizationManager {
public:
    static constexpr size_t MAX_CALCULATIONS = 64;
    
    AutoActualizationManager() = default;
    
    /**
     * @brief Инициализация менеджера
     * @param utcSource Источник текущего UTC времени
     * @return true при успешной инициализации
     */
    bool init(std::function<uint64_t()> utcSource) {
        if (!utcSource) {
            return false;
        }
        utcSource_ = utcSource;
        return freshnessMgr_.init(utcSource);
    }
    
    /**
     * @brief Регистрация расчёта для автоматической актуализации
     * @param config Конфигурация расчёта
     * @return true при успешной регистрации
     */
    bool registerCalculation(const CalculationActualizationConfig& config) {
        if (calculationCount_ >= MAX_CALCULATIONS) {
            return false;
        }
        
        configs_[calculationCount_] = config;
        lastActualizationTimes_[calculationCount_] = 0;
        actualizationCounts_[calculationCount_] = 0;
        lastStatuses_[calculationCount_] = DataFreshnessStatus::INVALID;
        
        // Регистрируем в DataFreshnessManager
        DataFreshnessConfig freshnessConfig = {
            .freshThresholdSeconds = config.freshThresholdSeconds,
            .staleThresholdSeconds = config.staleThresholdSeconds,
            .expireThresholdSeconds = config.expireThresholdSeconds,
            .autoUpdateOnStale = config.autoActualize,
            .autoUpdateOnExpire = config.autoActualize
        };
        
        std::string calcName = getCalculationName(config.type, config.name);
        freshnessMgr_.registerDataSource(calcName.c_str(), freshnessConfig);
        
        calculationCount_++;
        return true;
    }
    
    /**
     * @brief Проверить и актуализировать все расчёты
     * @return Количество актуализированных расчётов
     */
    size_t actualizeAllCalculations() {
        size_t actualizedCount = 0;
        
        for (size_t i = 0; i < calculationCount_; i++) {
            if (actualizeCalculation(i)) {
                actualizedCount++;
            }
        }
        
        return actualizedCount;
    }
    
    /**
     * @brief Проверить и актуализировать расчёт по типу
     * @param type Тип расчёта
     * @return true если расчёт был актуализирован
     */
    bool actualizeByType(CalculationType type) {
        for (size_t i = 0; i < calculationCount_; i++) {
            if (configs_[i].type == type) {
                return actualizeCalculation(i);
            }
        }
        return false;
    }
    
    /**
     * @brief Проверить и актуализировать расчёт по имени
     * @param name Имя расчёта
     * @return true если расчёт был актуализирован
     */
    bool actualizeByName(const char* name) {
        for (size_t i = 0; i < calculationCount_; i++) {
            std::string calcName = getCalculationName(configs_[i].type, configs_[i].name);
            if (calcName == name) {
                return actualizeCalculation(i);
            }
        }
        return false;
    }
    
    /**
     * @brief Проверить статус свежести расчёта
     * @param type Тип расчёта
     * @return Статус свежести
     */
    DataFreshnessStatus getCalculationStatus(CalculationType type) const {
        for (size_t i = 0; i < calculationCount_; i++) {
            if (configs_[i].type == type) {
                return lastStatuses_[i];
            }
        }
        return DataFreshnessStatus::INVALID;
    }
    
    /**
     * @brief Получить время последней актуализации (МСК)
     * @param type Тип расчёта
     * @return МСК timestamp или 0 если не найдено
     */
    uint64_t getLastActualizationTimeMSK(CalculationType type) const {
        for (size_t i = 0; i < calculationCount_; i++) {
            if (configs_[i].type == type) {
                return MoscowTimeConverter::utcToMSK(lastActualizationTimes_[i]);
            }
        }
        return 0;
    }
    
    /**
     * @brief Получить количество актуализаций
     * @param type Тип расчёта
     * @return Количество актуализаций
     */
    uint32_t getActualizationCount(CalculationType type) const {
        for (size_t i = 0; i < calculationCount_; i++) {
            if (configs_[i].type == type) {
                return actualizationCounts_[i];
            }
        }
        return 0;
    }
    
    /**
     * @brief Получить статистику по всем расчётам
     * @param freshCount[out] Количество свежих расчётов
     * @param staleCount[out] Количество устаревших расчётов
     * @param expiredCount[out] Количество истёкших расчётов
     * @param invalidCount[out] Количество невалидных расчётов
     */
    void getActualizationStats(uint8_t& freshCount, uint8_t& staleCount,
                               uint8_t& expiredCount, uint8_t& invalidCount) const {
        freshCount = 0;
        staleCount = 0;
        expiredCount = 0;
        invalidCount = 0;
        
        uint64_t currentTime = utcSource_ ? utcSource_() : 0;

        for (size_t i = 0; i < calculationCount_; i++) {
            // Рассчитываем статус напрямую (с защитой от обратного хода часов)
            uint64_t age = (lastActualizationTimes_[i] > 0 && currentTime >= lastActualizationTimes_[i]) ?
                           (currentTime - lastActualizationTimes_[i]) :
                           UINT64_MAX;
            
            const auto& config = configs_[i];
            DataFreshnessStatus status;
            if (age <= config.freshThresholdSeconds) {
                status = DataFreshnessStatus::FRESH;
            } else if (age <= config.staleThresholdSeconds) {
                status = DataFreshnessStatus::STALE;
            } else if (age <= config.expireThresholdSeconds) {
                status = DataFreshnessStatus::EXPIRED;
            } else {
                status = DataFreshnessStatus::INVALID;
            }
            
            switch (status) {
                case DataFreshnessStatus::FRESH: freshCount++; break;
                case DataFreshnessStatus::STALE: staleCount++; break;
                case DataFreshnessStatus::EXPIRED: expiredCount++; break;
                case DataFreshnessStatus::INVALID: invalidCount++; break;
            }
        }
    }
    
    /**
     * @brief Получить расчёты, требующие актуализации
     * @param calculations[out] Вектор с индексами расчётов, требующих актуализации
     */
    void getCalculationsNeedingActualization(std::vector<size_t>& calculations) const {
        calculations.clear();
        
        uint64_t currentTime = utcSource_ ? utcSource_() : 0;
        
        for (size_t i = 0; i < calculationCount_; i++) {
            uint64_t age = (lastActualizationTimes_[i] > 0 && currentTime > 0) ? 
                           (currentTime - lastActualizationTimes_[i]) : 
                           UINT64_MAX;
            
            const auto& config = configs_[i];
            
            if (age > config.freshThresholdSeconds) {
                calculations.push_back(i);
            }
        }
    }
    
    /**
     * @brief Проверить конкретный расчёт и актуализировать при необходимости
     * @param index Индекс расчёта
     * @return true если расчёт был актуализирован
     */
    bool actualizeCalculation(size_t index) {
        if (index >= calculationCount_) {
            return false;
        }
        
        auto& config = configs_[index];
        std::string calcName = getCalculationName(config.type, config.name);
        
        // Рассчитываем статус напрямую
        uint64_t currentTime = utcSource_();
        uint64_t age = (lastActualizationTimes_[index] > 0) ? 
                       (currentTime - lastActualizationTimes_[index]) : 
                       UINT64_MAX;
        
        DataFreshnessStatus status;
        if (age <= config.freshThresholdSeconds) {
            status = DataFreshnessStatus::FRESH;
        } else if (age <= config.staleThresholdSeconds) {
            status = DataFreshnessStatus::STALE;
        } else if (age <= config.expireThresholdSeconds) {
            status = DataFreshnessStatus::EXPIRED;
        } else {
            status = DataFreshnessStatus::INVALID;
        }
        
        // Определяем нужно ли актуализировать
        bool needsActualization = false;
        
        switch (status) {
            case DataFreshnessStatus::STALE:
                needsActualization = config.autoActualize;
                break;
            case DataFreshnessStatus::EXPIRED:
                needsActualization = config.autoActualize;
                break;
            case DataFreshnessStatus::INVALID:
                needsActualization = true;  // Всегда актуализируем невалидные
                break;
            default:
                // FRESH - не актуализируем
                lastStatuses_[index] = status;
                return false;
        }
        
        // Актуализируем если нужно
        bool actualized = false;
        if (needsActualization && config.actualizationCallback) {
            if (config.actualizationCallback()) {
                // Успешная актуализация - записываем UTC время
                lastActualizationTimes_[index] = currentTime;
                actualizationCounts_[index]++;
                lastStatuses_[index] = DataFreshnessStatus::FRESH;
                
                // Обновляем DataFreshnessManager для консистентности
                freshnessMgr_.recordUpdate(
                    calcName.c_str(),
                    MoscowTimeConverter::utcToMSK(currentTime)
                );
                
                actualized = true;
            }
        }
        
        // Если не актуализировали, сохраняем текущий статус
        if (!actualized) {
            lastStatuses_[index] = status;
        }
        
        return actualized;
    }
    
    /**
     * @brief Получить менеджер свежести для прямого доступа
     */
    DataFreshnessManager& getFreshnessManager() { return freshnessMgr_; }
    
    /**
     * @brief Получить количество зарегистрированных расчётов
     */
    size_t getCalculationCount() const { return calculationCount_; }
    
private:
    std::function<uint64_t()> utcSource_;
    DataFreshnessManager freshnessMgr_;
    
    CalculationActualizationConfig configs_[MAX_CALCULATIONS];
    uint64_t lastActualizationTimes_[MAX_CALCULATIONS];
    uint32_t actualizationCounts_[MAX_CALCULATIONS];
    DataFreshnessStatus lastStatuses_[MAX_CALCULATIONS];
    size_t calculationCount_ = 0;
    
    std::string getCalculationName(CalculationType type, const char* name) const {
        std::string prefix;
        switch (type) {
            case CalculationType::SGP4_ORBITAL: prefix = "SGP4_"; break;
            case CalculationType::TELEMETRY: prefix = "TEL_"; break;
            case CalculationType::FDIR_PARAMETERS: prefix = "FDIR_"; break;
            case CalculationType::HEALTH_MONITORING: prefix = "HLTH_"; break;
            case CalculationType::NAVIGATION: prefix = "NAV_"; break;
            case CalculationType::ADCS_ALGORITHMS: prefix = "ADCS_"; break;
            case CalculationType::POWER_CALCULATIONS: prefix = "PWR_"; break;
            case CalculationType::THERMAL_CALCULATIONS: prefix = "THRM_"; break;
            case CalculationType::COMMUNICATION: prefix = "COMM_"; break;
            case CalculationType::PAYLOAD_DATA: prefix = "PAYLD_"; break;
            default: prefix = "UNK_"; break;
        }
        return prefix + (name ? name : "");
    }
};

// ============================================================================
// Фоновый планировщик автоматической актуализации
// ============================================================================

/// Режим работы планировщика
enum class SchedulerMode : uint8_t {
    MANUAL = 0,         // Только ручной вызов actualize*()
    AUTO_BACKGROUND = 1 // Автоматический фоновый запуск
};

/// Конфигурация планировщика
struct SchedulerConfig {
    SchedulerMode mode = SchedulerMode::MANUAL;         // Режим работы
    uint32_t baseIntervalMs = 60000;                     // Базовый интервал (1 минута)
    uint32_t criticalIntervalMs = 10000;                 // Интервал для CRITICAL (10 секунд)
    uint32_t highIntervalMs = 60000;                     // Интервал для HIGH (1 минута)
    uint32_t mediumIntervalMs = 900000;                  // Интервал для MEDIUM (15 минут)
    uint32_t lowIntervalMs = 3600000;                    // Интервал для LOW (1 час)
    bool prioritizeByMSKTime = true;                     // Приоритезация по МСК времени
};

/**
 * @brief Фоновый планировщик автоматической актуализации
 *
 * Запускает периодическую проверку и актуализацию всех расчётов
 * с учётом приоритетов и Московского времени.
 *
 * Пример использования:
 * @code
 * AutoActualizationManager actualizationMgr;
 * actualizationMgr.init(utcSource);
 *
 * // Регистрация расчётов
 * actualizationMgr.registerCalculation(sgp4Config);
 * actualizationMgr.registerCalculation(telemetryConfig);
 *
 * // Запуск фонового планировщика
 * AutoActualizationScheduler scheduler;
 * scheduler.start(actualizationMgr, {
 *     .mode = SchedulerMode::AUTO_BACKGROUND,
 *     .baseIntervalMs = 30000,
 *     .criticalIntervalMs = 5000,
 *     .highIntervalMs = 30000,
 *     .mediumIntervalMs = 300000,
 *     .lowIntervalMs = 1800000
 * });
 *
 * // Работает автоматически...
 *
 * // Остановка при необходимости
 * scheduler.stop();
 * @endcode
 */
class AutoActualizationScheduler {
public:
    AutoActualizationScheduler() = default;
    ~AutoActualizationScheduler() { stop(); }

    // Запрещаем копирование
    AutoActualizationScheduler(const AutoActualizationScheduler&) = delete;
    AutoActualizationScheduler& operator=(const AutoActualizationScheduler&) = delete;

    // Разрешаем перемещение
    AutoActualizationScheduler(AutoActualizationScheduler&& other) noexcept {
        std::lock_guard<std::mutex> lock(other.mutex_);
        running_ = other.running_.exchange(false);
        manager_ = other.manager_;
        config_ = other.config_;
        other.manager_ = nullptr;
        if (other.thread_.joinable()) {
            thread_ = std::move(other.thread_);
        }
    }

    /**
     * @brief Запуск фонового планировщика
     * @param manager Ссылка на менеджер актуализации
     * @param config Конфигурация планировщика
     * @return true при успешном запуске
     */
    bool start(AutoActualizationManager& manager, const SchedulerConfig& config = {}) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (running_.load()) {
            return false; // Уже запущен
        }

        manager_ = &manager;
        config_ = config;
        running_.store(true);

        if (config.mode == SchedulerMode::AUTO_BACKGROUND) {
            thread_ = std::thread(&AutoActualizationScheduler::backgroundLoop, this);
        }

        return true;
    }

    /**
     * @brief Остановка фонового планировщика
     */
    void stop() {
        running_.store(false);
        cv_.notify_all();

        if (thread_.joinable()) {
            thread_.join();
        }
    }

    /**
     * @brief Проверка работы планировщика
     * @return true если планировщик запущен
     */
    bool isRunning() const {
        return running_.load();
    }

    /**
     * @brief Принудительно запустить актуализацию сейчас
     * @return Количество актуализированных расчётов
     */
    size_t actualizeNow() {
        if (!manager_) {
            return 0;
        }
        return manager_->actualizeAllCalculations();
    }

    /**
     * @brief Получить интервал для приоритета
     * @param priority Приоритет расчёта
     * @return Интервал в миллисекундах
     */
    uint32_t getIntervalForPriority(ActualizationPriority priority) const {
        switch (priority) {
            case ActualizationPriority::CRITICAL:
                return config_.criticalIntervalMs;
            case ActualizationPriority::HIGH:
                return config_.highIntervalMs;
            case ActualizationPriority::MEDIUM:
                return config_.mediumIntervalMs;
            case ActualizationPriority::LOW:
                return config_.lowIntervalMs;
            default:
                return config_.baseIntervalMs;
        }
    }

private:
    AutoActualizationManager* manager_ = nullptr;
    SchedulerConfig config_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable cv_;

    /**
     * @brief Фоновый цикл актуализации
     */
    void backgroundLoop() {
        while (running_.load()) {
            // Определяем минимальный интервал среди всех расчётов
            uint32_t sleepInterval = determineSleepInterval();

            // Ждём или до сигнала пробуждения
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait_for(lock, std::chrono::milliseconds(sleepInterval), [this] {
                    return !running_.load();
                });
            }

            // Проверяем что не остановлены
            if (!running_.load()) {
                break;
            }

            // Выполняем актуализацию если менеджер доступен
            if (manager_) {
                manager_->actualizeAllCalculations();
            }
        }
    }

    /**
     * @brief Определить интервал сна на основе приоритетов
     * @return Интервал в миллисекундах
     */
    uint32_t determineSleepInterval() const {
        if (!manager_ || !manager_->getCalculationCount()) {
            return config_.baseIntervalMs;
        }

        // Находим минимальный интервал среди всех зарегистрированных расчётов
        // Это обеспечивает актуализацию критичных расчётов вовремя
        uint32_t minInterval = config_.baseIntervalMs;

        // Проходим по всем расчётам и находим минимальный интервал
        // Для упрощения используем базовый интервал
        // В полной реализации можно итерировать по всем расчётам
        return minInterval;
    }

#ifdef MKA_USE_FREERTOS
    /**
     * @brief FreeRTOS задача для работы в реальном времени
     *
     * Альтернатива std::thread для встраиваемых систем на FreeRTOS
     * Используется когда определён MKA_USE_FREERTOS
     */
    static void freertosTask(void* pvParameters) {
        AutoActualizationScheduler* scheduler =
            static_cast<AutoActualizationScheduler*>(pvParameters);

        while (scheduler->running_.load()) {
            uint32_t sleepInterval = scheduler->determineSleepInterval();

            // Используем vTaskDelay для не-блокирующего ожидания
            vTaskDelay(pdMS_TO_TICKS(sleepInterval));

            if (scheduler->manager_) {
                scheduler->manager_->actualizeAllCalculations();
            }
        }

        vTaskDelete(nullptr);
    }

    /**
     * @brief Запуск FreeRTOS задачи
     * @param manager Ссылка на менеджер актуализации
     * @param config Конфигурация планировщика
     * @param taskName Имя задачи (до 16 символов)
     * @param stackSize Размер стека в словах (по умолчанию 256)
     * @param priority Приоритет FreeRTOS задачи (по умолчанию 1)
     * @return true при успешном запуске
     */
    bool startFreeRTOSTask(AutoActualizationManager& manager,
                           const SchedulerConfig& config,
                           const char* taskName = "MSKActual",
                           uint16_t stackSize = 256,
                           UBaseType_t priority = 1) {
#ifdef MKA_USE_FREERTOS
        std::lock_guard<std::mutex> lock(mutex_);

        if (running_.load()) {
            return false;
        }

        manager_ = &manager;
        config_ = config;
        running_.store(true);

        // Создаём FreeRTOS задачу
        TaskHandle_t taskHandle = nullptr;
        BaseType_t result = xTaskCreate(
            freertosTask,
            taskName,
            stackSize,
            this,
            priority,
            &taskHandle
        );

        if (result != pdPASS) {
            running_.store(false);
            return false;
        }

        return true;
#else
        // FreeRTOS не доступен
        (void)taskName;
        (void)stackSize;
        (void)priority;
        return false;
#endif
    }
#endif // MKA_USE_FREERTOS
};

} // namespace systems
} // namespace mka

#endif // AUTO_ACTUALIZATION_HPP
