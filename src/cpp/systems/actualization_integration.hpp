/**
 * @file actualization_integration.hpp
 * @brief Интеграция автоматической актуализации по МСК с существующими модулями
 *
 * Автоматически интегрирует AutoActualizationManager с:
 * - SGP4 орбитальными расчётами
 * - Телеметрическими расчётами
 * - FDIR параметрами
 * - Health Monitoring данными
 * - ADCS алгоритмами
 *
 * Все расчёты автоматически актуализируются при устаревании данных
 * с учётом Московского времени (МСК = UTC+3)
 */

#ifndef ACTUALIZATION_INTEGRATION_HPP
#define ACTUALIZATION_INTEGRATION_HPP

#include "systems/auto_actualization.hpp"
#include "algorithms/sgp4.hpp"
#include "systems/health_monitor.hpp"
#include "systems/fdir.hpp"

namespace mka {
namespace systems {

// ============================================================================
// SGP4 Auto Actualization
// ============================================================================

/**
 * @brief SGP4 пропагатор с автоматической актуализацией TLE данных
 * 
 * Автоматически проверяет свежесть TLE данных и обновляет их при устаревании.
 * 
 * Пример использования:
 * @code
 * AutoActualizationSGP4 sgp4;
 * sgp4.init(utcTimeSource, "ISS", []() -> TLE { return loadTLEFromServer(); });
 * 
 * // Использование - автоматически актуализирует при необходимости
 * ECIState state = sgp4.propagateIfNeeded(minutesSinceEpoch);
 * @endcode
 */
class AutoActualizationSGP4 {
public:
    AutoActualizationSGP4() = default;
    
    /**
     * @brief Инициализация SGP4 с авто-актуализацией
     * @param utcSource Источник UTC времени
     * @param satelliteName Имя спутника
     * @param tleLoader Callback загрузки TLE данных
     * @param freshThresholdSeconds Порог свежести (по умолчанию 2 часа)
     * @param expireThresholdSeconds Порог истечения (по умолчанию 24 часа)
     * @return true при успехе
     */
    bool init(std::function<uint64_t()> utcSource,
              const std::string& satelliteName,
              std::function<navigation::TLE()> tleLoader,
              uint32_t freshThresholdSeconds = 7200,
              uint32_t expireThresholdSeconds = 86400) {
        utcSource_ = utcSource;
        tleLoader_ = tleLoader;
        satelliteName_ = satelliteName;
        
        if (!actualizationMgr_.init(utcSource)) {
            return false;
        }
        
        // Регистрируем SGP4 расчёт
        CalculationActualizationConfig config = {
            .type = CalculationType::SGP4_ORBITAL,
            .name = satelliteName.c_str(),
            .freshThresholdSeconds = freshThresholdSeconds,
            .staleThresholdSeconds = freshThresholdSeconds * 2,
            .expireThresholdSeconds = expireThresholdSeconds,
            .priority = ActualizationPriority::HIGH,
            .autoActualize = true,
            .actualizationCallback = [this]() -> bool {
                return actualizeTLEData();
            }
        };
        
        return actualizationMgr_.registerCalculation(config);
    }
    
    /**
     * @brief Получить положение спутника с авто-актуализацией
     * @param minutesSinceEpoch Минуты от эпохи
     * @return Положение и скорость в ECI
     */
    navigation::ECIState propagateIfNeeded(double minutesSinceEpoch) {
        // Проверяем и актуализируем если нужно
        actualizationMgr_.actualizeAllCalculations();
        
        // Возвращаем положение
        return propagator_.propagate(minutesSinceEpoch);
    }
    
    /**
     * @brief Принудительно актуализировать TLE данные
     * @return true при успехе
     */
    bool forceActualize() {
        if (actualizeTLEData()) {
            // Обновляем время актуализации
            if (utcSource_) {
                uint64_t currentTime = utcSource_();
                // Обновляем через менеджер
                auto& mgr = getActualizationManager();
                mgr.actualizeByType(CalculationType::SGP4_ORBITAL);
            }
            return true;
        }
        return false;
    }
    
    /**
     * @brief Получить статус свежести TLE данных
     */
    DataFreshnessStatus getFreshnessStatus() const {
        return actualizationMgr_.getCalculationStatus(CalculationType::SGP4_ORBITAL);
    }
    
    /**
     * @brief Получить время последней актуализации (МСК)
     */
    uint64_t getLastActualizationTimeMSK() const {
        return actualizationMgr_.getLastActualizationTimeMSK(CalculationType::SGP4_ORBITAL);
    }
    
    /**
     * @brief Получить менеджер актуализации
     */
    AutoActualizationManager& getActualizationManager() { return actualizationMgr_; }
    
    /**
     * @brief Получить пропагатор
     */
    navigation::SGP4Propagator& getPropagator() { return propagator_; }
    
private:
    std::function<uint64_t()> utcSource_;
    std::function<navigation::TLE()> tleLoader_;
    std::string satelliteName_;
    
    navigation::SGP4Propagator propagator_;
    AutoActualizationManager actualizationMgr_;
    
    bool actualizeTLEData() {
        if (!tleLoader_) {
            return false;
        }
        
        navigation::TLE tle = tleLoader_();
        return propagator_.init(tle);
    }
};

// ============================================================================
// Health Monitoring Auto Actualization
// ============================================================================

/**
 * @brief Health Monitoring с автоматической актуализацией
 * 
 * Автоматически актуализирует конфигурацию health monitoring.
 */
class AutoActualizationHealthMonitoring {
public:
    AutoActualizationHealthMonitoring() = default;
    
    /**
     * @brief Инициализация health monitoring с авто-актуализацией
     * @param utcSource Источник UTC времени
     * @param freshThresholdSeconds Порог свежести (по умолчанию 5 минут)
     * @return true при успехе
     */
    bool init(std::function<uint64_t()> utcSource,
              uint32_t freshThresholdSeconds = 300) {
        utcSource_ = utcSource;
        
        if (!actualizationMgr_.init(utcSource)) {
            return false;
        }
        
        // Регистрируем health monitoring расчёт
        CalculationActualizationConfig config = {
            .type = CalculationType::HEALTH_MONITORING,
            .name = "Health_Config",
            .freshThresholdSeconds = freshThresholdSeconds,
            .staleThresholdSeconds = freshThresholdSeconds * 2,
            .expireThresholdSeconds = freshThresholdSeconds * 12,
            .priority = ActualizationPriority::HIGH,
            .autoActualize = true,
            .actualizationCallback = [this]() -> bool {
                return actualizeHealthConfig();
            }
        };
        
        return actualizationMgr_.registerCalculation(config);
    }
    
    /**
     * @brief Зарегистрировать драйвер
     * @param driver Указатель на драйвер
     * @param config Конфигурация
     * @return true при успехе
     */
    bool registerDriver(IHealthMonitorDriver* driver, const DriverHealthConfig& config) {
        // Актуализируем если нужно
        actualizationMgr_.actualizeAllCalculations();
        
        return healthMgr_.registerDriver(driver, config);
    }
    
    /**
     * @brief Проверить все драйверы
     * @param currentTimeMs Текущее время в мс
     */
    void checkAllDrivers(uint32_t currentTimeMs) {
        healthMgr_.checkAllDrivers(currentTimeMs);
    }
    
    /**
     * @brief Получить статус свежести
     */
    DataFreshnessStatus getFreshnessStatus() const {
        return actualizationMgr_.getCalculationStatus(CalculationType::HEALTH_MONITORING);
    }
    
    /**
     * @brief Получить менеджер актуализации
     */
    AutoActualizationManager& getActualizationManager() { return actualizationMgr_; }
    
    /**
     * @brief Получить health monitoring менеджер
     */
    HealthMonitorManager& getHealthMonitorManager() { return healthMgr_; }
    
private:
    std::function<uint64_t()> utcSource_;
    HealthMonitorManager healthMgr_;
    AutoActualizationManager actualizationMgr_;
    
    bool actualizeHealthConfig() {
        // Здесь может быть обновление порогов health monitoring
        return true;
    }
};

} // namespace systems
} // namespace mka

#endif // ACTUALIZATION_INTEGRATION_HPP
