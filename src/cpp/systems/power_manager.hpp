/**
 * @file power_manager.hpp
 * @brief Система управления питанием МКА
 *
 * Компоненты:
 * - MPPT алгоритм (Maximum Power Point Tracking)
 * - Балансировка нагрузки
 * - Прогноз доступной мощности
 * - Аварийное отключение нагрузки
 * - Гистерезисное управление
 *
 * Архитектура:
 * - SolarPanel: модель солнечной панели
 * - Battery: модель батареи
 * - Load: нагрузка (потребитель)
 * - PowerManager: управление всей системой
 */

#ifndef POWER_MANAGER_HPP
#define POWER_MANAGER_HPP

#include <cstdint>
#include <cstddef>
#include <array>
#include <vector>
#include <functional>
#include <algorithm>
#include <cmath>
#include <limits>

namespace mka {
namespace systems {

// ============================================================================
// Типы и константы
// ============================================================================

/// Максимальное число солнечных панелей
constexpr size_t MAX_SOLAR_PANELS = 6;

/// Максимальное число нагрузок
constexpr size_t MAX_LOADS = 16;

/// Максимальное число батарей
constexpr size_t MAX_BATTERIES = 2;

/// Напряжение (В)
using Voltage = float;

/// Ток (А)
using Current = float;

/// Мощность (Вт)
using Power = float;

/// Энергия (Вт·ч)
using Energy = float;

/// Температура (°C)
using Temperature = float;

/// Эффективность (0-1)
using Efficiency = float;

// ============================================================================
// Солнечная панель
// ============================================================================

/// Состояние солнечной панели
enum class PanelState : uint8_t {
    OK = 0,
    DEGRADED = 1,     // Деградация
    SHADOWED = 2,     // В тени
    FAILED = 3,       // Отказ
    DISCONNECTED = 4  // Отключена
};

/// Модель солнечной панели
struct SolarPanel {
    uint8_t id;                     // ID панели
    const char* name;               // Имя
    PanelState state;               // Состояние
    
    // Параметры панели
    Voltage voc;                    // Напряжение холостого хода (В)
    Voltage vmp;                    // Напряжение в точке макс. мощности (В)
    Current isc;                    // Ток короткого замыкания (А)
    Current imp;                    // Ток в точке макс. мощности (А)
    Power maxPower;                 // Максимальная мощность (Вт)
    
    // Текущие измерения
    Voltage voltage;                // Текущее напряжение (В)
    Current current;                // Текущий ток (А)
    Power power;                    // Текущая мощность (Вт)
    Temperature temperature;        // Температура панели (°C)
    float illumination;             // Освещённость (0-1, 1 = полное солнце)
    
    // MPPT
    Voltage mpptVoltage;            // Напряжение MPPT (В)
    float mpptEfficiency;           // Эффективность MPPT (0-1)
    
    // Деградация
    float degradationFactor;        // Фактор деградации (1.0 = новая, <1.0 = старая)
    uint32_t operationalHours;      // Часы работы
    
    SolarPanel() : id(0), name(nullptr), state(PanelState::OK),
                   voc(0), vmp(0), isc(0), imp(0), maxPower(0),
                   voltage(0), current(0), power(0), temperature(25),
                   illumination(0), mpptVoltage(0), mpptEfficiency(0),
                   degradationFactor(1.0f), operationalHours(0) {}
    
    /**
     * @brief Обновить текущие параметры панели
     * @param illum Освещённость (0-1)
     * @param temp Температура (°C)
     */
    void update(float illum, float temp) {
        illumination = illum;
        temperature = temp;
        
        if (state == PanelState::FAILED || state == PanelState::DISCONNECTED) {
            voltage = 0;
            current = 0;
            power = 0;
            return;
        }
        
        if (state == PanelState::SHADOWED) {
            illum *= 0.1f;  // В тени - 10% освещённости
        }
        
        // Учёт деградации
        float deg = degradationFactor;
        
        // Температурная коррекция (типично -0.4%/°C для Si)
        float tempCoeff = 1.0f - 0.004f * (temp - 25.0f);
        
        // Расчёт тока
        current = isc * illum * deg * tempCoeff;
        
        // Расчёт напряжения (упрощённая модель)
        voltage = voc * (1.0f - 0.1f * (1.0f - illum)) * deg * tempCoeff;
        
        // Мощность
        power = voltage * current;
    }
};

// ============================================================================
// Батарея
// ============================================================================

/// Состояние батареи
enum class BatteryState : uint8_t {
    CHARGING = 0,
    DISCHARGING = 1,
    FULL = 2,
    EMPTY = 3,
    OVERHEAT = 4,
    FAILED = 5
};

/// Модель батареи
struct Battery {
    uint8_t id;                     // ID батареи
    const char* name;               // Имя
    BatteryState state;             // Состояние
    
    // Параметры
    Voltage nominalVoltage;         // Номинальное напряжение (В)
    Energy capacity;                // Ёмкость (Вт·ч)
    Voltage minVoltage;             // Минимальное напряжение (В)
    Voltage maxVoltage;             // Максимальное напряжение (В)
    Current maxChargeCurrent;       // Макс. ток заряда (А)
    Current maxDischargeCurrent;    // Макс. ток разряда (А)
    Temperature maxTemp;            // Макс. температура (°C)
    
    // Текущие измерения
    Voltage voltage;                // Текущее напряжение (В)
    Current current;                // Ток (+ = заряд, - = разряд) (А)
    Energy remainingEnergy;         // Оставшаяся энергия (Вт·ч)
    Temperature temperature;        // Температура (°C)
    float soc;                      // State of Charge (0-1)
    float soh;                      // State of Health (0-1)
    
    // Циклы
    uint32_t chargeCycles;          // Число циклов заряда
    uint32_t dischargeCycles;       // Число циклов разряда
    
    Battery() : id(0), name(nullptr), state(BatteryState::DISCHARGING),
                nominalVoltage(0), capacity(0), minVoltage(0), maxVoltage(0),
                maxChargeCurrent(0), maxDischargeCurrent(0), maxTemp(45),
                voltage(0), current(0), remainingEnergy(0), temperature(25),
                soc(0), soh(1.0f), chargeCycles(0), dischargeCycles(0) {}
    
    /**
     * @brief Обновить параметры батареи
     * @param chargePower Мощность заряда (Вт, >0 = заряд, <0 = разряд)
     * @param dt Время (сек)
     */
    void update(Power chargePower, float dt) {
        float hours = dt / 3600.0f;
        
        if (state == BatteryState::FAILED) return;
        
        // Проверка температуры
        if (temperature > maxTemp) {
            state = BatteryState::OVERHEAT;
            current = 0;
            return;
        } else if (state == BatteryState::OVERHEAT && temperature < maxTemp - 5) {
            state = BatteryState::DISCHARGING;
        }
        
        // Ограничение мощности
        if (chargePower > 0) {
            // Заряд
            Power maxCharge = nominalVoltage * maxChargeCurrent;
            if (chargePower > maxCharge) chargePower = maxCharge;
            
            // Проверка полного заряда
            if (soc >= 0.99f) {
                state = BatteryState::FULL;
                current = 0;
                chargePower = 0;
            } else {
                state = BatteryState::CHARGING;
            }
        } else {
            // Разряд
            Power maxDischarge = nominalVoltage * maxDischargeCurrent;
            if (chargePower < -maxDischarge) chargePower = -maxDischarge;
            
            // Проверка полного разряда
            if (soc <= 0.01f) {
                state = BatteryState::EMPTY;
                current = 0;
                chargePower = 0;
            } else {
                state = BatteryState::DISCHARGING;
            }
        }
        
        // Обновление энергии
        float energyChange = chargePower * hours;
        remainingEnergy += energyChange;
        
        // Ограничение ёмкости
        if (remainingEnergy < 0) remainingEnergy = 0;
        if (remainingEnergy > capacity) remainingEnergy = capacity;
        
        // SOC
        soc = remainingEnergy / capacity;
        
        // Напряжение (упрощённая модель: линейная зависимость от SOC)
        voltage = minVoltage + (maxVoltage - minVoltage) * soc;
        
        // Ток
        current = (voltage > 0) ? chargePower / voltage : 0;
        
        // Подсчёт циклов
        if (state == BatteryState::CHARGING && soc > 0.8f) {
            chargeCycles++;
        } else if (state == BatteryState::DISCHARGING && soc < 0.2f) {
            dischargeCycles++;
        }
        
        // Деградация (упрощённая модель)
        soh = 1.0f - static_cast<float>(chargeCycles) / 5000.0f;
        if (soh < 0.0f) soh = 0.0f;
    }
};

// ============================================================================
// Нагрузка
// ============================================================================

/// Приоритет нагрузки
enum class LoadPriority : uint8_t {
    CRITICAL = 0,     // Критическая (никогда не отключать)
    HIGH = 1,         // Высокий приоритет
    MEDIUM = 2,       // Средний приоритет
    LOW = 3,          // Низкий приоритет (первая отключается)
    BEST_EFFORT = 4   // По возможности (первая отключается)
};

/// Нагрузка (потребитель)
struct Load {
    uint8_t id;                     // ID нагрузки
    const char* name;               // Имя
    LoadPriority priority;          // Приоритет
    bool enabled;                   // Включена
    bool autoManaged;               // Автоматическое управление
    
    Power nominalPower;             // Номинальная мощность (Вт)
    Power actualPower;              // Фактическая мощность (Вт)
    Voltage voltage;                // Напряжение питания (В)
    Current current;                // Ток (А)
    
    // Управление
    float minVoltage;               // Мин. напряжение для работы (В)
    float maxVoltage;               // Макс. напряжение для работы (В)
    
    // Статистика
    float totalEnergy;              // Потреблённая энергия (Вт·ч)
    uint32_t onTime;                // Время работы (сек)
    uint32_t offCount;              // Число отключений
    
    Load() : id(0), name(nullptr), priority(LoadPriority::MEDIUM),
             enabled(true), autoManaged(true),
             nominalPower(0), actualPower(0), voltage(0), current(0),
             minVoltage(3.0f), maxVoltage(4.2f),
             totalEnergy(0), onTime(0), offCount(0) {}
    
    /**
     * @brief Обновить параметры нагрузки
     * @param busVoltage Напряжение шины (В)
     * @param dt Время (сек)
     */
    void update(Voltage busVoltage, float dt) {
        if (!enabled) {
            actualPower = 0;
            current = 0;
            return;
        }
        
        voltage = busVoltage;
        
        // Проверка диапазона напряжения
        if (voltage < minVoltage || voltage > maxVoltage) {
            if (autoManaged) {
                enabled = false;
                offCount++;
            }
            actualPower = 0;
            current = 0;
            return;
        }
        
        // Расчёт тока
        current = (voltage > 0) ? nominalPower / voltage : 0;
        actualPower = nominalPower;
        
        // Статистика
        totalEnergy += actualPower * dt / 3600.0f;
        onTime += static_cast<uint32_t>(dt);
    }
};

// ============================================================================
// MPPT алгоритм
// ============================================================================

/**
 * @brief MPPT (Maximum Power Point Tracking) — алгоритм отслеживания точки
 *        максимальной мощности солнечных панелей
 *
 * Реализует алгоритм Perturb & Observe (P&O):
 * 1. Измерить текущую мощность P(k)
 * 2. Изменить напряжение на delta_V
 * 3. Измерить новую мощность P(k+1)
 * 4. Если P увеличилась — продолжить в том же направлении
 * 5. Если P уменьшилась — изменить направление
 *
 * Преимущества:
 * - Простая реализация
 * - Не требует знания параметров панели
 * - Адаптация к изменяющимся условиям
 *
 * @note Для реального применения требуется аппаратный DC-DC конвертер
 */
class MPPTController {
public:
    /// Конфигурация MPPT
    struct Config {
        Voltage initialVoltage = 15.0f;     // Начальное напряжение (В)
        Voltage perturbationStep = 0.1f;    // Шаг возмущения (В)
        Voltage minVoltage = 5.0f;          // Мин. напряжение (В)
        Voltage maxVoltage = 30.0f;         // Макс. напряжение (В)
        float updateInterval = 1.0f;        // Интервал обновления (сек)
        Voltage maxVoltageChange = 0.5f;    // Макс. изменение за шаг (В)
    };

    MPPTController() = default;
    explicit MPPTController(const Config& config) : config_(config) {
        currentVoltage_ = config.initialVoltage;
    }

    /**
     * @brief Обновить MPPT
     * @param voltage Текущее напряжение панели (В)
     * @param current Текущий ток панели (А)
     * @return Рекомендуемое напряжение для макс. мощности
     */
    Voltage update(Voltage voltage, Current current) {
        // Текущая мощность
        Power currentPower = voltage * current;
        
        // Разница мощности
        Power powerDelta = currentPower - lastPower_;
        Voltage voltageDelta = voltage - lastVoltage_;
        
        // Perturb & Observe логика
        if (std::abs(voltageDelta) > 1e-6f) {
            if (powerDelta > 0) {
                // Мощность увеличилась — продолжать в том же направлении
                if (voltageDelta > 0) {
                    perturbationDirection_ = 1;
                } else {
                    perturbationDirection_ = -1;
                }
            } else {
                // Мощность уменьшилась — изменить направление
                perturbationDirection_ = -perturbationDirection_;
            }
        }
        
        // Новое напряжение
        Voltage newVoltage = currentVoltage_ + 
            static_cast<float>(perturbationDirection_) * config_.perturbationStep;
        
        // Ограничение диапазона
        if (newVoltage < config_.minVoltage) newVoltage = config_.minVoltage;
        if (newVoltage > config_.maxVoltage) newVoltage = config_.maxVoltage;
        
        // Ограничение скорости изменения
        Voltage maxChange = config_.maxVoltageChange;
        if (newVoltage - currentVoltage_ > maxChange) {
            newVoltage = currentVoltage_ + maxChange;
        } else if (currentVoltage_ - newVoltage > maxChange) {
            newVoltage = currentVoltage_ - maxChange;
        }
        
        currentVoltage_ = newVoltage;
        lastVoltage_ = voltage;
        lastPower_ = currentPower;
        
        // Расчёт эффективности
        if (voltage > 0 && current > 0) {
            Voltage theoreticalVmp = voltage;  // Упрощённо
            currentEfficiency_ = std::min(voltage / theoreticalVmp, 1.0f);
        }
        
        return currentVoltage_;
    }

    /**
     * @brief Получить текущую эффективность MPPT
     */
    float getEfficiency() const { return currentEfficiency_; }
    
    /**
     * @brief Получить текущее MPPT напряжение
     */
    Voltage getMPPTVoltage() const { return currentVoltage_; }
    
    /**
     * @brief Сбросить контроллер
     */
    void reset() {
        currentVoltage_ = config_.initialVoltage;
        lastPower_ = 0;
        lastVoltage_ = 0;
        perturbationDirection_ = 1;
        currentEfficiency_ = 0;
    }
    
    void setConfig(const Config& config) { config_ = config; }
    const Config& getConfig() const { return config_; }

private:
    Config config_;
    Voltage currentVoltage_ = 0;
    Power lastPower_ = 0;
    Voltage lastVoltage_ = 0;
    int perturbationDirection_ = 1;
    float currentEfficiency_ = 0;
};

// ============================================================================
// Power Manager — Управление питанием
// ============================================================================

/**
 * @brief Power Manager — система управления электропитанием МКА
 *
 * Компоненты:
 * - Управление солнечными панелями (MPPT)
 * - Управление батареями (заряд/разряд)
 * - Управление нагрузками (включение/отключение)
 * - Балансировка мощности
 * - Аварийное отключение
 * - Прогнозирование энергии
 *
 * @note Thread-safe НЕ гарантируется (для RTOS использовать с мьютексами)
 */
class PowerManager {
public:
    /// Конфигурация Power Manager
    struct Config {
        Voltage busVoltage = 12.0f;             // Напряжение шины (В)
        float lowBatteryThreshold = 0.2f;       // Порог низкого заряда (SOC)
        float criticalBatteryThreshold = 0.1f;  // Порог критического заряда
        float highBatteryThreshold = 0.9f;      // Порог полного заряда
        float powerReserve = 0.1f;              // Резерв мощности (доля)
        float loadSheddingMargin = 5.0f;        // Запас для отключения нагрузок (Вт)
        bool enableMPPT = true;                 // Включить MPPT
        bool enableLoadShedding = true;         // Включить автоматическое отключение
        bool enablePowerForecast = true;        // Включить прогноз мощности
    };

    PowerManager() = default;
    explicit PowerManager(const Config& config) : config_(config) {}

    /**
     * @brief Добавить солнечную панель
     */
    bool addPanel(SolarPanel& panel) {
        if (numPanels_ >= MAX_SOLAR_PANELS) return false;
        panels_[numPanels_++] = &panel;
        if (config_.enableMPPT) {
            mpptControllers_[numPanels_ - 1] = std::make_unique<MPPTController>();
        }
        return true;
    }

    /**
     * @brief Добавить батарею
     */
    bool addBattery(Battery& battery) {
        if (numBatteries_ >= MAX_BATTERIES) return false;
        batteries_[numBatteries_++] = &battery;
        return true;
    }

    /**
     * @brief Добавить нагрузку
     */
    bool addLoad(Load& load) {
        if (numLoads_ >= MAX_LOADS) return false;
        loads_[numLoads_++] = &load;
        return true;
    }

    /**
     * @brief Обновить состояние системы
     * @param dt Время (сек)
     */
    void update(float dt) {
        // Обновление солнечных панелей
        Power totalSolarPower = 0;
        for (size_t i = 0; i < numPanels_; i++) {
            auto* panel = panels_[i];
            if (panel->state != PanelState::FAILED && 
                panel->state != PanelState::DISCONNECTED) {
                
                // MPPT
                if (config_.enableMPPT && mpptControllers_[i]) {
                    Voltage mpptV = mpptControllers_[i]->update(
                        panel->voltage, panel->current);
                    panel->mpptVoltage = mpptV;
                    panel->mpptEfficiency = mpptControllers_[i]->getEfficiency();
                }
                
                panel->update(panel->illumination, panel->temperature);
                totalSolarPower += panel->power;
            }
        }
        
        // Расчёт баланса мощности
        Power loadPower = 0;
        for (size_t i = 0; i < numLoads_; i++) {
            auto* load = loads_[i];
            if (load->enabled) {
                load->update(config_.busVoltage, dt);
                loadPower += load->actualPower;
            }
        }
        
        // Заряд/разряд батарей
        Power batteryPower = totalSolarPower - loadPower;
        
        // Резерв мощности
        if (config_.enablePowerForecast) {
            float reserve = totalSolarPower * config_.powerReserve;
            batteryPower -= reserve;
        }
        
        // Обновление батарей
        for (size_t i = 0; i < numBatteries_; i++) {
            auto* battery = batteries_[i];
            battery->update(batteryPower / static_cast<float>(numBatteries_), dt);
        }
        
        // Проверка аварийного отключения нагрузок
        if (config_.enableLoadShedding) {
            checkLoadShedding(totalSolarPower);
        }
        
        // Обновление статистики
        stats_.solarPower = totalSolarPower;
        stats_.loadPower = loadPower;
        stats_.batteryPower = batteryPower;
        stats_.powerBalance = totalSolarPower - loadPower;
        
        // Расчёт общей энергии батарей
        stats_.totalBatteryEnergy = 0;
        stats_.averageSOC = 0;
        for (size_t i = 0; i < numBatteries_; i++) {
            stats_.totalBatteryEnergy += batteries_[i]->remainingEnergy;
            stats_.averageSOC += batteries_[i]->soc;
        }
        if (numBatteries_ > 0) {
            stats_.averageSOC /= static_cast<float>(numBatteries_);
        }
    }

    /**
     * @brief Проверить необходимость отключения нагрузок
     */
    void checkLoadShedding(Power availablePower) {
        // Проверка критического уровня батарей
        bool criticalBattery = false;
        for (size_t i = 0; i < numBatteries_; i++) {
            if (batteries_[i]->soc < config_.criticalBatteryThreshold) {
                criticalBattery = true;
                break;
            }
        }
        
        if (!criticalBattery) return;
        
        // Отключение нагрузок по приоритету (от низшего к высшему)
        LoadPriority priorities[] = {
            LoadPriority::BEST_EFFORT,
            LoadPriority::LOW,
            LoadPriority::MEDIUM,
            LoadPriority::HIGH
            // CRITICAL никогда не отключается
        };
        
        for (auto priority : priorities) {
            for (size_t i = 0; i < numLoads_; i++) {
                auto* load = loads_[i];
                if (load->enabled && load->autoManaged && 
                    load->priority == priority) {
                    load->enabled = false;
                    load->offCount++;
                }
            }
        }
    }

    /**
     * @brief Включить нагрузку
     */
    bool enableLoad(uint8_t loadId) {
        for (size_t i = 0; i < numLoads_; i++) {
            if (loads_[i]->id == loadId) {
                loads_[i]->enabled = true;
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Отключить нагрузку
     */
    bool disableLoad(uint8_t loadId) {
        for (size_t i = 0; i < numLoads_; i++) {
            if (loads_[i]->id == loadId) {
                loads_[i]->enabled = false;
                loads_[i]->offCount++;
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Прогноз доступной энергии на период орбиты
     * @param orbitalPeriod Период орбиты (сек)
     * @param sunFraction Доля времени на свету (0-1)
     * @return Прогноз энергии (Вт·ч)
     */
    float forecastEnergy(float orbitalPeriod, float sunFraction) const {
        float sunTime = orbitalPeriod * sunFraction;
        float eclipseTime = orbitalPeriod - sunTime;
        
        Power avgSolarPower = 0;
        for (size_t i = 0; i < numPanels_; i++) {
            if (panels_[i]->state == PanelState::OK) {
                avgSolarPower += panels_[i]->maxPower * 
                    panels_[i]->degradationFactor * panels_[i]->illumination;
            }
        }
        
        Energy generated = avgSolarPower * sunTime / 3600.0f;
        
        // Потребление нагрузок
        Power totalLoad = 0;
        for (size_t i = 0; i < numLoads_; i++) {
            if (loads_[i]->enabled) {
                totalLoad += loads_[i]->nominalPower;
            }
        }
        Energy consumed = totalLoad * orbitalPeriod / 3600.0f;
        
        return generated - consumed;
    }

    /**
     * @brief Получить статистику
     */
    struct Statistics {
        Power solarPower = 0;           // Мощность солнечных панелей (Вт)
        Power loadPower = 0;            // Мощность нагрузок (Вт)
        Power batteryPower = 0;         // Мощность батарей (Вт, + = заряд)
        Power powerBalance = 0;         // Баланс мощности (Вт)
        Energy totalBatteryEnergy = 0;  // Общая энергия батарей (Вт·ч)
        float averageSOC = 0;           // Средний SOC (0-1)
    };
    
    Statistics getStats() const { return stats_; }
    
    /**
     * @brief Сбросить систему
     */
    void reset() {
        numPanels_ = 0;
        numBatteries_ = 0;
        numLoads_ = 0;
        stats_ = Statistics{};
    }

    void setConfig(const Config& config) { config_ = config; }
    const Config& getConfig() const { return config_; }

private:
    Config config_;
    Statistics stats_;
    
    SolarPanel* panels_[MAX_SOLAR_PANELS] = {nullptr};
    Battery* batteries_[MAX_BATTERIES] = {nullptr};
    Load* loads_[MAX_LOADS] = {nullptr};
    
    std::unique_ptr<MPPTController> mpptControllers_[MAX_SOLAR_PANELS];
    
    size_t numPanels_ = 0;
    size_t numBatteries_ = 0;
    size_t numLoads_ = 0;
};

} // namespace systems
} // namespace mka

#endif // POWER_MANAGER_HPP
