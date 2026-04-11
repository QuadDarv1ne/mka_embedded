/**
 * @file task_scheduler.hpp
 * @brief Планировщик задач для бортового ПО МКА
 *
 * Система планирования задач с поддержкой:
 * - Расписания по времени орбиты
 * - Приоритетов задач
 * - Зависимостей между задачами
 * - Обработки конфликтов ресурсов
 * - Энерго-балансировки
 *
 * Архитектура:
 * - Task: единица работы с приоритетом, ресурсами и зависимостями
 * - Schedule: расписание задач на орбитальный период
 * - Scheduler: планировщик с разрешением конфликтов
 */

#ifndef TASK_SCHEDULER_HPP
#define TASK_SCHEDULER_HPP

#include <cstdint>
#include <cstddef>
#include <array>
#include <vector>
#include <string>
#include <functional>
#include <algorithm>
#include <cmath>
#include <limits>

namespace mka {
namespace systems {

// ============================================================================
// Типы и константы
// ============================================================================

/// Максимальное число задач в расписании
constexpr size_t MAX_TASKS = 64;

/// Максимальное число зависимостей на задачу
constexpr size_t MAX_DEPENDENCIES = 8;

/// Максимальное число ресурсов на задачу
constexpr size_t MAX_RESOURCES = 4;

/// Идентификатор задачи
using TaskId = uint16_t;

/// Приоритет задачи (0 = низкий, 255 = высокий)
using Priority = uint8_t;

/// Энергия (в Вт·с)
using Energy = float;

/// Время (в секундах от начала орбиты)
using OrbitTime = float;

/// Период орбиты (в секундах, типично ~5400с для LEO)
using OrbitalPeriod = float;

// ============================================================================
// Ресурсы
// ============================================================================

/// Типы ресурсов спутника
enum class ResourceType : uint8_t {
    NONE = 0,
    EPS_PRIMARY = 1,      // Электропитание (основная шина)
    EPS_SECONDARY = 2,    // Электропитание (резервная шина)
    ADCS = 3,             // Система ориентации
    PAYLOAD = 4,          // Полезная нагрузка
    COMM_UHF = 5,         // Связь UHF
    COMM_S = 6,           // Связь S-band
    THERMAL_HEATER = 7,   // Нагреватель
    OBC = 8,              // Бортовой компьютер
    MEMORY = 9,           // Карта памяти
    MAX = 10
};

/// Запрос ресурса
struct ResourceRequest {
    ResourceType type;
    float powerConsumption;  // Потребляемая мощность (Вт)
    bool exclusive;          // Требуется монопольный доступ
};

// ============================================================================
// Задача
// ============================================================================

/// Состояние задачи
enum class TaskState : uint8_t {
    PENDING = 0,       // Ожидает выполнения
    READY = 1,         // Готова к выполнению
    RUNNING = 2,       // Выполняется
    COMPLETED = 3,     // Завершена
    FAILED = 4,        // Ошибка выполнения
    CANCELLED = 5,     // Отменена
    BLOCKED = 6        // Заблокирована (ресурсы/зависимости)
};

/// Тип расписания
enum class ScheduleType : uint8_t {
    ONCE = 0,          // Однократное выполнение
    PERIODIC = 1,      // Периодическое
    ORBITAL = 2,       // По времени орбиты
    EVENT_DRIVEN = 3   // По событию
};

/// Задача планировщика
struct Task {
    TaskId id;                          // Уникальный ID
    const char* name;                   // Имя задачи
    Priority priority;                  // Приоритет (0-255)
    ScheduleType scheduleType;          // Тип расписания
    TaskState state;                    // Текущее состояние
    
    // Временные параметры
    OrbitTime startTime;                // Время начала (с начала орбиты)
    OrbitTime duration;                 // Ожидаемая длительность (сек)
    OrbitTime period;                   // Период повторения (для PERIODIC/ORBITAL)
    
    // Ресурсы
    ResourceRequest resources[MAX_RESOURCES];
    size_t numResources;
    
    // Зависимости
    TaskId dependencies[MAX_DEPENDENCIES];
    size_t numDependencies;
    
    // Энергия
    Energy energyBudget;                // Бюджет энергии (Вт·с)
    Energy estimatedEnergy;             // Ожидаемое потребление
    
    // Callback выполнения
    std::function<bool()> execute;      // Функция выполнения задачи
    std::function<void(bool)> onComplete;  // Callback завершения
    
    // Метаданные
    uint32_t executionCount;            // Число выполнений
    uint32_t failureCount;              // Число ошибок
    OrbitTime lastExecutionTime;        // Время последнего выполнения
    
    Task() : id(0), name(nullptr), priority(0), scheduleType(ScheduleType::ONCE),
             state(TaskState::PENDING), startTime(0), duration(0), period(0),
             numResources(0), numDependencies(0),
             energyBudget(0), estimatedEnergy(0),
             executionCount(0), failureCount(0), lastExecutionTime(0) {}
};

// ============================================================================
/// Результат планирования
enum class ScheduleResult : uint8_t {
    OK = 0,
    CONFLICT = 1,           // Конфликт ресурсов
    DEPENDENCY_NOT_MET = 2, // Зависимости не выполнены
    NO_ENERGY = 3,          // Недостаточно энергии
    SCHEDULE_FULL = 4,      // Расписание заполнено
    INVALID_TIME = 5,       // Некорректное время
    DUPLICATE_TASK = 6      // Задача уже существует
};

/// Элемент расписания
struct ScheduleEntry {
    TaskId taskId;
    OrbitTime startTime;
    OrbitTime endTime;
    bool isScheduled;
};

/// Статистика планировщика
struct SchedulerStats {
    size_t totalTasks;          // Всего задач
    size_t scheduledTasks;      // Запланированных
    size_t runningTasks;        // Выполняющихся
    size_t completedTasks;      // Завершённых
    size_t failedTasks;         // Ошибочных
    Energy totalEnergyBudget;   // Общий бюджет энергии
    Energy consumedEnergy;      // Потраченная энергия
    OrbitTime currentOrbitTime; // Текущее время орбиты
};

// ============================================================================
// TaskScheduler — Планировщик задач
// ============================================================================

/**
 * @brief Планировщик задач для бортового ПО МКА
 *
 * Особенности:
 * - Планирование по времени орбиты
 * - Приоритизация задач
 * - Разрешение конфликтов ресурсов
 * - Проверка зависимостей
 * - Энерго-балансировка
 * - Поддержка периодических и однократных задач
 *
 * @note Thread-safe НЕ гарантируется (для RTOS использовать с мьютексами)
 */
class TaskScheduler {
public:
    /// Конфигурация планировщика
    struct Config {
        OrbitalPeriod orbitalPeriod = 5400.0f;   // Период орбиты (сек, ~90 мин LEO)
        float minEnergyMargin = 0.1f;            // Минимальный запас энергии (доля)
        float maxConcurrentPower = 50.0f;        // Макс. одновременная мощность (Вт)
        size_t maxTasks = MAX_TASKS;             // Макс. число задач
        bool enableEnergyBalancing = true;        // Включить энерго-балансировку
        bool enableConflictResolution = true;     // Включить разрешение конфликтов
    };

    TaskScheduler() = default;
    explicit TaskScheduler(const Config& config) : config_(config) {
        schedule_.reserve(config.maxTasks);
    }

    /**
     * @brief Добавить задачу в планировщик
     */
    ScheduleResult addTask(const Task& task) {
        if (tasks_.size() >= config_.maxTasks) {
            return ScheduleResult::SCHEDULE_FULL;
        }
        
        // Проверка дубликата
        for (const auto& t : tasks_) {
            if (t.id == task.id) {
                return ScheduleResult::DUPLICATE_TASK;
            }
        }
        
        // Валидация времени
        if (task.startTime < 0 || task.duration <= 0) {
            return ScheduleResult::INVALID_TIME;
        }
        
        if (task.scheduleType == ScheduleType::ORBITAL && 
            task.startTime >= config_.orbitalPeriod) {
            return ScheduleResult::INVALID_TIME;
        }
        
        tasks_.push_back(task);
        return ScheduleResult::OK;
    }

    /**
     * @brief Удалить задачу
     */
    bool removeTask(TaskId taskId) {
        for (auto it = tasks_.begin(); it != tasks_.end(); ++it) {
            if (it->id == taskId) {
                // Проверка что нет зависимых задач
                for (const auto& t : tasks_) {
                    if (t.id != taskId) {
                        for (size_t i = 0; i < t.numDependencies; i++) {
                            if (t.dependencies[i] == taskId) {
                                return false;  // Есть зависимые задачи
                            }
                        }
                    }
                }
                tasks_.erase(it);
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Построить расписание на орбитальный период
     */
    ScheduleResult buildSchedule() {
        schedule_.clear();
        
        // Сортировка задач по приоритету (высокий приоритет первый)
        std::vector<Task*> sortedTasks;
        for (auto& task : tasks_) {
            if (task.state != TaskState::CANCELLED) {
                sortedTasks.push_back(&task);
            }
        }
        
        std::sort(sortedTasks.begin(), sortedTasks.end(),
            [](const Task* a, const Task* b) {
                return a->priority > b->priority;
            });
        
        // Планирование задач
        for (auto* task : sortedTasks) {
            ScheduleResult result = scheduleTask(task);
            if (result != ScheduleResult::OK) {
                task->state = TaskState::BLOCKED;
            }
        }
        
        return ScheduleResult::OK;
    }

    /**
     * @brief Выполнить готовые задачи на текущем времени орбиты
     * @param currentOrbitTime Текущее время от начала орбиты (сек)
     * @return Число запущенных задач
     */
    size_t executeReadyTasks(OrbitTime currentOrbitTime) {
        currentOrbitTime_ = currentOrbitTime;
        size_t executed = 0;
        
        // Обновление состояний задач
        for (auto& task : tasks_) {
            if (task.state == TaskState::PENDING || task.state == TaskState::BLOCKED) {
                if (isTaskReady(task, currentOrbitTime)) {
                    task.state = TaskState::READY;
                }
            }
        }
        
        // Выполнение задач по приоритету
        std::vector<Task*> readyTasks;
        for (auto& task : tasks_) {
            if (task.state == TaskState::READY) {
                readyTasks.push_back(&task);
            }
        }
        
        std::sort(readyTasks.begin(), readyTasks.end(),
            [](const Task* a, const Task* b) {
                return a->priority > b->priority;
            });
        
        // Проверка ресурсов и выполнение
        float currentPower = 0.0f;
        for (auto* task : readyTasks) {
            float taskPower = calculateTaskPower(*task);
            
            // Проверка энерго-баланса
            if (config_.enableEnergyBalancing) {
                if (currentPower + taskPower > config_.maxConcurrentPower) {
                    continue;  // Пропуск — недостаточно мощности
                }
            }
            
            // Проверка ресурсов
            if (config_.enableConflictResolution) {
                if (!checkResourcesAvailable(*task, currentOrbitTime)) {
                    continue;  // Пропуск — ресурсы заняты
                }
            }
            
            // Запуск задачи
            if (task->execute) {
                task->state = TaskState::RUNNING;
                currentPower += taskPower;
                executed++;
                
                // Выполнение callback
                bool success = task->execute();
                task->state = success ? TaskState::COMPLETED : TaskState::FAILED;
                task->executionCount++;
                if (!success) task->failureCount++;
                task->lastExecutionTime = currentOrbitTime;
                
                if (task->onComplete) {
                    task->onComplete(success);
                }
            }
        }
        
        return executed;
    }

    /**
     * @brief Проверить готова ли задача к выполнению
     */
    bool isTaskReady(const Task& task, OrbitTime currentTime) const {
        // Проверка времени
        if (currentTime < task.startTime) {
            return false;
        }
        
        // Проверка периодичности
        if (task.scheduleType == ScheduleType::PERIODIC || 
            task.scheduleType == ScheduleType::ORBITAL) {
            OrbitTime elapsed = currentTime - task.startTime;
            OrbitTime nextTime = task.startTime + task.period * 
                static_cast<OrbitTime>(static_cast<int>(elapsed / task.period) + 1);
            
            if (currentTime < nextTime) {
                // Уже выполнялась в этом цикле
                if (task.lastExecutionTime > currentTime - task.period) {
                    return false;
                }
            }
        }
        
        // Проверка зависимостей
        for (size_t i = 0; i < task.numDependencies; i++) {
            TaskId depId = task.dependencies[i];
            bool depCompleted = false;
            
            for (const auto& t : tasks_) {
                if (t.id == depId && t.state == TaskState::COMPLETED) {
                    depCompleted = true;
                    break;
                }
            }
            
            if (!depCompleted) {
                return false;  // Зависимость не выполнена
            }
        }
        
        return true;
    }

    /**
     * @brief Проверить доступны ли ресурсы
     */
    bool checkResourcesAvailable(const Task& task, OrbitTime currentTime) const {
        for (size_t i = 0; i < task.numResources; i++) {
            const auto& req = task.resources[i];
            
            if (req.exclusive) {
                // Проверка монопольного доступа
                for (const auto& t : tasks_) {
                    if (t.id != task.id && t.state == TaskState::RUNNING) {
                        for (size_t j = 0; j < t.numResources; j++) {
                            if (t.resources[j].type == req.type && 
                                t.resources[j].exclusive) {
                                return false;  // Ресурс занят
                            }
                        }
                    }
                }
            }
        }
        return true;
    }

    /**
     * @brief Получить статистику планировщика
     */
    SchedulerStats getStats() const {
        SchedulerStats stats;
        stats.totalTasks = tasks_.size();
        stats.scheduledTasks = 0;
        stats.runningTasks = 0;
        stats.completedTasks = 0;
        stats.failedTasks = 0;
        stats.totalEnergyBudget = 0;
        stats.consumedEnergy = 0;
        stats.currentOrbitTime = currentOrbitTime_;
        
        for (const auto& task : tasks_) {
            switch (task.state) {
                case TaskState::PENDING:
                case TaskState::READY:
                case TaskState::BLOCKED:
                    stats.scheduledTasks++;
                    break;
                case TaskState::RUNNING:
                    stats.runningTasks++;
                    break;
                case TaskState::COMPLETED:
                    stats.completedTasks++;
                    break;
                case TaskState::FAILED:
                    stats.failedTasks++;
                    break;
                default:
                    break;
            }
            
            stats.totalEnergyBudget += task.energyBudget;
            stats.consumedEnergy += task.estimatedEnergy * 
                static_cast<float>(task.executionCount);
        }
        
        return stats;
    }

    /**
     * @brief Сбросить все задачи
     */
    void reset() {
        tasks_.clear();
        schedule_.clear();
        currentOrbitTime_ = 0;
    }

    /**
     * @brief Получить задачу по ID
     */
    const Task* getTask(TaskId taskId) const {
        for (const auto& task : tasks_) {
            if (task.id == taskId) {
                return &task;
            }
        }
        return nullptr;
    }

    /**
     * @brief Обновить конфигурацию
     */
    void setConfig(const Config& config) { config_ = config; }
    const Config& getConfig() const { return config_; }

private:
    Config config_;
    std::vector<Task> tasks_;
    std::vector<ScheduleEntry> schedule_;
    OrbitTime currentOrbitTime_ = 0;

    /**
     * @brief Запланировать одну задачу
     */
    ScheduleResult scheduleTask(Task* task) {
        // Проверка зависимостей
        for (size_t i = 0; i < task->numDependencies; i++) {
            TaskId depId = task->dependencies[i];
            bool found = false;
            
            for (const auto& t : tasks_) {
                if (t.id == depId) {
                    found = true;
                    // Проверка что зависимость заканчивается до начала текущей
                    if (t.startTime + t.duration > task->startTime) {
                        // Сдвиг времени начала
                        task->startTime = t.startTime + t.duration;
                    }
                    break;
                }
            }
            
            if (!found) {
                return ScheduleResult::DEPENDENCY_NOT_MET;
            }
        }
        
        // Проверка энерго-баланса
        if (config_.enableEnergyBalancing) {
            float orbitPower = task->estimatedEnergy / task->duration;
            if (orbitPower > config_.maxConcurrentPower) {
                return ScheduleResult::NO_ENERGY;
            }
        }
        
        // Добавление в расписание
        ScheduleEntry entry;
        entry.taskId = task->id;
        entry.startTime = task->startTime;
        entry.endTime = task->startTime + task->duration;
        entry.isScheduled = true;
        schedule_.push_back(entry);
        
        return ScheduleResult::OK;
    }

    /**
     * @brief Вычислить потребляемую мощность задачи
     */
    float calculateTaskPower(const Task& task) const {
        float power = 0.0f;
        for (size_t i = 0; i < task.numResources; i++) {
            power += task.resources[i].powerConsumption;
        }
        return task.estimatedEnergy / task.duration > power ? 
               task.estimatedEnergy / task.duration : power;
    }
};

} // namespace systems
} // namespace mka

#endif // TASK_SCHEDULER_HPP
