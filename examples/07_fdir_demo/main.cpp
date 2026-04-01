/**
 * @file main.cpp
 * @brief Пример системы FDIR (Fault Detection, Isolation, Recovery)
 *
 * Демонстрирует мониторинг параметров, детекцию аномалий
 * и автоматическое восстановление после сбоев.
 */

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <random>
#include <thread>
#include <chrono>

// Системные модули
#include "systems/fdir.hpp"
#include "systems/log_system.hpp"

// ============================================================================
// Конфигурация
// ============================================================================

namespace config {
    constexpr uint32_t MONITOR_PERIOD_MS = 500;   // Период мониторинга
    constexpr uint32_t SIMULATION_DURATION_S = 30; // Длительность симуляции

    // Параметры для мониторинга
    constexpr float BATTERY_NOMINAL_V = 7.4f;
    constexpr float TEMP_NOMINAL_C = 25.0f;
    constexpr float CURRENT_NOMINAL_A = 2.0f;
}

// ============================================================================
// Глобальные объекты
// ============================================================================

using namespace mka::fdir;
using namespace mka::logging;

// Менеджер FDIR
FDIRManager fdir;

// Логгер
Logger logger;

// Генератор случайных чисел для симуляции
std::random_device rd;
std::mt19937 gen(rd());

// Флаги состояния
bool batteryFault = false;
bool tempFault = false;
bool currentFault = false;

// ============================================================================
// Инициализация
// ============================================================================

bool init() {
    printf("=== FDIR Demo ===\n\n");

    // Инициализация логгера
    logger.init();
    logger.setLogLevel(LogLevel::INFO);

    LOG_INFO("FDIR system initializing...");

    // Регистрация параметров с порогами

    // Батарея (напряжение)
    auto batteryId = fdir.registerParameter({
        .name = "Battery Voltage",
        .nominalValue = config::BATTERY_NOMINAL_V,
        .warningLow = 6.5f,
        .warningHigh = 8.5f,
        .errorLow = 6.0f,
        .errorHigh = 9.0f,
        .criticalLow = 5.5f,
        .criticalHigh = 10.0f
    });
    printf("[OK] Registered: Battery Voltage (ID=%d)\n", (int)batteryId);

    // Температура
    auto tempId = fdir.registerParameter({
        .name = "Temperature",
        .nominalValue = config::TEMP_NOMINAL_C,
        .warningLow = 10.0f,
        .warningHigh = 40.0f,
        .errorLow = 0.0f,
        .errorHigh = 50.0f,
        .criticalLow = -10.0f,
        .criticalHigh = 60.0f
    });
    printf("[OK] Registered: Temperature (ID=%d)\n", (int)tempId);

    // Ток потребления
    auto currentId = fdir.registerParameter({
        .name = "Current",
        .nominalValue = config::CURRENT_NOMINAL_A,
        .warningLow = 0.1f,
        .warningHigh = 3.0f,
        .errorLow = 0.0f,
        .errorHigh = 5.0f,
        .criticalLow = 0.0f,
        .criticalHigh = 8.0f
    });
    printf("[OK] Registered: Current (ID=%d)\n", (int)currentId);

    // Регистрация детекторов аномалий

    // Детектор застывшего значения (Frozen Value)
    fdir.registerFrozenValueDetector(batteryId, 5000);  // 5 секунд
    printf("[OK] Registered: Frozen Value Detector (Battery)\n");

    // Детектор застывшего бита (Stuck Bit)
    fdir.registerStuckBitDetector(tempId, 10000);  // 10 секунд
    printf("[OK] Registered: Stuck Bit Detector (Temperature)\n");

    // Детектор скачков (Glitch)
    fdir.registerGlitchDetector(currentId, 2.0f);  // Порог скачка
    printf("[OK] Registered: Glitch Detector (Current)\n");

    LOG_INFO("FDIR system initialized");
    printf("\n");

    return true;
}

// ============================================================================
// Симуляция данных
// ============================================================================

float simulateBatteryVoltage() {
    static float voltage = config::BATTERY_NOMINAL_V;

    // Нормальные флуктуации
    std::uniform_real_distribution<float> dist(-0.1f, 0.1f);
    voltage += dist(gen);

    // Симуляция неисправности
    if (batteryFault) {
        voltage -= 0.3f;  // Постепенный разряд
    }

    // Ограничение
    if (voltage < 3.0f) voltage = 3.0f;
    if (voltage > 12.0f) voltage = 12.0f;

    return voltage;
}

float simulateTemperature() {
    static float temp = config::TEMP_NOMINAL_C;

    std::uniform_real_distribution<float> dist(-0.5f, 0.5f);
    temp += dist(gen);

    if (tempFault) {
        temp += 2.0f;  // Быстрый нагрев
    }

    return temp;
}

float simulateCurrent() {
    std::uniform_real_distribution<float> dist(-0.2f, 0.2f);
    float current = config::CURRENT_NOMINAL_A + dist(gen);

    if (currentFault) {
        current += 3.0f;  // Скачок тока
    }

    return current > 0 ? current : 0.1f;
}

// ============================================================================
// Мониторинг и вывод
// ============================================================================

void printStatus(const FDIRManager::Status& status) {
    const char* statusStr = "OK";
    switch (status) {
        case FDIRManager::Status::WARNING: statusStr = "WARNING"; break;
        case FDIRManager::Status::ERROR_STATE: statusStr = "ERROR"; break;
        case FDIRManager::Status::CRITICAL: statusStr = "CRITICAL"; break;
        default: break;
    }
    printf("System Status: %s\n", statusStr);
}

void printParameter(const char* name, float value, FDIRManager::Status status) {
    const char* statusStr = "OK";
    switch (status) {
        case FDIRManager::Status::WARNING: statusStr = "WARN"; break;
        case FDIRManager::Status::ERROR_STATE: statusStr = "ERR"; break;
        case FDIRManager::Status::CRITICAL: statusStr = "CRIT"; break;
        default: break;
    }

    printf("%-18s: %8.2f [%s]\n", name, value, statusStr);
}

void monitorAndPrint() {
    // Чтение "сенсоров"
    float battery = simulateBatteryVoltage();
    float temp = simulateTemperature();
    float current = simulateCurrent();

    // Обновление параметров в FDIR
    fdir.updateParameter(0, battery);
    fdir.updateParameter(1, temp);
    fdir.updateParameter(2, current);

    // Проверка всех параметров
    auto batteryStatus = fdir.checkParameter(0);
    auto tempStatus = fdir.checkParameter(1);
    auto currentStatus = fdir.checkParameter(2);

    // Вывод
    printf("\n=== FDIR Monitor ===\n");
    printParameter("Battery [V]", battery, batteryStatus);
    printParameter("Temperature [C]", temp, tempStatus);
    printParameter("Current [A]", current, currentStatus);

    // Общий статус
    auto systemStatus = fdir.getSystemStatus();
    printStatus(systemStatus);

    // Проверка детекторов аномалий
    auto anomalies = fdir.detectAnomalies();
    if (!anomalies.empty()) {
        printf("\n!!! ANOMALIES DETECTED: %zu\n", anomalies.size());
        for (const auto& anomaly : anomalies) {
            printf("    - Parameter %d: %s\n", anomaly.paramId, anomaly.description.c_str());
        }
    }

    // Статистика FDIR
    auto stats = fdir.getStatistics();
    printf("\nStatistics:\n");
    printf("  Updates: %lu\n", stats.totalUpdates);
    printf("  Warnings: %lu\n", stats.warningCount);
    printf("  Errors: %lu\n", stats.errorCount);
    printf("  Critical: %lu\n", stats.criticalCount);
    printf("====================\n");
}

// ============================================================================
// Сценарий демонстрации
// ============================================================================

void runDemoScenario() {
    printf("Running demo scenario...\n");
    printf("Simulating faults for demonstration.\n\n");

    auto startTime = std::chrono::steady_clock::now();

    while (true) {
        auto elapsed = std::chrono::steady_clock::now() - startTime;
        uint32_t elapsedSec = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

        if (elapsedSec >= config::SIMULATION_DURATION_S) {
            break;
        }

        // Сценарий по времени
        if (elapsedSec == 5) {
            printf("\n>>> Injecting battery fault...\n");
            batteryFault = true;
        }

        if (elapsedSec == 10) {
            printf(">>> Clearing battery fault\n");
            batteryFault = false;
        }

        if (elapsedSec == 15) {
            printf(">>> Injecting temperature fault...\n");
            tempFault = true;
        }

        if (elapsedSec == 20) {
            printf(">>> Clearing temperature fault\n");
            tempFault = false;
        }

        if (elapsedSec == 25) {
            printf(">>> Injecting current spike...\n");
            currentFault = true;
        }

        if (elapsedSec == 27) {
            printf(">>> Clearing current spike\n");
            currentFault = false;
        }

        monitorAndPrint();

        std::this_thread::sleep_for(std::chrono::milliseconds(config::MONITOR_PERIOD_MS));
    }
}

// ============================================================================
// Основная программа
// ============================================================================

int main() {
    if (!init()) {
        printf("[FATAL] Initialization failed\n");
        return 1;
    }

    runDemoScenario();

    // Финальная статистика
    printf("\n=== Final Report ===\n");
    auto stats = fdir.getStatistics();
    printf("Total updates: %lu\n", stats.totalUpdates);
    printf("Total warnings: %lu\n", stats.warningCount);
    printf("Total errors: %lu\n", stats.errorCount);
    printf("Total critical: %lu\n", stats.criticalCount);

    // История событий
    auto events = fdir.getEventHistory(10);  // Последние 10 событий
    if (!events.empty()) {
        printf("\nRecent events:\n");
        for (const auto& event : events) {
            printf("  [%lu] %s: %s\n",
                   (unsigned long)event.timestamp,
                   event.paramName.c_str(),
                   event.description.c_str());
        }
    }

    LOG_INFO("FDIR demo completed");

    printf("\nDemo completed successfully.\n");
    return 0;
}
