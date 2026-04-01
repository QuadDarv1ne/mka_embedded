/**
 * @file main.cpp
 * @brief Пример использования режимов энергосбережения
 *
 * Демонстрирует:
 * - Sleep mode — остановка CPU, периферия работает
 * - Stop mode — остановка CPU и периферии, SRAM сохранена
 * - Standby mode — максимальное энергосбережение
 * - Wake-up из внешних прерываний
 */

#include <cstdint>
#include <cstdio>
#include <chrono>
#include <thread>

// HAL интерфейсы
#include "hal/hal_full.hpp"

// ============================================================================
// Конфигурация
// ============================================================================

namespace config {
    // LED для индикации режима
    constexpr uint32_t LED_PIN = 13;

    // Кнопка для wake-up
    constexpr uint32_t BUTTON_PIN = 0;

    // RTC будильник (секунды)
    constexpr uint32_t RTC_ALARM_SECONDS = 5;

    // Период измерения (мс)
    constexpr uint32_t MEASURE_PERIOD_MS = 1000;
}

// ============================================================================
// Глобальные объекты
// ============================================================================

hal::GPIO* led = nullptr;
hal::GPIO* button = nullptr;

// Флаги состояния
volatile bool wakeUpRequested = false;
volatile bool rtcAlarmTriggered = false;

// Статистика энергопотребления
struct PowerStats {
    uint32_t activeTimeMs = 0;
    uint32_t sleepTimeMs = 0;
    uint32_t stopTimeMs = 0;
    uint32_t standbyTimeMs = 0;
    uint32_t wakeUpCount = 0;
};

PowerStats powerStats;

// ============================================================================
// Режимы энергосбережения STM32
// ============================================================================

enum class LowPowerMode {
    RUN,        // Активный режим
    SLEEP,      // CPU остановлен, периферия работает
    STOP,       // CPU и периферия остановлены, SRAM сохранена
    STANDBY     // Максимальное энергосбережение, только backup domain
};

// ============================================================================
// Инициализация
// ============================================================================

bool init() {
    printf("=== Low Power Mode Example ===\n\n");

    // Инициализация LED
    led = new hal::GPIO(config::LED_PIN, hal::GPIOMode::OUTPUT);
    led->write(hal::GPIOPinState::LOW);

    // Инициализация кнопки (wake-up)
    button = new hal::GPIO(config::BUTTON_PIN, hal::GPIOMode::INPUT_PULLDOWN);

    printf("[OK] GPIO initialized\n");
    printf("LED:    pin %lu\n", config::LED_PIN);
    printf("Button: pin %lu (wake-up source)\n\n", config::BUTTON_PIN);

    return true;
}

// ============================================================================
// Функции управления питанием
// ============================================================================

/**
 * @brief Переход в Sleep mode
 * 
 * CPU останавливается, периферия продолжает работать.
 * Wake-up: любое прерывание.
 */
void enterSleepMode() {
    printf("[POWER] Entering SLEEP mode...\n");
    
    // В реальной реализации:
    // 1. Очистить флаг прерывания
    // 2. Установить режим (например, WFE или WFI)
    // 3. Вызвать __WFI() или __WFE()
    
    // Симуляция задержки
    auto start = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto end = std::chrono::steady_clock::now();
    
    powerStats.sleepTimeMs += std::chrono::duration_cast<std::chrono::milliseconds>(
        end - start).count();
    
    printf("[POWER] Woke up from SLEEP\n");
}

/**
 * @brief Переход в Stop mode
 * 
 * CPU и периферия остановлены, SRAM и регистры сохранены.
 * Wake-up: EXTI, PVD, RTC alarm.
 */
void enterStopMode() {
    printf("[POWER] Entering STOP mode...\n");
    
    // В реальной реализации:
    // 1. Остановить PLL
    // 2. Выбрать источник тактирования HSI
    // 3. Установить биты LPDS и PDDS в PWR_CR
    // 4. Установить бит SLEEPDEEP в Cortex SCR
    // 5. Вызвать __WFI()
    
    // Симуляция задержки
    auto start = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    auto end = std::chrono::steady_clock::now();
    
    powerStats.stopTimeMs += std::chrono::duration_cast<std::chrono::milliseconds>(
        end - start).count();
    
    printf("[POWER] Woke up from STOP\n");
}

/**
 * @brief Переход в Standby mode
 * 
 * Максимальное энергосбережение. SRAM и регистры потеряны.
 * Wake-up: WKUP pin, RTC alarm, NRST.
 */
void enterStandbyMode() {
    printf("[POWER] Entering STANDBY mode...\n");
    
    // В реальной реализации:
    // 1. Очистить флаг wake-up (WUF в PWR_CSR)
    // 2. Установить бит PDDS в PWR_CR
    // 3. Установить бит SLEEPDEEP в Cortex SCR
    // 4. Вызвать __WFI()
    
    // Симуляция задержки
    auto start = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    auto end = std::chrono::steady_clock::now();
    
    powerStats.standbyTimeMs += std::chrono::duration_cast<std::chrono::milliseconds>(
        end - start).count();
    
    printf("[POWER] Woke up from STANDBY\n");
}

/**
 * @brief Выход из режима ожидания
 */
void wakeUpFromLowPower() {
    wakeUpRequested = true;
    powerStats.wakeUpCount++;
}

// ============================================================================
// Обработчики прерываний
// ============================================================================

/**
 * @brief Обработчик нажатия кнопки (EXTI)
 */
void onButtonPressed() {
    printf("[IRQ] Button pressed - wake-up requested\n");
    wakeUpFromLowPower();
}

/**
 * @brief Обработчик RTC будильника
 */
void onRtcAlarm() {
    printf("[IRQ] RTC alarm triggered\n");
    rtcAlarmTriggered = true;
    wakeUpFromLowPower();
}

// ============================================================================
// Измерение параметров
// ============================================================================

struct SensorData {
    float voltage;
    float current;
    float temperature;
};

SensorData measureSensors() {
    SensorData data;
    
    // Симуляция измерений
    data.voltage = 3.3f + (std::rand() % 100) / 1000.0f;
    data.current = 10.0f + (std::rand() % 500) / 100.0f;  // mA
    data.temperature = 25.0f + (std::rand() % 100) / 10.0f;
    
    return data;
}

void printSensorData(const SensorData& data) {
    printf("[SENSORS] V=%.3fV I=%.2fmA T=%.1fC\n",
           data.voltage, data.current, data.temperature);
}

// ============================================================================
// Демонстрация сценария
// ============================================================================

void runDemoScenario() {
    printf("Running low power demo scenario...\n\n");

    // Фаза 1: Активный режим
    printf("=== Phase 1: RUN mode ===\n");
    for (int i = 0; i < 3; i++) {
        auto start = std::chrono::steady_clock::now();
        
        SensorData data = measureSensors();
        printSensorData(data);
        
        led->toggle();
        
        auto end = std::chrono::steady_clock::now();
        powerStats.activeTimeMs += std::chrono::duration_cast<std::chrono::milliseconds>(
            end - start).count();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(config::MEASURE_PERIOD_MS));
    }
    printf("\n");

    // Фаза 2: Sleep mode
    printf("=== Phase 2: SLEEP mode ===\n");
    enterSleepMode();
    printf("\n");

    // Фаза 3: Stop mode
    printf("=== Phase 3: STOP mode ===\n");
    enterStopMode();
    printf("\n");

    // Фаза 4: Standby mode
    printf("=== Phase 4: STANDBY mode ===\n");
    enterStandbyMode();
    printf("\n");

    // Фаза 5: Циклический режим
    printf("=== Phase 5: Cyclic low power ===\n");
    for (int cycle = 0; cycle < 3; cycle++) {
        printf("\n--- Cycle %d ---\n", cycle + 1);
        
        // Измерение
        SensorData data = measureSensors();
        printSensorData(data);
        powerStats.activeTimeMs += 10;
        
        // Sleep между измерениями
        enterSleepMode();
    }
    printf("\n");
}

// ============================================================================
// Печать статистики
// ============================================================================

void printPowerStats() {
    uint32_t totalTime = powerStats.activeTimeMs + powerStats.stopTimeMs + 
                         powerStats.sleepTimeMs + powerStats.standbyTimeMs;
    
    printf("=== Power Statistics ===\n");
    printf("Active time:   %lu ms (%.1f%%)\n", 
           powerStats.activeTimeMs,
           totalTime > 0 ? 100.0f * powerStats.activeTimeMs / totalTime : 0);
    printf("Sleep time:    %lu ms (%.1f%%)\n",
           powerStats.sleepTimeMs,
           totalTime > 0 ? 100.0f * powerStats.sleepTimeMs / totalTime : 0);
    printf("Stop time:     %lu ms (%.1f%%)\n",
           powerStats.stopTimeMs,
           totalTime > 0 ? 100.0f * powerStats.stopTimeMs / totalTime : 0);
    printf("Standby time:  %lu ms (%.1f%%)\n",
           powerStats.standbyTimeMs,
           totalTime > 0 ? 100.0f * powerStats.standbyTimeMs / totalTime : 0);
    printf("\n");
    printf("Wake-up count: %lu\n", powerStats.wakeUpCount);
    
    // Оценка среднего тока
    float avgCurrent = (
        powerStats.activeTimeMs * 15.0f +     // ~15 mA в активном режиме
        powerStats.sleepTimeMs * 5.0f +       // ~5 mA в Sleep
        powerStats.stopTimeMs * 0.5f +        // ~0.5 mA в Stop
        powerStats.standbyTimeMs * 0.005f     // ~5 uA в Standby
    ) / (totalTime > 0 ? totalTime : 1);
    
    printf("Estimated avg current: %.2f mA\n", avgCurrent);
    printf("========================\n");
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
    printPowerStats();

    // Советы по оптимизации
    printf("\n=== Power Optimization Tips ===\n");
    printf("1. Отключайте неиспользуемую периферию (RCC)\n");
    printf("2. Используйте DMA для передачи данных\n");
    printf("3. Снижайте частоту CPU когда возможно\n");
    printf("4. Отключайте Brown-out Detector если допустимо\n");
    printf("5. Используйте RTC wake-up вместо polling\n");
    printf("6. Минимизируйте время в активном режиме\n");
    printf("===============================\n");

    // Очистка
    delete led;
    delete button;

    printf("\nExample completed.\n");
    return 0;
}

// ============================================================================
// Конфигурация низкого энергопотребления для STM32
// ============================================================================

#ifdef STM32F4

// Пример настройки RCC для низкого энергопотребления
void configureLowPowerRCC() {
    // В реальной реализации:
    // 1. Переключиться на HSI (внутренний RC генератор)
    // 2. Отключить PLL
    // 3. Снизить частоту AHB/APB
    // 4. Отключить тактирование неиспользуемой периферии
}

// Пример настройки GPIO для аналоговых входов
void configureAnalogGPIO() {
    // В реальной реализации:
    // 1. Установить все неиспользуемые GPIO в Analog mode
    // 2. Это минимизирует ток утечки
}

#endif

// ============================================================================
// Версия для FreeRTOS с tickless idle
// ============================================================================

#ifdef USE_FREERTOS

#include "rtos/freertos_wrapper.hpp"

using namespace mka::rtos;

// Задача измерения
void vMeasureTask(void* pvParameters) {
    uint32_t lastWakeTime = Task<1024>::getTickCount();
    
    for (;;) {
        SensorData data = measureSensors();
        printSensorData(data);
        
        // Tickless idle mode позволит FreeRTOS остановить tick timer
        Task<1024>::delayUntil(lastWakeTime, config::MEASURE_PERIOD_MS);
    }
}

void vStartLowPowerDemo(void) {
    static Task<2048> measureTask("Measure", 3, +[]() { vMeasureTask(nullptr); });
    measureTask.start();
    
    // Включение tickless idle mode
    // vPortEnableTicklessIdle();
}

#endif // USE_FREERTOS
