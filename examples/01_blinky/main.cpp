/**
 * @file main.cpp
 * @brief Пример использования GPIO — мигание светодиодом
 * 
 * Демонстрирует базовое использование HAL интерфейса IGPIO
 * для управления цифровым выходом (светодиод).
 */

#include <cstdint>
#include <thread>
#include <chrono>

// HAL интерфейс
#include "hal/hal_full.hpp"

// ============================================================================
// Конфигурация
// ============================================================================

namespace config {
    constexpr uint32_t LED_PIN = 13;           // PIN светодиода
    constexpr uint32_t BLINK_PERIOD_MS = 500;  // Период мигания (мс)
}

// ============================================================================
// Основная программа
// ============================================================================

int main() {
    // Инициализация HAL (платформенно-зависимая)
    // Для STM32: HAL_Init(), SystemClock_Config()
    
    // Создание объекта GPIO
    hal::GPIO led(config::LED_PIN, hal::GPIOMode::OUTPUT);
    
    printf("Blinky example started. LED on pin %lu\n", config::LED_PIN);
    
    // Основной цикл мигания
    while (true) {
        // Включить светодиод
        led.write(hal::GPIOPinState::HIGH);
        std::this_thread::sleep_for(std::chrono::milliseconds(config::BLINK_PERIOD_MS));
        
        // Выключить светодиод
        led.write(hal::GPIOPinState::LOW);
        std::this_thread::sleep_for(std::chrono::milliseconds(config::BLINK_PERIOD_MS));
    }
    
    return 0;
}

// ============================================================================
// Для FreeRTOS
// ============================================================================

#ifdef USE_FREERTOS

#include "FreeRTOS.h"
#include "task.h"

// Задача мигания
void vBlinkyTask(void* pvParameters) {
    hal::GPIO led(config::LED_PIN, hal::GPIOMode::OUTPUT);
    
    for (;;) {
        led.toggle();
        vTaskDelay(pdMS_TO_TICKS(config::BLINK_PERIOD_MS));
    }
}

// Создание задачи
void vStartBlinky(void) {
    xTaskCreate(
        vBlinkyTask,           // Функция задачи
        "Blinky",              // Имя задачи
        128,                   // Размер стека
        nullptr,               // Параметры
        1,                     // Приоритет
        nullptr                // Дескриптор задачи
    );
}

#endif // USE_FREERTOS
