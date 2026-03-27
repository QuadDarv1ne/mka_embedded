/**
 * @file main.cpp
 * @brief Пример использования UART — эхо-сервер
 * 
 * Демонстрирует приём и передачу данных через UART.
 * Все полученные символы отправляются обратно (echo).
 */

#include <cstdint>
#include <cstdio>
#include <string>

// HAL интерфейс
#include "hal/hal_full.hpp"

// ============================================================================
// Конфигурация
// ============================================================================

namespace config {
    constexpr uint32_t UART_BAUDRATE = 115200;  // Скорость UART
    constexpr size_t BUFFER_SIZE = 64;          // Размер буфера
}

// ============================================================================
// Основная программа
// ============================================================================

int main() {
    // Инициализация UART
    hal::UART uart(1, config::UART_BAUDRATE);
    
    printf("UART Echo example started. Baudrate: %lu\n", config::UART_BAUDRATE);
    printf("Type something and press Enter:\n");
    
    std::string buffer;
    char ch;
    
    // Основной цикл
    while (true) {
        // Чтение одного символа (blocking)
        if (uart.read(&ch, 1) > 0) {
            // Эхо обратно
            uart.write(&ch, 1);
            
            // Обработка Enter
            if (ch == '\n' || ch == '\r') {
                uart.write("\r\n", 2);
                
                if (!buffer.empty()) {
                    // Обработка команды
                    printf("Received: %s\n", buffer.c_str());
                    buffer.clear();
                }
            } else if (ch == '\b' || ch == 127) {
                // Backspace
                if (!buffer.empty()) {
                    buffer.pop_back();
                }
            } else {
                // Добавление символа в буфер
                if (buffer.size() < config::BUFFER_SIZE) {
                    buffer += ch;
                }
            }
        }
    }
    
    return 0;
}

// ============================================================================
// Для FreeRTOS — версия с прерываниями
// ============================================================================

#ifdef USE_FREERTOS

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static QueueHandle_t uartQueue;
static hal::UART* uartPtr;

// Прерывание UART (вызывается из ISR)
void UART_IRQHandler(void) {
    char ch;
    if (uartPtr->read(&ch, 1) > 0) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(uartQueue, &ch, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Задача обработки UART
void vUARTTask(void* pvParameters) {
    char ch;
    
    for (;;) {
        if (xQueueReceive(uartQueue, &ch, portMAX_DELAY) == pdPASS) {
            // Эхо
            uartPtr->write(&ch, 1);
        }
    }
}

void vStartUART(void) {
    uartPtr = new hal::UART(1, config::UART_BAUDRATE);
    uartQueue = xQueueCreate(config::BUFFER_SIZE, sizeof(char));
    
    xTaskCreate(vUARTTask, "UART", 256, nullptr, 2, nullptr);
    
    // Включение прерываний UART
    // NVIC_EnableIRQ(UART1_IRQn);
}

#endif // USE_FREERTOS
