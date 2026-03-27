/**
 * @file main.cpp
 * @brief Пример сканирования I2C шины
 *
 * Демонстрирует поиск подключенных устройств на I2C шине.
 * Выводит адреса найденных устройств в шестнадцатеричном формате.
 */

#include <cstdint>
#include <cstdio>
#include <vector>
#include <chrono>
#include <thread>

// HAL интерфейс
#include "hal/hal_full.hpp"

// ============================================================================
// Конфигурация
// ============================================================================

namespace config {
    constexpr uint32_t I2C_SPEED = 100000;  // Скорость I2C (100 kHz)
    constexpr uint8_t I2C_ADDR_MIN = 0x08;  // Минимальный адрес
    constexpr uint8_t I2C_ADDR_MAX = 0x77;  // Максимальный адрес
}

// ============================================================================
// Сканер I2C
// ============================================================================

class I2CScanner {
public:
    /**
     * @brief Сканирование I2C шины
     * @param i2c Объект I2C
     * @return Вектор найденных адресов
     */
    static std::vector<uint8_t> scan(mka::hal::I2C& i2c) {
        std::vector<uint8_t> foundDevices;
        
        printf("Scanning I2C bus...\n");
        printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
        
        for (uint8_t row = 0; row < 8; ++row) {
            printf("%02X: ", row * 16);
            
            for (uint8_t col = 0; col < 16; ++col) {
                uint8_t address = row * 16 + col;
                
                // Пропуск зарезервированных адресов
                if (address < config::I2C_ADDR_MIN || address > config::I2C_ADDR_MAX) {
                    printf("   ");
                    continue;
                }
                
                // Попытка записи
                uint8_t dummy = 0;
                if (i2c.write(address, &dummy, 1) == 0) {
                    printf("%02X ", address);
                    foundDevices.push_back(address);
                } else {
                    printf("-- ");
                }
            }
            
            printf("\n");
        }
        
        return foundDevices;
    }
    
    /**
     * @brief Вывод информации о найденных устройствах
     */
    static void printDevices(const std::vector<uint8_t>& devices) {
        printf("\nFound %zu device(s):\n", devices.size());
        
        for (uint8_t addr : devices) {
            printf("  - 0x%02X", addr);
            
            // Известные устройства
            switch (addr) {
                case 0x19:
                case 0x69:
                    printf(" (BMI160 IMU)");
                    break;
                case 0x1E:
                    printf(" (LIS3MDL Magnetometer)");
                    break;
                case 0x56:
                case 0x76:
                case 0x77:
                    printf(" (BMP388 Barometer)");
                    break;
                case 0x68:
                    printf(" (MPU6050/9250 IMU)");
                    break;
                case 0x0D:
                case 0x1A:
                case 0x1B:
                    printf(" (LIS3DSH Accelerometer)");
                    break;
                default:
                    printf(" (Unknown)");
                    break;
            }
            
            printf("\n");
        }
    }
};

// ============================================================================
// Основная программа
// ============================================================================

int main() {
    // Инициализация I2C
    mka::hal::I2C i2c(1, config::I2C_SPEED);

    printf("I2C Scanner example started\n");
    printf("I2C speed: %lu Hz\n", config::I2C_SPEED);
    printf("\n");

    // Сканирование
    auto devices = I2CScanner::scan(i2c);

    // Вывод результатов
    I2CScanner::printDevices(devices);

    printf("\nPress Reset to scan again\n");

    // Бесконечный цикл
    while (true) {
        // Ожидание (для STM32)
        // Для хост-системы можно использовать sleep
        #ifdef HOST_BUILD
            std::this_thread::sleep_for(std::chrono::seconds(1));
        #else
            __WFI();
        #endif
    }

    return 0;
}
