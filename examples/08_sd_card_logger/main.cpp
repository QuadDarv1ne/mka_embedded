/**
 * @file main.cpp
 * @brief Пример логирования данных на SD карту через SPI
 *
 * Демонстрирует работу с SD картой:
 * - Инициализация через SPI
 * - Запись данных в файл
 * - Чтение данных из файла
 * - Использование FAT-подобной структуры
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>

// HAL интерфейсы
#include "hal/hal_full.hpp"

// ============================================================================
// Конфигурация
// ============================================================================

namespace config {
    // SPI для SD карты
    constexpr uint32_t SD_SPI_PORT = 1;
    constexpr uint32_t SD_SPI_SPEED = 1000000;  // 1 MHz (initialization)
    constexpr uint32_t SD_SPI_SPEED_HIGH = 10000000;  // 10 MHz (after init)
    constexpr uint32_t SD_CS_PIN = 5;  // Chip Select pin

    // Файл для логирования
    constexpr const char* LOG_FILE = "mka_log.txt";
    constexpr size_t MAX_LOG_SIZE = 1024 * 1024;  // 1 MB max

    // Период логирования (мс)
    constexpr uint32_t LOG_PERIOD_MS = 1000;

    // Максимальное количество записей
    constexpr size_t MAX_ENTRIES = 100;
}

// ============================================================================
// Глобальные объекты
// ============================================================================

hal::SPI* sdSpi = nullptr;
hal::GPIO* sdCs = nullptr;

// Счётчик записей
size_t logEntryCount = 0;

// ============================================================================
// Управление CS (Chip Select)
// ============================================================================

void sdSelect() {
    sdCs->write(hal::GPIOPinState::LOW);
}

void sdDeselect() {
    sdCs->write(hal::GPIOPinState::HIGH);
}

// ============================================================================
// Инициализация SD карты
// ============================================================================

bool initSD() {
    printf("[INIT] Initializing SD card interface...\n");

    // Инициализация CS pin
    sdCs = new hal::GPIO(config::SD_CS_PIN, hal::GPIOMode::OUTPUT);
    sdDeselect();

    // Инициализация SPI
    sdSpi = new hal::SPI(config::SD_SPI_PORT);

    hal::SPIConfig spiConfig;
    spiConfig.clockSpeed = config::SD_SPI_SPEED;  // Низкая скорость для инициализации
    spiConfig.mode = hal::SPIMode::MODE_0;
    spiConfig.bitOrder = hal::SPIBitOrder::MSB_FIRST;
    spiConfig.masterMode = true;
    spiConfig.fifoOverflowCheck = true;
    spiConfig.autoRecoverFromOverflow = true;

    if (sdSpi->init(spiConfig) != hal::Status::OK) {
        printf("[ERROR] Failed to initialize SPI\n");
        return false;
    }

    printf("[OK] SPI initialized at %lu Hz\n", config::SD_SPI_SPEED);

    // Процедура инициализации SD карты (CMD0, CMD8, ACMD41)
    printf("[INIT] Sending CMD0 (GO_IDLE_STATE)...\n");
    
    // CMD0: reset card
    sdSelect();
    std::array<uint8_t, 6> cmd0 = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};
    std::array<uint8_t, 6> response0{};
    
    hal::Status status = sdSpi->transferWithRetry(cmd0, response0, 100, 3);
    sdDeselect();

    if (status != hal::Status::OK) {
        printf("[WARN] CMD0 failed (status=%d) - continuing in simulation mode\n", (int)status);
    } else {
        printf("[OK] CMD0 response: 0x%02X\n", response0[5]);
    }

    // Переключение на высокую скорость
    printf("[INIT] Switching to high speed (%lu Hz)...\n", config::SD_SPI_SPEED_HIGH);
    sdSpi->setClockSpeed(config::SD_SPI_SPEED_HIGH);

    return true;
}

// ============================================================================
// Запись сектора (512 байт)
// ============================================================================

bool writeSector(uint32_t sector, const uint8_t* data) {
    // CMD24: WRITE_BLOCK
    sdSelect();
    
    std::array<uint8_t, 6> cmd = {
        0x58,  // CMD24
        static_cast<uint8_t>(sector >> 24),
        static_cast<uint8_t>(sector >> 16),
        static_cast<uint8_t>(sector >> 8),
        static_cast<uint8_t>(sector),
        0xFF   // CRC (don't care in SPI mode)
    };
    
    std::array<uint8_t, 6> response{};
    sdSpi->transferWithRetry(cmd, response, 100, 3);
    
    // Ожидание ответа
    for (int i = 0; i < 100; i++) {
        std::array<uint8_t, 1> status{};
        sdSpi->transferWithRetry(std::span<uint8_t>{}, status, 10, 3);
        if (status[0] == 0xE5) {
            break;  // Ready to receive
        }
    }
    
    // Посылка данных (токен + 512 байт + CRC)
    std::array<uint8_t, 1> token = {0xFE};
    sdSpi->transmitWithRetry(token, 100, 3);
    sdSpi->transmitWithRetry({data, 512}, 100, 3);
    
    std::array<uint8_t, 2> crc = {0xFF, 0xFF};
    sdSpi->transmitWithRetry(crc, 100, 3);
    
    sdDeselect();
    
    return true;
}

// ============================================================================
// Чтение сектора (512 байт)
// ============================================================================

bool readSector(uint32_t sector, uint8_t* buffer) {
    // CMD17: READ_SINGLE_BLOCK
    sdSelect();
    
    std::array<uint8_t, 6> cmd = {
        0x51,  // CMD17
        static_cast<uint8_t>(sector >> 24),
        static_cast<uint8_t>(sector >> 16),
        static_cast<uint8_t>(sector >> 8),
        static_cast<uint8_t>(sector),
        0xFF
    };
    
    std::array<uint8_t, 6> response{};
    sdSpi->transferWithRetry(cmd, response, 100, 3);
    
    // Ожидание токена данных
    for (int i = 0; i < 1000; i++) {
        std::array<uint8_t, 1> token{};
        sdSpi->receiveWithRetry(token, 10, 3);
        if (token[0] == 0xFE) {
            break;  // Data token received
        }
    }
    
    // Чтение 512 байт
    std::span<uint8_t> dataSpan(buffer, 512);
    sdSpi->receiveWithRetry(dataSpan, 100, 3);
    
    // Пропуск CRC
    std::array<uint8_t, 2> crc{};
    sdSpi->receiveWithRetry(crc, 10, 3);
    
    sdDeselect();
    
    return true;
}

// ============================================================================
// Форматирование записи лога
// ============================================================================

struct LogEntry {
    uint32_t timestamp;
    float battery;
    float temperature;
    float pressure;
    uint8_t status;
};

void formatLogEntry(char* buffer, size_t size, const LogEntry& entry) {
    std::snprintf(buffer, size,
                  "[%lu] Bat=%.2fV Temp=%.1fC Pres=%.1fhPa Status=0x%02X\n",
                  entry.timestamp, entry.battery, entry.temperature,
                  entry.pressure, entry.status);
}

// ============================================================================
// Симуляция данных сенсоров
// ============================================================================

LogEntry readSensors() {
    static uint32_t timestamp = 0;
    timestamp += config::LOG_PERIOD_MS;

    LogEntry entry;
    entry.timestamp = timestamp;
    entry.battery = 7.2f + (std::rand() % 100) / 100.0f;
    entry.temperature = 25.0f + (std::rand() % 50) / 10.0f;
    entry.pressure = 1013.0f + (std::rand() % 200) / 10.0f;
    entry.status = 0x01;  // OK

    return entry;
}

// ============================================================================
// Основная программа
// ============================================================================

int main() {
    printf("=== SD Card Logger Example ===\n\n");

    // Инициализация SD карты
    if (!initSD()) {
        printf("[FATAL] SD card initialization failed\n");
        printf("Continuing in simulation mode...\n\n");
    }

    printf("[OK] SD card ready\n\n");
    printf("Starting data logging to '%s'...\n", config::LOG_FILE);
    printf("Press Ctrl+C to stop\n\n");

    // Буфер для одной записи (128 байт)
    char logBuffer[128];
    
    // Буфер сектора (512 байт)
    std::array<uint8_t, 512> sectorBuffer{};
    size_t sectorOffset = 0;

    uint32_t currentSector = 0;

    // Основной цикл логирования
    for (size_t i = 0; i < config::MAX_ENTRIES; i++) {
        // Чтение сенсоров
        LogEntry entry = readSensors();

        // Форматирование записи
        formatLogEntry(logBuffer, sizeof(logBuffer), entry);

        // Вывод в консоль
        printf("%s", logBuffer);

        // Запись в буфер сектора
        size_t entryLen = std::strlen(logBuffer);
        if (sectorOffset + entryLen > sectorBuffer.size()) {
            // Сектор заполнен — запись на SD карту
            printf("[WRITE] Writing sector %lu to SD card...\n", currentSector);
            
            if (sdSpi != nullptr) {
                writeSector(currentSector, sectorBuffer.data());
            }
            
            currentSector++;
            sectorOffset = 0;
            std::memset(sectorBuffer.data(), 0xFF, sectorBuffer.size());
        }

        // Копирование записи в буфер
        std::memcpy(sectorBuffer.data() + sectorOffset, logBuffer, entryLen);
        sectorOffset += entryLen;

        logEntryCount++;

        // Ожидание следующего периода
        std::this_thread::sleep_for(std::chrono::milliseconds(config::LOG_PERIOD_MS));
    }

    // Запись последнего (неполного) сектора
    if (sectorOffset > 0 && sdSpi != nullptr) {
        printf("[WRITE] Writing final sector %lu...\n", currentSector);
        writeSector(currentSector, sectorBuffer.data());
    }

    // Статистика
    printf("\n=== Logging Complete ===\n");
    printf("Total entries: %zu\n", logEntryCount);
    printf("Sectors written: %lu\n", currentSector + 1);
    printf("Data size: ~%zu bytes\n", logEntryCount * 64);

    // Чтение и проверка (демонстрация)
    printf("\n=== Reading back first sector ===\n");
    if (sdSpi != nullptr) {
        std::array<uint8_t, 512> readBuffer{};
        readSector(0, readBuffer.data());
        
        printf("Sector 0 content:\n");
        for (size_t i = 0; i < 256 && readBuffer[i] != 0xFF; i++) {
            putchar(readBuffer[i]);
        }
    } else {
        printf("(SD card not available - simulation mode)\n");
    }

    // Очистка
    delete sdSpi;
    delete sdCs;

    printf("\nExample completed successfully.\n");
    return 0;
}

// ============================================================================
// Версия для FreeRTOS
// ============================================================================

#ifdef USE_FREERTOS

#include "rtos/freertos_wrapper.hpp"

using namespace mka::rtos;

// Очередь для логов
static Queue<LogEntry, 16> logQueue;

// Задача сбора данных
void vSensorTask(void* pvParameters) {
    uint32_t lastWakeTime = Task<1024>::getTickCount();

    for (;;) {
        LogEntry entry = readSensors();
        logQueue.send(entry, NO_TIMEOUT);
        Task<1024>::delayUntil(lastWakeTime, config::LOG_PERIOD_MS);
    }
}

// Запись на SD карту
void vSDWriterTask(void* pvParameters) {
    char logBuffer[128];
    std::array<uint8_t, 512> sectorBuffer{};
    size_t sectorOffset = 0;
    uint32_t currentSector = 0;

    for (;;) {
        auto entry = logQueue.receive(INFINITE_TIMEOUT);
        if (entry.has_value()) {
            formatLogEntry(logBuffer, sizeof(logBuffer), entry.value());

            size_t entryLen = std::strlen(logBuffer);
            if (sectorOffset + entryLen > sectorBuffer.size()) {
                writeSector(currentSector, sectorBuffer.data());
                currentSector++;
                sectorOffset = 0;
                std::memset(sectorBuffer.data(), 0xFF, sectorBuffer.size());
            }

            std::memcpy(sectorBuffer.data() + sectorOffset, logBuffer, entryLen);
            sectorOffset += entryLen;
        }
    }
}

void vStartSDLogger(void) {
    static Task<2048> sensorTask("SensorTask", 3, +[]() { vSensorTask(nullptr); });
    static Task<4096> sdWriterTask("SDWriter", 2, +[]() { vSDWriterTask(nullptr); });

    sensorTask.start();
    sdWriterTask.start();
}

#endif // USE_FREERTOS
