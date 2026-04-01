/**
 * @file main.cpp
 * @brief Пример логирования GPS данных с u-blox NEO-M8
 *
 * Демонстрирует чтение NMEA/UBX данных с GPS приёмника,
 * парсинг координат, скорости и времени.
 */

#include <cstdint>
#include <cstdio>
#include <thread>
#include <chrono>
#include <cstring>

// HAL интерфейсы
#include "hal/hal_full.hpp"
// Драйверы
#include "drivers/sensors_drivers.hpp"

// ============================================================================
// Конфигурация
// ============================================================================

namespace config {
    // UART для GPS
    constexpr uint32_t GPS_UART_PORT = 2;
    constexpr uint32_t GPS_BAUDRATE = 9600;  // u-blox NEO-M8 default

    // Период опроса (мс)
    constexpr uint32_t READ_PERIOD_MS = 1000;  // 1 Гц

    // Формат вывода
    constexpr bool PRINT_NMEA = false;       // Показывать сырые NMEA
    constexpr bool PRINT_UBX = true;         // Показывать UBX данные
}

// ============================================================================
// Глобальные объекты
// ============================================================================

hal::UART* gpsUart = nullptr;
mka::drivers::GPSDriver* gps = nullptr;

// Флаги состояния
bool gpsConnected = false;
bool hasFix = false;

// ============================================================================
// Инициализация
// ============================================================================

bool init() {
    printf("=== GPS Logger Example ===\n\n");

    // Инициализация UART
    printf("[INIT] Initializing UART%lu for GPS...\n", GPS_UART_PORT);
    gpsUart = new hal::UART(GPS_UART_PORT);

    hal::UARTConfig uartConfig;
    uartConfig.baudrate = config::GPS_BAUDRATE;
    uartConfig.wordLength = hal::UARTWordLength::BITS_8;
    uartConfig.stopBits = hal::UARTStopBits::STOP_1;
    uartConfig.parity = hal::UARTParity::NONE;
    uartConfig.txEnable = true;
    uartConfig.rxEnable = true;

    if (gpsUart->init(uartConfig) != hal::Status::OK) {
        printf("[ERROR] Failed to initialize UART\n");
        return false;
    }
    printf("[OK] UART initialized at %lu baud\n", config::GPS_BAUDRATE);

    // Инициализация GPS драйвера
    printf("[INIT] Initializing u-blox GPS driver...\n");
    gps = new mka::drivers::GPSDriver(*gpsUart);

    // Попытка инициализации (не всегда успешна без конфигурации)
    hal::Status status = gps->init();
    if (status == hal::Status::OK) {
        printf("[OK] GPS driver initialized\n");
        gpsConnected = true;
    } else {
        printf("[WARN] GPS init returned %d, continuing anyway...\n", (int)status);
        gpsConnected = true;  // Пробуем работать
    }

    printf("\n");
    return true;
}

// ============================================================================
// Форматирование координат
// ============================================================================

void printCoordinates(const mka::drivers::GPSData& data) {
    // Широта
    char latDir = data.latitude >= 0 ? 'N' : 'S';
    float latAbs = std::abs(data.latitude);
    int latDeg = (int)latAbs;
    float latMin = (latAbs - latDeg) * 60.0f;

    // Долгота
    char lonDir = data.longitude >= 0 ? 'E' : 'W';
    float lonAbs = std::abs(data.longitude);
    int lonDeg = (int)lonAbs;
    float lonMin = (lonAbs - lonDeg) * 60.0f;

    printf("Position: %02d°%06.3f'%c  %03d°%06.3f'%c\n",
           latDeg, latMin, latDir, lonDeg, lonMin, lonDir);
}

// ============================================================================
// Чтение и вывод данных
// ============================================================================

void readAndLog() {
    mka::drivers::GPSData data;

    // Чтение данных из UART
    hal::Status status = gps->readData(data);

    if (status != hal::Status::OK) {
        printf("[WARN] No GPS data available (status=%d)\n", (int)status);
        return;
    }

    // Проверка качества фиксации
    hasFix = data.fixType >= mka::drivers::GPSFixType::FIX_2D;

    printf("=== GPS Data ===\n");

    // Время
    if (data.hour != 0xFF) {
        printf("Time:    %02d:%02d:%02d UTC\n", data.hour, data.minute, data.second);
    } else {
        printf("Time:    --:--:-- (no time)\n");
    }

    // Дата
    if (data.day != 0xFF) {
        printf("Date:    %02d.%02d.%04d\n", data.day, data.month, data.year);
    } else {
        printf("Date:    --.--.---- (no date)\n");
    }

    // Статус фиксации
    const char* fixStr = "None";
    switch (data.fixType) {
        case mka::drivers::GPSFixType::NO_FIX: fixStr = "No Fix"; break;
        case mka::drivers::GPSFixType::FIX_2D: fixStr = "2D Fix"; break;
        case mka::drivers::GPSFixType::FIX_3D: fixStr = "3D Fix"; break;
    }
    printf("Fix:     %s (%d satellites)\n", fixStr, (int)data.numSatellites);

    // Координаты
    if (hasFix) {
        printCoordinates(data);
    } else {
        printf("Position: Waiting for fix...\n");
    }

    // Высота
    if (data.altitude != 0x7FFFFFFF) {
        printf("Altitude: %.1f m [MSL]\n", data.altitude / 1000.0f);
    }

    // Скорость
    if (data.speed != 0xFFFFFFFF) {
        float speedKmh = data.speed / 1000.0f * 3.6f;
        printf("Speed:   %.2f km/h (%.1f m/s)\n", speedKmh, data.speed / 1000.0f);
    }

    // Курс
    if (data.heading != 0xFFFF) {
        printf("Heading: %.1f°\n", data.heading / 100000.0f);
    }

    // HDOP (точность)
    if (data.hDOP != 0xFFFF) {
        printf("HDOP:    %.2f\n", data.hDOP / 100.0f);
    }

    printf("\n");
}

// ============================================================================
// Основная программа
// ============================================================================

int main() {
    if (!init()) {
        printf("[FATAL] Initialization failed. Exiting.\n");
        return 1;
    }

    printf("Starting GPS logging...\n");
    printf("Press Ctrl+C to stop\n\n");

    uint32_t fixCount = 0;
    uint32_t totalReads = 0;

    // Основной цикл чтения
    while (true) {
        readAndLog();

        totalReads++;
        if (hasFix) fixCount++;

        // Статистика
        if (totalReads % 10 == 0) {
            float fixRate = (float)fixCount / totalReads * 100.0f;
            printf("--- Fix rate: %.1f%% (%lu/%lu) ---\n\n", fixRate, fixCount, totalReads);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config::READ_PERIOD_MS));
    }

    // Очистка
    delete gps;
    delete gpsUart;

    return 0;
}

// ============================================================================
// Версия для FreeRTOS
// ============================================================================

#ifdef USE_FREERTOS

#include "rtos/freertos_wrapper.hpp"

using namespace mka::rtos;

// Очередь для GPS данных
static Queue<mka::drivers::GPSData, 8> gpsQueue;

// Задача чтения GPS
void vGPSReaderTask(void* pvParameters) {
    mka::drivers::GPSData data;
    uint32_t lastWakeTime = Task<1024>::getTickCount();

    for (;;) {
        if (gps->readData(data) == hal::Status::OK) {
            gpsQueue.send(data, NO_TIMEOUT);
        }

        Task<1024>::delayUntil(lastWakeTime, config::READ_PERIOD_MS);
    }
}

// Задача обработки GPS
void vGPSProcessorTask(void* pvParameters) {
    for (;;) {
        auto data = gpsQueue.receive(INFINITE_TIMEOUT);
        if (data.has_value()) {
            printf("GPS: Lat=%.6f Lon=%.6f Alt=%.1fm\n",
                   data->latitude, data->longitude, data->altitude / 1000.0f);
        }
    }
}

// Создание задач GPS
void vStartGPSLogger(void) {
    static Task<2048> readerTask("GPSReader", 3, +[]() {
        vGPSReaderTask(nullptr);
    });

    static Task<2048> processorTask("GPSProcessor", 3, +[]() {
        vGPSProcessorTask(nullptr);
    });

    readerTask.start();
    processorTask.start();
}

#endif // USE_FREERTOS
