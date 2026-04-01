/**
 * @file main.cpp
 * @brief Пример CAN коммуникации
 *
 * Демонстрирует:
 * - Инициализацию CAN шины
 * - Передачу стандартных и расширенных кадров
 * - Приём кадров с фильтрацией
 * - Обработку ошибок
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>
#include <array>

// HAL интерфейсы
#include "hal/hal_full.hpp"

// ============================================================================
// Конфигурация
// ============================================================================

namespace config {
    // CAN параметры
    constexpr uint32_t CAN_PORT = 1;
    constexpr uint32_t CAN_BAUDRATE = 500000;  // 500 kbps

    // CAN ID для тестовых сообщений
    constexpr uint32_t CAN_ID_TELEMETRY = 0x100;
    constexpr uint32_t CAN_ID_COMMAND = 0x200;
    constexpr uint32_t CAN_ID_HEARTBEAT = 0x700;

    // Периоды отправки (мс)
    constexpr uint32_t TELEMETRY_PERIOD_MS = 100;
    constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;
}

// ============================================================================
// Глобальные объекты
// ============================================================================

hal::CAN* can = nullptr;

// Статистика
struct CANStats {
    uint32_t txFrames = 0;
    uint32_t rxFrames = 0;
    uint32_t errors = 0;
    uint32_t busOffCount = 0;
};

CANStats canStats;

// Флаги
volatile bool busOff = false;

// ============================================================================
// CAN Frame структура
// ============================================================================

struct CANFrame {
    uint32_t id;
    uint8_t dlc;  // Data Length Code (0-8)
    bool isExtended;
    bool isRTR;
    std::array<uint8_t, 8> data;
};

// ============================================================================
// Инициализация
// ============================================================================

bool initCAN() {
    printf("=== CAN Communication Example ===\n\n");
    printf("[INIT] Initializing CAN%lu at %lu bps...\n", 
           config::CAN_PORT, config::CAN_BAUDRATE);

    can = new hal::CAN(config::CAN_PORT);

    hal::CANConfig canConfig;
    canConfig.baudrate = config::CAN_BAUDRATE;
    canConfig.mode = hal::CANMode::NORMAL;
    canConfig.autoRetransmit = true;
    canConfig.autoBusOffManagement = true;

    hal::Status status = can->init(canConfig);
    
    if (status != hal::Status::OK) {
        printf("[WARN] CAN init returned %d\n", (int)status);
        printf("Continuing in simulation mode...\n\n");
    } else {
        printf("[OK] CAN initialized\n\n");
    }

    // Настройка фильтров
    printf("[INIT] Configuring CAN filters...\n");
    
    // Фильтр 0: принимаем телеметрию (0x100-0x10F)
    can->configureFilter(0, hal::CANFilterType::MASK16, 
                         CAN_ID_TELEMETRY << 5, 0xFFF0);
    
    // Фильтр 1: принимаем команды (0x200-0x20F)
    can->configureFilter(1, hal::CANFilterType::MASK16,
                         CAN_ID_COMMAND << 5, 0xFFF0);
    
    // Фильтр 2: принимаем heartbeat (0x700)
    can->configureFilter(2, hal::CANFilterType::MASK16,
                         CAN_ID_HEARTBEAT << 5, 0xFFF8);

    printf("[OK] Filters configured\n\n");

    return true;
}

// ============================================================================
// Передача кадров
// ============================================================================

bool sendFrame(const CANFrame& frame) {
    if (busOff) {
        printf("[ERROR] Bus OFF - cannot transmit\n");
        canStats.errors++;
        return false;
    }

    hal::Status status = can->transmit(frame.id, frame.data.data(), 
                                        frame.dlc, frame.isExtended);
    
    if (status == hal::Status::OK) {
        canStats.txFrames++;
        return true;
    } else if (status == hal::Status::BUS) {
        printf("[WARN] Bus OFF detected\n");
        busOff = true;
        canStats.busOffCount++;
    } else {
        printf("[ERROR] Transmit failed: %d\n", (int)status);
        canStats.errors++;
    }
    
    return false;
}

// ============================================================================
// Приём кадров
// ============================================================================

bool receiveFrame(CANFrame& frame) {
    uint32_t id;
    std::array<uint8_t, 8> data;
    uint8_t dlc;
    bool extended;

    hal::Status status = can->receive(id, data.data(), dlc, extended, 10);
    
    if (status == hal::Status::OK) {
        frame.id = id;
        frame.dlc = dlc;
        frame.isExtended = extended;
        frame.data = data;
        canStats.rxFrames++;
        return true;
    }
    
    return false;
}

// ============================================================================
// Формирование телеметрии
// ============================================================================

struct TelemetryData {
    uint16_t voltage;      // mV * 10
    uint16_t current;      // mA * 10
    int16_t temperature;   // C * 10
    uint8_t status;
    uint8_t reserved;
};

CANFrame createTelemetryFrame(float voltage, float current, float temperature, 
                               uint8_t status) {
    CANFrame frame;
    frame.id = CAN_ID_TELEMETRY;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRTR = false;

    TelemetryData* telemetry = reinterpret_cast<TelemetryData*>(frame.data.data());
    telemetry->voltage = static_cast<uint16_t>(voltage * 10);
    telemetry->current = static_cast<uint16_t>(current * 10);
    telemetry->temperature = static_cast<int16_t>(temperature * 10);
    telemetry->status = status;
    telemetry->reserved = 0;

    return frame;
}

CANFrame createHeartbeatFrame(uint8_t nodeState) {
    CANFrame frame;
    frame.id = CAN_ID_HEARTBEAT;
    frame.dlc = 1;
    frame.isExtended = false;
    frame.isRTR = false;
    frame.data[0] = nodeState;

    return frame;
}

// ============================================================================
// Обработка принятых кадров
// ============================================================================

void processReceivedFrame(const CANFrame& frame) {
    printf("[RX] ID=0x%03X DLC=%u Data:", frame.id, frame.dlc);
    
    for (int i = 0; i < frame.dlc; i++) {
        printf(" %02X", frame.data[i]);
    }
    printf("\n");

    // Обработка по типам
    if (frame.id == CAN_ID_COMMAND) {
        printf("  -> Command received\n");
        // Обработка команды
    } else if (frame.id == CAN_ID_HEARTBEAT) {
        printf("  -> Heartbeat from node %u\n", frame.data[0]);
    } else if (frame.id == CAN_ID_TELEMETRY) {
        printf("  -> Telemetry from another node\n");
    }
}

// ============================================================================
// Симуляция данных
// ============================================================================

float simulateVoltage() {
    static float v = 12.0f;
    v += (std::rand() % 20 - 10) / 100.0f;
    return v;
}

float simulateCurrent() {
    return 100.0f + (std::rand() % 500);
}

float simulateTemperature() {
    return 25.0f + (std::rand() % 100 - 50) / 10.0f;
}

// ============================================================================
// Основная программа
// ============================================================================

int main() {
    initCAN();

    printf("Starting CAN communication demo...\n");
    printf("Press Ctrl+C to stop\n\n");

    auto startTime = std::chrono::steady_clock::now();
    uint32_t telemetryCount = 0;
    uint32_t heartbeatCount = 0;

    // Основной цикл
    while (telemetryCount < 20) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - startTime).count();

        // Отправка телеметрии каждые 100 мс
        if (elapsed % config::TELEMETRY_PERIOD_MS < 20) {
            if (telemetryCount % 5 == 0) {  // Каждые 500 мс
                float v = simulateVoltage();
                float c = simulateCurrent();
                float t = simulateTemperature();
                
                CANFrame frame = createTelemetryFrame(v, c, t, 0x01);
                
                printf("[TX] Telemetry: V=%.2fV I=%.1fmA T=%.1fC\n", v, c, t);
                sendFrame(frame);
                
                telemetryCount++;
            }
        }

        // Отправка heartbeat каждые 1 секунду
        if (elapsed % config::HEARTBEAT_PERIOD_MS < 20 && 
            elapsed >= heartbeatCount * config::HEARTBEAT_PERIOD_MS) {
            
            CANFrame frame = createHeartbeatFrame(0x05);  // Operational state
            printf("[TX] Heartbeat #%u\n", heartbeatCount + 1);
            sendFrame(frame);
            
            heartbeatCount++;
        }

        // Приём кадров (неблокирующий)
        CANFrame rxFrame;
        while (receiveFrame(rxFrame)) {
            processReceivedFrame(rxFrame);
        }

        // Проверка Bus Off recovery
        if (busOff) {
            printf("[RECOVER] Attempting bus recovery...\n");
            can->recoverBus();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            busOff = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Статистика
    printf("\n=== CAN Statistics ===\n");
    printf("TX frames: %lu\n", canStats.txFrames);
    printf("RX frames: %lu\n", canStats.rxFrames);
    printf("Errors:    %lu\n", canStats.errors);
    printf("Bus OFF:   %lu\n", canStats.busOffCount);
    printf("======================\n");

    // Тест loopback (если доступен)
    printf("\n=== Loopback Test ===\n");
    can->setMode(hal::CANMode::LOOPBACK);
    
    CANFrame testFrame;
    testFrame.id = 0x7FF;
    testFrame.dlc = 8;
    testFrame.isExtended = false;
    testFrame.data = {1, 2, 3, 4, 5, 6, 7, 8};
    
    printf("[TEST] Sending loopback frame...\n");
    sendFrame(testFrame);
    
    CANFrame receivedFrame;
    if (receiveFrame(receivedFrame)) {
        printf("[TEST] Received: ID=0x%03X Data:", receivedFrame.id);
        for (int i = 0; i < receivedFrame.dlc; i++) {
            printf(" %02X", receivedFrame.data[i]);
        }
        printf("\n[TEST] Loopback OK\n");
    } else {
        printf("[TEST] No response (expected in simulation)\n");
    }

    // Возврат в нормальный режим
    can->setMode(hal::CANMode::NORMAL);

    // Очистка
    delete can;

    printf("\nExample completed.\n");
    return 0;
}

// ============================================================================
// Версия для FreeRTOS
// ============================================================================

#ifdef USE_FREERTOS

#include "rtos/freertos_wrapper.hpp"

using namespace mka::rtos;

// Очередь для CAN кадров
static Queue<CANFrame, 32> canRxQueue;
static Queue<CANFrame, 16> canTxQueue;

// Задача приёма CAN
void vCANRxTask(void* pvParameters) {
    for (;;) {
        CANFrame frame;
        if (receiveFrame(frame)) {
            canRxQueue.send(frame, NO_TIMEOUT);
        }
        
        Task<1024>::delay(10);
    }
}

// Задача обработки CAN
void vCANProcessTask(void* pvParameters) {
    for (;;) {
        auto frame = canRxQueue.receive(INFINITE_TIMEOUT);
        if (frame.has_value()) {
            processReceivedFrame(frame.value());
        }
    }
}

// Задача передачи CAN
void vCANTxTask(void* pvParameters) {
    uint32_t lastWakeTime = Task<1024>::getTickCount();
    
    for (;;) {
        CANFrame frame;
        if (canTxQueue.receive(&frame, NO_TIMEOUT)) {
            sendFrame(frame);
        }
        
        Task<1024>::delayUntil(lastWakeTime, config::TELEMETRY_PERIOD_MS);
    }
}

void vStartCANCommunication(void) {
    static Task<2048> rxTask("CAN_Rx", 4, +[]() { vCANRxTask(nullptr); });
    static Task<2048> procTask("CAN_Proc", 3, +[]() { vCANProcessTask(nullptr); });
    static Task<2048> txTask("CAN_Tx", 3, +[]() { vCANTxTask(nullptr); });

    rxTask.start();
    procTask.start();
    txTask.start();
}

#endif // USE_FREERTOS
