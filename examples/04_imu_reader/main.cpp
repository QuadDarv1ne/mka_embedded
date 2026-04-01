/**
 * @file main.cpp
 * @brief Пример чтения данных с IMU BMI160
 *
 * Демонстрирует использование драйвера BMI160 через I2C интерфейс.
 * Чтение акселерометра и гироскопа, вывод данных в консоль.
 */

#include <cstdint>
#include <cstdio>
#include <thread>
#include <chrono>
#include <cmath>

// HAL интерфейсы
#include "hal/hal_full.hpp"
// Драйверы
#include "drivers/sensors_drivers.hpp"

// ============================================================================
// Конфигурация
// ============================================================================

namespace config {
    // BMI160 на I2C
    constexpr uint8_t BMI160_I2C_ADDR = 0x69;  // Или 0x68
    constexpr uint32_t I2C_PORT = 1;
    constexpr uint32_t I2C_SPEED = 400000;     // 400 kHz (Fast mode)

    // Период опроса (мс)
    constexpr uint32_t READ_PERIOD_MS = 100;

    // Пороги детекции движения
    constexpr float ACCEL_THRESHOLD = 0.1f;    // g
    constexpr float GYRO_THRESHOLD = 1.0f;     // deg/s
}

// ============================================================================
// Глобальные объекты
// ============================================================================

// I2C интерфейс (статический для простоты)
hal::I2C* i2c = nullptr;

// IMU драйвер
mka::drivers::BMI160Driver* imu = nullptr;

// ============================================================================
// Инициализация
// ============================================================================

bool init() {
    printf("=== IMU Reader Example ===\n\n");

    // Инициализация I2C
    printf("[INIT] Initializing I2C...\n");
    i2c = new hal::I2C(I2C_PORT);

    hal::I2CConfig i2cConfig;
    i2cConfig.speed = config::I2C_SPEED;
    i2cConfig.dutyCycle = hal::I2CDutyCycle::DUTY_2;
    i2cConfig.ownAddress = 0;  // Master mode

    if (i2c->init(i2cConfig) != hal::Status::OK) {
        printf("[ERROR] Failed to initialize I2C\n");
        return false;
    }
    printf("[OK] I2C initialized on port %lu\n", I2C_PORT);

    // Инициализация BMI160
    printf("[INIT] Initializing BMI160...\n");
    imu = new mka::drivers::BMI160Driver(*i2c, config::BMI160_I2C_ADDR);

    if (imu->init() != hal::Status::OK) {
        printf("[ERROR] Failed to initialize BMI160. Check connection.\n");
        return false;
    }
    printf("[OK] BMI160 initialized at address 0x%02X\n", config::BMI160_I2C_ADDR);

    // Настройка диапазонов
    imu->setAccelRange(mka::drivers::BMI160AccelRange::RANGE_2G);
    imu->setGyroRange(mka::drivers::BMI160GyroRange::RANGE_2000);

    printf("[OK] BMI160 configured:\n");
    printf("      - Accel range: ±2g\n");
    printf("      - Gyro range: ±2000 deg/s\n\n");

    return true;
}

// ============================================================================
// Чтение и вывод данных
// ============================================================================

void readAndPrint() {
    mka::drivers::IMUData data;

    if (imu->readData(data) != hal::Status::OK) {
        printf("[ERROR] Failed to read IMU data\n");
        return;
    }

    // Акселерометр [g]
    printf("Accel: X=%+7.3f Y=%+7.3f Z=%+7.3f [g]\n",
           data.accelX, data.accelY, data.accelZ);

    // Гироскоп [deg/s]
    printf("Gyro:  X=%+7.2f Y=%+7.2f Z=%+7.2f [deg/s]\n",
           data.gyroX, data.gyroY, data.gyroZ);

    // Температура [°C]
    printf("Temp:  %+6.1f [°C]\n", data.temperature);

    // Детекция движения
    float accelMagnitude = std::sqrt(
        data.accelX * data.accelX +
        data.accelY * data.accelY +
        data.accelZ * data.accelZ
    );

    if (accelMagnitude > 1.0f + config::ACCEL_THRESHOLD ||
        accelMagnitude < 1.0f - config::ACCEL_THRESHOLD) {
        printf(">>> MOTION DETECTED (accel=%.2fg)\n", accelMagnitude);
    }

    // Детекция вращения
    float gyroMagnitude = std::sqrt(
        data.gyroX * data.gyroX +
        data.gyroY * data.gyroY +
        data.gyroZ * data.gyroZ
    );

    if (gyroMagnitude > config::GYRO_THRESHOLD) {
        printf(">>> ROTATION DETECTED (gyro=%.1f deg/s)\n", gyroMagnitude);
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

    printf("Starting IMU data acquisition...\n");
    printf("Press Ctrl+C to stop\n\n");

    // Основной цикл чтения
    while (true) {
        readAndPrint();
        std::this_thread::sleep_for(std::chrono::milliseconds(config::READ_PERIOD_MS));
    }

    // Очистка (недостижимо в данном примере)
    delete imu;
    delete i2c;

    return 0;
}

// ============================================================================
// Версия для FreeRTOS
// ============================================================================

#ifdef USE_FREERTOS

#include "rtos/freertos_wrapper.hpp"

using namespace mka::rtos;

// Задача чтения IMU
void vIMUReaderTask(void* pvParameters) {
    mka::drivers::IMUData data;
    uint32_t lastWakeTime = Task<1024>::getTickCount();

    for (;;) {
        if (imu->readData(data) == hal::Status::OK) {
            printf("Accel: X=%.3f Y=%.3f Z=%.3f [g]\n",
                   data.accelX, data.accelY, data.accelZ);
        }

        Task<1024>::delayUntil(lastWakeTime, config::READ_PERIOD_MS);
    }
}

// Создание задачи IMU
void vStartIMUReader(void) {
    static Task<2048> imuTask("IMUReader", 3, +[]() {
        vIMUReaderTask(nullptr);
    });

    imuTask.start();
}

#endif // USE_FREERTOS
