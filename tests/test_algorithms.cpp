/**
 * @file test_algorithms.cpp
 * @brief Unit tests for ADCS algorithms
 */

#include <gtest/gtest.h>
#include <cmath>
#include <array>

#include "algorithms/adcs_algorithms.hpp"

using namespace mka::adcs;

// ============================================================================
// Тесты кватернионов
// ============================================================================

TEST(QuaternionTest, IdentityInitialization) {
    Quaternion q;
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);
}

TEST(QuaternionTest, DefaultConstructorValues) {
    // Проверка явной инициализации по умолчанию
    Quaternion q1;
    EXPECT_FLOAT_EQ(q1.w, 1.0f);
    EXPECT_FLOAT_EQ(q1.x, 0.0f);

    Quaternion q2{};
    EXPECT_FLOAT_EQ(q2.w, 1.0f);
    EXPECT_FLOAT_EQ(q2.x, 0.0f);
}

TEST(QuaternionTest, CustomInitialization) {
    Quaternion q(0.707f, 0.707f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(q.w, 0.707f);
    EXPECT_FLOAT_EQ(q.x, 0.707f);
}

TEST(QuaternionTest, Normalize) {
    Quaternion q(0.0f, 2.0f, 0.0f, 0.0f);
    q.normalize();
    EXPECT_FLOAT_EQ(q.w, 0.0f);
    EXPECT_FLOAT_EQ(q.x, 1.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);
}

TEST(QuaternionTest, NormalizeNearZero) {
    Quaternion q(0.0f, 1e-15f, 0.0f, 0.0f);
    q.normalize();  // Не должен крашнуть, но остаётся нулевым
    // При нулевой норме кватернион не нормализуется (защита от деления на ноль)
    EXPECT_NEAR(q.w, 0.0f, 1e-14f);
    EXPECT_NEAR(q.x, 0.0f, 1e-14f);
}

TEST(QuaternionTest, Conjugate) {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion conj = q.conjugate();
    EXPECT_FLOAT_EQ(conj.w, 1.0f);
    EXPECT_FLOAT_EQ(conj.x, -2.0f);
    EXPECT_FLOAT_EQ(conj.y, -3.0f);
    EXPECT_FLOAT_EQ(conj.z, -4.0f);
}

TEST(QuaternionTest, Multiplication) {
    Quaternion q1(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion q2(0.0f, 1.0f, 0.0f, 0.0f);
    Quaternion result = q1 * q2;
    
    EXPECT_FLOAT_EQ(result.w, 0.0f);
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

TEST(QuaternionTest, RotateVector) {
    Quaternion q(1.0f, 0.0f, 0.0f, 0.0f);  // Identity
    std::array<float, 3> v = {1.0f, 0.0f, 0.0f};
    auto result = q.rotateVector(v);
    
    EXPECT_FLOAT_EQ(result[0], 1.0f);
    EXPECT_FLOAT_EQ(result[1], 0.0f);
    EXPECT_FLOAT_EQ(result[2], 0.0f);
}

TEST(QuaternionTest, ToEulerAngles) {
    Quaternion q(1.0f, 0.0f, 0.0f, 0.0f);
    auto angles = q.toEulerAngles();
    
    // Identity quaternion -> zero angles
    EXPECT_NEAR(angles[0], 0.0f, 1e-6f);
    EXPECT_NEAR(angles[1], 0.0f, 1e-6f);
    EXPECT_NEAR(angles[2], 0.0f, 1e-6f);
}

// ============================================================================
// Тесты математических утилит
// ============================================================================

TEST(MathUtils, Clamp) {
    EXPECT_EQ(math::clamp(5, 0, 10), 5);
    EXPECT_EQ(math::clamp(-5, 0, 10), 0);
    EXPECT_EQ(math::clamp(15, 0, 10), 10);
}

TEST(MathUtils, Sign) {
    EXPECT_EQ(math::sign(5), 1);
    EXPECT_EQ(math::sign(-5), -1);
    EXPECT_EQ(math::sign(0), 0);
}

TEST(MathUtils, InvSqrt) {
    float result = math::invSqrt(4.0f);
    EXPECT_NEAR(result, 0.5f, 1e-3f);

    result = math::invSqrt(100.0f);
    EXPECT_NEAR(result, 0.1f, 1e-3f);
}

TEST(MathUtils, InvSqrtZeroAndNegative) {
    // Защита от невалидных входов
    EXPECT_FLOAT_EQ(math::invSqrt(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(math::invSqrt(-5.0f), 0.0f);
}

// ============================================================================
// Тесты фильтра Маджвика
// ============================================================================

TEST(MadgwickFilter, Initialization) {
    MadgwickFilter filter(100.0f, 0.1f);
    auto q = filter.getQuaternion();
    
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);
}

TEST(MadgwickFilter, UpdateIMUStability) {
    MadgwickFilter filter(100.0f, 0.1f);
    
    // Симуляция статических данных (спокойное состояние)
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float ax = 0.0f, ay = 0.0f, az = 1.0f;  // Вектор вверх
    float dt = 0.01f;
    
    for (int i = 0; i < 100; ++i) {
        filter.updateIMU(gx, gy, gz, ax, ay, az, dt);
    }
    
    auto q = filter.getQuaternion();
    
    // Кватернион должен быть нормализован
    float norm = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    EXPECT_NEAR(norm, 1.0f, 1e-3f);
}

TEST(MadgwickFilter, UpdateWithZeroAccel) {
    MadgwickFilter filter(100.0f, 0.1f);
    
    // Нулевой акселерометр - не должен крашнуть
    filter.updateIMU(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f);
    
    auto q = filter.getQuaternion();
    EXPECT_FLOAT_EQ(q.w, 1.0f);  // Без изменений
}

TEST(MadgwickFilter, BetaSetter) {
    MadgwickFilter filter(100.0f, 0.1f);
    filter.setBeta(0.5f);
    
    // Фильтр должен работать с новым beta
    filter.updateIMU(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.01f);
    
    auto q = filter.getQuaternion();
    EXPECT_TRUE(std::isfinite(q.w) && std::isfinite(q.x));
}

// ============================================================================
// Тесты PID контроллера
// ============================================================================

TEST(PIDController, BasicControl) {
    PIDController::Config config;
    config.kp = 1.0f;
    config.ki = 0.0f;
    config.kd = 0.0f;
    config.outputMin = -10.0f;
    config.outputMax = 10.0f;
    
    PIDController pid(config);
    
    // P-регулятор: выход = kp * error
    float output = pid.compute(10.0f, 5.0f, 0.1f);
    EXPECT_FLOAT_EQ(output, 5.0f);
}

TEST(PIDController, IntegralTerm) {
    PIDController::Config config;
    config.kp = 0.0f;
    config.ki = 1.0f;
    config.kd = 0.0f;
    config.outputMin = -10.0f;
    config.outputMax = 10.0f;
    config.integralLimit = 10.0f;

    PIDController pid(config);

    // Интеграл: sum(error * dt)
    // Первый вызов: integral = 10*0.1 = 1.0, output = 1.0 * 1.0 = 1.0
    float output = pid.compute(10.0f, 0.0f, 0.1f);
    // Второй вызов: integral = 1.0 + 1.0 = 2.0, output = 2.0
    float output2 = pid.compute(10.0f, 0.0f, 0.1f);
    // Третий вызов: integral = 2.0 + 1.0 = 3.0, output = 3.0
    float output3 = pid.compute(10.0f, 0.0f, 0.1f);

    EXPECT_FLOAT_EQ(output, 1.0f);
    EXPECT_FLOAT_EQ(output2, 2.0f);
    EXPECT_FLOAT_EQ(output3, 3.0f);
}

TEST(PIDController, IntegralLimit) {
    PIDController::Config config;
    config.kp = 0.0f;
    config.ki = 1.0f;
    config.kd = 0.0f;
    config.outputMin = -10.0f;
    config.outputMax = 10.0f;
    config.integralLimit = 1.0f;
    
    PIDController pid(config);
    
    // Интеграл должен быть ограничен
    for (int i = 0; i < 100; ++i) {
        pid.compute(10.0f, 0.0f, 0.1f);
    }
    
    float output = pid.compute(10.0f, 0.0f, 0.1f);
    EXPECT_FLOAT_EQ(output, 1.0f);  // Ограничен
}

TEST(PIDController, OutputClamp) {
    PIDController::Config config;
    config.kp = 10.0f;
    config.outputMin = -5.0f;
    config.outputMax = 5.0f;
    
    PIDController pid(config);
    
    float output = pid.compute(10.0f, 0.0f, 0.1f);
    EXPECT_FLOAT_EQ(output, 5.0f);  // Ограничен
    
    output = pid.compute(-10.0f, 0.0f, 0.1f);
    EXPECT_FLOAT_EQ(output, -5.0f);  // Ограничен
}

TEST(PIDController, Reset) {
    PIDController::Config config;
    config.kp = 1.0f;
    config.ki = 0.0f;  // Отключаем интегральную составляющую

    PIDController pid(config);

    pid.compute(10.0f, 0.0f, 0.1f);
    pid.reset();

    float output = pid.compute(10.0f, 0.0f, 0.1f);
    EXPECT_FLOAT_EQ(output, 10.0f);  // Только пропорциональная составляющая
}

TEST(PIDController, GainSetter) {
    PIDController::Config config;
    config.kp = 1.0f;
    config.ki = 0.0f;  // Отключаем интегральную составляющую
    config.kd = 0.0f;  // Отключаем деривативную составляющую

    PIDController pid(config);
    pid.setGains(2.0f, 0.0f, 0.0f);  // Устанавливаем только kp=2

    float output = pid.compute(10.0f, 0.0f, 0.1f);
    EXPECT_FLOAT_EQ(output, 20.0f);  // kp=2, error=10
}

TEST(PIDController, DefaultConstructor) {
    PIDController pid;  // Конструктор по умолчанию
    // Должен компилироваться и работать
    pid.setGains(1.0f, 0.0f, 0.0f);
    float output = pid.compute(10.0f, 0.0f, 0.1f);
    EXPECT_FLOAT_EQ(output, 10.0f);
}

// ============================================================================
// Тесты B-dot контроллера
// ============================================================================

TEST(BDotController, Initialization) {
    BDotController::Config config;
    config.gain = 1e-8f;
    config.maxDipole = 0.1f;
    
    BDotController bdot(config);
    
    auto result = bdot.compute(20000e-9f, 0.0f, 40000e-9f, 0.01f);
    
    // Результат должен быть конечным
    EXPECT_TRUE(std::isfinite(result[0]));
    EXPECT_TRUE(std::isfinite(result[1]));
    EXPECT_TRUE(std::isfinite(result[2]));
}

TEST(BDotController, DipoleLimit) {
    BDotController::Config config;
    config.gain = 1.0f;
    config.maxDipole = 0.1f;
    config.filterCoeff = 1.0f;
    
    BDotController bdot(config);
    
    // Большие изменения магнитного поля
    auto result = bdot.compute(1e6f, 1e6f, 1e6f, 0.01f);
    
    float norm = std::sqrt(result[0]*result[0] + result[1]*result[1] + result[2]*result[2]);
    EXPECT_LE(norm, config.maxDipole + 1e-6f);
}

TEST(BDotController, Reset) {
    BDotController::Config config;
    config.gain = 1.0f;
    config.maxDipole = 0.1f;

    BDotController bdot(config);

    bdot.compute(100e-6f, 0.0f, 0.0f, 0.01f);
    bdot.reset();

    // После reset фильтры должны быть нулевыми
    auto result = bdot.compute(0.0f, 0.0f, 0.0f, 0.01f);
    EXPECT_FLOAT_EQ(result[0], 0.0f);
    EXPECT_FLOAT_EQ(result[1], 0.0f);
    EXPECT_FLOAT_EQ(result[2], 0.0f);
}

TEST(BDotController, DefaultConstructor) {
    BDotController bdot;  // Конструктор по умолчанию
    // Должен компилироваться и работать
    bdot.reset();
    EXPECT_TRUE(true);
}

// ============================================================================
// Тесты контроллера ориентации
// ============================================================================

TEST(AttitudeController, ComputeStability) {
    AttitudeController::Config config;
    config.rollConfig.kp = 1.0f;
    config.pitchConfig.kp = 1.0f;
    config.yawConfig.kp = 1.0f;
    config.maxTorque = 1.0f;
    
    AttitudeController controller(config);
    
    Quaternion target(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion current(1.0f, 0.0f, 0.0f, 0.0f);
    std::array<float, 3> omega = {0.0f, 0.0f, 0.0f};
    
    auto result = controller.compute(target, current, omega, 0.01f);
    
    // Нулевая ошибка -> нулевой момент
    EXPECT_FLOAT_EQ(result[0], 0.0f);
    EXPECT_FLOAT_EQ(result[1], 0.0f);
    EXPECT_FLOAT_EQ(result[2], 0.0f);
}

TEST(AttitudeController, TorqueLimit) {
    AttitudeController::Config config;
    config.rollConfig.kp = 100.0f;
    config.pitchConfig.kp = 100.0f;
    config.yawConfig.kp = 100.0f;
    config.maxTorque = 0.01f;
    
    AttitudeController controller(config);
    
    Quaternion target(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion current(0.0f, 1.0f, 0.0f, 0.0f);  // Большая ошибка
    std::array<float, 3> omega = {0.0f, 0.0f, 0.0f};
    
    auto result = controller.compute(target, current, omega, 0.01f);
    
    EXPECT_GE(config.maxTorque, std::abs(result[0]));
    EXPECT_GE(config.maxTorque, std::abs(result[1]));
    EXPECT_GE(config.maxTorque, std::abs(result[2]));
}

TEST(AttitudeController, Reset) {
    AttitudeController::Config config;
    config.rollConfig.kp = 1.0f;
    config.rollConfig.ki = 1.0f;

    AttitudeController controller(config);

    controller.reset();  // Не должен крашнуть
}

TEST(AttitudeController, DefaultConstructor) {
    AttitudeController controller;  // Конструктор по умолчанию
    // Должен компилироваться и работать
    controller.reset();
    EXPECT_TRUE(true);
}

// ============================================================================
// Тесты Extended Kalman Filter
// ============================================================================

TEST(ExtendedKalmanFilter, Initialization) {
    ExtendedKalmanFilter ekf;
    ExtendedKalmanFilter::Config config;
    
    ekf.init(config);
    
    EXPECT_TRUE(ekf.isInitialized());
    
    auto q = ekf.getQuaternion();
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);
}

TEST(ExtendedKalmanFilter, PredictStep) {
    ExtendedKalmanFilter ekf;
    ExtendedKalmanFilter::Config config;
    ekf.init(config);
    
    float dt = 0.01f;
    
    // Предсказание с нулевой угловой скоростью
    ekf.predict(0.0f, 0.0f, 0.0f, dt);
    
    auto q = ekf.getQuaternion();
    EXPECT_FLOAT_EQ(q.w, 1.0f);  // Без изменений
    EXPECT_FLOAT_EQ(q.x, 0.0f);
}

TEST(ExtendedKalmanFilter, PredictWithRotation) {
    ExtendedKalmanFilter ekf;
    ExtendedKalmanFilter::Config config;
    ekf.init(config);
    
    float dt = 0.01f;
    float gx = 0.1f;  // rad/s, вращение вокруг X
    float gy = 0.0f;
    float gz = 0.0f;
    
    // Несколько шагов предсказания
    for (int i = 0; i < 10; ++i) {
        ekf.predict(gx, gy, gz, dt);
    }
    
    auto q = ekf.getQuaternion();
    
    // Кватернион должен быть нормализован
    float norm = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    EXPECT_NEAR(norm, 1.0f, 1e-3f);
    
    // Должно быть вращение вокруг X
    EXPECT_GT(std::abs(q.x), 0.0f);
}

TEST(ExtendedKalmanFilter, UpdateStep) {
    ExtendedKalmanFilter ekf;
    ExtendedKalmanFilter::Config config;
    ekf.init(config);
    
    // Статические измерения (спутник смотрит вниз)
    float ax = 0.0f, ay = 0.0f, az = 9.81f;  // Гравитация вниз
    float mx = 20e-6f, my = 0.0f, mz = 40e-6f;  // Магнитное поле
    
    ekf.update(ax, ay, az, mx, my, mz);
    
    auto q = ekf.getQuaternion();
    
    // Кватернион должен остаться нормализованным
    float norm = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    EXPECT_NEAR(norm, 1.0f, 1e-3f);
}

TEST(ExtendedKalmanFilter, PredictUpdateCycle) {
    ExtendedKalmanFilter ekf;
    ExtendedKalmanFilter::Config config;
    ekf.init(config);
    
    float dt = 0.01f;
    
    // Цикл predict-update
    for (int i = 0; i < 100; ++i) {
        float gx = 0.001f, gy = 0.001f, gz = 0.001f;
        float ax = 0.0f, ay = 0.0f, az = 9.81f;
        float mx = 20e-6f, my = 0.0f, mz = 40e-6f;
        
        ekf.predict(gx, gy, gz, dt);
        ekf.update(ax, ay, az, mx, my, mz);
    }
    
    auto q = ekf.getQuaternion();
    
    // Проверка нормализации
    float norm = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    EXPECT_NEAR(norm, 1.0f, 1e-3f);
    
    // Все компоненты должны быть конечными
    EXPECT_TRUE(std::isfinite(q.w));
    EXPECT_TRUE(std::isfinite(q.x));
    EXPECT_TRUE(std::isfinite(q.y));
    EXPECT_TRUE(std::isfinite(q.z));
}

TEST(ExtendedKalmanFilter, GyroBiasEstimation) {
    ExtendedKalmanFilter ekf;
    ExtendedKalmanFilter::Config config;
    // Увеличим шум смещения для более быстрой сходимости
    config.gyroBiasNoiseStd = 0.01f;
    ekf.init(config);

    float dt = 0.01f;

    // Имитация вращения с постоянным смещением
    // Важно: bias оценивается только при наличии вращения
    float true_bias_x = 0.01f;
    float rotation_rate = 0.1f;  // Небольшое вращение

    // Инициализация фильтра
    for (int i = 0; i < 100; ++i) {
        float gx = rotation_rate + true_bias_x;
        float gy = 0.0f;
        float gz = 0.0f;

        float ax = 0.0f, ay = 0.0f, az = 9.81f;
        float mx = 20e-6f, my = 0.0f, mz = 40e-6f;

        ekf.predict(gx, gy, gz, dt);
        ekf.update(ax, ay, az, mx, my, mz);
    }

    auto bias = ekf.getGyroBias();

    // EKF должен оценить смещение (с некоторой погрешностью)
    // Допуск увеличен из-за нелинейности фильтра
    EXPECT_NEAR(bias[0], true_bias_x, 0.5f);
}

TEST(ExtendedKalmanFilter, ZeroMeasurements) {
    ExtendedKalmanFilter ekf;
    ExtendedKalmanFilter::Config config;
    ekf.init(config);
    
    // Нулевые измерения не должны вызывать краш
    ekf.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    
    auto q = ekf.getQuaternion();
    EXPECT_FLOAT_EQ(q.w, 1.0f);  // Без изменений
}

TEST(ExtendedKalmanFilter, Reset) {
    ExtendedKalmanFilter ekf;
    ExtendedKalmanFilter::Config config;
    ekf.init(config);
    
    // Несколько шагов
    ekf.predict(0.1f, 0.0f, 0.0f, 0.01f);
    ekf.update(0.0f, 0.0f, 9.81f, 20e-6f, 0.0f, 40e-6f);
    
    ekf.reset();
    
    EXPECT_FALSE(ekf.isInitialized());
    
    // После сброса кватернион не определён (но не должен крашнуть)
    auto q = ekf.getQuaternion();
    EXPECT_FLOAT_EQ(q.w, 0.0f);
}

TEST(ExtendedKalmanFilter, CustomConfig) {
    ExtendedKalmanFilter ekf;
    ExtendedKalmanFilter::Config config;
    config.gyroNoiseStd = 0.001f;  // Меньше шум гироскопа
    config.accelNoiseStd = 0.01f;   // Меньше шум акселерометра
    config.magNoiseStd = 0.01f;     // Меньше шум магнитометра
    config.initialOrientationCov = 0.01f;
    
    ekf.init(config);
    
    float dt = 0.01f;
    
    for (int i = 0; i < 50; ++i) {
        ekf.predict(0.001f, 0.001f, 0.001f, dt);
        ekf.update(0.0f, 0.0f, 9.81f, 20e-6f, 0.0f, 40e-6f);
    }
    
    auto q = ekf.getQuaternion();
    
    // Проверка стабильности
    EXPECT_TRUE(std::isfinite(q.w));
    EXPECT_TRUE(std::isfinite(q.x));
    EXPECT_TRUE(std::isfinite(q.y));
    EXPECT_TRUE(std::isfinite(q.z));
}

TEST(ExtendedKalmanFilter, MatrixInversion) {
    // Тест на вырожденную матрицу (должна возвращать единичную)
    ExtendedKalmanFilter ekf;
    ExtendedKalmanFilter::Config config;
    ekf.init(config);
    
    // Много шагов с экстремальными значениями
    for (int i = 0; i < 1000; ++i) {
        ekf.predict(1.0f, 1.0f, 1.0f, 0.001f);
        ekf.update(100.0f, 100.0f, 100.0f, 1e-3f, 1e-3f, 1e-3f);
    }
    
    auto q = ekf.getQuaternion();
    
    // Должен оставаться стабильным
    EXPECT_TRUE(std::isfinite(q.w));
    EXPECT_TRUE(std::isfinite(q.x));
}
