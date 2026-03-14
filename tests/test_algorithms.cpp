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
    pid.compute(10.0f, 0.0f, 0.1f);
    float output = pid.compute(10.0f, 0.0f, 0.1f);
    
    EXPECT_FLOAT_EQ(output, 2.0f);  // 1.0 + 1.0
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
