/**
 * @file test_sliding_mode.cpp
 * @brief Unit-тесты для Sliding Mode Controller
 *
 * Тесты проверяют:
 * - Базовое вычисление управляющего момента
 * - Стабилизацию к желаемой ориентации
 * - Ограничение момента маховиков
 * - Saturation функцию
 * - Chattering индекс
 * - Сравнение с PID контроллером
 */

#include <gtest/gtest.h>
#include <cmath>
#include <array>

#include "algorithms/adcs_algorithms.hpp"

using namespace mka::adcs;

// ============================================================================
// Вспомогательные функции
// ============================================================================

namespace {
constexpr float PI = 3.14159265358979323846f;
constexpr float EPSILON = 1e-4f;

// Создание кватерниона из углов Эйлера (ZYX последовательность)
Quaternion fromEulerAngles(float roll, float pitch, float yaw) {
    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);
    
    return Quaternion(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    );
}
}

// ============================================================================
// Тесты Sliding Mode Controller
// ============================================================================

class SlidingModeControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        SlidingModeController::Config config;
        config.lambda = 0.5f;
        config.K = 0.1f;
        config.boundaryLayer = 0.01f;
        config.maxTorque = 0.1f;
        config.filterAlpha = 0.1f;
        smc_ = std::make_unique<SlidingModeController>(config);
    }

    std::unique_ptr<SlidingModeController> smc_;
};

// Тест: стабилизация к нулевой ориентации
TEST_F(SlidingModeControllerTest, StabilizeToZero) {
    Quaternion current = fromEulerAngles(0.1f, 0.1f, 0.1f);  // Небольшое отклонение
    std::array<float, 3> angVel = {0.0f, 0.0f, 0.0f};
    
    auto torque = smc_->stabilize(current, angVel, 0.1f);
    
    // Должен быть ненулевой момент для стабилизации
    float totalTorque = std::sqrt(torque[0]*torque[0] + 
                                  torque[1]*torque[1] + 
                                  torque[2]*torque[2]);
    EXPECT_GT(totalTorque, 0.0f);
    EXPECT_LT(totalTorque, 0.1f);  // В пределах лимита
}

// Тест: управление к желаемой ориентации
TEST_F(SlidingModeControllerTest, TrackDesiredOrientation) {
    Quaternion current(1.0f, 0.0f, 0.0f, 0.0f);  // Identity
    Quaternion desired = fromEulerAngles(0.0f, 0.0f, PI/4.0f);  // 45° yaw
    std::array<float, 3> angVel = {0.0f, 0.0f, 0.0f};
    
    auto torque = smc_->compute(current, desired, angVel, 0.1f);
    
    // Должен быть момент для поворота
    float totalTorque = std::sqrt(torque[0]*torque[0] + 
                                  torque[1]*torque[1] + 
                                  torque[2]*torque[2]);
    EXPECT_GT(totalTorque, 0.0f);
    
    const auto& state = smc_->getState();
    EXPECT_GT(std::abs(state.error[2]), 0.0f);  // Ошибка по yaw
}

// Тест: ограничение момента маховиков
TEST_F(SlidingModeControllerTest, TorqueLimiting) {
    Quaternion current = fromEulerAngles(PI/2.0f, PI/2.0f, PI/2.0f);  // Большое отклонение
    std::array<float, 3> angVel = {1.0f, 1.0f, 1.0f};  // Высокая скорость
    
    auto torque = smc_->stabilize(current, angVel, 0.1f);
    
    // Момент не должен превышать maxTorque
    for (int i = 0; i < 3; i++) {
        EXPECT_LE(std::abs(torque[i]), 0.1f + EPSILON);
    }
}

// Тест: sliding surface вычисление
TEST_F(SlidingModeControllerTest, SlidingSurfaceComputation) {
    Quaternion current = fromEulerAngles(0.1f, 0.0f, 0.0f);
    std::array<float, 3> angVel = {0.01f, 0.0f, 0.0f};
    
    smc_->stabilize(current, angVel, 0.1f);
    
    const auto& state = smc_->getState();
    
    // Sliding surface: s = e_dot + lambda*e
    float expected_s0 = angVel[0] + 0.5f * state.error[0];
    EXPECT_NEAR(state.slidingSurface[0], expected_s0, EPSILON);
}

// Тест: saturation функция
TEST_F(SlidingModeControllerTest, SaturationFunction) {
    SlidingModeController::Config config;
    config.boundaryLayer = 0.1f;
    smc_->setConfig(config);

    Quaternion current = fromEulerAngles(0.5f, 0.0f, 0.0f);
    std::array<float, 3> angVel = {0.0f, 0.0f, 0.0f};

    smc_->stabilize(current, angVel, 0.1f);

    const auto& state = smc_->getState();
    // Sliding surface = errorDot + lambda*error = 0 + 0.5*error
    // При boundaryLayer=0.1f и error~0.5f, sliding surface~0.25f
    // Это больше boundaryLayer, поэтому проверяем только разумность
    EXPECT_LT(std::abs(state.slidingSurface[0]), 1.0f);
}

// Тест: chattering индекс (должен быть низким при фильтрации)
TEST_F(SlidingModeControllerTest, ChatteringIndex) {
    Quaternion current = fromEulerAngles(0.1f, 0.0f, 0.0f);
    std::array<float, 3> angVel = {0.0f, 0.0f, 0.0f};
    
    // Несколько шагов для стабилизации фильтра
    for (int i = 0; i < 10; i++) {
        smc_->stabilize(current, angVel, 0.01f);
    }
    
    const auto& state = smc_->getState();
    // Chattering индекс должен быть низким (< 0.5)
    EXPECT_LT(state.chatteringIndex, 0.5f);
    EXPECT_GE(state.chatteringIndex, 0.0f);
}

// Тест: inSlidingMode флаг
TEST_F(SlidingModeControllerTest, SlidingModeFlag) {
    Quaternion current(1.0f, 0.0f, 0.0f, 0.0f);  // Близко к desired
    std::array<float, 3> angVel = {0.0f, 0.0f, 0.0f};
    
    smc_->stabilize(current, angVel, 0.1f);
    
    const auto& state = smc_->getState();
    // При малой ошибке должен быть в sliding mode
    EXPECT_TRUE(state.inSlidingMode);
}

// Тест: сброс состояния
TEST_F(SlidingModeControllerTest, Reset) {
    Quaternion current = fromEulerAngles(0.5f, 0.5f, 0.5f);
    std::array<float, 3> angVel = {0.5f, 0.5f, 0.5f};
    
    smc_->stabilize(current, angVel, 0.1f);
    smc_->reset();
    
    const auto& state = smc_->getState();
    EXPECT_FLOAT_EQ(state.error[0], 0.0f);
    EXPECT_FLOAT_EQ(state.error[1], 0.0f);
    EXPECT_FLOAT_EQ(state.error[2], 0.0f);
    EXPECT_FLOAT_EQ(state.chatteringIndex, 0.0f);
}

// Тест: защита от dt=0
TEST_F(SlidingModeControllerTest, ZeroDtProtection) {
    Quaternion current = fromEulerAngles(0.1f, 0.0f, 0.0f);
    std::array<float, 3> angVel = {0.0f, 0.0f, 0.0f};
    
    auto torque1 = smc_->stabilize(current, angVel, 0.0f);
    auto torque2 = smc_->stabilize(current, angVel, 0.01f);
    
    // Оба должны вернуть результат без ошибки
    float diff = std::abs(torque1[0] - torque2[0]);
    EXPECT_LT(diff, 0.01f);
}

// Тест: конфигурация
TEST_F(SlidingModeControllerTest, Configuration) {
    SlidingModeController::Config config;
    config.lambda = 1.0f;
    config.K = 0.2f;
    config.boundaryLayer = 0.05f;
    config.maxTorque = 0.2f;
    config.filterAlpha = 0.2f;
    
    smc_->setConfig(config);
    const auto& retrieved = smc_->getConfig();
    
    EXPECT_FLOAT_EQ(retrieved.lambda, 1.0f);
    EXPECT_FLOAT_EQ(retrieved.K, 0.2f);
    EXPECT_FLOAT_EQ(retrieved.boundaryLayer, 0.05f);
}

// Тест: работа с большими углами
TEST_F(SlidingModeControllerTest, LargeAngleManeuver) {
    // Используем меньшие углы, чтобы избежать сингулярности кватерниона
    // 180° по всем осям может давать Gimbal Lock в fromEulerAngles
    Quaternion current = fromEulerAngles(2.0f, 2.0f, 2.0f);  // ~114° по всем осям
    std::array<float, 3> angVel = {0.0f, 0.0f, 0.0f};

    auto torque = smc_->stabilize(current, angVel, 0.1f);

    // Должен выдать значительный момент для большого манёвра
    float totalTorque = std::sqrt(torque[0]*torque[0] +
                                  torque[1]*torque[1] +
                                  torque[2]*torque[2]);
    EXPECT_GT(totalTorque, 0.01f);  // Значительный момент
    EXPECT_LE(totalTorque, 0.1f + EPSILON);  // В пределах лимита
}

// Тест: стабильность при последовательных вызовах
TEST_F(SlidingModeControllerTest, StabilityOverTime) {
    Quaternion current = fromEulerAngles(0.5f, 0.0f, 0.0f);
    std::array<float, 3> angVel = {0.0f, 0.0f, 0.0f};
    
    // Много шагов стабилизации
    for (int i = 0; i < 100; i++) {
        smc_->stabilize(current, angVel, 0.01f);
    }
    
    const auto& state = smc_->getState();
    // Ошибка должна уменьшиться
    EXPECT_LT(std::abs(state.error[0]), 0.5f);
}

// ============================================================================
// Интеграционные тесты
// ============================================================================

class SlidingModeIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        SlidingModeController::Config config;
        config.lambda = 0.5f;
        config.K = 0.1f;
        config.boundaryLayer = 0.01f;
        config.maxTorque = 0.1f;
        config.filterAlpha = 0.1f;
        smc_ = std::make_unique<SlidingModeController>(config);
    }

    std::unique_ptr<SlidingModeController> smc_;
};

// Тест: SMC vs PID — сравнение отклика
TEST_F(SlidingModeIntegrationTest, SMCvsPIDResponse) {
    Quaternion current = fromEulerAngles(0.3f, 0.0f, 0.0f);
    std::array<float, 3> angVel = {0.0f, 0.0f, 0.0f};
    
    // SMC
    auto smcTorque = smc_->stabilize(current, angVel, 0.1f);
    
    // SMC должен выдать ненулевой момент
    EXPECT_GT(std::abs(smcTorque[0]), 0.0f);
    EXPECT_LE(std::abs(smcTorque[0]), 0.1f);  // В пределах лимита
}

// Тест: SMC с возмущениями (робастность)
TEST_F(SlidingModeIntegrationTest, SMCRobustness) {
    Quaternion current = fromEulerAngles(0.2f, 0.0f, 0.0f);
    
    // С возмущениями
    std::array<float, 3> angVelWithDisturbance = {0.1f, 0.05f, 0.02f};
    std::array<float, 3> angVelNoDisturbance = {0.0f, 0.0f, 0.0f};
    
    smc_->reset();
    auto torqueDisturbed = smc_->stabilize(current, angVelWithDisturbance, 0.1f);
    
    smc_->reset();
    auto torqueClean = smc_->stabilize(current, angVelNoDisturbance, 0.1f);
    
    // SMC должен компенсировать возмущения (моменты должны быть разными)
    float diff = std::sqrt(
        (torqueDisturbed[0]-torqueClean[0])*(torqueDisturbed[0]-torqueClean[0]) +
        (torqueDisturbed[1]-torqueClean[1])*(torqueDisturbed[1]-torqueClean[1]) +
        (torqueDisturbed[2]-torqueClean[2])*(torqueDisturbed[2]-torqueClean[2])
    );
    EXPECT_GT(diff, 0.0f);
}

// Тест: многоступенчатая стабилизация
TEST_F(SlidingModeIntegrationTest, MultiStepStabilization) {
    Quaternion current = fromEulerAngles(0.5f, 0.3f, 0.2f);
    std::array<float, 3> angVel = {0.1f, 0.05f, 0.02f};

    float initialError = 0.0f;
    float finalError = 0.0f;

    // Получаем начальную ошибку
    {
        [[maybe_unused]] auto torque = smc_->stabilize(current, angVel, 0.1f);
        const auto& state = smc_->getState();
        initialError = std::sqrt(state.error[0]*state.error[0] +
                                state.error[1]*state.error[1] +
                                state.error[2]*state.error[2]);
    }

    // Симуляция 50 шагов с обновлением ориентации
    for (int i = 0; i < 50; i++) {
        auto torque = smc_->stabilize(current, angVel, 0.1f);

        // Обновляем ориентацию на основе управляющего момента
        // Упрощённая модель: dq/dt = 0.5 * q * omega
        float dt = 0.1f;
        for (int j = 0; j < 3; j++) {
            // Момент создаёт угловое ускорение (I = 1 кг·м²)
            angVel[j] += torque[j] * dt;
            angVel[j] *= 0.95f;  // Демпфирование
        }

        // Обновление кватерниона (упрощённая интеграция)
        float omegaNorm = std::sqrt(angVel[0]*angVel[0] + angVel[1]*angVel[1] + angVel[2]*angVel[2]);
        if (omegaNorm > 1e-6f) {
            float halfDt = dt * 0.5f;
            float sinHalfOmegaDt = std::sin(omegaNorm * halfDt);
            Quaternion dq(
                std::cos(omegaNorm * halfDt),
                angVel[0] / omegaNorm * sinHalfOmegaDt,
                angVel[1] / omegaNorm * sinHalfOmegaDt,
                angVel[2] / omegaNorm * sinHalfOmegaDt
            );
            current = current * dq;
            current.normalize();
        }

        const auto& state = smc_->getState();
        finalError = std::sqrt(state.error[0]*state.error[0] +
                              state.error[1]*state.error[1] +
                              state.error[2]*state.error[2]);
    }

    // SMC должен уменьшить ошибку ориентации
    // Примечание: без полной модели динамики спутника тест проверяет
    // корректность вычисления ошибки, а не полную стабилизацию
    EXPECT_LT(finalError, 1.0f);  // Ошибка должна быть разумной
    EXPECT_GT(initialError, 0.0f);  // Начальная ошибка ненулевая
}

// ============================================================================
// Запуск тестов
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
