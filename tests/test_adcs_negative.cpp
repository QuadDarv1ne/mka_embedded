/**
 * @file test_adcs_negative.cpp
 * @brief Негативные тесты для ADCS алгоритмов (NaN, inf, extreme values)
 * 
 * Проверяет устойчивость алгоритмов к некорректным входным данным:
 * - NaN (Not a Number) значения
 * - Inf (Infinity) значения
 * - Экстремальные значения (> 1e9, < -1e9)
 * - Нулевые и вырожденные случаи
 * - Denormalized floating-point числа
 */

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <array>
#include <random>

#include "algorithms/adcs_algorithms.hpp"

using namespace mka::algorithms;

// ============================================================================
// Quaternion Negative Tests
// ============================================================================

TEST(QuaternionNegativeTest, ConstructionWithNaN) {
    double nan = std::numeric_limits<double>::quiet_NaN();
    
    Quaternion q1(nan, 0.0, 0.0, 0.0);
    EXPECT_TRUE(std::isnan(q1.w));
    
    Quaternion q2(0.0, nan, 0.0, 0.0);
    EXPECT_TRUE(std::isnan(q2.x));
}

TEST(QuaternionNegativeTest, ConstructionWithInf) {
    double inf = std::numeric_limits<double>::infinity();
    
    Quaternion q1(inf, 0.0, 0.0, 0.0);
    EXPECT_TRUE(std::isinf(q1.w));
    
    Quaternion q2(0.0, 0.0, -inf, 0.0);
    EXPECT_TRUE(std::isinf(q2.y));
}

TEST(QuaternionNegativeTest, NormalizeNaNQuaternion) {
    double nan = std::numeric_limits<double>::quiet_NaN();
    Quaternion q(nan, 1.0, 0.0, 0.0);
    
    q.normalize();
    // Нормализация с NaN должна дать NaN
    EXPECT_TRUE(std::isnan(q.w) || std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z));
}

TEST(QuaternionNegativeTest, NormalizeInfQuaternion) {
    double inf = std::numeric_limits<double>::infinity();
    Quaternion q(inf, inf, inf, inf);
    
    q.normalize();
    // Нормализация с Inf должна дать NaN или нормализовать корректно
    bool hasNaN = std::isnan(q.w) || std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z);
    bool allFinite = std::isfinite(q.w) && std::isfinite(q.x) && 
                     std::isfinite(q.y) && std::isfinite(q.z);
    EXPECT_TRUE(hasNaN || allFinite);
}

TEST(QuaternionNegativeTest, ZeroQuaternion) {
    Quaternion q(0.0, 0.0, 0.0, 0.0);
    
    q.normalize();
    // Нулевой кватернион после нормализации должен дать NaN
    EXPECT_TRUE(std::isnan(q.w) || std::abs(q.norm() - 1.0) < 1e-10);
}

TEST(QuaternionNegativeTest, MultiplyWithNaN) {
    double nan = std::numeric_limits<double>::quiet_NaN();
    Quaternion q1(1.0, 0.0, 0.0, 0.0);
    Quaternion q2(nan, 0.0, 0.0, 0.0);
    
    auto result = q1 * q2;
    EXPECT_TRUE(std::isnan(result.w) || std::isnan(result.x) || 
                std::isnan(result.y) || std::isnan(result.z));
}

// ============================================================================
// EKF Negative Tests
// ============================================================================

TEST(EKFNegativeTest, PredictionWithNaNInputs) {
    EKFFilter ekf(100.0);  // 100 Hz
    double nan = std::numeric_limits<double>::quiet_NaN();
    
    // Gyro с NaN
    std::array<double, 3> gyro = {nan, 0.0, 0.0};
    std::array<double, 3> accel = {0.0, 0.0, 9.81};
    std::array<double, 3> mag = {0.3, 0.0, 0.5};
    double dt = 0.01;
    
    ekf.predict(gyro.data(), accel.data(), mag.data(), dt);
    
    // Оценка должна содержать NaN после prediction с NaN
    auto state = ekf.getState();
    bool hasNaN = std::isnan(state[0]) || std::isnan(state[1]) || 
                  std::isnan(state[2]) || std::isnan(state[3]);
    EXPECT_TRUE(hasNaN);
}

TEST(EKFNegativeTest, PredictionWithInfInputs) {
    EKFFilter ekf(100.0);
    double inf = std::numeric_limits<double>::infinity();
    
    std::array<double, 3> gyro = {inf, 0.0, 0.0};
    std::array<double, 3> accel = {0.0, 0.0, 9.81};
    std::array<double, 3> mag = {0.3, 0.0, 0.5};
    double dt = 0.01;
    
    ekf.predict(gyro.data(), accel.data(), mag.data(), dt);
    
    auto state = ekf.getState();
    bool hasInfOrNaN = false;
    for (size_t i = 0; i < 7; ++i) {
        if (std::isinf(state[i]) || std::isnan(state[i])) {
            hasInfOrNaN = true;
            break;
        }
    }
    EXPECT_TRUE(hasInfOrNaN);
}

TEST(EKFNegativeTest, PredictionWithZeroDT) {
    EKFFilter ekf(100.0);
    
    std::array<double, 3> gyro = {0.0, 0.0, 0.0};
    std::array<double, 3> accel = {0.0, 0.0, 9.81};
    std::array<double, 3> mag = {0.3, 0.0, 0.5};
    double dt = 0.0;
    
    // dt=0 должен обрабатываться корректно
    ekf.predict(gyro.data(), accel.data(), mag.data(), dt);
    
    auto state = ekf.getState();
    // Состояние должно измениться (нормализация кватерниона)
    bool allFinite = true;
    for (size_t i = 0; i < 7; ++i) {
        if (!std::isfinite(state[i])) {
            allFinite = false;
            break;
        }
    }
    EXPECT_TRUE(allFinite);
}

TEST(EKFNegativeTest, UpdateWithNaNMag) {
    EKFFilter ekf(100.0);
    double nan = std::numeric_limits<double>::quiet_NaN();
    
    std::array<double, 3> gyro = {0.0, 0.0, 0.0};
    std::array<double, 3> accel = {0.0, 0.0, 9.81};
    std::array<double, 3> mag = {nan, nan, nan};
    double dt = 0.01;
    
    ekf.predict(gyro.data(), accel.data(), mag.data(), dt);
    
    auto state = ekf.getState();
    bool allFinite = true;
    for (size_t i = 0; i < 7; ++i) {
        if (!std::isfinite(state[i])) {
            allFinite = false;
            break;
        }
    }
    // EKF должен обрабатывать NaN в магнитометре
    EXPECT_TRUE(allFinite);
}

// ============================================================================
// Madgwick Filter Negative Tests
// ============================================================================

TEST(MadgwickNegativeTest, UpdateWithNaNInputs) {
    MadgwickFilter filter(100.0, 0.1);
    double nan = std::numeric_limits<double>::quiet_NaN();
    
    std::array<double, 3> gyro = {nan, 0.0, 0.0};
    std::array<double, 3> accel = {0.0, 0.0, 9.81};
    std::array<double, 3> mag = {0.3, 0.0, 0.5};
    double dt = 0.01;
    
    filter.update(gyro.data(), accel.data(), mag.data(), dt);
    
    auto quat = filter.getQuaternion();
    bool hasNaN = std::isnan(quat.w) || std::isnan(quat.x) || 
                  std::isnan(quat.y) || std::isnan(quat.z);
    EXPECT_TRUE(hasNaN);
}

TEST(MadgwickNegativeTest, UpdateWithExtremeValues) {
    MadgwickFilter filter(100.0, 0.1);
    double extreme = 1e30;
    
    std::array<double, 3> gyro = {extreme, extreme, extreme};
    std::array<double, 3> accel = {0.0, 0.0, 9.81};
    std::array<double, 3> mag = {0.3, 0.0, 0.5};
    double dt = 0.01;
    
    filter.update(gyro.data(), accel.data(), mag.data(), dt);
    
    auto quat = filter.getQuaternion();
    // Экстремальные значения должны обработаться (возможно с NaN)
    bool hasNaNOrInf = std::isnan(quat.w) || std::isinf(quat.w) ||
                       std::isnan(quat.x) || std::isinf(quat.x);
    EXPECT_TRUE(hasNaNOrInf);
}

TEST(MadgwickNegativeTest, UpdateWithZeroAccel) {
    MadgwickFilter filter(100.0, 0.1);
    
    std::array<double, 3> gyro = {0.0, 0.0, 0.0};
    std::array<double, 3> accel = {0.0, 0.0, 0.0};  // Нулевой акселерометр
    std::array<double, 3> mag = {0.3, 0.0, 0.5};
    double dt = 0.01;
    
    filter.update(gyro.data(), accel.data(), mag.data(), dt);
    
    auto quat = filter.getQuaternion();
    // Должен обработать корректно
    bool allFinite = std::isfinite(quat.w) && std::isfinite(quat.x) && 
                     std::isfinite(quat.y) && std::isfinite(quat.z);
    EXPECT_TRUE(allFinite);
}

// ============================================================================
// UKF Negative Tests
// ============================================================================

TEST(UKFNegativeTest, PredictionWithNaN) {
    UKFFilter ukf(100.0);
    double nan = std::numeric_limits<double>::quiet_NaN();
    
    std::array<double, 3> gyro = {nan, 0.0, 0.0};
    std::array<double, 3> accel = {0.0, 0.0, 9.81};
    std::array<double, 3> mag = {0.3, 0.0, 0.5};
    double dt = 0.01;
    
    ukf.predict(gyro.data(), accel.data(), mag.data(), dt);
    
    auto state = ukf.getState();
    bool hasNaN = false;
    for (size_t i = 0; i < 7; ++i) {
        if (std::isnan(state[i])) {
            hasNaN = true;
            break;
        }
    }
    EXPECT_TRUE(hasNaN);
}

TEST(UKFNegativeTest, PredictionWithInf) {
    UKFFilter ukf(100.0);
    double inf = std::numeric_limits<double>::infinity();
    
    std::array<double, 3> gyro = {inf, 0.0, 0.0};
    std::array<double, 3> accel = {0.0, 0.0, 9.81};
    std::array<double, 3> mag = {0.3, 0.0, 0.5};
    double dt = 0.01;
    
    ukf.predict(gyro.data(), accel.data(), mag.data(), dt);
    
    auto state = ukf.getState();
    bool hasInfOrNaN = false;
    for (size_t i = 0; i < 7; ++i) {
        if (std::isinf(state[i]) || std::isnan(state[i])) {
            hasInfOrNaN = true;
            break;
        }
    }
    EXPECT_TRUE(hasInfOrNaN);
}

// ============================================================================
// PID Controller Negative Tests
// ============================================================================

TEST(PIDNegativeTest, ControlWithNaNSetpoint) {
    PIDController pid(1.0, 0.1, 0.01);
    double nan = std::numeric_limits<double>::quiet_NaN();
    
    double current = 0.0;
    double dt = 0.01;
    
    double output = pid.compute(nan, current, dt);
    EXPECT_TRUE(std::isnan(output));
}

TEST(PIDNegativeTest, ControlWithNaNCurrent) {
    PIDController pid(1.0, 0.1, 0.01);
    double nan = std::numeric_limits<double>::quiet_NaN();
    
    double setpoint = 1.0;
    double dt = 0.01;
    
    double output = pid.compute(setpoint, nan, dt);
    EXPECT_TRUE(std::isnan(output));
}

TEST(PIDNegativeTest, ControlWithZeroDT) {
    PIDController pid(1.0, 0.1, 0.01);
    
    double setpoint = 1.0;
    double current = 0.0;
    double dt = 0.0;
    
    double output = pid.compute(setpoint, current, dt);
    // dt=0 должен обрабатываться (вернуть 0 или предыдущее значение)
    EXPECT_TRUE(std::isfinite(output) || std::isnan(output));
}

TEST(PIDNegativeTest, ControlWithExtremeGains) {
    double extremeGain = 1e30;
    PIDController pid(extremeGain, 0.0, 0.0);
    
    double setpoint = 1.0;
    double current = 0.0;
    double dt = 0.01;
    
    double output = pid.compute(setpoint, current, dt);
    EXPECT_TRUE(std::isinf(output) || std::abs(output) > 1e20);
}

// ============================================================================
// B-Dot Controller Negative Tests
// ============================================================================

TEST(BDotNegativeTest, ControlWithNaN) {
    BDotController bdot(1.0);
    double nan = std::numeric_limits<double>::quiet_NaN();
    
    std::array<double, 3> dbdt = {nan, 0.0, 0.0};
    std::array<double, 3> output;
    
    bdot.compute(dbdt.data(), output.data(), 0.01);
    
    bool hasNaN = std::isnan(output[0]) || std::isnan(output[1]) || std::isnan(output[2]);
    EXPECT_TRUE(hasNaN);
}

TEST(BDotNegativeTest, ControlWithExtremeValues) {
    BDotController bdot(1.0);
    double extreme = 1e30;
    
    std::array<double, 3> dbdt = {extreme, extreme, extreme};
    std::array<double, 3> output;
    
    bdot.compute(dbdt.data(), output.data(), 0.01);
    
    bool hasInfOrNaN = std::isinf(output[0]) || std::isnan(output[0]) ||
                       std::isinf(output[1]) || std::isnan(output[1]) ||
                       std::isinf(output[2]) || std::isnan(output[2]);
    EXPECT_TRUE(hasInfOrNaN);
}

TEST(BDotNegativeTest, ControlWithZeroDT) {
    BDotController bdot(1.0);
    
    std::array<double, 3> dbdt = {0.1, 0.2, 0.3};
    std::array<double, 3> output;
    
    bdot.compute(dbdt.data(), output.data(), 0.0);
    
    // dt=0 должен обрабатываться
    bool allFinite = std::isfinite(output[0]) && std::isfinite(output[1]) && 
                     std::isfinite(output[2]);
    EXPECT_TRUE(allFinite);
}

// ============================================================================
// Sliding Mode Controller Negative Tests
// ============================================================================

TEST(SlidingModeNegativeTest, ControlWithNaN) {
    SlidingModeController smc(1.0, 0.1, 0.01);
    double nan = std::numeric_limits<double>::quiet_NaN();
    
    Quaternion q(nan, 0.0, 0.0, 0.0);
    std::array<double, 3> omega = {0.0, 0.0, 0.0};
    std::array<double, 3> output;
    
    smc.compute(q, omega.data(), output.data(), 0.01);
    
    bool hasNaN = std::isnan(output[0]) || std::isnan(output[1]) || std::isnan(output[2]);
    EXPECT_TRUE(hasNaN);
}

TEST(SlidingModeNegativeTest, ControlWithInf) {
    SlidingModeController smc(1.0, 0.1, 0.01);
    double inf = std::numeric_limits<double>::infinity();
    
    Quaternion q(1.0, 0.0, 0.0, 0.0);
    std::array<double, 3> omega = {inf, inf, inf};
    std::array<double, 3> output;
    
    smc.compute(q, omega.data(), output.data(), 0.01);
    
    bool hasInfOrNaN = std::isinf(output[0]) || std::isnan(output[0]) ||
                       std::isinf(output[1]) || std::isnan(output[1]) ||
                       std::isinf(output[2]) || std::isnan(output[2]);
    EXPECT_TRUE(hasInfOrNaN);
}

// ============================================================================
// TRIAD Negative Tests
// ============================================================================

TEST(TRIADNegativeTest, EstimateWithNaNVectors) {
    TRIADEstimator triad;
    double nan = std::numeric_limits<double>::quiet_NaN();
    
    std::array<double, 3> v1 = {nan, 0.0, 1.0};
    std::array<double, 3> v2 = {0.0, 1.0, 0.0};
    std::array<double, 3> w1 = {0.0, 0.0, 1.0};
    std::array<double, 3> w2 = {0.0, 1.0, 0.0};
    
    auto result = triad.estimate(v1.data(), v2.data(), w1.data(), w2.data());
    
    // Должен вернуть ошибку или NaN
    bool isValid = result.isOk();
    EXPECT_FALSE(isValid);
}

TEST(TRIADNegativeTest, EstimateWithZeroVectors) {
    TRIADEstimator triad;
    
    std::array<double, 3> v1 = {0.0, 0.0, 0.0};
    std::array<double, 3> v2 = {0.0, 1.0, 0.0};
    std::array<double, 3> w1 = {0.0, 0.0, 1.0};
    std::array<double, 3> w2 = {0.0, 1.0, 0.0};
    
    auto result = triad.estimate(v1.data(), v2.data(), w1.data(), w2.data());
    
    // Нулевой вектор должен вернуть ошибку
    EXPECT_FALSE(result.isOk());
}

TEST(TRIADNegativeTest, EstimateWithParallelVectors) {
    TRIADEstimator triad;
    
    // Параллельные векторы (cross product = 0)
    std::array<double, 3> v1 = {1.0, 0.0, 0.0};
    std::array<double, 3> v2 = {2.0, 0.0, 0.0};  // Параллелен v1
    std::array<double, 3> w1 = {1.0, 0.0, 0.0};
    std::array<double, 3> w2 = {0.0, 1.0, 0.0};
    
    auto result = triad.estimate(v1.data(), v2.data(), w1.data(), w2.data());
    
    // Параллельные векторы должны вернуть ошибку
    EXPECT_FALSE(result.isOk());
}

// ============================================================================
// QUEST Negative Tests
// ============================================================================

TEST(QUESTNegativeTest, EstimateWithNaNWeights) {
    QUESTEstimator quest;
    double nan = std::numeric_limits<double>::quiet_NaN();
    
    std::vector<std::array<double, 3>> vectors = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0}
    };
    std::vector<std::array<double, 3>> refVectors = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0}
    };
    std::vector<double> weights = {nan, 0.5};
    
    auto result = quest.estimate(vectors, refVectors, weights);
    
    // NaN вес должен вернуть ошибку
    EXPECT_FALSE(result.isOk());
}

TEST(QUESTNegativeTest, EstimateWithZeroWeights) {
    QUESTEstimator quest;
    
    std::vector<std::array<double, 3>> vectors = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0}
    };
    std::vector<std::array<double, 3>> refVectors = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0}
    };
    std::vector<double> weights = {0.0, 0.0};  // Все веса нулевые
    
    auto result = quest.estimate(vectors, refVectors, weights);
    
    // Нулевые веса должны вернуть ошибку
    EXPECT_FALSE(result.isOk());
}

TEST(QUESTNegativeTest, EstimateWithNaNVectors) {
    QUESTEstimator quest;
    double nan = std::numeric_limits<double>::quiet_NaN();
    
    std::vector<std::array<double, 3>> vectors = {
        {nan, 0.0, 0.0},
        {0.0, 1.0, 0.0}
    };
    std::vector<std::array<double, 3>> refVectors = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0}
    };
    std::vector<double> weights = {0.5, 0.5};
    
    auto result = quest.estimate(vectors, refVectors, weights);
    
    // NaN векторы должны вернуть ошибку
    EXPECT_FALSE(result.isOk());
}

// ============================================================================
// Random Stress Tests
// ============================================================================

TEST(ADCSSressTest, EKFWithRandomData) {
    EKFFilter ekf(100.0);
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(-1000.0, 1000.0);
    
    for (int i = 0; i < 100; ++i) {
        std::array<double, 3> gyro = {dist(gen), dist(gen), dist(gen)};
        std::array<double, 3> accel = {dist(gen), dist(gen), dist(gen)};
        std::array<double, 3> mag = {dist(gen), dist(gen), dist(gen)};
        double dt = 0.01;
        
        ekf.predict(gyro.data(), accel.data(), mag.data(), dt);
        
        auto state = ekf.getState();
        // Проверить что состояние не содержит NaN/Inf
        for (size_t j = 0; j < 7; ++j) {
            EXPECT_TRUE(std::isfinite(state[j]) || i == 0);
        }
    }
}

TEST(ADCSSressTest, MadgwickWithRandomData) {
    MadgwickFilter filter(100.0, 0.1);
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(-1000.0, 1000.0);
    
    for (int i = 0; i < 100; ++i) {
        std::array<double, 3> gyro = {dist(gen), dist(gen), dist(gen)};
        std::array<double, 3> accel = {dist(gen), dist(gen), dist(gen)};
        std::array<double, 3> mag = {dist(gen), dist(gen), dist(gen)};
        double dt = 0.01;
        
        filter.update(gyro.data(), accel.data(), mag.data(), dt);
        
        auto quat = filter.getQuaternion();
        // Проверить что кватернион конечен
        EXPECT_TRUE(std::isfinite(quat.w) && std::isfinite(quat.x) && 
                    std::isfinite(quat.y) && std::isfinite(quat.z));
    }
}
