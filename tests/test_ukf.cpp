/**
 * @file test_ukf.cpp
 * @brief Unit tests for Unscented Kalman Filter
 */

#include <gtest/gtest.h>
#include <cmath>
#include <array>

#include "algorithms/adcs_algorithms.hpp"

using namespace mka::adcs;

// ============================================================================
// Тесты инициализации UKF
// ============================================================================

TEST(UKFTest, Initialization) {
    UnscentedKalmanFilter::Config config;
    UnscentedKalmanFilter ukf(config);
    
    ukf.init();
    
    EXPECT_TRUE(ukf.isInitialized());
    
    auto q = ukf.getQuaternion();
    EXPECT_NEAR(q.w, 1.0f, 0.01f);
    EXPECT_NEAR(q.x, 0.0f, 0.01f);
    EXPECT_NEAR(q.y, 0.0f, 0.01f);
    EXPECT_NEAR(q.z, 0.0f, 0.01f);
}

TEST(UKFTest, InitializationWithBias) {
    UnscentedKalmanFilter::Config config;
    UnscentedKalmanFilter ukf(config);
    
    std::array<float, 3> gyroBias = {0.01f, -0.02f, 0.005f};
    ukf.init(Quaternion(), gyroBias);
    
    auto bias = ukf.getGyroBias();
    EXPECT_NEAR(bias[0], 0.01f, 0.001f);
    EXPECT_NEAR(bias[1], -0.02f, 0.001f);
    EXPECT_NEAR(bias[2], 0.005f, 0.001f);
}

// ============================================================================
// Тесты обновления гироскопом
// ============================================================================

TEST(UKFTest, GyroUpdate) {
    UnscentedKalmanFilter::Config config;
    UnscentedKalmanFilter ukf(config);
    ukf.init();
    
    float dt = 0.01f;
    std::array<float, 3> gyro = {0.0f, 0.0f, 0.1f};
    
    ukf.predict(gyro, dt);
    
    auto q = ukf.getQuaternion();
    float norm = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    EXPECT_NEAR(norm, 1.0f, 0.01f);
}

// ============================================================================
// Тесты обновления акселерометром
// ============================================================================

TEST(UKFTest, AccelUpdate) {
    UnscentedKalmanFilter::Config config;
    UnscentedKalmanFilter ukf(config);
    ukf.init();
    
    float dt = 0.01f;
    std::array<float, 3> gyro = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> accel = {0.0f, 0.0f, 9.81f};
    
    ukf.predict(gyro, dt);
    ukf.update(accel[0], accel[1], accel[2], 0.0f, 0.0f, 0.0f);
    
    auto q = ukf.getQuaternion();
    float norm = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    EXPECT_NEAR(norm, 1.0f, 0.01f);
}

// ============================================================================
// Тесты стабильности
// ============================================================================

TEST(UKFTest, NumericalStability) {
    UnscentedKalmanFilter::Config config;
    UnscentedKalmanFilter ukf(config);
    ukf.init();
    
    float dt = 0.01f;
    
    for (int i = 0; i < 1000; i++) {
        std::array<float, 3> gyro = {
            0.1f * std::sin(i * 0.01f),
            0.1f * std::cos(i * 0.01f),
            0.05f
        };
        std::array<float, 3> accel = {0.0f, 0.0f, 9.81f};
        
        ukf.predict(gyro, dt);
        ukf.update(accel[0], accel[1], accel[2], 0.0f, 0.0f, 0.0f);
    }
    
    auto q = ukf.getQuaternion();
    float norm = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    EXPECT_NEAR(norm, 1.0f, 0.001f);
    
    EXPECT_FALSE(std::isnan(q.w));
    EXPECT_FALSE(std::isnan(q.x));
    EXPECT_FALSE(std::isnan(q.y));
    EXPECT_FALSE(std::isnan(q.z));
}

TEST(UKFTest, ZeroDeltaTime) {
    UnscentedKalmanFilter::Config config;
    UnscentedKalmanFilter ukf(config);
    ukf.init();
    
    std::array<float, 3> gyro = {0.1f, 0.0f, 0.0f};
    ukf.predict(gyro, 0.0f);
    
    auto q = ukf.getQuaternion();
    EXPECT_FALSE(std::isnan(q.w));
}
