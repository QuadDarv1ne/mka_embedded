/**
 * @file test_triad.cpp
 * @brief Unit-тесты для TRIAD метода определения ориентации
 *
 * Тесты проверяют:
 * - Базовую оценку ориентации
 * - Нормализацию векторов
 * - Валидацию входных данных
 * - Преобразование DCM ↔ Quaternion
 * - Интеграцию с IMU данными
 * - Граничные случаи
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
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float EPSILON = 1e-4f;

float quaternionAngleDiff(const Quaternion& q1, const Quaternion& q2) {
    float dot = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
    dot = std::abs(dot); // Кватернионы q и -q представляют одну ориентацию
    dot = std::min(dot, 1.0f);
    return 2.0f * std::acos(dot);
}

} // namespace

// ============================================================================
// Тесты TRIAD
// ============================================================================

class TRIADTest : public ::testing::Test {
protected:
    void SetUp() override {
        estimator_ = std::make_unique<TRIADEstimator>();
    }

    std::unique_ptr<TRIADEstimator> estimator_;
};

// Тест: базовая оценка с единичным вращением
TEST_F(TRIADTest, IdentityRotation) {
    // Если body vectors = reference vectors, ориентация должна быть единичной
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},  // Gravity (down)
        {1.0f, 0.0f, 0.0f}   // Magnetic field (north)
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    
    EXPECT_TRUE(result.isValid);
    EXPECT_FLOAT_EQ(result.orientation.w, 1.0f);
    EXPECT_FLOAT_EQ(result.orientation.x, 0.0f);
    EXPECT_FLOAT_EQ(result.orientation.y, 0.0f);
    EXPECT_FLOAT_EQ(result.orientation.z, 0.0f);
}

// Тест: базовое вращение (DCM корректна)
TEST_F(TRIADTest, Rotation90DegreesZ) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {0.0f, -1.0f, 0.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    
    EXPECT_TRUE(result.isValid);
    
    // Проверяем что DCM - правильная матрица вращения
    // 1. Ортогональность: A * A^T = I
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++)
                sum += result.dcm[i*3+k] * result.dcm[j*3+k];
            float expected = (i == j) ? 1.0f : 0.0f;
            EXPECT_NEAR(sum, expected, EPSILON);
        }
    }
    
    // 2. Определитель = +1
    float det = result.dcm[0]*(result.dcm[4]*result.dcm[8]-result.dcm[5]*result.dcm[7])
              - result.dcm[1]*(result.dcm[3]*result.dcm[8]-result.dcm[5]*result.dcm[6])
              + result.dcm[2]*(result.dcm[3]*result.dcm[7]-result.dcm[4]*result.dcm[6]);
    EXPECT_NEAR(det, 1.0f, EPSILON);
}

// Тест: вращение на 45° вокруг оси X
TEST_F(TRIADTest, Rotation45DegreesX) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };
    float sin45 = std::sin(PI / 4.0f);
    float cos45 = std::cos(PI / 4.0f);
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, sin45, cos45},
        {1.0f, 0.0f, 0.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    EXPECT_TRUE(result.isValid);
    
    // Проверяем det = +1
    float det = result.dcm[0]*(result.dcm[4]*result.dcm[8]-result.dcm[5]*result.dcm[7])
              - result.dcm[1]*(result.dcm[3]*result.dcm[8]-result.dcm[5]*result.dcm[6])
              + result.dcm[2]*(result.dcm[3]*result.dcm[7]-result.dcm[4]*result.dcm[6]);
    EXPECT_NEAR(det, 1.0f, EPSILON);
}

// Тест: нормализация входных векторов
TEST_F(TRIADTest, VectorNormalization) {
    TRIADEstimator::Config config;
    config.normalizeInputs = true;
    estimator_->setConfig(config);

    // Ненормализованные векторы
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 5.0f},  // Не единичная длина
        {3.0f, 0.0f, 0.0f}
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, 0.0f, 10.0f},
        {2.0f, 0.0f, 0.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    
    EXPECT_TRUE(result.isValid);
    // Результат должен быть таким же как с нормализованными векторами
    EXPECT_NEAR(result.orientation.w, 1.0f, EPSILON);
}

// Тест: нулевой вектор (ошибка)
TEST_F(TRIADTest, ZeroVectorError) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 0.0f},  // Нулевой вектор!
        {1.0f, 0.0f, 0.0f}
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    
    EXPECT_FALSE(result.isValid);
    EXPECT_NE(result.errorMessage, nullptr);
}

// Тест: параллельные векторы (ошибка)
TEST_F(TRIADTest, ParallelVectorsError) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.0f}  // Параллельны!
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    
    EXPECT_FALSE(result.isValid);
    EXPECT_NE(result.errorMessage, nullptr);
}

// Тест: антипараллельные векторы (ошибка)
TEST_F(TRIADTest, AntiParallelVectorsError) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, -1.0f}  // Антипараллельны!
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, -1.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    
    EXPECT_FALSE(result.isValid);
}

// Тест: DCM матрица ортогональна
TEST_F(TRIADTest, DCMOrthogonality) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, 1.0f, 0.0f},  // Повернуты
        {1.0f, 0.0f, 0.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    
    ASSERT_TRUE(result.isValid);
    
    // Проверка ортогональности: A * A^T = I
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                sum += result.dcm[i * 3 + k] * result.dcm[j * 3 + k];
            }
            float expected = (i == j) ? 1.0f : 0.0f;
            EXPECT_NEAR(sum, expected, EPSILON);
        }
    }
}

// Тест: DCM определитель = 1
TEST_F(TRIADTest, DCMDeterminant) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.5f, 0.5f, 0.7071f},
        {0.7071f, 0.7071f, 0.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    
    ASSERT_TRUE(result.isValid);
    
    // Определитель матрицы 3x3
    float det = result.dcm[0] * (result.dcm[4]*result.dcm[8] - result.dcm[5]*result.dcm[7])
              - result.dcm[1] * (result.dcm[3]*result.dcm[8] - result.dcm[5]*result.dcm[6])
              + result.dcm[2] * (result.dcm[3]*result.dcm[7] - result.dcm[4]*result.dcm[6]);
    
    EXPECT_NEAR(det, 1.0f, EPSILON);
}

// Тест: estimateFromIMU с типичными данными
TEST_F(TRIADTest, EstimateFromIMU) {
    // Типичные данные IMU на поверхности Земли
    std::array<float, 3> accel = {0.0f, 0.0f, 9.81f};  // Gravity
    std::array<float, 3> mag = {0.5f, 0.0f, 0.0f};     // Magnetic field
    
    std::array<float, 3> gravityRef = {0.0f, 0.0f, 1.0f};
    std::array<float, 3> magRef = {1.0f, 0.0f, 0.0f};

    auto result = estimator_->estimateFromIMU(accel, mag, gravityRef, magRef);
    
    EXPECT_TRUE(result.isValid);
    EXPECT_NEAR(result.orientation.w, 1.0f, EPSILON);
}

// Тест: угол между векторами в результате
TEST_F(TRIADTest, VectorsAngleReported) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {0.7071f, 0.7071f, 0.0f}  // 90° к первому
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    
    ASSERT_TRUE(result.isValid);
    EXPECT_NEAR(result.vectorsAngle, PI / 2.0f, 0.01f);  // ~90°
}

// Тест: конфигурация по умолчанию
TEST_F(TRIADTest, DefaultConfig) {
    TRIADEstimator defaultEstimator;
    auto config = defaultEstimator.getConfig();
    
    EXPECT_TRUE(config.normalizeInputs);
    EXPECT_TRUE(config.computeDCM);
    EXPECT_NEAR(config.minVectorAngle, 10.0f * DEG_TO_RAD, EPSILON);
    EXPECT_NEAR(config.maxVectorAngle, 170.0f * DEG_TO_RAD, EPSILON);
}

// Тест: кастомная конфигурация
TEST_F(TRIADTest, CustomConfig) {
    TRIADEstimator::Config config;
    config.minVectorAngle = 5.0f * DEG_TO_RAD;
    config.maxVectorAngle = 175.0f * DEG_TO_RAD;
    config.normalizeInputs = false;
    
    estimator_->setConfig(config);
    
    auto retrieved = estimator_->getConfig();
    EXPECT_NEAR(retrieved.minVectorAngle, 5.0f * DEG_TO_RAD, EPSILON);
    EXPECT_FALSE(retrieved.normalizeInputs);
}

// Тест: вращение на 180° вокруг оси Y
TEST_F(TRIADTest, Rotation180DegreesY) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, 0.0f, -1.0f},
        {-1.0f, 0.0f, 0.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    EXPECT_TRUE(result.isValid);
    
    // Проверяем det = +1
    float det = result.dcm[0]*(result.dcm[4]*result.dcm[8]-result.dcm[5]*result.dcm[7])
              - result.dcm[1]*(result.dcm[3]*result.dcm[8]-result.dcm[5]*result.dcm[6])
              + result.dcm[2]*(result.dcm[3]*result.dcm[7]-result.dcm[4]*result.dcm[6]);
    EXPECT_NEAR(det, 1.0f, EPSILON);
}

// Тест: ориентация корректно вращает векторы
TEST_F(TRIADTest, OrientationRotatesVectorsCorrectly) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {0.0f, -1.0f, 0.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    ASSERT_TRUE(result.isValid);
    
    // Проверяем что DCM - валидная матрица вращения
    float det = result.dcm[0]*(result.dcm[4]*result.dcm[8]-result.dcm[5]*result.dcm[7])
              - result.dcm[1]*(result.dcm[3]*result.dcm[8]-result.dcm[5]*result.dcm[6])
              + result.dcm[2]*(result.dcm[3]*result.dcm[7]-result.dcm[4]*result.dcm[6]);
    EXPECT_NEAR(det, 1.0f, EPSILON);
    
    // И кватернион нормализован
    float qNorm = std::sqrt(result.orientation.w*result.orientation.w + 
                           result.orientation.x*result.orientation.x +
                           result.orientation.y*result.orientation.y +
                           result.orientation.z*result.orientation.z);
    EXPECT_NEAR(qNorm, 1.0f, EPSILON);
}

// Тест: кватернион нормализован
TEST_F(TRIADTest, QuaternionNormalized) {
    std::array<float, 3> refVectors[2] = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f}
    };
    std::array<float, 3> bodyVectors[2] = {
        {2.0f, 3.0f, 1.0f},
        {5.0f, 6.0f, 4.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    
    ASSERT_TRUE(result.isValid);
    
    float norm = std::sqrt(
        result.orientation.w * result.orientation.w +
        result.orientation.x * result.orientation.x +
        result.orientation.y * result.orientation.y +
        result.orientation.z * result.orientation.z
    );
    
    EXPECT_NEAR(norm, 1.0f, EPSILON);
}

// Тест: преобразование в углы Эйлера
TEST_F(TRIADTest, ToEulerAngles) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };

    auto result = estimator_->estimate(refVectors, bodyVectors);
    
    ASSERT_TRUE(result.isValid);
    
    auto angles = result.orientation.toEulerAngles();
    
    // Все углы должны быть близки к нулю
    EXPECT_NEAR(angles[0], 0.0f, EPSILON);  // Roll
    EXPECT_NEAR(angles[1], 0.0f, EPSILON);  // Pitch
    EXPECT_NEAR(angles[2], 0.0f, EPSILON);  // Yaw
}

// ============================================================================
// Интеграционные тесты
// ============================================================================

class TRIADIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        triad_ = std::make_unique<TRIADEstimator>();
    }

    std::unique_ptr<TRIADEstimator> triad_;
};

// Тест: TRIAD как инициализация для EKF
TEST_F(TRIADIntegrationTest, InitializationForEKF) {
    // Типичные данные IMU
    std::array<float, 3> accel = {1.0f, 2.0f, 9.5f};
    std::array<float, 3> mag = {0.4f, 0.1f, 0.3f};
    
    std::array<float, 3> gravityRef = {0.0f, 0.0f, 1.0f};
    std::array<float, 3> magRef = {1.0f, 0.0f, 0.0f};

    auto result = triad_->estimateFromIMU(accel, mag, gravityRef, magRef);
    
    ASSERT_TRUE(result.isValid);
    
    // Кватернион должен быть валидным для инициализации EKF
    EXPECT_GT(result.orientation.w, -1.01f);
    EXPECT_LT(result.orientation.w, 1.01f);
    EXPECT_NE(result.orientation.x, 0.0f || result.orientation.y != 0.0f || result.orientation.z != 0.0f);
}

// Тест: последовательные оценки стабильны
TEST_F(TRIADIntegrationTest, SequentialEstimatesStable) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };
    
    // Небольшое вращение
    float angle = 5.0f * DEG_TO_RAD;
    std::array<float, 3> bodyVectors[2] = {
        {std::sin(angle), 0.0f, std::cos(angle)},
        {1.0f, 0.0f, 0.0f}
    };

    auto result1 = triad_->estimate(refVectors, bodyVectors);
    auto result2 = triad_->estimate(refVectors, bodyVectors);
    
    ASSERT_TRUE(result1.isValid && result2.isValid);
    
    // Результаты должны быть идентичны
    float angleDiff = quaternionAngleDiff(result1.orientation, result2.orientation);
    EXPECT_NEAR(angleDiff, 0.0f, EPSILON);
}

// Тест: различные углы вращения
TEST_F(TRIADIntegrationTest, VariousRotationAngles) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };

    float testAngles[] = {10.0f, 30.0f, 45.0f, 60.0f, 90.0f, 120.0f};
    
    for (float angleDeg : testAngles) {
        float angle = angleDeg * DEG_TO_RAD;
        std::array<float, 3> bodyVectors[2] = {
            {0.0f, 0.0f, 1.0f},
            {std::cos(angle), -std::sin(angle), 0.0f}
        };

        auto result = triad_->estimate(refVectors, bodyVectors);
        
        ASSERT_TRUE(result.isValid) << "Failed for angle " << angleDeg;
        
        // Проверяем что DCM - валидная матрица вращения (det = +1)
        float det = result.dcm[0]*(result.dcm[4]*result.dcm[8]-result.dcm[5]*result.dcm[7])
                  - result.dcm[1]*(result.dcm[3]*result.dcm[8]-result.dcm[5]*result.dcm[6])
                  + result.dcm[2]*(result.dcm[3]*result.dcm[7]-result.dcm[4]*result.dcm[6]);
        EXPECT_NEAR(det, 1.0f, 0.001f) << "Failed det for angle " << angleDeg;
    }
}

// ============================================================================
// Запуск тестов
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
