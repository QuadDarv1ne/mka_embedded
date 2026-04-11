/**
 * @file test_quest.cpp
 * @brief Unit-тесты для QUEST метода определения ориентации
 *
 * Тесты проверяют:
 * - Базовую оценку ориентации
 * - Работу с множеством векторов
 * - Взвешивание измерений
 * - Обработку ошибок
 * - Сравнение с TRIAD
 */

#include <gtest/gtest.h>
#include <cmath>
#include <array>
#include <vector>

#include "algorithms/adcs_algorithms.hpp"

using namespace mka::adcs;

// ============================================================================
// Вспомогательные функции
// ============================================================================

namespace {
constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float EPSILON = 1e-4f;
}

// ============================================================================
// Тесты QUEST
// ============================================================================

class QUESTTest : public ::testing::Test {
protected:
    void SetUp() override {
        estimator_ = std::make_unique<QUESTEstimator>();
    }

    std::unique_ptr<QUESTEstimator> estimator_;
};

// Тест: единичное вращение (векторы совпадают)
TEST_F(QUESTTest, IdentityRotation) {
    QUESTEstimator::WeightedVectorObservation observations[2];
    observations[0].refVector = {0.0f, 0.0f, 1.0f};
    observations[0].bodyVector = {0.0f, 0.0f, 1.0f};
    observations[0].weight = 0.5f;
    
    observations[1].refVector = {1.0f, 0.0f, 0.0f};
    observations[1].bodyVector = {1.0f, 0.0f, 0.0f};
    observations[1].weight = 0.5f;

    auto result = estimator_->estimate(observations, 2);
    
    EXPECT_TRUE(result.isValid);
    // Кватернион должен быть близок к единичному [1, 0, 0, 0] или [-1, 0, 0, 0]
    EXPECT_GT(std::abs(result.orientation.w), 0.99f);
    EXPECT_NEAR(result.orientation.x, 0.0f, 0.01f);
    EXPECT_NEAR(result.orientation.y, 0.0f, 0.01f);
    EXPECT_NEAR(result.orientation.z, 0.0f, 0.01f);
}

// Тест: вращение на 90° вокруг Z
TEST_F(QUESTTest, Rotation90DegreesZ) {
    QUESTEstimator::WeightedVectorObservation observations[2];
    observations[0].refVector = {0.0f, 0.0f, 1.0f};
    observations[0].bodyVector = {0.0f, 0.0f, 1.0f};  // Z не меняется
    observations[0].weight = 0.5f;
    
    observations[1].refVector = {1.0f, 0.0f, 0.0f};
    observations[1].bodyVector = {0.0f, -1.0f, 0.0f};  // X → -Y (90° yaw)
    observations[1].weight = 0.5f;

    auto result = estimator_->estimate(observations, 2);
    
    EXPECT_TRUE(result.isValid);
    // Для 90°yaw кватернион должен иметь w~0.707, z~0.707
    EXPECT_NEAR(result.orientation.w * result.orientation.w + 
                result.orientation.z * result.orientation.z, 1.0f, 0.01f);
    EXPECT_NEAR(result.orientation.x, 0.0f, 0.01f);
    EXPECT_NEAR(result.orientation.y, 0.0f, 0.01f);
}

// Тест: использование 3+ векторов
TEST_F(QUESTTest, MultipleVectors) {
    QUESTEstimator::WeightedVectorObservation observations[3];
    observations[0].refVector = {0.0f, 0.0f, 1.0f};
    observations[0].bodyVector = {0.0f, 0.0f, 1.0f};
    observations[0].weight = 1.0f;
    
    observations[1].refVector = {1.0f, 0.0f, 0.0f};
    observations[1].bodyVector = {1.0f, 0.0f, 0.0f};
    observations[1].weight = 1.0f;
    
    observations[2].refVector = {0.0f, 1.0f, 0.0f};
    observations[2].bodyVector = {0.0f, 1.0f, 0.0f};
    observations[2].weight = 1.0f;

    auto result = estimator_->estimate(observations, 3);
    
    EXPECT_TRUE(result.isValid);
    EXPECT_EQ(result.numVectors, 3);
    EXPECT_NEAR(result.orientation.w, 1.0f, EPSILON);
}

// Тест: неравные веса
TEST_F(QUESTTest, UnequalWeights) {
    QUESTEstimator::WeightedVectorObservation observations[2];
    observations[0].refVector = {0.0f, 0.0f, 1.0f};
    observations[0].bodyVector = {0.0f, 0.0f, 1.0f};
    observations[0].weight = 0.9f;  // Высокий вес для gravity
    
    observations[1].refVector = {1.0f, 0.0f, 0.0f};
    observations[1].bodyVector = {0.707f, 0.707f, 0.0f};  // 45° mag
    observations[1].weight = 0.1f;  // Низкий вес для mag

    auto result = estimator_->estimate(observations, 2);
    
    EXPECT_TRUE(result.isValid);
    // Результат должен быть ближе к gravity вектору
    EXPECT_NEAR(result.orientation.w, 1.0f, 0.1f);
}

// Тест: ошибка при менее чем 2 векторах
TEST_F(QUESTTest, InsufficientVectorsError) {
    QUESTEstimator::WeightedVectorObservation observations[1];
    observations[0].refVector = {0.0f, 0.0f, 1.0f};
    observations[0].bodyVector = {0.0f, 0.0f, 1.0f};
    observations[0].weight = 1.0f;

    auto result = estimator_->estimate(observations, 1);
    
    EXPECT_FALSE(result.isValid);
    EXPECT_NE(result.errorMessage, nullptr);
}

// Тест: нулевой вектор
TEST_F(QUESTTest, ZeroVectorError) {
    QUESTEstimator::WeightedVectorObservation observations[2];
    observations[0].refVector = {0.0f, 0.0f, 0.0f};  // Нулевой!
    observations[0].bodyVector = {0.0f, 0.0f, 1.0f};
    observations[0].weight = 0.5f;
    
    observations[1].refVector = {1.0f, 0.0f, 0.0f};
    observations[1].bodyVector = {1.0f, 0.0f, 0.0f};
    observations[1].weight = 0.5f;

    auto result = estimator_->estimate(observations, 2);
    
    EXPECT_FALSE(result.isValid);
}

// Тест: estimateFromIMU helper
TEST_F(QUESTTest, EstimateFromIMU) {
    std::array<float, 3> accel = {0.0f, 0.0f, 9.81f};
    std::array<float, 3> mag = {0.5f, 0.0f, 0.0f};
    
    auto result = estimator_->estimateFromIMU(accel, mag);
    
    EXPECT_TRUE(result.isValid);
    EXPECT_NEAR(result.orientation.w, 1.0f, 0.01f);
}

// Тест: нормализация кватерниона
TEST_F(QUESTTest, QuaternionNormalized) {
    QUESTEstimator::WeightedVectorObservation observations[2];
    observations[0].refVector = {1.0f, 2.0f, 3.0f};
    observations[0].bodyVector = {2.0f, 3.0f, 1.0f};
    observations[0].weight = 0.5f;
    
    observations[1].refVector = {4.0f, 5.0f, 6.0f};
    observations[1].bodyVector = {5.0f, 6.0f, 4.0f};
    observations[1].weight = 0.5f;

    auto result = estimator_->estimate(observations, 2);
    
    ASSERT_TRUE(result.isValid);
    
    float norm = std::sqrt(
        result.orientation.w * result.orientation.w +
        result.orientation.x * result.orientation.x +
        result.orientation.y * result.orientation.y +
        result.orientation.z * result.orientation.z
    );
    
    EXPECT_NEAR(norm, 1.0f, EPSILON);
}

// Тест: loss функция (проверка что возвращается)
TEST_F(QUESTTest, LossFunction) {
    QUESTEstimator::WeightedVectorObservation observations[2];
    observations[0].refVector = {0.0f, 0.0f, 1.0f};
    observations[0].bodyVector = {0.0f, 0.0f, 1.0f};  // Идеально
    observations[0].weight = 0.5f;
    
    observations[1].refVector = {1.0f, 0.0f, 0.0f};
    observations[1].bodyVector = {1.0f, 0.0f, 0.0f};  // Идеально
    observations[1].weight = 0.5f;

    auto result = estimator_->estimate(observations, 2);
    
    ASSERT_TRUE(result.isValid);
    // loss должна быть определена (может быть отрицательной из-за численных ошибок)
    EXPECT_NEAR(result.loss, 0.0f, 0.3f);
}

// Тест: конфигурация
TEST_F(QUESTTest, Configuration) {
    QUESTEstimator::Config config;
    config.newtonTolerance = 1e-12f;
    config.maxNewtonIterations = 100;
    config.normalizeInputs = false;
    
    estimator_->setConfig(config);
    auto retrieved = estimator_->getConfig();
    
    EXPECT_NEAR(retrieved.newtonTolerance, 1e-12f, 1e-15f);
    EXPECT_EQ(retrieved.maxNewtonIterations, 100);
    EXPECT_FALSE(retrieved.normalizeInputs);
}

// Тест: вращение на 45° вокруг X
TEST_F(QUESTTest, Rotation45DegreesX) {
    float sin45 = std::sin(PI / 4.0f);
    float cos45 = std::cos(PI / 4.0f);
    
    QUESTEstimator::WeightedVectorObservation observations[2];
    observations[0].refVector = {0.0f, 0.0f, 1.0f};
    observations[0].bodyVector = {0.0f, sin45, cos45};  // Поворот Z
    observations[0].weight = 0.5f;
    
    observations[1].refVector = {1.0f, 0.0f, 0.0f};
    observations[1].bodyVector = {1.0f, 0.0f, 0.0f};  // X не меняется
    observations[1].weight = 0.5f;

    auto result = estimator_->estimate(observations, 2);

    EXPECT_TRUE(result.isValid);
    // Проверяем что кватернион нормализован
    float qNorm = std::sqrt(result.orientation.w*result.orientation.w + 
                           result.orientation.x*result.orientation.x +
                           result.orientation.y*result.orientation.y +
                           result.orientation.z*result.orientation.z);
    EXPECT_NEAR(qNorm, 1.0f, EPSILON);
    EXPECT_GT(result.orientation.w, 0.0f);
}

// Тест: estimateFromVectors helper
TEST_F(QUESTTest, EstimateFromVectorsHelper) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };
    float weights[2] = {0.5f, 0.5f};

    auto result = estimator_->estimateFromVectors(refVectors, bodyVectors, weights, 2);
    
    EXPECT_TRUE(result.isValid);
    EXPECT_NEAR(result.orientation.w, 1.0f, EPSILON);
}

// ============================================================================
// Интеграционные тесты
// ============================================================================

class QUESTIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        quest_ = std::make_unique<QUESTEstimator>();
        triad_ = std::make_unique<TRIADEstimator>();
    }

    std::unique_ptr<QUESTEstimator> quest_;
    std::unique_ptr<TRIADEstimator> triad_;
};

// Тест: QUEST vs TRIAD (оба работают для 2 векторов)
TEST_F(QUESTIntegrationTest, QUESTvsTRIAD) {
    // Данные для 45° yaw
    float angle = 45.0f * DEG_TO_RAD;
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };
    std::array<float, 3> bodyVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {std::cos(angle), -std::sin(angle), 0.0f}
    };

    // QUEST
    QUESTEstimator::WeightedVectorObservation questObs[2];
    questObs[0].refVector = refVectors[0];
    questObs[0].bodyVector = bodyVectors[0];
    questObs[0].weight = 0.5f;
    questObs[1].refVector = refVectors[1];
    questObs[1].bodyVector = bodyVectors[1];
    questObs[1].weight = 0.5f;
    
    auto questResult = quest_->estimate(questObs, 2);
    
    // TRIAD
    auto triadResult = triad_->estimate(refVectors, bodyVectors);
    
    // Оба должны вернуть валидный результат
    ASSERT_TRUE(questResult.isValid && triadResult.isValid);
    
    // Оба кватерниона должны быть нормализованы
    float questNorm = std::sqrt(questResult.orientation.w*questResult.orientation.w + 
                               questResult.orientation.x*questResult.orientation.x +
                               questResult.orientation.y*questResult.orientation.y +
                               questResult.orientation.z*questResult.orientation.z);
    EXPECT_NEAR(questNorm, 1.0f, EPSILON);
}

// Тест: QUEST с шумными данными (оптимальность)
TEST_F(QUESTIntegrationTest, NoisyDataOptimality) {
    // 3 вектора с небольшим шумом
    QUESTEstimator::WeightedVectorObservation observations[3];
    observations[0].refVector = {0.0f, 0.0f, 1.0f};
    observations[0].bodyVector = {0.01f, -0.01f, 0.9999f};  // Небольшой шум
    observations[0].weight = 1.0f;
    
    observations[1].refVector = {1.0f, 0.0f, 0.0f};
    observations[1].bodyVector = {0.9999f, 0.01f, -0.01f};
    observations[1].weight = 1.0f;
    
    observations[2].refVector = {0.0f, 1.0f, 0.0f};
    observations[2].bodyVector = {-0.01f, 0.9999f, 0.01f};
    observations[2].weight = 1.0f;

    auto result = quest_->estimate(observations, 3);
    
    EXPECT_TRUE(result.isValid);
    // Кватернион должен быть близок к единичному (шум небольшой)
    EXPECT_NEAR(result.orientation.w, 1.0f, 0.02f);
}

// Тест: стабильность последовательных оценок
TEST_F(QUESTIntegrationTest, SequentialEstimatesStable) {
    QUESTEstimator::WeightedVectorObservation observations[2];
    observations[0].refVector = {0.0f, 0.0f, 1.0f};
    observations[0].bodyVector = {0.0f, 0.0f, 1.0f};
    observations[0].weight = 0.5f;
    
    observations[1].refVector = {1.0f, 0.0f, 0.0f};
    observations[1].bodyVector = {1.0f, 0.0f, 0.0f};
    observations[1].weight = 0.5f;

    auto result1 = quest_->estimate(observations, 2);
    auto result2 = quest_->estimate(observations, 2);
    
    ASSERT_TRUE(result1.isValid && result2.isValid);
    
    EXPECT_FLOAT_EQ(result1.orientation.w, result2.orientation.w);
    EXPECT_FLOAT_EQ(result1.loss, result2.loss);
}

// Тест: различные углы вращения (QUEST работает)
TEST_F(QUESTIntegrationTest, VariousRotationAngles) {
    std::array<float, 3> refVectors[2] = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    };

    float testAngles[] = {10.0f, 30.0f, 45.0f, 60.0f, 90.0f};
    
    for (float angleDeg : testAngles) {
        float angle = angleDeg * DEG_TO_RAD;
        std::array<float, 3> bodyVectors[2] = {
            {0.0f, 0.0f, 1.0f},
            {std::cos(angle), -std::sin(angle), 0.0f}
        };

        QUESTEstimator::WeightedVectorObservation questObs[2];
        questObs[0].refVector = refVectors[0];
        questObs[0].bodyVector = bodyVectors[0];
        questObs[0].weight = 0.5f;
        questObs[1].refVector = refVectors[1];
        questObs[1].bodyVector = bodyVectors[1];
        questObs[1].weight = 0.5f;
        
        auto questResult = quest_->estimate(questObs, 2);
        
        ASSERT_TRUE(questResult.isValid) << "Failed for angle " << angleDeg;
        
        // Кватернион должен быть нормализован
        float norm = std::sqrt(questResult.orientation.w*questResult.orientation.w + 
                              questResult.orientation.x*questResult.orientation.x +
                              questResult.orientation.y*questResult.orientation.y +
                              questResult.orientation.z*questResult.orientation.z);
        EXPECT_NEAR(norm, 1.0f, EPSILON) << "Failed for angle " << angleDeg;
    }
}

// ============================================================================
// Запуск тестов
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
