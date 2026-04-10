/**
 * @file test_anomaly_detector.cpp
 * @brief Unit tests for Isolation Forest Anomaly Detector
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <random>

#include "algorithms/anomaly_detector.hpp"

using namespace mka::ml;

// ============================================================================
// Тесты инициализации
// ============================================================================

TEST(AnomalyDetectorTest, Initialization) {
    AnomalyDetector detector;
    AnomalyDetectorConfig config;
    config.numTrees = 100;
    config.sampleSize = 256;
    config.contamination = 0.1f;
    
    detector.configure(config);
    
    EXPECT_FALSE(detector.isReady());  // Ещё не обучен
}

// ============================================================================
// Тесты обучения (fit)
// ============================================================================

TEST(AnomalyDetectorTest, FitNormalData) {
    AnomalyDetector detector;
    AnomalyDetectorConfig config;
    config.numTrees = 50;
    config.sampleSize = 128;
    detector.configure(config);
    
    std::vector<std::vector<float>> data;
    std::mt19937 rng(42);
    std::normal_distribution<float> dist(0.0f, 1.0f);
    
    for (int i = 0; i < 200; i++) {
        data.push_back({dist(rng), dist(rng), dist(rng)});
    }
    
    bool success = detector.fit(data);
    EXPECT_TRUE(success);
    EXPECT_TRUE(detector.isReady());
}

TEST(AnomalyDetectorTest, FitEmptyData) {
    AnomalyDetector detector;
    std::vector<std::vector<float>> emptyData;
    
    bool success = detector.fit(emptyData);
    EXPECT_FALSE(success);
}

// ============================================================================
// Тесты детекции (detect)
// ============================================================================

TEST(AnomalyDetectorTest, DetectNormalPoint) {
    AnomalyDetector detector;
    AnomalyDetectorConfig config;
    config.numTrees = 50;
    config.sampleSize = 128;
    detector.configure(config);
    
    std::vector<std::vector<float>> data;
    for (int i = 0; i < 100; i++) {
        data.push_back({0.0f, 0.0f, 0.0f});
    }
    detector.fit(data);
    
    AnomalyResult result = detector.detect({0.0f, 0.0f, 0.0f});
    EXPECT_LT(result.anomalyScore, 0.6f);
    EXPECT_FALSE(result.isAnomaly);
}

TEST(AnomalyDetectorTest, DetectAnomalousPoint) {
    AnomalyDetector detector;
    AnomalyDetectorConfig config;
    config.numTrees = 50;
    config.sampleSize = 128;
    detector.configure(config);
    
    std::vector<std::vector<float>> data;
    for (int i = 0; i < 100; i++) {
        data.push_back({0.0f, 0.0f, 0.0f});
    }
    detector.fit(data);
    
    AnomalyResult result = detector.detect({100.0f, 100.0f, 100.0f});
    EXPECT_GT(result.anomalyScore, 0.4f);
}

// ============================================================================
// Тесты partialFit (онлайн обучение)
// ============================================================================

TEST(AnomalyDetectorTest, PartialFit) {
    AnomalyDetector detector;
    AnomalyDetectorConfig config;
    config.numTrees = 50;
    config.sampleSize = 128;
    detector.configure(config);
    
    std::vector<std::vector<float>> initialData;
    for (int i = 0; i < 50; i++) {
        initialData.push_back({0.0f, 0.0f, 0.0f});
    }
    detector.fit(initialData);
    
    detector.partialFit({0.1f, 0.1f, 0.1f});
    EXPECT_TRUE(detector.isReady());
}

// ============================================================================
// Тесты статистики
// ============================================================================

TEST(AnomalyDetectorTest, GetStatistics) {
    AnomalyDetector detector;
    AnomalyDetectorConfig config;
    config.numTrees = 50;
    config.sampleSize = 128;
    detector.configure(config);
    
    std::vector<std::vector<float>> data;
    for (int i = 0; i < 100; i++) {
        data.push_back({0.0f, 0.0f, 0.0f});
    }
    detector.fit(data);
    
    detector.detect({0.0f, 0.0f, 0.0f});
    
    auto stats = detector.getStatistics();
    EXPECT_GE(stats.totalSamples, 100);
}

// ============================================================================
// Тесты граничных случаев
// ============================================================================

TEST(AnomalyDetectorTest, SingleDimension) {
    AnomalyDetector detector;
    AnomalyDetectorConfig config;
    config.numTrees = 50;
    config.sampleSize = 128;
    detector.configure(config);
    
    std::vector<std::vector<float>> data;
    for (int i = 0; i < 100; i++) {
        data.push_back({static_cast<float>(i)});
    }
    detector.fit(data);
    
    AnomalyResult result = detector.detect({50.0f});
    EXPECT_GE(result.anomalyScore, 0.0f);
    EXPECT_LE(result.anomalyScore, 1.0f);
}

TEST(AnomalyDetectorTest, NegativeValues) {
    AnomalyDetector detector;
    AnomalyDetectorConfig config;
    config.numTrees = 50;
    config.sampleSize = 128;
    detector.configure(config);
    
    std::vector<std::vector<float>> data;
    for (int i = 0; i < 100; i++) {
        data.push_back({-1.0f, -2.0f, -3.0f});
    }
    detector.fit(data);
    
    AnomalyResult normalResult = detector.detect({-1.0f, -2.0f, -3.0f});
    AnomalyResult anomalousResult = detector.detect({100.0f, 100.0f, 100.0f});
    
    EXPECT_LT(normalResult.anomalyScore, anomalousResult.anomalyScore);
}
