/**
 * @file test_fdir.cpp
 * @brief Unit tests for FDIR system
 */

#include <gtest/gtest.h>
#include <cmath>

#include "systems/fdir.hpp"

using namespace mka::fdir;

// ============================================================================
// Тесты ParameterStats
// ============================================================================

TEST(ParameterStatsTest, InitialState) {
    ParameterStats stats;
    EXPECT_FLOAT_EQ(stats.mean, 0.0f);
    EXPECT_FLOAT_EQ(stats.variance, 0.0f);
    EXPECT_FLOAT_EQ(stats.min, 0.0f);
    EXPECT_FLOAT_EQ(stats.max, 0.0f);
    EXPECT_EQ(stats.count, 0u);
    EXPECT_EQ(stats.anomalyCount, 0u);
}

TEST(ParameterStatsTest, Update) {
    ParameterStats stats;
    stats.update(10.0f);
    stats.update(20.0f);
    stats.update(30.0f);
    
    EXPECT_FLOAT_EQ(stats.mean, 20.0f);
    EXPECT_FLOAT_EQ(stats.min, 10.0f);
    EXPECT_FLOAT_EQ(stats.max, 30.0f);
    EXPECT_EQ(stats.count, 3u);
}

TEST(ParameterStatsTest, StdDev) {
    ParameterStats stats;
    // Значения: 2, 4, 4, 4, 5, 5, 7, 9
    stats.update(2.0f);
    stats.update(4.0f);
    stats.update(4.0f);
    stats.update(4.0f);
    stats.update(5.0f);
    stats.update(5.0f);
    stats.update(7.0f);
    stats.update(9.0f);
    
    float stddev = stats.stddev();
    EXPECT_GT(stddev, 2.0f);
    EXPECT_LT(stddev, 3.0f);
}

TEST(ParameterStatsTest, IsAnomaly) {
    ParameterStats stats;
    
    // Недостаточно данных
    EXPECT_FALSE(stats.isAnomaly(100.0f));
    
    // Добавляем данные
    for (int i = 0; i < 20; i++) {
        stats.update(10.0f + (i % 5) * 0.1f);
    }
    
    // Нормальное значение
    EXPECT_FALSE(stats.isAnomaly(10.0f));
    
    // Аномальное значение (далеко от среднего)
    EXPECT_TRUE(stats.isAnomaly(100.0f));
}

TEST(ParameterStatsTest, Reset) {
    ParameterStats stats;
    stats.update(10.0f);
    stats.update(20.0f);
    
    stats.reset();
    
    EXPECT_FLOAT_EQ(stats.mean, 0.0f);
    EXPECT_EQ(stats.count, 0u);
}

// ============================================================================
// Тесты ParameterMonitor
// ============================================================================

TEST(ParameterMonitorTest, NormalValue) {
    ParameterConfig config{
        .nominalValue = 7.4f,
        .warningLow = 6.5f,
        .warningHigh = 8.5f,
        .errorLow = 6.0f,
        .errorHigh = 9.0f,
        .criticalLow = 5.5f,
        .criticalHigh = 10.0f,
        .debounceMs = 0,
        .subsystem = 0,
        .parameterId = 0
    };
    
    ParameterMonitor monitor(config);
    
    // Нормальное значение
    Severity result = monitor.check(7.4f, 100);
    EXPECT_EQ(result, Severity::INFO);
}

TEST(ParameterMonitorTest, WarningThreshold) {
    ParameterConfig config{
        .nominalValue = 7.4f,
        .warningLow = 6.5f,
        .warningHigh = 8.5f,
        .errorLow = 6.0f,
        .errorHigh = 9.0f,
        .criticalLow = 5.5f,
        .criticalHigh = 10.0f,
        .debounceMs = 0,
        .subsystem = 0,
        .parameterId = 0
    };
    
    ParameterMonitor monitor(config);
    
    Severity result = monitor.check(6.0f, 100);
    EXPECT_EQ(result, Severity::WARNING);
}

TEST(ParameterMonitorTest, ErrorThreshold) {
    ParameterConfig config{
        .nominalValue = 7.4f,
        .warningLow = 6.5f,
        .warningHigh = 8.5f,
        .errorLow = 6.0f,
        .errorHigh = 9.0f,
        .criticalLow = 5.5f,
        .criticalHigh = 10.0f,
        .debounceMs = 0,
        .subsystem = 0,
        .parameterId = 0
    };
    
    ParameterMonitor monitor(config);
    
    Severity result = monitor.check(5.5f, 100);
    EXPECT_EQ(result, Severity::ERROR);
}

TEST(ParameterMonitorTest, CriticalThreshold) {
    ParameterConfig config{
        .nominalValue = 7.4f,
        .warningLow = 6.5f,
        .warningHigh = 8.5f,
        .errorLow = 6.0f,
        .errorHigh = 9.0f,
        .criticalLow = 5.5f,
        .criticalHigh = 10.0f,
        .debounceMs = 0,
        .subsystem = 0,
        .parameterId = 0
    };
    
    ParameterMonitor monitor(config);
    
    Severity result = monitor.check(5.0f, 100);
    EXPECT_EQ(result, Severity::CRITICAL);
}

TEST(ParameterMonitorTest, Debounce) {
    ParameterConfig config{
        .nominalValue = 7.4f,
        .warningLow = 6.5f,
        .warningHigh = 8.5f,
        .errorLow = 6.0f,
        .errorHigh = 9.0f,
        .criticalLow = 5.5f,
        .criticalHigh = 10.0f,
        .debounceMs = 100,  // 100ms debounce
        .subsystem = 0,
        .parameterId = 0
    };
    
    ParameterMonitor monitor(config);
    
    // Быстрое изменение - не должно пройти debounce
    monitor.check(6.0f, 50);
    Severity result = monitor.check(6.0f, 100);  // Всего 50ms
    
    EXPECT_EQ(result, Severity::INFO);  // Ещё не прошло debounce
}

TEST(ParameterMonitorTest, CheckAnomaly) {
    ParameterConfig config{};
    ParameterMonitor monitor(config);
    
    // Недостаточно данных
    EXPECT_FALSE(monitor.checkAnomaly(10.0f));
    
    // Добавляем данные
    for (int i = 0; i < 20; i++) {
        monitor.check(10.0f + (i % 3) * 0.1f, i * 10);
    }
    
    EXPECT_FALSE(monitor.checkAnomaly(10.0f));
    EXPECT_TRUE(monitor.checkAnomaly(100.0f));
}

TEST(ParameterMonitorTest, Reset) {
    ParameterConfig config{};
    ParameterMonitor monitor(config);
    
    for (int i = 0; i < 10; i++) {
        monitor.check(10.0f, i * 10);
    }
    
    monitor.reset();
    
    const auto& stats = monitor.getStats();
    EXPECT_EQ(stats.count, 0u);
}

// ============================================================================
// Тесты FDIRManager
// ============================================================================

TEST(FDIRManagerTest, RegisterParameter) {
    FDIRManager manager;
    
    ParameterConfig config{
        .nominalValue = 7.4f,
        .warningLow = 6.5f,
        .warningHigh = 8.5f
    };
    
    uint8_t id = manager.registerParameter(config);
    EXPECT_LT(id, 64u);
}

TEST(FDIRManagerTest, UpdateParameter) {
    FDIRManager manager;
    
    ParameterConfig config{
        .nominalValue = 7.4f,
        .warningLow = 6.5f,
        .warningHigh = 8.5f,
        .errorLow = 6.0f,
        .errorHigh = 9.0f,
        .criticalLow = 5.5f,
        .criticalHigh = 10.0f
    };
    
    uint8_t id = manager.registerParameter(config);
    
    Severity result = manager.updateParameter(id, 7.4f, 100);
    EXPECT_EQ(result, Severity::INFO);
    
    result = manager.updateParameter(id, 5.0f, 200);
    EXPECT_EQ(result, Severity::CRITICAL);
}

TEST(FDIRManagerTest, EventLogging) {
    FDIRManager manager;
    
    manager.reportError(ErrorCode::EPS_BATTERY_LOW, Severity::ERROR, 
                       Subsystem::EPS, 1, 55, 65);
    
    EXPECT_EQ(manager.getEventCount(), 1u);
    
    EventLogEntry entry;
    bool found = manager.getEventLogEntry(0, entry);
    EXPECT_TRUE(found);
    EXPECT_EQ(entry.code, ErrorCode::EPS_BATTERY_LOW);
    EXPECT_EQ(entry.severity, Severity::ERROR);
    EXPECT_EQ(entry.subsystem, Subsystem::EPS);
}

TEST(FDIRManagerTest, ClearEventLog) {
    FDIRManager manager;
    
    manager.reportError(ErrorCode::EPS_BATTERY_LOW, Severity::ERROR, Subsystem::EPS);
    manager.clearEventLog();
    
    EXPECT_EQ(manager.getEventCount(), 0u);
}

TEST(FDIRManagerTest, ResetAllMonitors) {
    FDIRManager manager;
    
    ParameterConfig config{
        .nominalValue = 7.4f,
        .warningLow = 6.5f,
        .warningHigh = 8.5f
    };
    
    uint8_t id = manager.registerParameter(config);
    manager.updateParameter(id, 5.0f, 100);  // Нарушение
    
    manager.resetAllMonitors();
    
    Severity result = manager.updateParameter(id, 5.0f, 200);
    EXPECT_EQ(result, Severity::INFO);  // Сброшено
}

// ============================================================================
// Тесты FrozenValueDetector
// ============================================================================

TEST(FrozenValueDetectorTest, NormalOperation) {
    FrozenValueDetector detector(5);
    
    // Изменяющиеся значения
    EXPECT_FALSE(detector.check(1.0f));
    EXPECT_FALSE(detector.check(2.0f));
    EXPECT_FALSE(detector.check(3.0f));
    EXPECT_FALSE(detector.check(4.0f));
    EXPECT_FALSE(detector.check(5.0f));
}

TEST(FrozenValueDetectorTest, FrozenValueDetected) {
    FrozenValueDetector detector(3);
    
    EXPECT_FALSE(detector.check(10.0f));
    EXPECT_FALSE(detector.check(10.0f));  // 2 раза
    EXPECT_TRUE(detector.check(10.0f));   // 3 раза - заморозка
    EXPECT_TRUE(detector.check(10.0f));   // Продолжает
}

TEST(FrozenValueDetectorTest, Reset) {
    FrozenValueDetector detector(3);
    
    detector.check(10.0f);
    detector.check(10.0f);
    detector.reset();
    detector.check(10.0f);
    
    EXPECT_FALSE(detector.check(10.0f));  // Сброшено
}

// ============================================================================
// Тесты StuckBitDetector
// ============================================================================

TEST(StuckBitDetectorTest, NormalOperation) {
    StuckBitDetector detector(2, 10);
    
    // Частые переключения
    EXPECT_FALSE(detector.check(true));
    EXPECT_FALSE(detector.check(false));
    EXPECT_FALSE(detector.check(true));
    EXPECT_FALSE(detector.check(false));
    EXPECT_FALSE(detector.check(true));
    EXPECT_FALSE(detector.check(false));
    EXPECT_FALSE(detector.check(true));
    EXPECT_FALSE(detector.check(false));
    EXPECT_FALSE(detector.check(true));
    EXPECT_FALSE(detector.check(false));
}

TEST(StuckBitDetectorTest, StuckBitDetected) {
    StuckBitDetector detector(2, 5);
    
    // Все значения одинаковые
    EXPECT_FALSE(detector.check(true));
    EXPECT_FALSE(detector.check(true));
    EXPECT_FALSE(detector.check(true));
    EXPECT_FALSE(detector.check(true));
    EXPECT_FALSE(detector.check(true));  // 5 - недостаточно переходов
    EXPECT_TRUE(detector.check(true));   // Застрявший бит
}

TEST(StuckBitDetectorTest, Reset) {
    StuckBitDetector detector(2, 5);
    
    detector.check(true);
    detector.check(true);
    detector.reset();
    
    EXPECT_FALSE(detector.check(true));
    EXPECT_FALSE(detector.check(true));
}

// ============================================================================
// Тесты GlitchDetector
// ============================================================================

TEST(GlitchDetectorTest, NormalOperation) {
    GlitchDetector detector(5.0f, 3);
    
    // Плавные изменения
    EXPECT_FALSE(detector.check(1.0f));
    EXPECT_FALSE(detector.check(1.5f));
    EXPECT_FALSE(detector.check(2.0f));
    EXPECT_FALSE(detector.check(2.5f));
}

TEST(GlitchDetectorTest, GlitchDetected) {
    GlitchDetector detector(2.0f, 3);
    
    EXPECT_FALSE(detector.check(1.0f));
    EXPECT_FALSE(detector.check(10.0f));  // Глитч 1
    EXPECT_FALSE(detector.check(10.0f));  // Глитч 2
    EXPECT_TRUE(detector.check(10.0f));   // Подтверждённый глитч
}

TEST(GlitchDetectorTest, Reset) {
    GlitchDetector detector(2.0f, 3);
    
    detector.check(1.0f);
    detector.check(10.0f);
    detector.reset();
    
    // После reset глитч не детектируется сразу
    EXPECT_FALSE(detector.check(10.0f));
}
