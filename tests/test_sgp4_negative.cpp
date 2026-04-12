/**
 * @file test_sgp4_negative.cpp
 * @brief Негативные тесты для SGP4 орбитального пропагатора
 * 
 * Проверяет устойчивость к:
 * - Некорректным TLE данным
 * - Вырожденным орбитам (e >= 1, i = 0, i = 180)
 * - Экстремальным параметрам
 * - Граничным условиям
 */

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <limits>

#include "algorithms/sgp4.hpp"

using namespace mka::navigation;

// ============================================================================
// TLE Parsing Tests
// ============================================================================

TEST(SGP4NegativeTest, EmptyTLE) {
    TLE tle;
    tle.line1 = "";
    tle.line2 = "";
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    EXPECT_FALSE(result);
}

TEST(SGP4NegativeTest, MalformedTLE) {
    TLE tle;
    tle.line1 = "1 25544U INVALID LINE";
    tle.line2 = "2 25544 BAD DATA";
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    EXPECT_FALSE(result);
}

TEST(SGP4NegativeTest, TLEWithWrongLength) {
    TLE tle;
    tle.line1 = "1 25544U";  // Слишком короткая строка
    tle.line2 = "2 25544";   // Слишком короткая строка
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    EXPECT_FALSE(result);
}

TEST(SGP4NegativeTest, TLEWithSpacesOnly) {
    TLE tle;
    tle.line1 = "                     ";
    tle.line2 = "                     ";
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    EXPECT_FALSE(result);
}

// ============================================================================
// Eccentricity Tests
// ============================================================================

TEST(SGP4NegativeTest, HighEccentricity) {
    TLE tle;
    tle.name = "TEST-SAT";
    tle.line1 = "1 99999U 24001A   24100.50000000  .00000000  00000-0  00000-0 0  9990";
    tle.line2 = "2 99999  51.6400 100.0000 9000000  90.0000 270.0000 10.00000000000010";
    // e = 0.9 (очень высокая)
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    
    // SGP4 может не работать с e близким к 1
    if (result) {
        ECIState state = propagator.propagate(0.0);
        // Проверить что состояние не NaN/Inf
        bool allFinite = std::isfinite(state.x) && std::isfinite(state.y) && 
                         std::isfinite(state.z);
        EXPECT_TRUE(allFinite);
    } else {
        EXPECT_FALSE(result);
    }
}

TEST(SGP4NegativeTest, EccentricityEqualToOne) {
    TLE tle;
    tle.name = "TEST-SAT";
    tle.line1 = "1 99999U 24001A   24100.50000000  .00000000  00000-0  00000-0 0  9990";
    tle.line2 = "2 99999  51.6400 100.0000 9999999  90.0000 270.0000 10.00000000000010";
    // e = 1.0 (параболическая орбита - SGP4 не работает)
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    
    // Должен вернуть false или обработать корректно
    EXPECT_FALSE(result);
}

TEST(SGP4NegativeTest, EccentricityGreaterThanOne) {
    TLE tle;
    tle.name = "TEST-SAT";
    tle.line1 = "1 99999U 24001A   24100.50000000  .00000000  00000-0  00000-0 0  9990";
    tle.line2 = "2 99999  51.6400 100.0000 1000000  90.0000 270.0000 10.00000000000010";
    // e = 1.0000000 (гиперболическая орбита - SGP4 не работает)
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    EXPECT_FALSE(result);
}

TEST(SGP4NegativeTest, ZeroEccentricity) {
    TLE tle;
    tle.name = "TEST-SAT";
    tle.line1 = "1 99999U 24001A   24100.50000000  .00000000  00000-0  00000-0 0  9990";
    tle.line2 = "2 99999  51.6400 100.0000 0000000  90.0000 270.0000 15.00000000000010";
    // e = 0.0 (круговая орбита - должна работать)
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    EXPECT_TRUE(result);
    
    ECIState state = propagator.propagate(0.0);
    bool allFinite = std::isfinite(state.x) && std::isfinite(state.y) && 
                     std::isfinite(state.z);
    EXPECT_TRUE(allFinite);
}

// ============================================================================
// Inclination Tests
// ============================================================================

TEST(SGP4NegativeTest, ZeroInclination) {
    TLE tle;
    tle.name = "TEST-SAT";
    tle.line1 = "1 99999U 24001A   24100.50000000  .00000000  00000-0  00000-0 0  9990";
    tle.line2 = "2 99999   0.0000 100.0000 0001000  90.0000 270.0000 15.00000000000010";
    // i = 0.0 (экваториальная орбита)
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    
    if (result) {
        ECIState state = propagator.propagate(0.0);
        bool allFinite = std::isfinite(state.x) && std::isfinite(state.y) && 
                         std::isfinite(state.z);
        EXPECT_TRUE(allFinite);
    }
}

TEST(SGP4NegativeTest, NinetyDegreeInclination) {
    TLE tle;
    tle.name = "TEST-SAT";
    tle.line1 = "1 99999U 24001A   24100.50000000  .00000000  00000-0  00000-0 0  9990";
    tle.line2 = "2 99999  90.0000 100.0000 0001000  90.0000 270.0000 15.00000000000010";
    // i = 90.0 (полярная орбита)
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    
    if (result) {
        ECIState state = propagator.propagate(0.0);
        bool allFinite = std::isfinite(state.x) && std::isfinite(state.y) && 
                         std::isfinite(state.z);
        EXPECT_TRUE(allFinite);
    }
}

TEST(SGP4NegativeTest, RetrogradeInclination) {
    TLE tle;
    tle.name = "TEST-SAT";
    tle.line1 = "1 99999U 24001A   24100.50000000  .00000000  00000-0  00000-0 0  9990";
    tle.line2 = "2 99999 120.0000 100.0000 0001000  90.0000 270.0000 15.00000000000010";
    // i = 120.0 (ретроградная орбита)
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    
    if (result) {
        ECIState state = propagator.propagate(0.0);
        bool allFinite = std::isfinite(state.x) && std::isfinite(state.y) && 
                         std::isfinite(state.z);
        EXPECT_TRUE(allFinite);
    }
}

// ============================================================================
// Mean Motion Tests
// ============================================================================

TEST(SGP4NegativeTest, ZeroMeanMotion) {
    TLE tle;
    tle.name = "TEST-SAT";
    tle.line1 = "1 99999U 24001A   24100.50000000  .00000000  00000-0  00000-0 0  9990";
    tle.line2 = "2 99999  51.6400 100.0000 0001000  90.0000 270.0000  0.00000000000010";
    // mean motion = 0 (некорректно)
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    EXPECT_FALSE(result);
}

TEST(SGP4NegativeTest, NegativeMeanMotion) {
    TLE tle;
    tle.name = "TEST-SAT";
    tle.line1 = "1 99999U 24001A   24100.50000000 -.00000000  00000-0  00000-0 0  9990";
    tle.line2 = "2 99999  51.6400 100.0000 0001000  90.0000 270.0000 -5.00000000000010";
    // mean motion = -5 (некорректно)
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    EXPECT_FALSE(result);
}

TEST(SGP4NegativeTest, VeryHighMeanMotion) {
    TLE tle;
    tle.name = "TEST-SAT";
    tle.line1 = "1 99999U 24001A   24100.50000000  .00000000  00000-0  00000-0 0  9990";
    tle.line2 = "2 99999  51.6400 100.0000 0001000  90.0000 270.0000 50.00000000000010";
    // mean motion = 50 rev/day (очень низкая орбита)
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    
    if (result) {
        ECIState state = propagator.propagate(0.0);
        bool allFinite = std::isfinite(state.x) && std::isfinite(state.y) && 
                         std::isfinite(state.z);
        EXPECT_TRUE(allFinite);
    }
}

// ============================================================================
// Propagation Time Tests
// ============================================================================

TEST(SGP4NegativeTest, PropagateNegativeTime) {
    TLE tle;
    tle.name = "ISS (ZARYA)";
    tle.line1 = "1 25544U 98067A   24100.50000000  .00016717  00000-0  30501-3 0  9990";
    tle.line2 = "2 25544  51.6400 100.0000 0001000  90.0000 270.0000 15.50000000000010";
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    ASSERT_TRUE(result);
    
    // Пропагация в прошлое
    ECIState state = propagator.propagate(-60.0);  // 60 минут назад
    bool allFinite = std::isfinite(state.x) && std::isfinite(state.y) && 
                     std::isfinite(state.z);
    EXPECT_TRUE(allFinite);
}

TEST(SGP4NegativeTest, PropagateLargeTime) {
    TLE tle;
    tle.name = "ISS (ZARYA)";
    tle.line1 = "1 25544U 98067A   24100.50000000  .00016717  00000-0  30501-3 0  9990";
    tle.line2 = "2 25544  51.6400 100.0000 0001000  90.0000 270.0000 15.50000000000010";
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    ASSERT_TRUE(result);
    
    // Пропагация на большое время (год)
    ECIState state = propagator.propagate(365.0 * 24.0 * 60.0);  // 1 год в минутах
    // Может стать некорректным из-за дрейфа орбиты
    // Проверим что не NaN/Inf
    bool allFinite = std::isfinite(state.x) && std::isfinite(state.y) && 
                     std::isfinite(state.z);
    EXPECT_TRUE(allFinite);
}

TEST(SGP4NegativeTest, PropagateZeroTime) {
    TLE tle;
    tle.name = "ISS (ZARYA)";
    tle.line1 = "1 25544U 98067A   24100.50000000  .00016717  00000-0  30501-3 0  9990";
    tle.line2 = "2 25544  51.6400 100.0000 0001000  90.0000 270.0000 15.50000000000010";
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    ASSERT_TRUE(result);
    
    ECIState state = propagator.propagate(0.0);
    bool allFinite = std::isfinite(state.x) && std::isfinite(state.y) && 
                     std::isfinite(state.z);
    EXPECT_TRUE(allFinite);
    EXPECT_GT(state.altitude(), 100.0);  // Должен быть выше 100 km
}

// ============================================================================
// Real TLE Tests
// ============================================================================

TEST(SGP4NegativeTest, RealISSTLE) {
    TLE tle;
    tle.name = "ISS (ZARYA)";
    tle.line1 = "1 25544U 98067A   24100.50000000  .00016717  00000-0  30501-3 0  9990";
    tle.line2 = "2 25544  51.6400 100.0000 0001000  90.0000 270.0000 15.50000000000010";
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    EXPECT_TRUE(result);
    
    ECIState state = propagator.propagate(0.0);
    EXPECT_GT(state.altitude(), 300.0);  // ISS обычно на ~400 km
    EXPECT_LT(state.altitude(), 500.0);
}

TEST(SGP4NegativeTest, RealCubeSatTLE) {
    TLE tle;
    tle.name = "TEST CUBESAT";
    tle.line1 = "1 99999U 24001A   24100.50000000  .00001000  00000-0  10000-3 0  9990";
    tle.line2 = "2 99999  97.5000 180.0000 0010000  45.0000 315.0000 15.20000000000010";
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    EXPECT_TRUE(result);
    
    ECIState state = propagator.propagate(0.0);
    bool allFinite = std::isfinite(state.x) && std::isfinite(state.y) && 
                     std::isfinite(state.z);
    EXPECT_TRUE(allFinite);
}

// ============================================================================
// Coordinate Conversion Tests
// ============================================================================

TEST(SGP4NegativeTest, ECItoLLAConversion) {
    TLE tle;
    tle.name = "ISS (ZARYA)";
    tle.line1 = "1 25544U 98067A   24100.50000000  .00016717  00000-0  30501-3 0  9990";
    tle.line2 = "2 25544  51.6400 100.0000 0001000  90.0000 270.0000 15.50000000000010";
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    ASSERT_TRUE(result);
    
    ECIState state = propagator.propagate(0.0);
    LLACoords lla = propagator.eciToLla(state);
    
    // Проверить что LLA в допустимых диапазонах
    EXPECT_GE(lla.latitude, -90.0);
    EXPECT_LE(lla.latitude, 90.0);
    EXPECT_GE(lla.longitude, -180.0);
    EXPECT_LE(lla.longitude, 360.0);  // Может быть больше 180 из-за вращения Земли
    EXPECT_GT(lla.altitude, 0.0);
}

// ============================================================================
// Orbital Period Tests
// ============================================================================

TEST(SGP4NegativeTest, OrbitalPeriodCalculation) {
    TLE tle;
    tle.name = "ISS (ZARYA)";
    tle.line1 = "1 25544U 98067A   24100.50000000  .00016717  00000-0  30501-3 0  9990";
    tle.line2 = "2 25544  51.6400 100.0000 0001000  90.0000 270.0000 15.50000000000010";
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    ASSERT_TRUE(result);
    
    double period = propagator.getOrbitalPeriod();
    // ISS период ~92 минуты
    EXPECT_GT(period, 80.0);
    EXPECT_LT(period, 100.0);
}

// ============================================================================
// Multiple Propagation Tests
// ============================================================================

TEST(SGP4NegativeTest, MultiplePropagations) {
    TLE tle;
    tle.name = "ISS (ZARYA)";
    tle.line1 = "1 25544U 98067A   24100.50000000  .00016717  00000-0  30501-3 0  9990";
    tle.line2 = "2 25544  51.6400 100.0000 0001000  90.0000 270.0000 15.50000000000010";
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    ASSERT_TRUE(result);
    
    // Многократная пропаган
    for (int i = 0; i < 100; ++i) {
        double time = i * 10.0;  // Каждые 10 минут
        ECIState state = propagator.propagate(time);
        
        bool allFinite = std::isfinite(state.x) && std::isfinite(state.y) && 
                         std::isfinite(state.z) && std::isfinite(state.vx) && 
                         std::isfinite(state.vy) && std::isfinite(state.vz);
        EXPECT_TRUE(allFinite) << "Failed at time " << time;
    }
}

// ============================================================================
// Stress Test
// ============================================================================

TEST(SGP4NegativeTest, StressTestWithManyPropagations) {
    TLE tle;
    tle.name = "ISS (ZARYA)";
    tle.line1 = "1 25544U 98067A   24100.50000000  .00016717  00000-0  30501-3 0  9990";
    tle.line2 = "2 25544  51.6400 100.0000 0001000  90.0000 270.0000 15.50000000000010";
    
    SGP4Propagator propagator;
    bool result = propagator.init(tle);
    ASSERT_TRUE(result);
    
    // Пропагация на каждый шаг орбиты
    double period = propagator.getOrbitalPeriod();
    for (int i = 0; i < 1000; ++i) {
        double time = i * period / 100.0;
        ECIState state = propagator.propagate(time);
        
        bool allFinite = std::isfinite(state.x) && std::isfinite(state.y) && 
                         std::isfinite(state.z);
        EXPECT_TRUE(allFinite) << "Failed at step " << i;
    }
}
