/**
 * @file test_sgp4.cpp
 * @brief Unit tests for SGP4 orbital propagator
 */

#include <gtest/gtest.h>
#include <cmath>
#include <string>

#include "algorithms/sgp4.hpp"

using namespace mka::navigation;

// ============================================================================
// Тесты инициализации TLE
// ============================================================================

TEST(SGP4Test, TLEInitialization) {
    TLE tle;
    tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
    tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
    
    EXPECT_EQ(tle.catalogNumber, 25544);
    EXPECT_NEAR(tle.inclination, 51.64, 0.01);
    EXPECT_NEAR(tle.raan, 208.57, 0.01);
    EXPECT_NEAR(tle.eccentricity, 0.0006703, 0.0000001);
    EXPECT_NEAR(tle.argPerigee, 85.62, 0.01);
    EXPECT_NEAR(tle.meanAnomaly, 274.52, 0.01);
    EXPECT_NEAR(tle.meanMotion, 15.495606, 0.000001);
}

TEST(SGP4Test, TLEParsing) {
    TLE tle;
    tle.line1 = "1 43013U 18004A   24001.25000000  .00000123  00000-0  12345-4 0  9991";
    tle.line2 = "2 43013  97.5000 120.3000 0012345  45.6000 314.6000 15.20000000345678";
    
    EXPECT_EQ(tle.catalogNumber, 43013);
    EXPECT_NEAR(tle.inclination, 97.5, 0.01);
    EXPECT_NEAR(tle.raan, 120.3, 0.01);
    EXPECT_NEAR(tle.eccentricity, 0.0012345, 0.0000001);
}

// ============================================================================
// Тесты пропагатора
// ============================================================================

TEST(SGP4Test, PropagatorInitialization) {
    TLE tle;
    tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
    tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
    
    SGP4Propagator propagator;
    bool success = propagator.init(tle);
    EXPECT_TRUE(success);
    EXPECT_TRUE(propagator.isValid());
}

TEST(SGP4Test, OrbitalPeriodCalculation) {
    TLE tle;
    tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
    tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
    
    SGP4Propagator propagator;
    propagator.init(tle);
    
    double period = propagator.getOrbitalPeriod();
    
    // ISS орбитальный период ~92 минуты
    EXPECT_GT(period, 85.0);
    EXPECT_LT(period, 95.0);
}

TEST(SGP4Test, ECIPositionReasonable) {
    TLE tle;
    tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
    tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
    
    SGP4Propagator propagator;
    propagator.init(tle);
    
    ECIState state = propagator.propagate(0.0);
    
    // Радиус орбиты Земли ~6378 км + высота ISS ~400 км = ~6778 км
    double distance = state.radius();
    
    EXPECT_GT(distance, 6000.0);
    EXPECT_LT(distance, 8000.0);
}

TEST(SGP4Test, PropagationOverTime) {
    TLE tle;
    tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
    tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
    
    SGP4Propagator propagator;
    propagator.init(tle);
    
    ECIState state0 = propagator.propagate(0.0);
    ECIState state60 = propagator.propagate(60.0);  // 60 минут
    
    double dx = state60.x - state0.x;
    double dy = state60.y - state0.y;
    double dz = state60.z - state0.z;
    double displacement = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // ISS движется ~7.66 км/с, за 60 минут ~27,600 км
    EXPECT_GT(displacement, 10000.0);
}

// ============================================================================
// Тесты преобразования координат
// ============================================================================

TEST(SGP4Test, LLAReasonableValues) {
    TLE tle;
    tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
    tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
    
    SGP4Propagator propagator;
    propagator.init(tle);
    
    ECIState eci = propagator.propagate(0.0);
    LLACoords lla = utils::eciToLLA(eci, 2460300.5);  // JD для 2024 года
    
    EXPECT_GE(lla.latitude, -90.0);
    EXPECT_LE(lla.latitude, 90.0);
    EXPECT_GE(lla.longitude, -180.0);
    EXPECT_LE(lla.longitude, 180.0);
}

// ============================================================================
// Тесты специальных случаев
// ============================================================================

TEST(SGP4Test, CircularOrbit) {
    TLE tle;
    tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
    tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
    
    SGP4Propagator propagator;
    propagator.init(tle);
    EXPECT_LT(propagator.getEccentricity(), 0.01);
}

TEST(SGP4Test, PolarOrbit) {
    TLE tle;
    tle.line1 = "1 43013U 18004A   24001.25000000  .00000123  00000-0  12345-4 0  9991";
    tle.line2 = "2 43013  97.5000 120.3000 0012345  45.6000 314.6000 15.20000000345678";
    
    SGP4Propagator propagator;
    propagator.init(tle);
    EXPECT_GT(propagator.getInclination(), 95.0 * 3.14159265 / 180.0 - 0.1);
    EXPECT_LT(propagator.getInclination(), 100.0 * 3.14159265 / 180.0 + 0.1);
}

// ============================================================================
// Тесты валидации
// ============================================================================

TEST(SGP4Test, EccentricityValidation) {
    TLE tle;
    tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
    tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
    
    SGP4Propagator propagator;
    propagator.init(tle);
    double e = propagator.getEccentricity();
    EXPECT_GE(e, 0.0);
}

TEST(SGP4Test, InclinationValidation) {
    TLE tle;
    tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
    tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
    
    SGP4Propagator propagator;
    propagator.init(tle);
    double inc = propagator.getInclination();
    EXPECT_GE(inc, 0.0);
    EXPECT_LE(inc, 3.14159265);  // [0, pi] rad
}
