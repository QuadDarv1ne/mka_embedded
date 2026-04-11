/**
 * @file test_actualization_integration.cpp
 * @brief Unit tests для интеграции auto actualization с модулями
 */

#include <gtest/gtest.h>
#include <cstdint>
#include <string>

#include "systems/actualization_integration.hpp"

using namespace mka::systems;
using namespace mka::navigation;

// ============================================================================
// Тесты AutoActualizationSGP4
// ============================================================================

TEST(ActualizationIntegrationTest, SGP4Initialization) {
    AutoActualizationSGP4 sgp4;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    bool tleLoaded = false;
    auto tleLoader = [&tleLoaded]() -> TLE {
        tleLoaded = true;
        TLE tle;
        tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
        tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
        return tle;
    };
    
    EXPECT_TRUE(sgp4.init(utcSource, "ISS", tleLoader));
    EXPECT_EQ(sgp4.getFreshnessStatus(), DataFreshnessStatus::INVALID);
}

TEST(ActualizationIntegrationTest, SGP4PropagationWithActualization) {
    AutoActualizationSGP4 sgp4;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    int tleLoadCount = 0;
    auto tleLoader = [&tleLoadCount]() -> TLE {
        tleLoadCount++;
        TLE tle;
        tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
        tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
        return tle;
    };
    
    sgp4.init(utcSource, "ISS", tleLoader);
    
    // Первое propagation - должно загрузить TLE
    ECIState state1 = sgp4.propagateIfNeeded(0.0);
    
    EXPECT_EQ(tleLoadCount, 1);
    EXPECT_GT(state1.radius(), 6000.0);  // Разумный радиус орбиты
    EXPECT_LT(state1.radius(), 10000.0);
}

TEST(ActualizationIntegrationTest, SGP4ForceActualize) {
    AutoActualizationSGP4 sgp4;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    int tleLoadCount = 0;
    auto tleLoader = [&tleLoadCount]() -> TLE {
        tleLoadCount++;
        TLE tle;
        tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
        tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
        return tle;
    };
    
    sgp4.init(utcSource, "ISS", tleLoader, 3600, 86400);
    
    // Принудительная актуализация
    EXPECT_TRUE(sgp4.forceActualize());
    EXPECT_EQ(tleLoadCount, 1);
    EXPECT_EQ(sgp4.getFreshnessStatus(), DataFreshnessStatus::FRESH);
}

TEST(ActualizationIntegrationTest, SGP4LastActualizationTimeMSK) {
    AutoActualizationSGP4 sgp4;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    auto tleLoader = []() -> navigation::TLE {
        navigation::TLE tle;
        tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
        tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
        return tle;
    };
    
    sgp4.init(utcSource, "ISS", tleLoader);
    sgp4.forceActualize();
    
    // Проверяем МСК время
    uint64_t mskTime = sgp4.getLastActualizationTimeMSK();
    EXPECT_EQ(mskTime, MoscowTimeConverter::utcToMSK(currentTime));
}

// ============================================================================
// Тесты комплексной интеграции
// ============================================================================

TEST(ActualizationIntegrationTest, SGP4AndHealthIntegration) {
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    // Инициализируем SGP4
    AutoActualizationSGP4 sgp4;
    auto tleLoader = []() -> navigation::TLE {
        navigation::TLE tle;
        tle.line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9002";
        tle.line2 = "2 25544  51.6400 208.5700 0006703  85.6200 274.5200 15.49560600123456";
        return tle;
    };
    EXPECT_TRUE(sgp4.init(utcSource, "ISS", tleLoader));
    
    // Инициализируем Health Monitoring
    AutoActualizationHealthMonitoring health;
    EXPECT_TRUE(health.init(utcSource));
    
    // Все модули инициализированы
    EXPECT_EQ(sgp4.getFreshnessStatus(), DataFreshnessStatus::INVALID);
    EXPECT_EQ(health.getFreshnessStatus(), DataFreshnessStatus::INVALID);
    
    // Актуализируем SGP4
    sgp4.forceActualize();
    EXPECT_EQ(sgp4.getFreshnessStatus(), DataFreshnessStatus::FRESH);
    
    // Проверяем что SGP4 работает
    auto state = sgp4.propagateIfNeeded(0.0);
    EXPECT_GT(state.radius(), 6000.0);
    EXPECT_LT(state.radius(), 10000.0);
}
