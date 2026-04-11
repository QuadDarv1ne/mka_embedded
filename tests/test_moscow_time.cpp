/**
 * @file test_moscow_time.cpp
 * @brief Unit tests для Moscow Time системы и Data Freshness Manager
 */

#include <gtest/gtest.h>
#include <cstdint>
#include <string>

#include "systems/moscow_time.hpp"

using namespace mka::systems;

// ============================================================================
// Тесты конвертации времени
// ============================================================================

TEST(MoscowTimeTest, UTCtoMSKConversion) {
    // Тест: UTC 12:00 -> МСК 15:00
    UTCDateTime utc;
    utc.year = 2026;
    utc.month = 4;
    utc.day = 11;
    utc.hour = 12;
    utc.minute = 0;
    utc.second = 0;
    
    MSKDateTime msk = MSKDateTime::fromUTC(utc);
    
    EXPECT_EQ(msk.hour, 15);
    EXPECT_EQ(msk.minute, 0);
    EXPECT_EQ(msk.second, 0);
}

TEST(MoscowTimeTest, MSKtoUTCConversion) {
    // Тест: МСК 15:00 -> UTC 12:00
    MSKDateTime msk;
    msk.year = 2026;
    msk.month = 4;
    msk.day = 11;
    msk.hour = 15;
    msk.minute = 0;
    msk.second = 0;
    
    UTCDateTime utc = msk.toUTC();
    
    EXPECT_EQ(utc.hour, 12);
    EXPECT_EQ(utc.minute, 0);
    EXPECT_EQ(utc.second, 0);
}

TEST(MoscowTimeTest, TimestampConversion) {
    MoscowTimeConverter converter;
    
    // Unix timestamp: 2026-04-11 12:00:00 UTC
    uint64_t utcTimestamp = 1744372800;  // Примерное значение
    
    uint64_t mskTimestamp = converter.utcToMSK(utcTimestamp);
    uint64_t convertedBack = converter.mskToUTC(mskTimestamp);
    
    EXPECT_EQ(mskTimestamp, utcTimestamp + 3 * 3600);
    EXPECT_EQ(convertedBack, utcTimestamp);
}

TEST(MoscowTimeTest, UTCOffsetConstant) {
    EXPECT_EQ(MoscowTimeConverter::getUTCOffset(), 3 * 3600);
}

// ============================================================================
// Тесты Data Freshness Manager
// ============================================================================

TEST(DataFreshnessTest, Initialization) {
    DataFreshnessManager freshnessMgr;
    
    // Без источника времени
    EXPECT_FALSE(freshnessMgr.init(nullptr));
}

TEST(DataFreshnessTest, RegisterDataSource) {
    DataFreshnessManager freshnessMgr;
    auto utcSource = []() -> uint64_t { return 1000000; };
    
    EXPECT_TRUE(freshnessMgr.init(utcSource));
    
    DataFreshnessConfig config = {
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .autoUpdateOnStale = true,
        .autoUpdateOnExpire = true
    };
    
    EXPECT_TRUE(freshnessMgr.registerDataSource("SGP4_TLE", config));
}

TEST(DataFreshnessTest, FreshDataStatus) {
    DataFreshnessManager freshnessMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    freshnessMgr.init(utcSource);
    
    DataFreshnessConfig config = {
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .autoUpdateOnStale = true,
        .autoUpdateOnExpire = true
    };
    
    freshnessMgr.registerDataSource("TEST_DATA", config);
    
    // Данные обновлены 100 секунд назад (в пределах fresh threshold)
    auto status = freshnessMgr.checkFreshNESS("TEST_DATA", currentTime - 100);
    EXPECT_EQ(status, DataFreshnessStatus::FRESH);
}

TEST(DataFreshnessTest, StaleDataStatus) {
    DataFreshnessManager freshnessMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    freshnessMgr.init(utcSource);
    
    DataFreshnessConfig config = {
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .autoUpdateOnStale = true,
        .autoUpdateOnExpire = true
    };
    
    freshnessMgr.registerDataSource("TEST_DATA", config);
    
    // Данные обновлены 5000 секунд назад (между fresh и stale)
    auto status = freshnessMgr.checkFreshNESS("TEST_DATA", currentTime - 5000);
    EXPECT_EQ(status, DataFreshnessStatus::STALE);
}

TEST(DataFreshnessTest, ExpiredDataStatus) {
    DataFreshnessManager freshnessMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    freshnessMgr.init(utcSource);
    
    DataFreshnessConfig config = {
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .autoUpdateOnStale = true,
        .autoUpdateOnExpire = true
    };
    
    freshnessMgr.registerDataSource("TEST_DATA", config);
    
    // Данные обновлены 50000 секунд назад (между stale и expire)
    auto status = freshnessMgr.checkFreshNESS("TEST_DATA", currentTime - 50000);
    EXPECT_EQ(status, DataFreshnessStatus::EXPIRED);
}

TEST(DataFreshnessTest, InvalidDataStatus) {
    DataFreshnessManager freshnessMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    freshnessMgr.init(utcSource);
    
    DataFreshnessConfig config = {
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .autoUpdateOnStale = true,
        .autoUpdateOnExpire = true
    };
    
    freshnessMgr.registerDataSource("TEST_DATA", config);
    
    // Данные обновлены 100000 секунд назад (больше expire threshold)
    auto status = freshnessMgr.checkFreshNESS("TEST_DATA", currentTime - 100000);
    EXPECT_EQ(status, DataFreshnessStatus::INVALID);
}

// ============================================================================
// Тесты автоматического обновления
// ============================================================================

TEST(DataFreshnessTest, AutoUpdateOnStale) {
    DataFreshnessManager freshnessMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    freshnessMgr.init(utcSource);
    
    DataFreshnessConfig config = {
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .autoUpdateOnStale = true,
        .autoUpdateOnExpire = true
    };
    
    freshnessMgr.registerDataSource("TEST_DATA", config);
    
    bool updateCalled = false;
    auto updateCallback = [&updateCalled]() -> bool {
        updateCalled = true;
        return true;
    };
    
    // Данные устарели, должно вызвать обновление
    auto status = freshnessMgr.checkAndAutoUpdate("TEST_DATA", currentTime - 5000, updateCallback);
    
    EXPECT_TRUE(updateCalled);
    EXPECT_EQ(status, DataFreshnessStatus::FRESH);  // После обновления статус должен быть FRESH
}

TEST(DataFreshnessTest, NoAutoUpdateOnFresh) {
    DataFreshnessManager freshnessMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    freshnessMgr.init(utcSource);
    
    DataFreshnessConfig config = {
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .autoUpdateOnStale = true,
        .autoUpdateOnExpire = true
    };
    
    freshnessMgr.registerDataSource("TEST_DATA", config);
    
    bool updateCalled = false;
    auto updateCallback = [&updateCalled]() -> bool {
        updateCalled = true;
        return true;
    };
    
    // Данные свежие, обновление не должно вызываться
    auto status = freshnessMgr.checkAndAutoUpdate("TEST_DATA", currentTime - 100, updateCallback);
    
    EXPECT_FALSE(updateCalled);
    EXPECT_EQ(status, DataFreshnessStatus::FRESH);
}

// ============================================================================
// Тесты статистики
// ============================================================================

TEST(DataFreshnessTest, FreshnessStats) {
    DataFreshnessManager freshnessMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    freshnessMgr.init(utcSource);
    
    DataFreshnessConfig config = {
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .autoUpdateOnStale = true,
        .autoUpdateOnExpire = true
    };
    
    // Регистрируем 3 источника
    freshnessMgr.registerDataSource("FRESH", config);
    freshnessMgr.registerDataSource("STALE", config);
    freshnessMgr.registerDataSource("EXPIRED", config);
    
    // Устанавливаем разное время обновления
    freshnessMgr.checkFreshNESS("FRESH", currentTime - 100);
    freshnessMgr.checkFreshNESS("STALE", currentTime - 5000);
    freshnessMgr.checkFreshNESS("EXPIRED", currentTime - 50000);
    
    uint8_t freshCount, staleCount, expiredCount, invalidCount;
    freshnessMgr.getFreshnessStats(freshCount, staleCount, expiredCount, invalidCount);
    
    EXPECT_EQ(freshCount, 1);
    EXPECT_EQ(staleCount, 1);
    EXPECT_EQ(expiredCount, 1);
    EXPECT_EQ(invalidCount, 0);
}

// ============================================================================
// Тесты SGP4 Data Freshness Manager
// ============================================================================

TEST(DataFreshnessTest, SGP4DataFreshnessManager) {
    SGP4DataFreshnessManager sgp4Mgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    EXPECT_TRUE(sgp4Mgr.init(utcSource));
    
    // Регистрируем TLE данные ISS
    EXPECT_TRUE(sgp4Mgr.registerTLEData("ISS", 7200, 86400));
    
    bool tleUpdated = false;
    auto updateCallback = [&tleUpdated]() -> bool {
        tleUpdated = true;
        return true;
    };
    
    // Проверяем и обновляем
    auto status = sgp4Mgr.checkAndUpdateTLE("ISS", currentTime - 10000, updateCallback);
    
    EXPECT_TRUE(tleUpdated);
    EXPECT_EQ(status, DataFreshnessStatus::FRESH);
}
