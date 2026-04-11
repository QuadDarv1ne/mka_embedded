/**
 * @file test_auto_actualization.cpp
 * @brief Unit tests для системы автоматической актуализации расчётов по МСК
 */

#include <gtest/gtest.h>
#include <cstdint>
#include <vector>

#include "systems/auto_actualization.hpp"

using namespace mka::systems;

// ============================================================================
// Тесты инициализации и регистрации
// ============================================================================

TEST(AutoActualizationTest, Initialization) {
    AutoActualizationManager actualizationMgr;
    
    // Без источника времени
    EXPECT_FALSE(actualizationMgr.init(nullptr));
    
    // С источником времени
    auto utcSource = []() -> uint64_t { return 1000000; };
    EXPECT_TRUE(actualizationMgr.init(utcSource));
}

TEST(AutoActualizationTest, RegisterSGP4Calculation) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    actualizationMgr.init(utcSource);
    
    bool actualizationCalled = false;
    CalculationActualizationConfig config = {
        .type = CalculationType::SGP4_ORBITAL,
        .name = "ISS_TLE",
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .priority = ActualizationPriority::HIGH,
        .autoActualize = true,
        .actualizationCallback = [&actualizationCalled]() -> bool {
            actualizationCalled = true;
            return true;
        }
    };
    
    EXPECT_TRUE(actualizationMgr.registerCalculation(config));
    EXPECT_EQ(actualizationMgr.getCalculationCount(), 1);
}

TEST(AutoActualizationTest, RegisterMultipleCalculations) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    actualizationMgr.init(utcSource);
    
    auto dummyCallback = []() -> bool { return true; };
    
    // Регистрируем расчёты разных типов
    CalculationActualizationConfig configs[] = {
        {
            .type = CalculationType::SGP4_ORBITAL,
            .name = "ISS",
            .freshThresholdSeconds = 3600,
            .staleThresholdSeconds = 7200,
            .expireThresholdSeconds = 86400,
            .priority = ActualizationPriority::HIGH,
            .autoActualize = true,
            .actualizationCallback = dummyCallback
        },
        {
            .type = CalculationType::TELEMETRY,
            .name = "TM_Generator",
            .freshThresholdSeconds = 600,
            .staleThresholdSeconds = 1200,
            .expireThresholdSeconds = 3600,
            .priority = ActualizationPriority::MEDIUM,
            .autoActualize = true,
            .actualizationCallback = dummyCallback
        },
        {
            .type = CalculationType::FDIR_PARAMETERS,
            .name = "FDIR_Params",
            .freshThresholdSeconds = 1800,
            .staleThresholdSeconds = 3600,
            .expireThresholdSeconds = 7200,
            .priority = ActualizationPriority::LOW,
            .autoActualize = true,
            .actualizationCallback = dummyCallback
        }
    };
    
    for (const auto& config : configs) {
        EXPECT_TRUE(actualizationMgr.registerCalculation(config));
    }
    
    EXPECT_EQ(actualizationMgr.getCalculationCount(), 3);
}

// ============================================================================
// Тесты автоматической актуализации
// ============================================================================

TEST(AutoActualizationTest, AutoActualizeStaleCalculation) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    actualizationMgr.init(utcSource);
    
    bool actualizationCalled = false;
    CalculationActualizationConfig config = {
        .type = CalculationType::SGP4_ORBITAL,
        .name = "ISS_TLE",
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .priority = ActualizationPriority::HIGH,
        .autoActualize = true,
        .actualizationCallback = [&actualizationCalled]() -> bool {
            actualizationCalled = true;
            return true;
        }
    };
    
    actualizationMgr.registerCalculation(config);
    
    // Актуализируем все расчёты (данные устарели, т.к. lastActualizationTime = 0)
    size_t actualizedCount = actualizationMgr.actualizeAllCalculations();
    
    EXPECT_EQ(actualizedCount, 1);
    EXPECT_TRUE(actualizationCalled);
    EXPECT_EQ(
        actualizationMgr.getCalculationStatus(CalculationType::SGP4_ORBITAL),
        DataFreshnessStatus::FRESH
    );
}

TEST(AutoActualizationTest, NoActualizeFreshCalculation) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    actualizationMgr.init(utcSource);
    
    int actualizationCallCount = 0;
    CalculationActualizationConfig config = {
        .type = CalculationType::SGP4_ORBITAL,
        .name = "ISS_TLE",
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .priority = ActualizationPriority::HIGH,
        .autoActualize = true,
        .actualizationCallback = [&actualizationCallCount]() -> bool {
            actualizationCallCount++;
            return true;
        }
    };
    
    actualizationMgr.registerCalculation(config);
    
    // Первая актуализация
    size_t count1 = actualizationMgr.actualizeAllCalculations();
    EXPECT_EQ(count1, 1);
    EXPECT_EQ(actualizationCallCount, 1);
    
    // Вторая актуализация (данные ещё свежие - прошло 100 секунд при threshold 3600)
    currentTime += 100;
    size_t count2 = actualizationMgr.actualizeAllCalculations();
    
    // Callback не должен был вызваться второй раз
    EXPECT_EQ(count2, 0);
    EXPECT_EQ(actualizationCallCount, 1);
}

TEST(AutoActualizationTest, ActualizeByType) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    actualizationMgr.init(utcSource);
    
    bool sgp4Called = false;
    bool telemetryCalled = false;
    
    CalculationActualizationConfig sgp4Config = {
        .type = CalculationType::SGP4_ORBITAL,
        .name = "ISS",
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .priority = ActualizationPriority::HIGH,
        .autoActualize = true,
        .actualizationCallback = [&sgp4Called]() -> bool {
            sgp4Called = true;
            return true;
        }
    };
    
    CalculationActualizationConfig telConfig = {
        .type = CalculationType::TELEMETRY,
        .name = "TM",
        .freshThresholdSeconds = 600,
        .staleThresholdSeconds = 1200,
        .expireThresholdSeconds = 3600,
        .priority = ActualizationPriority::MEDIUM,
        .autoActualize = true,
        .actualizationCallback = [&telemetryCalled]() -> bool {
            telemetryCalled = true;
            return true;
        }
    };
    
    actualizationMgr.registerCalculation(sgp4Config);
    actualizationMgr.registerCalculation(telConfig);
    
    // Актуализируем только SGP4
    bool result = actualizationMgr.actualizeByType(CalculationType::SGP4_ORBITAL);
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(sgp4Called);
    EXPECT_FALSE(telemetryCalled);  // Телеметрия не должна была актуализироваться
}

TEST(AutoActualizationTest, ActualizeByName) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    actualizationMgr.init(utcSource);
    
    bool actualizationCalled = false;
    CalculationActualizationConfig config = {
        .type = CalculationType::SGP4_ORBITAL,
        .name = "ISS_TLE",
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .priority = ActualizationPriority::HIGH,
        .autoActualize = true,
        .actualizationCallback = [&actualizationCalled]() -> bool {
            actualizationCalled = true;
            return true;
        }
    };
    
    actualizationMgr.registerCalculation(config);
    
    // Актуализируем по имени
    bool result = actualizationMgr.actualizeByName("SGP4_ISS_TLE");
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(actualizationCalled);
}

// ============================================================================
// Тесты статистики
// ============================================================================

TEST(AutoActualizationTest, ActualizationStats) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    actualizationMgr.init(utcSource);
    
    int fdirCallCount = 0;
    auto dummyCallback = []() -> bool { return true; };
    auto fdirCallback = [&fdirCallCount]() -> bool {
        fdirCallCount++;
        return false;  // Симулируем ошибку актуализации
    };
    
    // Регистрируем 4 расчёта
    CalculationActualizationConfig configs[] = {
        {
            .type = CalculationType::SGP4_ORBITAL,
            .name = "Fresh_SGP4",
            .freshThresholdSeconds = 3600,
            .staleThresholdSeconds = 7200,
            .expireThresholdSeconds = 86400,
            .priority = ActualizationPriority::HIGH,
            .autoActualize = true,
            .actualizationCallback = dummyCallback
        },
        {
            .type = CalculationType::TELEMETRY,
            .name = "Fresh_Telemetry",
            .freshThresholdSeconds = 3600,
            .staleThresholdSeconds = 7200,
            .expireThresholdSeconds = 86400,
            .priority = ActualizationPriority::MEDIUM,
            .autoActualize = true,
            .actualizationCallback = dummyCallback
        },
        {
            .type = CalculationType::FDIR_PARAMETERS,
            .name = "Expired_FDIR",
            .freshThresholdSeconds = 100,
            .staleThresholdSeconds = 200,
            .expireThresholdSeconds = 300,
            .priority = ActualizationPriority::LOW,
            .autoActualize = false,  // Не актуализируем автоматически
            .actualizationCallback = fdirCallback
        },
        {
            .type = CalculationType::HEALTH_MONITORING,
            .name = "Invalid_Health",
            .freshThresholdSeconds = 3600,
            .staleThresholdSeconds = 7200,
            .expireThresholdSeconds = 86400,
            .priority = ActualizationPriority::HIGH,
            .autoActualize = false,
            .actualizationCallback = dummyCallback
        }
    };
    
    for (const auto& config : configs) {
        actualizationMgr.registerCalculation(config);
    }
    
    // Актуализируем SGP4 и Telemetry (они станут FRESH)
    actualizationMgr.actualizeByType(CalculationType::SGP4_ORBITAL);
    actualizationMgr.actualizeByType(CalculationType::TELEMETRY);
    
    // Перемещаем время вперёд на 500 секунд (> expireThreshold для FDIR)
    currentTime += 500;
    
    // FDIR и Health никогда не актуализировались (lastActualizationTime = 0), поэтому оба INVALID
    // SGP4 и Telemetry актуализированы и fresh (500 < 3600)
    uint8_t freshCount, staleCount, expiredCount, invalidCount;
    actualizationMgr.getActualizationStats(freshCount, staleCount, expiredCount, invalidCount);
    
    EXPECT_EQ(freshCount, 2);    // SGP4 и Telemetry
    EXPECT_EQ(staleCount, 0);
    EXPECT_EQ(expiredCount, 0);  // Нет расчётов в EXPIRED состоянии
    EXPECT_EQ(invalidCount, 2);  // FDIR и Health (никогда не актуализированы)
}

TEST(AutoActualizationTest, CalculationsNeedingActualization) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    actualizationMgr.init(utcSource);
    
    auto dummyCallback = []() -> bool { return true; };
    
    // Регистрируем 3 расчёта
    CalculationActualizationConfig configs[] = {
        {
            .type = CalculationType::SGP4_ORBITAL,
            .name = "Fresh_SGP4",
            .freshThresholdSeconds = 3600,
            .staleThresholdSeconds = 7200,
            .expireThresholdSeconds = 86400,
            .priority = ActualizationPriority::HIGH,
            .autoActualize = true,
            .actualizationCallback = dummyCallback
        },
        {
            .type = CalculationType::TELEMETRY,
            .name = "Stale_Telemetry",
            .freshThresholdSeconds = 100,
            .staleThresholdSeconds = 200,
            .expireThresholdSeconds = 300,
            .priority = ActualizationPriority::MEDIUM,
            .autoActualize = false,  // Не актуализируем автоматически
            .actualizationCallback = dummyCallback
        },
        {
            .type = CalculationType::FDIR_PARAMETERS,
            .name = "Invalid_FDIR",
            .freshThresholdSeconds = 3600,
            .staleThresholdSeconds = 7200,
            .expireThresholdSeconds = 86400,
            .priority = ActualizationPriority::LOW,
            .autoActualize = false,
            .actualizationCallback = dummyCallback
        }
    };
    
    for (const auto& config : configs) {
        actualizationMgr.registerCalculation(config);
    }
    
    // Актуализируем только SGP4
    actualizationMgr.actualizeByType(CalculationType::SGP4_ORBITAL);
    
    // Перемещаем время вперёд на 500 секунд (> staleThreshold для Telemetry)
    currentTime += 500;
    
    std::vector<size_t> needingActualization;
    actualizationMgr.getCalculationsNeedingActualization(needingActualization);
    
    // Telemetry устарел (500 > 100), FDIR never актуализирован
    EXPECT_EQ(needingActualization.size(), 2);
}

// ============================================================================
// Тесты времени МСК
// ============================================================================

TEST(AutoActualizationTest, LastActualizationTimeMSK) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;  // UTC timestamp
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };
    
    actualizationMgr.init(utcSource);
    
    CalculationActualizationConfig config = {
        .type = CalculationType::SGP4_ORBITAL,
        .name = "ISS",
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .priority = ActualizationPriority::HIGH,
        .autoActualize = true,
        .actualizationCallback = []() -> bool { return true; }
    };
    
    actualizationMgr.registerCalculation(config);
    actualizationMgr.actualizeAllCalculations();
    
    // Получаем МСК время
    uint64_t mskTime = actualizationMgr.getLastActualizationTimeMSK(CalculationType::SGP4_ORBITAL);
    
    // МСК = UTC + 3 часа
    EXPECT_EQ(mskTime, currentTime + 3 * 3600);
}

TEST(AutoActualizationTest, ActualizationCount) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };

    actualizationMgr.init(utcSource);

    int callCount = 0;
    CalculationActualizationConfig config = {
        .type = CalculationType::SGP4_ORBITAL,
        .name = "ISS",
        .freshThresholdSeconds = 100,  // Короткий fresh для тестирования
        .staleThresholdSeconds = 200,
        .expireThresholdSeconds = 300,
        .priority = ActualizationPriority::HIGH,
        .autoActualize = true,
        .actualizationCallback = [&callCount]() -> bool {
            callCount++;
            return true;
        }
    };

    actualizationMgr.registerCalculation(config);

    // Первая актуализация
    actualizationMgr.actualizeAllCalculations();
    EXPECT_EQ(actualizationMgr.getActualizationCount(CalculationType::SGP4_ORBITAL), 1);
    EXPECT_EQ(callCount, 1);

    // Перемещаем время вперёд (данные устарели)
    currentTime += 250;  // > staleThreshold

    // Вторая актуализация
    actualizationMgr.actualizeAllCalculations();
    EXPECT_EQ(actualizationMgr.getActualizationCount(CalculationType::SGP4_ORBITAL), 2);
    EXPECT_EQ(callCount, 2);
}

// ============================================================================
// Тесты фонового планировщика
// ============================================================================

TEST(AutoActualizationSchedulerTest, SchedulerInitialization) {
    AutoActualizationScheduler scheduler;

    // Планировщик не запущен по умолчанию
    EXPECT_FALSE(scheduler.isRunning());
}

TEST(AutoActualizationSchedulerTest, StartAndStopManualMode) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };

    actualizationMgr.init(utcSource);

    auto dummyCallback = []() -> bool { return true; };
    CalculationActualizationConfig config = {
        .type = CalculationType::SGP4_ORBITAL,
        .name = "ISS",
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .priority = ActualizationPriority::HIGH,
        .autoActualize = true,
        .actualizationCallback = dummyCallback
    };

    actualizationMgr.registerCalculation(config);

    // Запускаем планищик в ручном режиме
    AutoActualizationScheduler scheduler;
    SchedulerConfig config_sched = {
        .mode = SchedulerMode::MANUAL,
        .baseIntervalMs = 60000
    };

    EXPECT_TRUE(scheduler.start(actualizationMgr, config_sched));
    EXPECT_TRUE(scheduler.isRunning());

    // Ручной вызов актуализации
    size_t count = scheduler.actualizeNow();
    EXPECT_EQ(count, 1);

    // Останавливаем
    scheduler.stop();
    EXPECT_FALSE(scheduler.isRunning());
}

TEST(AutoActualizationSchedulerTest, StartAndStopAutoMode) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };

    actualizationMgr.init(utcSource);

    int callCount = 0;
    auto dummyCallback = [&callCount]() -> bool {
        callCount++;
        return true;
    };

    CalculationActualizationConfig config = {
        .type = CalculationType::SGP4_ORBITAL,
        .name = "ISS",
        .freshThresholdSeconds = 100,  // Короткий для теста
        .staleThresholdSeconds = 200,
        .expireThresholdSeconds = 300,
        .priority = ActualizationPriority::HIGH,
        .autoActualize = true,
        .actualizationCallback = dummyCallback
    };

    actualizationMgr.registerCalculation(config);

    // Запускаем планировщик в автоматическом режиме
    AutoActualizationScheduler scheduler;
    SchedulerConfig config_sched = {
        .mode = SchedulerMode::AUTO_BACKGROUND,
        .baseIntervalMs = 100,  // Очень короткий интервал для теста (100мс)
        .criticalIntervalMs = 50,
        .highIntervalMs = 100,
        .mediumIntervalMs = 200,
        .lowIntervalMs = 500
    };

    EXPECT_TRUE(scheduler.start(actualizationMgr, config_sched));
    EXPECT_TRUE(scheduler.isRunning());

    // Ждём немного чтобы фоновый поток успел сработать
    std::this_thread::sleep_for(std::chrono::milliseconds(350));

    // Останавливаем
    scheduler.stop();
    EXPECT_FALSE(scheduler.isRunning());

    // Callback должен был вызваться хотя бы один раз
    EXPECT_GE(callCount, 1);
}

TEST(AutoActualizationSchedulerTest, CannotStartTwice) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };

    actualizationMgr.init(utcSource);
    actualizationMgr.registerCalculation({
        .type = CalculationType::SGP4_ORBITAL,
        .name = "ISS",
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .priority = ActualizationPriority::HIGH,
        .autoActualize = true,
        .actualizationCallback = []() -> bool { return true; }
    });

    AutoActualizationScheduler scheduler;
    SchedulerConfig config = {
        .mode = SchedulerMode::MANUAL
    };

    EXPECT_TRUE(scheduler.start(actualizationMgr, config));

    // Вторая попытка запуска должна вернуть false
    EXPECT_FALSE(scheduler.start(actualizationMgr, config));

    scheduler.stop();
}

TEST(AutoActualizationSchedulerTest, GetIntervalForPriority) {
    AutoActualizationScheduler scheduler;
    SchedulerConfig config = {
        .mode = SchedulerMode::MANUAL,
        .baseIntervalMs = 60000,
        .criticalIntervalMs = 10000,
        .highIntervalMs = 60000,
        .mediumIntervalMs = 900000,
        .lowIntervalMs = 3600000
    };

    // Запускаем чтобы инициализировать config
    AutoActualizationManager mgr;
    mgr.init([]() -> uint64_t { return 1000000; });
    scheduler.start(mgr, config);

    EXPECT_EQ(scheduler.getIntervalForPriority(ActualizationPriority::CRITICAL), 10000);
    EXPECT_EQ(scheduler.getIntervalForPriority(ActualizationPriority::HIGH), 60000);
    EXPECT_EQ(scheduler.getIntervalForPriority(ActualizationPriority::MEDIUM), 900000);
    EXPECT_EQ(scheduler.getIntervalForPriority(ActualizationPriority::LOW), 3600000);

    scheduler.stop();
}

TEST(AutoActualizationSchedulerTest, MoveConstructor) {
    AutoActualizationManager actualizationMgr;
    uint64_t currentTime = 1000000;
    auto utcSource = [&currentTime]() -> uint64_t { return currentTime; };

    actualizationMgr.init(utcSource);
    actualizationMgr.registerCalculation({
        .type = CalculationType::SGP4_ORBITAL,
        .name = "ISS",
        .freshThresholdSeconds = 3600,
        .staleThresholdSeconds = 7200,
        .expireThresholdSeconds = 86400,
        .priority = ActualizationPriority::HIGH,
        .autoActualize = true,
        .actualizationCallback = []() -> bool { return true; }
    });

    AutoActualizationScheduler scheduler1;
    SchedulerConfig config = {.mode = SchedulerMode::MANUAL};
    scheduler1.start(actualizationMgr, config);

    EXPECT_TRUE(scheduler1.isRunning());

    // Перемещаем
    AutoActualizationScheduler scheduler2(std::move(scheduler1));
    EXPECT_TRUE(scheduler2.isRunning());
    EXPECT_FALSE(scheduler1.isRunning());

    scheduler2.stop();
}

TEST(AutoActualizationSchedulerTest, ActualizeNowWithoutManager) {
    AutoActualizationScheduler scheduler;

    // Вызов без менеджера должен вернуть 0
    size_t count = scheduler.actualizeNow();
    EXPECT_EQ(count, 0);
}
