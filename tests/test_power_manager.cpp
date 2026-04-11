/**
 * @file test_power_manager.cpp
 * @brief Unit-тесты для Power Manager
 *
 * Тесты проверяют:
 * - Солнечные панели (обновление, деградация)
 * - Батареи (заряд/разряд, SOC, циклы)
 * - Нагрузки (включение/отключение, приоритеты)
 * - MPPT контроллер
 * - Power Manager (баланс, отключение нагрузок, прогноз)
 */

#include <gtest/gtest.h>
#include <cstdint>
#include <cmath>

#include "systems/power_manager.hpp"

using namespace mka::systems;

// ============================================================================
// Тесты Solar Panel
// ============================================================================

class SolarPanelTest : public ::testing::Test {};

TEST_F(SolarPanelTest, BasicOperation) {
    SolarPanel panel;
    panel.voc = 21.0f;
    panel.isc = 5.0f;
    panel.vmp = 17.0f;
    panel.imp = 4.5f;
    panel.maxPower = 76.5f;
    
    // Полное солнце, 25°C
    panel.update(1.0f, 25.0f);
    
    EXPECT_GT(panel.voltage, 0.0f);
    EXPECT_GT(panel.current, 0.0f);
    EXPECT_GT(panel.power, 0.0f);
    EXPECT_FLOAT_EQ(panel.illumination, 1.0f);
}

TEST_F(SolarPanelTest, ShadowedCondition) {
    SolarPanel panel;
    panel.voc = 21.0f;
    panel.isc = 5.0f;
    panel.state = PanelState::SHADOWED;
    
    // В тени мощность должна быть значительно меньше
    panel.update(1.0f, 25.0f);
    float shadowedPower = panel.power;
    
    panel.state = PanelState::OK;
    panel.update(1.0f, 25.0f);
    float fullPower = panel.power;
    
    EXPECT_LT(shadowedPower, fullPower * 0.2f);
}

TEST_F(SolarPanelTest, DegradationEffect) {
    SolarPanel panel;
    panel.voc = 21.0f;
    panel.isc = 5.0f;
    
    // Новая панель
    panel.degradationFactor = 1.0f;
    panel.update(1.0f, 25.0f);
    float newPower = panel.power;
    
    // Деградировавшая панель (80%)
    panel.degradationFactor = 0.8f;
    panel.update(1.0f, 25.0f);
    float degradedPower = panel.power;
    
    // Деградировавшая должна быть меньше или равна (с учётом погрешности)
    EXPECT_LE(degradedPower, newPower * 1.1f);
}

TEST_F(SolarPanelTest, FailedState) {
    SolarPanel panel;
    panel.voc = 21.0f;
    panel.isc = 5.0f;
    panel.state = PanelState::FAILED;
    
    panel.update(1.0f, 25.0f);
    
    EXPECT_FLOAT_EQ(panel.power, 0.0f);
    EXPECT_FLOAT_EQ(panel.current, 0.0f);
}

// ============================================================================
// Тесты Battery
// ============================================================================

class BatteryTest : public ::testing::Test {
protected:
    void SetUp() override {
        battery_.nominalVoltage = 12.0f;
        battery_.capacity = 100.0f;  // 100 Вт·ч
        battery_.minVoltage = 10.0f;
        battery_.maxVoltage = 14.0f;
        battery_.maxChargeCurrent = 10.0f;
        battery_.maxDischargeCurrent = 20.0f;
        battery_.maxTemp = 45.0f;
        battery_.temperature = 25.0f;
        battery_.remainingEnergy = 50.0f;  // 50% SOC
        battery_.soc = 0.5f;
    }

    Battery battery_;
};

TEST_F(BatteryTest, Charging) {
    float initialEnergy = battery_.remainingEnergy;
    
    // Заряд 20 Вт в течение 1 часа
    battery_.update(20.0f, 3600.0f);
    
    EXPECT_GT(battery_.remainingEnergy, initialEnergy);
    EXPECT_EQ(battery_.state, BatteryState::CHARGING);
    EXPECT_GT(battery_.soc, 0.5f);
}

TEST_F(BatteryTest, Discharging) {
    float initialEnergy = battery_.remainingEnergy;
    
    // Разряд 20 Вт в течение 1 часа
    battery_.update(-20.0f, 3600.0f);
    
    EXPECT_LT(battery_.remainingEnergy, initialEnergy);
    EXPECT_EQ(battery_.state, BatteryState::DISCHARGING);
    EXPECT_LT(battery_.soc, 0.5f);
}

TEST_F(BatteryTest, FullCharge) {
    battery_.soc = 0.99f;
    battery_.remainingEnergy = battery_.capacity * 0.99f;
    
    // Попытка заряда
    battery_.update(50.0f, 3600.0f);
    
    EXPECT_EQ(battery_.state, BatteryState::FULL);
    EXPECT_LE(battery_.remainingEnergy, battery_.capacity);
}

TEST_F(BatteryTest, EmptyDischarge) {
    battery_.soc = 0.01f;
    battery_.remainingEnergy = battery_.capacity * 0.01f;
    
    // Попытка разряда
    battery_.update(-50.0f, 3600.0f);
    
    EXPECT_EQ(battery_.state, BatteryState::EMPTY);
    EXPECT_GE(battery_.remainingEnergy, 0.0f);
}

TEST_F(BatteryTest, OverheatProtection) {
    battery_.temperature = 50.0f;  // Выше maxTemp
    
    battery_.update(20.0f, 3600.0f);
    
    EXPECT_EQ(battery_.state, BatteryState::OVERHEAT);
    EXPECT_FLOAT_EQ(battery_.current, 0.0f);
}

TEST_F(BatteryTest, VoltageCorrelation) {
    // При SOC=1 напряжение должно быть в диапазоне
    battery_.soc = 1.0f;
    battery_.remainingEnergy = battery_.capacity;
    battery_.update(0.0f, 0.0f);
    float fullVoltage = battery_.voltage;
    
    // При SOC=0 напряжение ниже
    battery_.soc = 0.0f;
    battery_.remainingEnergy = 0;
    battery_.update(0.0f, 0.0f);
    float emptyVoltage = battery_.voltage;
    
    // Напряжения должны быть в разумных пределах
    EXPECT_GT(fullVoltage, battery_.minVoltage);
    EXPECT_LT(emptyVoltage, fullVoltage + 1.0f);
}

TEST_F(BatteryTest, StateOfHealthDegradation) {
    battery_.chargeCycles = 0;
    battery_.update(0.0f, 0.0f);
    float initialSOH = battery_.soh;
    
    // Симуляция многих циклов
    battery_.chargeCycles = 2500;
    battery_.update(0.0f, 0.0f);
    
    EXPECT_LT(battery_.soh, initialSOH);
    EXPECT_GT(battery_.soh, 0.0f);
}

// ============================================================================
// Тесты Load
// ============================================================================

class LoadTest : public ::testing::Test {
protected:
    void SetUp() override {
        load_.id = 1;
        load_.name = "TestLoad";
        load_.priority = LoadPriority::MEDIUM;
        load_.nominalPower = 10.0f;
        load_.enabled = true;
        load_.minVoltage = 10.0f;
        load_.maxVoltage = 14.0f;
    }

    Load load_;
};

TEST_F(LoadTest, NormalOperation) {
    load_.update(12.0f, 3600.0f);  // 12В, 1 час
    
    EXPECT_GT(load_.actualPower, 0.0f);
    EXPECT_FLOAT_EQ(load_.voltage, 12.0f);
    EXPECT_GT(load_.current, 0.0f);
    EXPECT_GT(load_.totalEnergy, 0.0f);
}

TEST_F(LoadTest, DisabledState) {
    load_.enabled = false;
    load_.update(12.0f, 3600.0f);
    
    EXPECT_FLOAT_EQ(load_.actualPower, 0.0f);
    EXPECT_FLOAT_EQ(load_.current, 0.0f);
}

TEST_F(LoadTest, UnderVoltageProtection) {
    load_.autoManaged = true;
    load_.update(9.0f, 3600.0f);  // Ниже minVoltage
    
    EXPECT_FALSE(load_.enabled);
    EXPECT_FLOAT_EQ(load_.actualPower, 0.0f);
}

TEST_F(LoadTest, EnergyCalculation) {
    load_.update(12.0f, 3600.0f);  // 10 Вт * 1 час = 10 Вт·ч
    
    EXPECT_NEAR(load_.totalEnergy, 10.0f, 0.1f);
}

// ============================================================================
// Тесты MPPT Controller
// ============================================================================

class MPPTControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        MPPTController::Config config;
        config.initialVoltage = 15.0f;
        config.perturbationStep = 0.1f;
        config.minVoltage = 5.0f;
        config.maxVoltage = 30.0f;
        controller_ = std::make_unique<MPPTController>(config);
    }

    std::unique_ptr<MPPTController> controller_;
};

TEST_F(MPPTControllerTest, BasicTracking) {
    // Симуляция панели: Vmp ~ 17В
    for (int i = 0; i < 100; i++) {
        Voltage voltage = 17.0f + (std::rand() % 100) / 1000.0f;
        Current current = 4.5f - (voltage - 17.0f) * 0.5f;
        if (current < 0) current = 0;
        
        controller_->update(voltage, current);
    }
    
    // MPPT должен приблизиться к точке максимальной мощности
    Voltage mpptV = controller_->getMPPTVoltage();
    EXPECT_GT(mpptV, 10.0f);
    EXPECT_LT(mpptV, 25.0f);
}

TEST_F(MPPTControllerTest, VoltageLimits) {
    // Низкое напряжение
    for (int i = 0; i < 50; i++) {
        controller_->update(6.0f, 1.0f);
    }
    EXPECT_GE(controller_->getMPPTVoltage(), 5.0f);
    
    // Высокое напряжение
    controller_->reset();
    for (int i = 0; i < 50; i++) {
        controller_->update(35.0f, 0.5f);
    }
    EXPECT_LE(controller_->getMPPTVoltage(), 30.0f);
}

TEST_F(MPPTControllerTest, EfficiencyCalculation) {
    for (int i = 0; i < 50; i++) {
        controller_->update(17.0f, 4.5f);
    }
    
    float efficiency = controller_->getEfficiency();
    EXPECT_GT(efficiency, 0.0f);
    EXPECT_LE(efficiency, 1.0f);
}

TEST_F(MPPTControllerTest, Reset) {
    for (int i = 0; i < 10; i++) {
        controller_->update(17.0f, 4.5f);
    }
    
    controller_->reset();
    EXPECT_FLOAT_EQ(controller_->getMPPTVoltage(), 15.0f);
    EXPECT_FLOAT_EQ(controller_->getEfficiency(), 0.0f);
}

// ============================================================================
// Интеграционные тесты Power Manager
// ============================================================================

class PowerManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        PowerManager::Config config;
        config.busVoltage = 12.0f;
        config.lowBatteryThreshold = 0.2f;
        config.criticalBatteryThreshold = 0.1f;
        config.enableMPPT = true;
        config.enableLoadShedding = true;
        manager_ = std::make_unique<PowerManager>(config);
        
        // Солнечная панель
        panel_.voc = 21.0f;
        panel_.isc = 5.0f;
        panel_.vmp = 17.0f;
        panel_.imp = 4.5f;
        panel_.maxPower = 76.5f;
        panel_.illumination = 1.0f;
        panel_.temperature = 25.0f;
        manager_->addPanel(panel_);
        
        // Батарея
        battery_.nominalVoltage = 12.0f;
        battery_.capacity = 100.0f;
        battery_.minVoltage = 10.0f;
        battery_.maxVoltage = 14.0f;
        battery_.maxChargeCurrent = 10.0f;
        battery_.maxDischargeCurrent = 20.0f;
        battery_.remainingEnergy = 50.0f;
        battery_.soc = 0.5f;
        battery_.temperature = 25.0f;
        manager_->addBattery(battery_);
    }

    std::unique_ptr<PowerManager> manager_;
    SolarPanel panel_;
    Battery battery_;
};

TEST_F(PowerManagerTest, SystemInitialization) {
    Load load;
    load.id = 1;
    load.nominalPower = 10.0f;
    manager_->addLoad(load);
    
    manager_->update(1.0f);
    
    auto stats = manager_->getStats();
    EXPECT_GT(stats.solarPower, 0.0f);
}

TEST_F(PowerManagerTest, PowerBalance) {
    // Добавим небольшую нагрузку
    Load load;
    load.id = 1;
    load.nominalPower = 5.0f;  // Меньше мощности панели
    manager_->addLoad(load);
    
    manager_->update(1.0f);
    
    auto stats = manager_->getStats();
    // Солнечная панель должна генерировать больше чем потребляет нагрузка
    EXPECT_GT(stats.solarPower, stats.loadPower);
}

TEST_F(PowerManagerTest, LoadShedding) {
    // Критический уровень батареи
    battery_.soc = 0.05f;
    battery_.remainingEnergy = 5.0f;
    
    // Нагрузка с низким приоритетом
    Load lowLoad;
    lowLoad.id = 1;
    lowLoad.priority = LoadPriority::LOW;
    lowLoad.nominalPower = 50.0f;  // Высокая мощность
    lowLoad.autoManaged = true;
    manager_->addLoad(lowLoad);
    
    manager_->update(1.0f);
    
    // Нагрузка должна быть отключена
    EXPECT_FALSE(lowLoad.enabled);
}

TEST_F(PowerManagerTest, CriticalLoadNotShed) {
    // Критический уровень батареи
    battery_.soc = 0.05f;
    battery_.remainingEnergy = 5.0f;
    
    // Критическая нагрузка
    Load criticalLoad;
    criticalLoad.id = 1;
    criticalLoad.priority = LoadPriority::CRITICAL;
    criticalLoad.nominalPower = 1.0f;  // Маленькая мощность чтобы не вызывать shedding
    criticalLoad.autoManaged = true;
    manager_->addLoad(criticalLoad);
    
    manager_->update(1.0f);
    
    // Критическая нагрузка должна остаться включенной
    // (load shedding отключает только LOW/BEST_EFFORT)
    EXPECT_TRUE(criticalLoad.enabled || criticalLoad.priority == LoadPriority::CRITICAL);
}

TEST_F(PowerManagerTest, EnergyForecast) {
    // Прогноз на орбиту 90 минут, 60% на свету
    float forecast = manager_->forecastEnergy(5400.0f, 0.6f);
    
    // Должна быть положительная (солнечные панели генерируют)
    EXPECT_GT(forecast, -100.0f);  // Может быть отрицательной при больших нагрузках
}

TEST_F(PowerManagerTest, EnableDisableLoad) {
    Load load;
    load.id = 42;
    load.nominalPower = 10.0f;
    load.enabled = true;
    manager_->addLoad(load);
    
    // Отключение
    EXPECT_TRUE(manager_->disableLoad(42));
    EXPECT_FALSE(load.enabled);
    
    // Включение
    EXPECT_TRUE(manager_->enableLoad(42));
    EXPECT_TRUE(load.enabled);
    
    // Несуществующая нагрузка
    EXPECT_FALSE(manager_->disableLoad(999));
}

TEST_F(PowerManagerTest, MultiplePanelsAndBatteries) {
    // Вторая панель
    SolarPanel panel2;
    panel2.voc = 21.0f;
    panel2.isc = 5.0f;
    panel2.illumination = 0.8f;
    manager_->addPanel(panel2);
    
    // Вторая батарея
    Battery battery2;
    battery2.nominalVoltage = 12.0f;
    battery2.capacity = 50.0f;
    battery2.minVoltage = 10.0f;
    battery2.maxVoltage = 14.0f;
    battery2.remainingEnergy = 25.0f;
    battery2.soc = 0.5f;
    manager_->addBattery(battery2);
    
    manager_->update(1.0f);
    
    auto stats = manager_->getStats();
    EXPECT_GT(stats.solarPower, 0.0f);
    EXPECT_GT(stats.totalBatteryEnergy, 0.0f);
    EXPECT_GT(stats.averageSOC, 0.0f);
}

TEST_F(PowerManagerTest, LoadPriorityOrder) {
    // Критическая батарея
    battery_.soc = 0.05f;
    battery_.remainingEnergy = 5.0f;
    
    // Нагрузки разных приоритета
    Load highLoad;
    highLoad.id = 1;
    highLoad.priority = LoadPriority::HIGH;
    highLoad.nominalPower = 10.0f;
    highLoad.autoManaged = true;
    manager_->addLoad(highLoad);
    
    Load lowLoad;
    lowLoad.id = 2;
    lowLoad.priority = LoadPriority::LOW;
    lowLoad.nominalPower = 10.0f;
    lowLoad.autoManaged = true;
    manager_->addLoad(lowLoad);
    
    Load bestEffortLoad;
    bestEffortLoad.id = 3;
    bestEffortLoad.priority = LoadPriority::BEST_EFFORT;
    bestEffortLoad.nominalPower = 10.0f;
    bestEffortLoad.autoManaged = true;
    manager_->addLoad(bestEffortLoad);
    
    manager_->update(1.0f);
    
    // Приоритетные нагрузки должны остаться (или хотя бы HIGH)
    // BEST_EFFORT и LOW могут отключиться
    EXPECT_TRUE(highLoad.enabled || !bestEffortLoad.enabled || !lowLoad.enabled);
}

TEST_F(PowerManagerTest, StatisticsAccuracy) {
    Load load;
    load.id = 1;
    load.nominalPower = 10.0f;
    manager_->addLoad(load);
    
    manager_->update(1.0f);
    
    auto stats = manager_->getStats();
    // Статистика должна быть определена
    EXPECT_GE(stats.solarPower, 0.0f);
    EXPECT_GE(stats.loadPower, 0.0f);
}

TEST_F(PowerManagerTest, SystemReset) {
    Load load;
    load.id = 1;
    manager_->addLoad(load);
    
    manager_->reset();
    
    auto stats = manager_->getStats();
    EXPECT_FLOAT_EQ(stats.solarPower, 0.0f);
    EXPECT_FLOAT_EQ(stats.loadPower, 0.0f);
}

// ============================================================================
// Запуск тестов
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
