/**
 * @file test_statemachine.cpp
 * @brief Unit tests for State Machine
 */

#include <gtest/gtest.h>

#include "systems/state_machine.hpp"

using namespace mka::statemachine;

// ============================================================================
// Тесты базовой функциональности
// ============================================================================

TEST(SatelliteStateMachineTest, InitialState) {
    SatelliteStateMachine sm;
    
    EXPECT_EQ(sm.getCurrentMode(), SatelliteMode::OFF);
    EXPECT_EQ(sm.getTransitionsCount(), 0u);
    EXPECT_EQ(sm.getTimeInCurrentMode(), 0u);
}

TEST(SatelliteStateMachineTest, GetModeName) {
    EXPECT_STREQ(SatelliteStateMachine::getModeName(SatelliteMode::OFF), "OFF");
    EXPECT_STREQ(SatelliteStateMachine::getModeName(SatelliteMode::INIT), "INIT");
    EXPECT_STREQ(SatelliteStateMachine::getModeName(SatelliteMode::SAFE), "SAFE");
    EXPECT_STREQ(SatelliteStateMachine::getModeName(SatelliteMode::STANDBY), "STANDBY");
    EXPECT_STREQ(SatelliteStateMachine::getModeName(SatelliteMode::NOMINAL), "NOMINAL");
    EXPECT_STREQ(SatelliteStateMachine::getModeName(SatelliteMode::MISSION), "MISSION");
    EXPECT_STREQ(SatelliteStateMachine::getModeName(SatelliteMode::MAINTENANCE), "MAINTENANCE");
    EXPECT_STREQ(SatelliteStateMachine::getModeName(SatelliteMode::EMERGENCY), "EMERGENCY");
    EXPECT_STREQ(SatelliteStateMachine::getModeName(static_cast<SatelliteMode>(99)), "UNKNOWN");
}

TEST(SatelliteStateMachineTest, GetReasonName) {
    EXPECT_STREQ(SatelliteStateMachine::getReasonName(TransitionReason::COMMAND), "COMMAND");
    EXPECT_STREQ(SatelliteStateMachine::getReasonName(TransitionReason::AUTONOMOUS), "AUTONOMOUS");
    EXPECT_STREQ(SatelliteStateMachine::getReasonName(TransitionReason::POWER_LOW), "POWER_LOW");
    EXPECT_STREQ(SatelliteStateMachine::getReasonName(TransitionReason::POWER_CRITICAL), "POWER_CRITICAL");
}

// ============================================================================
// Тесты переходов
// ============================================================================

TEST(SatelliteStateMachineTest, ForceTransition) {
    SatelliteStateMachine sm;
    
    sm.forceTransition(SatelliteMode::INIT, TransitionReason::COMMAND);
    
    EXPECT_EQ(sm.getCurrentMode(), SatelliteMode::INIT);
    EXPECT_EQ(sm.getTransitionsCount(), 1u);
    EXPECT_EQ(sm.getLastTransitionReason(), TransitionReason::COMMAND);
}

TEST(SatelliteStateMachineTest, RequestTransitionSuccess) {
    SatelliteStateMachine sm;
    
    // OFF -> INIT (без условий)
    auto result = sm.requestTransition(SatelliteMode::INIT, TransitionReason::COMMAND);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.previousMode, SatelliteMode::OFF);
    EXPECT_EQ(result.newMode, SatelliteMode::INIT);
    EXPECT_EQ(result.reason, TransitionReason::COMMAND);
    EXPECT_EQ(result.errorMessage, nullptr);
}

TEST(SatelliteStateMachineTest, RequestTransitionNotAllowed) {
    SatelliteStateMachine sm;
    
    // Попытка перехода OFF -> NOMINAL (напрямую, не разрешено)
    auto result = sm.requestTransition(SatelliteMode::NOMINAL, TransitionReason::COMMAND);
    
    EXPECT_FALSE(result.success);
    EXPECT_STREQ(result.errorMessage, "Transition not allowed");
}

TEST(SatelliteStateMachineTest, TransitionWithValidator) {
    SatelliteStateMachine sm;
    
    // OFF -> INIT -> SAFE
    sm.forceTransition(SatelliteMode::INIT, TransitionReason::COMMAND);
    
    // INIT -> SAFE требует antennaDeployed = true
    StateMachineContext ctx1{};
    ctx1.antennaDeployed = false;
    sm.updateContext(ctx1);
    
    auto result = sm.requestTransition(SatelliteMode::SAFE, TransitionReason::COMMAND);
    EXPECT_FALSE(result.success);
    EXPECT_STREQ(result.errorMessage, "Transition conditions not met");
    
    // С антенной
    ctx1.antennaDeployed = true;
    sm.updateContext(ctx1);
    
    result = sm.requestTransition(SatelliteMode::SAFE, TransitionReason::COMMAND);
    EXPECT_TRUE(result.success);
}

TEST(SatelliteStateMachineTest, SameModeTransition) {
    SatelliteStateMachine sm;
    
    // Переход в тот же режим
    auto result = sm.requestTransition(SatelliteMode::OFF, TransitionReason::COMMAND);
    EXPECT_TRUE(result.success);
}

// ============================================================================
// Тесты EnterSafeMode
// ============================================================================

TEST(SatelliteStateMachineTest, EnterSafeMode) {
    SatelliteStateMachine sm;
    
    sm.forceTransition(SatelliteMode::NOMINAL, TransitionReason::COMMAND);
    sm.enterSafeMode(TransitionReason::POWER_CRITICAL);
    
    EXPECT_EQ(sm.getCurrentMode(), SatelliteMode::SAFE);
    EXPECT_EQ(sm.getLastTransitionReason(), TransitionReason::POWER_CRITICAL);
}

// ============================================================================
// Тесты автономного управления
// ============================================================================

TEST(SatelliteStateMachineTest, AutonomousCheckPowerCritical) {
    SatelliteStateMachine sm;
    
    sm.forceTransition(SatelliteMode::NOMINAL, TransitionReason::COMMAND);
    
    StateMachineContext ctx{};
    ctx.batteryLevel = 5.0f;  // Критически низкий
    sm.updateContext(ctx);
    
    sm.autonomousCheck();
    
    EXPECT_EQ(sm.getCurrentMode(), SatelliteMode::SAFE);
}

TEST(SatelliteStateMachineTest, AutonomousCheckPowerLow) {
    SatelliteStateMachine sm;
    
    sm.forceTransition(SatelliteMode::MISSION, TransitionReason::COMMAND);
    
    StateMachineContext ctx{};
    ctx.batteryLevel = 15.0f;  // Низкий
    ctx.adcsNominal = true;
    sm.updateContext(ctx);
    
    sm.autonomousCheck();
    
    EXPECT_EQ(sm.getCurrentMode(), SatelliteMode::NOMINAL);
}

TEST(SatelliteStateMachineTest, AutonomousCheckThermalCritical) {
    SatelliteStateMachine sm;
    
    sm.forceTransition(SatelliteMode::NOMINAL, TransitionReason::COMMAND);
    
    StateMachineContext ctx{};
    ctx.batteryLevel = 50.0f;
    ctx.temperatureOBC = 65.0f;  // Перегрев
    sm.updateContext(ctx);
    
    sm.autonomousCheck();
    
    EXPECT_EQ(sm.getCurrentMode(), SatelliteMode::SAFE);
}

TEST(SatelliteStateMachineTest, AutonomousRecoveryFromSafe) {
    SatelliteStateMachine sm;
    
    sm.forceTransition(SatelliteMode::SAFE, TransitionReason::POWER_CRITICAL);
    
    StateMachineContext ctx{};
    ctx.batteryLevel = 40.0f;
    ctx.temperatureOBC = 25.0f;
    sm.updateContext(ctx);
    
    sm.autonomousCheck();
    
    EXPECT_EQ(sm.getCurrentMode(), SatelliteMode::STANDBY);
}

TEST(SatelliteStateMachineTest, AutonomousStandbyToNominal) {
    SatelliteStateMachine sm;
    
    sm.forceTransition(SatelliteMode::STANDBY, TransitionReason::COMMAND);
    
    StateMachineContext ctx{};
    ctx.batteryLevel = 60.0f;
    ctx.adcsNominal = true;
    ctx.commNominal = true;
    sm.updateContext(ctx);
    
    sm.autonomousCheck();
    
    EXPECT_EQ(sm.getCurrentMode(), SatelliteMode::NOMINAL);
}

// ============================================================================
// Тесты истории
// ============================================================================

TEST(SatelliteStateMachineTest, HistoryRecording) {
    SatelliteStateMachine sm;
    
    sm.forceTransition(SatelliteMode::INIT, TransitionReason::COMMAND);
    sm.forceTransition(SatelliteMode::SAFE, TransitionReason::AUTONOMOUS);
    
    EXPECT_EQ(sm.getHistorySize(), 2u);
    
    SatelliteStateMachine::HistoryEntry entry;
    bool found = sm.getHistoryEntry(0, entry);
    EXPECT_TRUE(found);
    EXPECT_EQ(entry.fromMode, SatelliteMode::OFF);
    EXPECT_EQ(entry.toMode, SatelliteMode::INIT);
    
    found = sm.getHistoryEntry(1, entry);
    EXPECT_TRUE(found);
    EXPECT_EQ(entry.fromMode, SatelliteMode::INIT);
    EXPECT_EQ(entry.toMode, SatelliteMode::SAFE);
}

TEST(SatelliteStateMachineTest, ClearHistory) {
    SatelliteStateMachine sm;
    
    sm.forceTransition(SatelliteMode::INIT, TransitionReason::COMMAND);
    sm.forceTransition(SatelliteMode::SAFE, TransitionReason::COMMAND);
    
    sm.clearHistory();
    
    EXPECT_EQ(sm.getHistorySize(), 0u);
}

// ============================================================================
// Тесты tick и времени
// ============================================================================

TEST(SatelliteStateMachineTest, Tick) {
    SatelliteStateMachine sm;
    
    sm.forceTransition(SatelliteMode::INIT, TransitionReason::COMMAND);
    
    sm.tick(10);
    EXPECT_EQ(sm.getTimeInCurrentMode(), 10u);
    
    sm.tick(20);
    EXPECT_EQ(sm.getTimeInCurrentMode(), 30u);
}

TEST(SatelliteStateMachineTest, TimeResetsOnTransition) {
    SatelliteStateMachine sm;
    
    sm.forceTransition(SatelliteMode::INIT, TransitionReason::COMMAND);
    sm.tick(100);
    
    sm.forceTransition(SatelliteMode::SAFE, TransitionReason::COMMAND);
    
    EXPECT_EQ(sm.getTimeInCurrentMode(), 0u);
}

// ============================================================================
// Тесты callback
// ============================================================================

TEST(SatelliteStateMachineTest, ModeChangeCallback) {
    SatelliteStateMachine sm;
    
    bool callbackCalled = false;
    SatelliteMode fromMode = SatelliteMode::OFF;
    SatelliteMode toMode = SatelliteMode::OFF;
    
    sm.setModeChangeCallback([&](SatelliteMode from, SatelliteMode to, TransitionReason) {
        callbackCalled = true;
        fromMode = from;
        toMode = to;
    });
    
    sm.forceTransition(SatelliteMode::INIT, TransitionReason::COMMAND);
    
    EXPECT_TRUE(callbackCalled);
    EXPECT_EQ(fromMode, SatelliteMode::OFF);
    EXPECT_EQ(toMode, SatelliteMode::INIT);
}

// ============================================================================
// Тесты контекста
// ============================================================================

TEST(SatelliteStateMachineTest, UpdateAndGetContext) {
    SatelliteStateMachine sm;
    
    StateMachineContext ctx{};
    ctx.batteryLevel = 75.0f;
    ctx.batteryVoltage = 7.8f;
    ctx.temperatureOBC = 35.0f;
    ctx.adcsNominal = true;
    
    sm.updateContext(ctx);
    
    const auto& retrieved = sm.getContext();
    EXPECT_FLOAT_EQ(retrieved.batteryLevel, 75.0f);
    EXPECT_FLOAT_EQ(retrieved.batteryVoltage, 7.8f);
    EXPECT_FLOAT_EQ(retrieved.temperatureOBC, 35.0f);
    EXPECT_TRUE(retrieved.adcsNominal);
}
