/**
 * @file test_canopen.cpp
 * @brief Unit tests for CANopen stack
 */

#include <gtest/gtest.h>
#include <cstring>

#include "systems/canopen.hpp"

using namespace mka::systems;

// ============================================================================
// Тесты CANopen констант
// ============================================================================

TEST(CANopenTest, COBIdConstants) {
    EXPECT_EQ(COBId::NMT_COMMAND, 0x000);
    EXPECT_EQ(COBId::NMT_ERROR_CTRL, 0x700);
    EXPECT_EQ(COBId::SDO_RX, 0x600);
    EXPECT_EQ(COBId::SDO_TX, 0x580);
    EXPECT_EQ(COBId::EMCY, 0x080);
    EXPECT_EQ(COBId::PDO_BASE, 0x180);
}

TEST(CANopenTest, NMTCommandValues) {
    EXPECT_EQ(static_cast<uint8_t>(NMTCommand::START_REMOTE_NODE), 0x01);
    EXPECT_EQ(static_cast<uint8_t>(NMTCommand::STOP_REMOTE_NODE), 0x02);
    EXPECT_EQ(static_cast<uint8_t>(NMTCommand::ENTER_PRE_OPERATIONAL), 0x80);
    EXPECT_EQ(static_cast<uint8_t>(NMTCommand::RESET_NODE), 0x81);
    EXPECT_EQ(static_cast<uint8_t>(NMTCommand::RESET_COMMUNICATION), 0x82);
}

TEST(CANopenTest, NodeStateValues) {
    EXPECT_EQ(static_cast<uint8_t>(NodeState::INITIALIZING), 0x00);
    EXPECT_EQ(static_cast<uint8_t>(NodeState::STOPPED), 0x04);
    EXPECT_EQ(static_cast<uint8_t>(NodeState::OPERATIONAL), 0x05);
    EXPECT_EQ(static_cast<uint8_t>(NodeState::PRE_OPERATIONAL), 0x7F);
}

TEST(CANopenTest, SDOErrorCodes) {
    EXPECT_EQ(static_cast<uint32_t>(SDOErrorCode::NO_ERROR), 0x00000000);
    EXPECT_EQ(static_cast<uint32_t>(SDOErrorCode::READ_ONLY_OBJECT), 0x06010002);
    EXPECT_EQ(static_cast<uint32_t>(SDOErrorCode::OBJECT_NOT_EXIST), 0x06020000);
    EXPECT_EQ(static_cast<uint32_t>(SDOErrorCode::DATA_WRITE_ERROR), 0x08000000);
}

// ============================================================================
// Тесты CANopen Stack
// ============================================================================

TEST(CANopenTest, StackConstruction) {
    CANopenStack stack(1);
    // Stack создан, но не инициализирован
}

TEST(CANopenTest, StackDifferentNodeId) {
    CANopenStack stack(5);
    // Stack создан с nodeId = 5
}
