/**
 * @file simple_planner_lvs_assign_no_ik_move_unit.cpp
 * @brief Unit tests for SimplePlannerLVSAssignNoIKMoveProfile
 *
 * @author Roelof Oomen
 * @date July 16, 2025
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "simple_planner_test_utils.hpp"

#include <gtest/gtest.h>
#include <tesseract_common/types.h>
#include <tesseract_common/unit_test_utils.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/simple/cereal_serialization.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_assign_no_ik_move_profile.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>

using namespace tesseract::motion_planners;
using namespace tesseract::command_language;

/**
 * @brief Test fixture for SimplePlannerLVSAssignNoIKMoveProfile unit tests
 *
 * This test suite validates the SimplePlannerLVSAssignNoIKMoveProfile class, which generates
 * a variable number of intermediate waypoints based on longest valid segment (LVS) criteria
 * while using NoIK assignment behavior. The "NoIK" behavior means no inverse kinematics
 * solving is performed. Instead, joint positions are assigned based on priority:
 * 1. If target waypoint has joint position: use target joint position
 * 2. Else if start waypoint has joint position: use start joint position
 * 3. Else: use current environment joint values
 * This combines adaptive step sizing with NoIK position assignment behavior.
 */
class TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit : public TesseractPlanningSimplePlannerUnit
{
};

TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, Serialization)  // NOLINT
{
  auto profile = std::make_shared<SimplePlannerLVSAssignNoIKMoveProfile>(3.14, 0.5, 1.57, 5);
  // Serialization
  tesseract::common::testSerializationDerivedClass<tesseract::common::Profile, SimplePlannerLVSAssignNoIKMoveProfile>(
      profile, "SimplePlannerLVSAssignNoIKMoveProfile");
}

/**
 * @brief Test Joint-to-Cartesian movement with NoIK behavior
 *
 * This test verifies that when the start waypoint is a joint waypoint and the end
 * waypoint is a Cartesian waypoint (without seed), the start joint position is
 * assigned to all intermediate waypoints (no IK solving). Step count adapts based on LVS criteria.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, JointCart_AssignStartJointPosition_NoIK)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSAssignNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For NoIK behavior, all intermediate waypoints should be joint waypoints
  // with the START joint position assigned (not IK solution of target)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp1.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
    if (instr2.getPathProfile().empty())
    {
      EXPECT_EQ(mi.getProfile(), instr2.getProfile());
      EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
    }
    else
    {
      EXPECT_EQ(mi.getProfile(), instr2.getPathProfile());
      EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
    }
  }

  // The final waypoint should be the original Cartesian waypoint with start position as seed
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(wp1.getPosition().isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test Cartesian-to-Joint movement with NoIK behavior
 *
 * This test verifies that when the start waypoint is a Cartesian waypoint (without seed)
 * and the end waypoint is a joint waypoint, the target joint position is assigned to all
 * intermediate waypoints (no IK solving). Step count adapts based on LVS criteria.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, CartJoint_AssignTargetJointPosition_NoIK)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSAssignNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For NoIK behavior, all intermediate waypoints should be joint waypoints
  // with the TARGET joint position assigned
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
    if (instr2.getPathProfile().empty())
    {
      EXPECT_EQ(mi.getProfile(), instr2.getProfile());
      EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
    }
    else
    {
      EXPECT_EQ(mi.getProfile(), instr2.getPathProfile());
      EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
    }
  }

  // The final waypoint should be the original joint waypoint target
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
  EXPECT_TRUE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test Cartesian-to-Cartesian movement with NoIK behavior
 *
 * This test verifies that when both start and end waypoints are Cartesian waypoints
 * (without seeds), the current environment joint values are assigned to all
 * intermediate waypoints (no IK solving). Step count adapts based on LVS criteria.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, CartCart_AssignSeedJointPosition_NoIK)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, -0.1, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0.1, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSAssignNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // Get the seed position from the current joint values
  auto fwd_kin = env_->getJointGroup(manip_info_.manipulator);
  Eigen::VectorXd seed_position = env_->getCurrentJointValues(fwd_kin->getJointNames());

  // For NoIK behavior, all intermediate waypoints should be joint waypoints
  // with the SEED joint position assigned (not IK solution)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(seed_position.isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
    if (instr2.getPathProfile().empty())
    {
      EXPECT_EQ(mi.getProfile(), instr2.getProfile());
      EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
    }
    else
    {
      EXPECT_EQ(mi.getProfile(), instr2.getPathProfile());
      EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
    }
  }

  // The final waypoint should be the original Cartesian waypoint with seed position
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(seed_position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test Joint-to-Joint movement with NoIK behavior
 *
 * This test verifies that when both start and end waypoints are joint waypoints,
 * the target joint position is assigned to all intermediate waypoints (no IK solving).
 * This follows the priority-based assignment: target joint position has priority.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, JointJoint_AssignTargetJointPosition_NoIK)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSAssignNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For joint-to-joint moves, all intermediate waypoints should be joint waypoints
  // with the TARGET joint position assigned (standard behavior)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
    if (instr2.getPathProfile().empty())
    {
      EXPECT_EQ(mi.getProfile(), instr2.getProfile());
      EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
    }
    else
    {
      EXPECT_EQ(mi.getProfile(), instr2.getPathProfile());
      EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
    }
  }

  // The final waypoint should match the target exactly and be constrained
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
  EXPECT_TRUE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test Cartesian-to-Cartesian movement with explicit seed and NoIK behavior
 *
 * This test verifies that when a Cartesian waypoint has an explicit seed provided,
 * that seed is used instead of the current joint values. This demonstrates the
 * priority-based assignment: target waypoint with seed has highest priority.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, CartCart_WithExplicitSeed_NoIK)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, -0.1, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0.1, 1);
  // Add an explicit seed to the waypoint
  tesseract::common::JointState joint_seed;
  joint_seed.joint_names = joint_names_;
  joint_seed.position = Eigen::VectorXd::Ones(7) * 0.5;
  wp2.setSeed(joint_seed);
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSAssignNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // When explicit seed is provided, NoIK behavior should use that seed
  // instead of the current joint values
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(joint_seed.position.isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
    if (instr2.getPathProfile().empty())
    {
      EXPECT_EQ(mi.getProfile(), instr2.getProfile());
      EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
    }
    else
    {
      EXPECT_EQ(mi.getProfile(), instr2.getPathProfile());
      EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
    }
  }

  // The final waypoint should maintain the explicit seed
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(joint_seed.position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test mixed motion types with NoIK behavior
 *
 * This test verifies that motion type (FREESPACE vs LINEAR) affects waypoint generation
 * but the NoIK assignment behavior is consistent - using start joint position since
 * target is Cartesian without seed (priority-based assignment).
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, MixedMotionTypes_NoIK_Behavior)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSAssignNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For LINEAR motion type, intermediate waypoints are Cartesian but NoIK behavior
  // should assign start joint position as seed (not IK solution)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(wp1.getPosition().isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
    if (instr2.getPathProfile().empty())
    {
      EXPECT_EQ(mi.getProfile(), instr2.getProfile());
      EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
    }
    else
    {
      EXPECT_EQ(mi.getProfile(), instr2.getPathProfile());
      EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
    }
  }

  // The final waypoint should be the original Cartesian waypoint with start position as seed
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(wp1.getPosition().isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test state segment length validation for joint distance with NoIK behavior
 *
 * This test verifies that the state_longest_valid_segment_length parameter
 * correctly controls the number of steps based on joint distance while
 * maintaining NoIK assignment behavior.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, StateSegmentLength_NoIK_Validation)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Test with large state segment length (should use min_steps)
  int min_steps = 5;
  SimplePlannerLVSAssignNoIKMoveProfile profile_large(6.28, 10.0, 6.28, min_steps);
  auto move_instructions_large =
      profile_large.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions_large.size(), min_steps);

  // Test with small state segment length (should create more steps)
  double small_state_length = 0.05;
  SimplePlannerLVSAssignNoIKMoveProfile profile_small(small_state_length, 10.0, 6.28, min_steps);
  auto move_instructions_small =
      profile_small.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Calculate expected steps based on joint distance
  double joint_dist = (wp2.getPosition() - wp1.getPosition()).norm();
  int expected_steps = static_cast<int>(joint_dist / small_state_length) + 1;
  expected_steps = std::max(expected_steps, min_steps);

  EXPECT_GT(static_cast<int>(move_instructions_small.size()), min_steps);
  EXPECT_EQ(move_instructions_small.size(), expected_steps);

  // Verify that all intermediate waypoints still have the assigned target position (NoIK behavior)
  for (std::size_t i = 0; i < move_instructions_small.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions_small.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  }
}

/**
 * @brief Test translation segment length validation for Cartesian moves with NoIK behavior
 *
 * This test verifies that the translation_longest_valid_segment_length parameter
 * correctly controls the number of steps based on translation distance while
 * maintaining NoIK assignment behavior.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, TranslationSegmentLength_NoIK_Validation)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, -0.1, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0.1, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Test with large translation segment length (should use min_steps)
  int min_steps = 5;
  SimplePlannerLVSAssignNoIKMoveProfile profile_large(6.28, 10.0, 6.28, min_steps);
  auto move_instructions_large =
      profile_large.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions_large.size(), min_steps);

  // Test with small translation segment length (should create more steps)
  double small_trans_length = 0.01;
  SimplePlannerLVSAssignNoIKMoveProfile profile_small(6.28, small_trans_length, 6.28, min_steps);
  auto move_instructions_small =
      profile_small.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Calculate expected steps based on translation distance
  double trans_dist = (wp2.getTransform().translation() - wp1.getTransform().translation()).norm();
  int expected_steps = static_cast<int>(trans_dist / small_trans_length) + 1;
  expected_steps = std::max(expected_steps, min_steps);

  EXPECT_GT(static_cast<int>(move_instructions_small.size()), min_steps);
  EXPECT_EQ(move_instructions_small.size(), expected_steps);

  // Get the seed position for verification
  auto fwd_kin = env_->getJointGroup(manip_info_.manipulator);
  Eigen::VectorXd seed_position = env_->getCurrentJointValues(fwd_kin->getJointNames());

  // Verify that all intermediate waypoints use the seed position (NoIK behavior)
  // For LINEAR motion, intermediate waypoints are Cartesian with seed position
  for (std::size_t i = 0; i < move_instructions_small.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions_small.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(seed_position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  }
}

/**
 * @brief Test rotation segment length validation for orientation changes with NoIK behavior
 *
 * This test verifies that the rotation_longest_valid_segment_length parameter
 * correctly controls the number of steps based on rotation distance while
 * maintaining NoIK assignment behavior.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, RotationSegmentLength_NoIK_Validation)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  // Create a rotation difference
  wp2.getTransform().rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Test with large rotation segment length (should use min_steps)
  int min_steps = 5;
  SimplePlannerLVSAssignNoIKMoveProfile profile_large(6.28, 10.0, 6.28, min_steps);
  auto move_instructions_large =
      profile_large.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions_large.size(), min_steps);

  // Test with small rotation segment length (should create more steps)
  double small_rot_length = 0.01;
  SimplePlannerLVSAssignNoIKMoveProfile profile_small(6.28, 10.0, small_rot_length, min_steps);
  auto move_instructions_small =
      profile_small.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Calculate expected steps based on rotation distance
  double rot_dist =
      Eigen::Quaterniond(wp1.getTransform().linear()).angularDistance(Eigen::Quaterniond(wp2.getTransform().linear()));
  int expected_steps = static_cast<int>(rot_dist / small_rot_length) + 1;
  expected_steps = std::max(expected_steps, min_steps);

  EXPECT_GT(static_cast<int>(move_instructions_small.size()), min_steps);
  EXPECT_EQ(move_instructions_small.size(), expected_steps);

  // Get the seed position for verification
  auto fwd_kin = env_->getJointGroup(manip_info_.manipulator);
  Eigen::VectorXd seed_position = env_->getCurrentJointValues(fwd_kin->getJointNames());

  // Verify that all intermediate waypoints use the seed position (NoIK behavior)
  // For LINEAR motion, intermediate waypoints are Cartesian with seed position
  for (std::size_t i = 0; i < move_instructions_small.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions_small.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(seed_position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  }
}

/**
 * @brief Test minimum steps constraint validation with NoIK behavior
 *
 * This test verifies that the min_steps parameter is always respected,
 * even when segment lengths would suggest fewer steps are needed,
 * while maintaining NoIK assignment behavior.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, MinSteps_NoIK_Validation)  // NOLINT
{
  // Use nearly identical waypoints to minimize distance
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Zero(7) + Eigen::VectorXd::Constant(7, 0.001) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Test with large segment lengths but higher min_steps
  int min_steps = 15;
  SimplePlannerLVSAssignNoIKMoveProfile profile(10.0, 10.0, 10.0, min_steps);
  auto move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should always respect min_steps regardless of segment lengths
  EXPECT_EQ(move_instructions.size(), min_steps);

  // Verify all intermediate waypoints have the assigned target position (NoIK behavior)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  }
}

/**
 * @brief Test maximum steps constraint validation with NoIK behavior
 *
 * This test verifies that the max_steps parameter limits the number of steps
 * even when segment lengths would suggest more steps are needed,
 * while maintaining NoIK assignment behavior.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, MaxSteps_NoIK_Validation)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Test with very small segment lengths but limited max_steps
  int min_steps = 5;
  int max_steps = 8;
  SimplePlannerLVSAssignNoIKMoveProfile profile(0.001, 0.001, 0.001, min_steps, max_steps);
  auto move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should be limited by max_steps
  EXPECT_EQ(move_instructions.size(), max_steps);

  // Verify all intermediate waypoints have the assigned target position (NoIK behavior)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  }
}

/**
 * @brief Test edge case with large parameters (should use min_steps) with NoIK behavior
 *
 * This test verifies that when all segment length parameters are set to very large values,
 * the profile falls back to using the minimum number of steps while maintaining NoIK behavior.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, LargeParameters_MinSteps_NoIK)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Use very large segment lengths - should default to min_steps
  int min_steps = 8;
  SimplePlannerLVSAssignNoIKMoveProfile profile(100.0, 100.0, 100.0, min_steps);
  auto move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should use exactly min_steps
  EXPECT_EQ(move_instructions.size(), min_steps);

  // Verify all intermediate waypoints have the start joint position (NoIK behavior)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp1.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  }
}

/**
 * @brief Test edge case with very small parameters (should be limited by max_steps) with NoIK behavior
 *
 * This test verifies that when all segment length parameters are set to very small values,
 * the profile is limited by the maximum number of steps while maintaining NoIK behavior.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignNoIKMoveProfileUnit, SmallParameters_MaxSteps_NoIK)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, -0.1, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0.1, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Use very small segment lengths with limited max_steps
  int min_steps = 5;
  int max_steps = 12;
  SimplePlannerLVSAssignNoIKMoveProfile profile(0.0001, 0.0001, 0.0001, min_steps, max_steps);
  auto move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should be limited by max_steps
  EXPECT_EQ(move_instructions.size(), max_steps);

  // Get the seed position for verification
  auto fwd_kin = env_->getJointGroup(manip_info_.manipulator);
  Eigen::VectorXd seed_position = env_->getCurrentJointValues(fwd_kin->getJointNames());

  // Verify all intermediate waypoints have the seed position (NoIK behavior)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(seed_position.isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  }
}

/**
 * @brief Main test runner
 *
 * Initializes Google Test framework and runs all test cases.
 * This comprehensive test suite validates the SimplePlannerLVSAssignNoIKMoveProfile
 * across all waypoint type combinations, motion types, LVS parameter validation,
 * constraint validation, and edge case scenarios. The NoIK behavior ensures that
 * no inverse kinematics solving is performed, using priority-based joint position
 * assignment based on available waypoint information and seed data.
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
