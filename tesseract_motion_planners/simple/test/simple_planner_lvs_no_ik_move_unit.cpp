/**
 * @file simple_planner_lvs_no_ik_move_unit.cpp
 * @brief Unit tests for SimplePlannerLVSNoIKMoveProfile
 *
 * @author Roelof Oomen
 * @date July 16, 2025
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/types.h>
#include <tesseract_common/unit_test_utils.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/simple/cereal_serialization.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_move_profile.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>

using namespace tesseract_planning;

/**
 * @brief Test fixture for SimplePlannerLVSNoIKMoveProfile unit tests
 *
 * This test suite validates the SimplePlannerLVSNoIKMoveProfile class, which generates
 * a variable number of intermediate waypoints based on longest valid segment (LVS) criteria
 * while using NoIK behavior and position assignment. The "NoIK" behavior means:
 * - Joint-to-Cartesian: Uses start joint position as seed (no IK solving)
 * - Cartesian-to-Joint: Uses target joint position as seed (no IK solving)
 * - Cartesian-to-Cartesian: Uses environment's current joint state as seed (no IK solving)
 * This combines adaptive step sizing with NoIK seed assignment behavior.
 */
class TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit : public TesseractPlanningSimplePlannerUnit
{
};

TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, Serialization)  // NOLINT
{
  auto profile = std::make_shared<SimplePlannerLVSNoIKMoveProfile>(3.14, 0.5, 1.57, 5);
  // Serialization
  tesseract_common::testSerializationDerivedClass<tesseract_common::Profile, SimplePlannerLVSNoIKMoveProfile>(profile,
                                                                                                              "SimplePl"
                                                                                                              "annerLVS"
                                                                                                              "NoIKMove"
                                                                                                              "Profil"
                                                                                                              "e");
}

/**
 * @brief Test Joint-to-Joint movement with NoIK behavior for freespace motion
 *
 * This test verifies that when both start and end waypoints are joint waypoints
 * with freespace motion, the profile interpolates between them normally.
 * Step count adapts based on LVS joint distance criteria.
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, JointJoint_NoIK_Freespace)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For freespace joint-to-joint moves, intermediate waypoints should be joint waypoints
  // with interpolated positions between start and end
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
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
 * @brief Test Joint-to-Joint movement with NoIK behavior for linear motion
 *
 * This test verifies that when both start and end waypoints are joint waypoints
 * with linear motion, intermediate waypoints are Cartesian with interpolated poses
 * and use seed positions for joint values.
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, JointJoint_NoIK_Linear)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For linear joint-to-joint moves, intermediate waypoints should be Cartesian
  // with interpolated poses between start and end transforms
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
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
 * @brief Test Joint-to-Cartesian movement with NoIK behavior for freespace motion
 *
 * This test verifies that when the start waypoint is a joint waypoint and the end
 * waypoint is a Cartesian waypoint with freespace motion, the NoIK behavior creates
 * intermediate joint waypoints that all use the start joint position (no IK solving
 * for intermediate waypoints).
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, JointCart_NoIK_Freespace)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For freespace joint-to-cartesian moves with NoIK, intermediate waypoints should be joint waypoints
  // All intermediate waypoints use the start joint position (no IK solving)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
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

  // The final waypoint should be the original Cartesian waypoint
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test Joint-to-Cartesian movement with NoIK behavior for linear motion
 *
 * This test verifies that when the start waypoint is a joint waypoint and the end
 * waypoint is a Cartesian waypoint with linear motion, intermediate waypoints are
 * Cartesian with start joint position used as seed (no IK solving).
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, JointCart_NoIK_Linear)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For linear joint-to-cartesian moves with NoIK, intermediate waypoints should be Cartesian
  // with start joint position used as seed for all intermediate waypoints
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
 * @brief Test Cartesian-to-Joint movement with NoIK behavior for freespace motion
 *
 * This test verifies that when the start waypoint is a Cartesian waypoint and the end
 * waypoint is a joint waypoint with freespace motion, the NoIK behavior creates
 * intermediate joint waypoints that all use the target joint position (no IK solving).
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, CartJoint_NoIK_Freespace)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  CartesianWaypoint wp1{ joint_group->calcFwdKin(Eigen::VectorXd::Zero(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For freespace cartesian-to-joint moves with NoIK, intermediate waypoints should be joint waypoints
  // All intermediate waypoints use the target joint position (no IK solving)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
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
 * @brief Test Cartesian-to-Joint movement with NoIK behavior for linear motion
 *
 * This test verifies that when the start waypoint is a Cartesian waypoint and the end
 * waypoint is a joint waypoint with linear motion, intermediate waypoints are Cartesian
 * with the target joint position used as seed (no IK solving).
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, CartJoint_NoIK_Linear)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  CartesianWaypoint wp1{ joint_group->calcFwdKin(Eigen::VectorXd::Zero(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For Cartesian-to-Joint linear motion, the target joint position is used as seed
  // This is because the interpolation uses j2.replicate(1, steps + 1) where j2 is the target joint position
  const Eigen::VectorXd& expected_seed = wp2.getPosition();

  // For linear cartesian-to-joint moves with NoIK, intermediate waypoints should be Cartesian
  // with target joint position used as seed
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(expected_seed.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
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
 * @brief Test Cartesian-to-Cartesian movement with NoIK behavior for freespace motion
 *
 * This test verifies that when both start and end waypoints are Cartesian waypoints
 * with freespace motion, the NoIK behavior creates intermediate joint waypoints that
 * all use the environment's current joint state (no IK solving).
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, CartCart_NoIK_Freespace)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  CartesianWaypoint wp1{ joint_group->calcFwdKin(Eigen::VectorXd::Zero(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For freespace cartesian-to-cartesian moves with NoIK, intermediate waypoints should be joint waypoints
  // All intermediate waypoints use the environment's current joint state (no IK solving)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
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

  // The final waypoint should be the original Cartesian waypoint with the environment's current joint state as seed
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  EXPECT_TRUE(instr1_seed.getWaypoint().as<JointWaypointPoly>().getPosition().isApprox(
      mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test Cartesian-to-Cartesian movement with NoIK behavior for linear motion
 *
 * This test verifies that when both start and end waypoints are Cartesian waypoints
 * with linear motion, intermediate waypoints are Cartesian with the environment's
 * current joint state used as seed consistently throughout the trajectory (no IK solving).
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, CartCart_NoIK_Linear)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  CartesianWaypoint wp1{ joint_group->calcFwdKin(Eigen::VectorXd::Zero(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // Get seed position from the seed instruction waypoint (environment's current joint state)
  const Eigen::VectorXd& seed_position = instr1_seed.getWaypoint().as<JointWaypointPoly>().getPosition();

  // For linear cartesian-to-cartesian moves with NoIK, intermediate waypoints should be Cartesian
  // with the environment's current joint state used as seed
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(seed_position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
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

  // The final waypoint should be the original Cartesian waypoint with the environment's current joint state as seed
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  EXPECT_TRUE(seed_position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

// ============================================================================
// Phase 2: LVS Parameter Validation Tests
// ============================================================================

/**
 * @brief Test LVS translation segment length validation with NoIK behavior
 *
 * This test validates that the LVS translation segment length parameter correctly
 * controls the number of intermediate waypoints generated based on translation distance.
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, TranslationSegmentLength_NoIK_Validation)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  // Create start and end poses with significant translation distance
  Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();
  start_pose.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  CartesianWaypoint wp1{ start_pose };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  Eigen::Isometry3d end_pose = Eigen::Isometry3d::Identity();
  end_pose.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);  // 2 meters translation distance
  CartesianWaypoint wp2{ end_pose };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Test with stricter translation segment length (should produce more waypoints)
  SimplePlannerLVSNoIKMoveProfile profile_strict(3.14, 0.1, 1.57, 5);  // 0.1m translation segments
  std::vector<MoveInstructionPoly> move_instructions_strict =
      profile_strict.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Test with looser translation segment length (should produce fewer waypoints)
  SimplePlannerLVSNoIKMoveProfile profile_loose(3.14, 1.0, 1.57, 5);  // 1.0m translation segments
  std::vector<MoveInstructionPoly> move_instructions_loose =
      profile_loose.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Stricter translation limits should produce more waypoints
  EXPECT_GT(move_instructions_strict.size(), move_instructions_loose.size());

  // Both should have at least min_steps
  EXPECT_GE(move_instructions_strict.size(), 5);
  EXPECT_GE(move_instructions_loose.size(), 5);

  // All waypoints should be Cartesian for linear motion with NoIK
  for (const auto& mi : move_instructions_strict)
  {
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  }
  for (const auto& mi : move_instructions_loose)
  {
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  }
}

/**
 * @brief Test LVS rotation segment length validation with NoIK behavior
 *
 * This test validates that the LVS rotation segment length parameter correctly
 * controls the number of intermediate waypoints generated based on rotation distance.
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, RotationSegmentLength_NoIK_Validation)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  // Create start and end poses with significant rotation distance
  Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();
  CartesianWaypoint wp1{ start_pose };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  Eigen::Isometry3d end_pose = Eigen::Isometry3d::Identity();
  end_pose.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();  // 180 degree rotation
  CartesianWaypoint wp2{ end_pose };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Test with stricter rotation segment length (should produce more waypoints)
  SimplePlannerLVSNoIKMoveProfile profile_strict(3.14, 0.5, 0.1, 5);  // 0.1 rad rotation segments
  std::vector<MoveInstructionPoly> move_instructions_strict =
      profile_strict.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Test with looser rotation segment length (should produce fewer waypoints)
  SimplePlannerLVSNoIKMoveProfile profile_loose(3.14, 0.5, 1.0, 5);  // 1.0 rad rotation segments
  std::vector<MoveInstructionPoly> move_instructions_loose =
      profile_loose.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Stricter rotation limits should produce more waypoints
  EXPECT_GT(move_instructions_strict.size(), move_instructions_loose.size());

  // Both should have at least min_steps
  EXPECT_GE(move_instructions_strict.size(), 5);
  EXPECT_GE(move_instructions_loose.size(), 5);

  // All waypoints should be Cartesian for linear motion with NoIK
  for (const auto& mi : move_instructions_strict)
  {
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  }
  for (const auto& mi : move_instructions_loose)
  {
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  }
}

/**
 * @brief Test LVS min_steps parameter validation with NoIK behavior
 *
 * This test validates that the min_steps parameter correctly enforces the minimum
 * number of waypoints generated regardless of segment length parameters.
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, MinSteps_NoIK_Validation)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  // Very small movement that normally wouldn't require many steps
  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Constant(7, 0.001) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Test with high min_steps requirement
  SimplePlannerLVSNoIKMoveProfile profile_high_min(3.14, 0.5, 1.57, 20);  // 20 minimum steps
  std::vector<MoveInstructionPoly> move_instructions_high_min =
      profile_high_min.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Test with low min_steps requirement
  SimplePlannerLVSNoIKMoveProfile profile_low_min(3.14, 0.5, 1.57, 3);  // 3 minimum steps
  std::vector<MoveInstructionPoly> move_instructions_low_min =
      profile_low_min.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // High min_steps should produce more waypoints
  EXPECT_GT(move_instructions_high_min.size(), move_instructions_low_min.size());

  // Both should respect their minimum step requirements
  EXPECT_GE(move_instructions_high_min.size(), 20);
  EXPECT_GE(move_instructions_low_min.size(), 3);

  // All waypoints should be joint waypoints for freespace motion
  for (const auto& mi : move_instructions_high_min)
  {
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
  }
  for (const auto& mi : move_instructions_low_min)
  {
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
  }
}

// ============================================================================
// Phase 3: Constraint Validation Tests
// ============================================================================

/**
 * @brief Test constraint preservation with NoIK behavior
 *
 * This test validates that when waypoints have constraints (fixed/toleranced),
 * the NoIK behavior properly preserves these constraints in the generated trajectory.
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, ConstraintPreservation_NoIK_Behavior)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  // Create constrained joint waypoint
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  wp1.setIsConstrained(true);  // Set start waypoint as constrained
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  // Create constrained joint waypoint
  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  wp2.setIsConstrained(true);  // Set end waypoint as constrained
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // Intermediate waypoints should be unconstrained joint waypoints
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  }

  // Final waypoint should preserve constraint from target
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
  EXPECT_TRUE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
}

/**
 * @brief Test mixed motion types with constraint handling and NoIK behavior
 *
 * This test validates that when waypoints have mixed motion types (FREESPACE/LINEAR)
 * and constraints, the NoIK behavior properly handles both while maintaining constraints.
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, MixedMotionTypes_NoIK_Behavior)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  // Create constrained joint waypoint with freespace motion
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  wp1.setIsConstrained(true);
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  // Create constrained cartesian waypoint with linear motion
  CartesianWaypoint wp2{ joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For joint-to-cartesian linear motion with NoIK, intermediate waypoints should be Cartesian
  // with start joint position used as seed
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(wp1.getPosition().isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  }

  // Final waypoint should be the original Cartesian waypoint with start position as seed
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  EXPECT_TRUE(wp1.getPosition().isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
}

// ============================================================================
// Phase 4: Seed Handling Tests
// ============================================================================

/**
 * @brief Test seed position handling with NoIK behavior
 *
 * This test validates that the NoIK behavior properly uses seed positions
 * for Joint-to-Cartesian linear motion where the start joint position is used as seed.
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, SeedPositionHandling_NoIK_Behavior)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  // Create custom start joint position
  Eigen::VectorXd custom_start = Eigen::VectorXd::Constant(7, 0.5);

  JointWaypoint wp1{ joint_names_, custom_start };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For Joint-to-Cartesian linear motion with NoIK, all waypoints should be Cartesian
  // with the start joint position used as seed
  for (const auto& mi : move_instructions)
  {
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(custom_start.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  }
}

/**
 * @brief Test seed position consistency with NoIK behavior
 *
 * This test validates that the NoIK behavior maintains consistent seed positions
 * throughout different trajectory segments and motion types.
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, SeedPositionConsistency_NoIK_Behavior)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  // Create custom seed position
  Eigen::VectorXd custom_seed = Eigen::VectorXd::Constant(7, 0.25);

  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, custom_seed);

  CartesianWaypoint wp2{ joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 10);  // More steps for better validation
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 10);

  // For joint-to-cartesian linear motion with NoIK, intermediate waypoints should be Cartesian
  // with start joint position used as seed (from wp1, not custom_seed)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(wp1.getPosition().isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  }

  // Final waypoint should be the original Cartesian waypoint with start position as seed
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  EXPECT_TRUE(wp1.getPosition().isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
}

// ============================================================================
// Phase 5: Edge Cases Tests
// ============================================================================

/**
 * @brief Test empty instruction handling with NoIK behavior
 *
 * This test validates that the profile properly handles empty or null instructions
 * and returns appropriate results without crashing.
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, EmptyInstructions_NoIK_Behavior)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;  // Empty instruction

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should still generate valid trajectory with at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // All waypoints should be valid joint waypoints
  for (const auto& mi : move_instructions)
  {
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
  }

  // Final waypoint should match target
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
}

/**
 * @brief Test identical waypoints with NoIK behavior
 *
 * This test validates that the profile properly handles cases where start and end
 * waypoints are identical, ensuring it still generates the minimum required steps.
 */
TEST_F(TesseractPlanningSimplePlannerLVSNoIKMoveProfileUnit, IdenticalWaypoints_NoIK_Behavior)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  // Create identical waypoints
  Eigen::VectorXd same_position = Eigen::VectorXd::Constant(7, 0.5);

  JointWaypoint wp1{ joint_names_, same_position };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, same_position };  // Identical to wp1
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSNoIKMoveProfile profile(3.14, 0.5, 1.57, 8);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());

  // Should still generate at least min_steps even with identical waypoints
  EXPECT_GE(move_instructions.size(), 8);

  // All waypoints should be valid joint waypoints
  for (const auto& mi : move_instructions)
  {
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
  }

  // All waypoints should have the same position (interpolating between identical points)
  for (const auto& mi : move_instructions)
  {
    EXPECT_TRUE(same_position.isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
  }

  // Final waypoint should match target
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
  EXPECT_TRUE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
