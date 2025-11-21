/**
 * @file simple_planner_fixed_size_assign_no_ik_move_unit.cpp
 * @brief Unit tests for SimplePlannerFixedSizeAssignNoIKMoveProfile
 *
 * @author Levi Armstrong
 * @date July 28, 2020
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
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_no_ik_move_profile.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>

using namespace tesseract_planning;

/**
 * @brief Test fixture for SimplePlannerFixedSizeAssignNoIKMoveProfile unit tests
 *
 * This test suite validates the SimplePlannerFixedSizeAssignNoIKMoveProfile class, which generates
 * a fixed number of intermediate waypoints using NoIK assignment behavior. The "NoIK" behavior means:
 * - Joint-to-Joint: Target joint position is assigned to all intermediate waypoints
 * - Joint-to-Cartesian: Start joint position is assigned (no IK solving)
 * - Cartesian-to-Joint: Target joint position is assigned to all intermediate waypoints
 * - Cartesian-to-Cartesian: Seed joint position is assigned (no IK solving)
 * This combines fixed step sizing with NoIK assignment behavior.
 */
class TesseractPlanningSimplePlannerFixedSizeAssignNoIKMoveProfileUnit : public TesseractPlanningSimplePlannerUnit
{
};

TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignNoIKMoveProfileUnit, Serialization)  // NOLINT
{
  auto profile = std::make_shared<SimplePlannerFixedSizeAssignNoIKMoveProfile>(10, 10);
  // Serialization
  tesseract_common::testSerializationDerivedClass<tesseract_common::Profile,
                                                  SimplePlannerFixedSizeAssignNoIKMoveProfile>(profile,
                                                                                               "SimplePlannerFixedSizeA"
                                                                                               "ssignNoIKMoveProfile");
}

/**
 * @brief Test Joint-to-Joint movement with NoIK assign behavior for freespace motion
 *
 * This test verifies that when both start and end waypoints are joint waypoints
 * and the motion type is FREESPACE, all intermediate waypoints are assigned
 * the target joint position directly without interpolation. This is the NoIK
 * assignment behavior for joint-to-joint movements.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignNoIKMoveProfileUnit, JointToJoint_AssignTarget_Freespace)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignNoIKMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // For joint-to-joint moves with NoIK, all intermediate waypoints should be joint waypoints
  // with the target position assigned directly (no interpolation)
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
 * @brief Test Joint-to-Joint movement with NoIK assign behavior for linear motion
 *
 * This test verifies that when both start and end waypoints are joint waypoints
 * and the motion type is LINEAR, intermediate waypoints are converted to Cartesian
 * waypoints with the target joint position used as the seed for all intermediate waypoints.
 * This demonstrates NoIK behavior for linear joint-to-joint movements.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignNoIKMoveProfileUnit, JointToJoint_AssignTarget_Linear)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignNoIKMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // For linear joint-to-joint moves with NoIK, intermediate waypoints should be Cartesian
  // with the target joint position used as seed for all intermediate waypoints
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
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
 * @brief Test Joint-to-Cartesian movement with NoIK assign behavior for freespace motion
 *
 * This test verifies that when the start waypoint is a joint waypoint and the end
 * waypoint is a Cartesian waypoint with freespace motion, the start joint position
 * is assigned to all intermediate waypoints (no IK solving). This demonstrates
 * NoIK behavior where the start joint position is used instead of solving IK.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignNoIKMoveProfileUnit, JointToCart_AssignStart_Freespace)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignNoIKMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // For joint-to-cart moves with NoIK, all intermediate waypoints should be joint waypoints
  // with the start joint position assigned (no IK solving)
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
 * @brief Test Joint-to-Cartesian movement with NoIK assign behavior for linear motion
 *
 * This test verifies that when the start waypoint is a joint waypoint and the end
 * waypoint is a Cartesian waypoint with linear motion, intermediate waypoints are
 * Cartesian waypoints with the start joint position used as the seed for all
 * intermediate waypoints (no IK solving). This demonstrates NoIK behavior for
 * linear joint-to-Cartesian movements.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignNoIKMoveProfileUnit, JointToCart_AssignStart_Linear)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignNoIKMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // For linear joint-to-cart moves with NoIK, intermediate waypoints should be Cartesian
  // with the start joint position used as seed for all intermediate waypoints (no IK solving)
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
 * @brief Test Joint-to-Cartesian movement with NoIK assign behavior (deprecated naming)
 *
 * This test verifies the same behavior as JointToCart_AssignStart_Freespace but with
 * legacy naming. The start joint position is assigned to all intermediate waypoints
 * instead of solving IK for the target Cartesian position.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignNoIKMoveProfileUnit, JointCartesian_AssignJointPosition)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignNoIKMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);
  // For joint-to-cart moves with NoIK, all intermediate waypoints should be joint waypoints
  // with the start joint position assigned (no IK solving)
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
 * @brief Test Cartesian-to-Joint movement with NoIK assign behavior
 *
 * This test verifies that when the start waypoint is a Cartesian waypoint and the end
 * waypoint is a joint waypoint, the target joint position is assigned to all
 * intermediate waypoints. This demonstrates NoIK behavior for Cartesian-to-joint movements.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignNoIKMoveProfileUnit, CartesianJoint_AssignJointPosition)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypointPoly(JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_)));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignNoIKMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);
  // For cart-to-joint moves with NoIK, all intermediate waypoints should be joint waypoints
  // with the target joint position assigned
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
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
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test Cartesian-to-Cartesian movement with NoIK assign behavior for freespace motion
 *
 * This test verifies that when both start and end waypoints are Cartesian waypoints
 * with freespace motion, the seed joint position is assigned to all intermediate
 * waypoints (no IK solving). This demonstrates NoIK behavior for Cartesian-to-Cartesian
 * movements where the current joint values are used consistently.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignNoIKMoveProfileUnit,
       CartesianCartesian_AssignJointPosition)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignNoIKMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  auto fwd_kin = env_->getJointGroup(manip_info_.manipulator);
  Eigen::VectorXd position = env_->getCurrentJointValues(fwd_kin->getJointNames());
  EXPECT_EQ(move_instructions.size(), 10);
  // For cart-to-cart moves with NoIK, all intermediate waypoints should be joint waypoints
  // with the seed joint position assigned (no IK solving)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(position.isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
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
  EXPECT_TRUE(position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test Cartesian-to-Joint movement with NoIK assign behavior for linear motion
 *
 * This test verifies that when the start waypoint is a Cartesian waypoint and the end
 * waypoint is a joint waypoint with linear motion, intermediate waypoints are Cartesian
 * waypoints with the target joint position used as the seed. This demonstrates NoIK
 * behavior for linear Cartesian-to-joint movements.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignNoIKMoveProfileUnit, CartToJoint_AssignTarget_Linear)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypointPoly(JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_)));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignNoIKMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // For linear cart-to-joint moves with NoIK, intermediate waypoints should be Cartesian
  // with the target joint position used as seed
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
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
 * @brief Test Cartesian-to-Cartesian movement with NoIK assign behavior for linear motion
 *
 * This test verifies that when both start and end waypoints are Cartesian waypoints
 * with linear motion, intermediate waypoints are Cartesian waypoints with the seed
 * joint position assigned to all intermediate waypoints (no IK solving). This demonstrates
 * NoIK behavior for linear Cartesian-to-Cartesian movements with pose interpolation.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignNoIKMoveProfileUnit, CartToCart_AssignSeed_Linear)  // NOLINT
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

  SimplePlannerFixedSizeAssignNoIKMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  auto fwd_kin = env_->getJointGroup(manip_info_.manipulator);
  Eigen::VectorXd position = env_->getCurrentJointValues(fwd_kin->getJointNames());
  EXPECT_EQ(move_instructions.size(), 10);

  // For linear cart-to-cart moves with NoIK, intermediate waypoints should be Cartesian
  // with the seed joint position assigned throughout (no IK solving)
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
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
  EXPECT_TRUE(position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Main test runner
 *
 * Initializes Google Test framework and runs all test cases.
 * This comprehensive test suite validates the SimplePlannerFixedSizeAssignNoIKMoveProfile
 * across all waypoint type combinations and motion types. The NoIK behavior ensures that
 * no inverse kinematics solving is performed, using predetermined joint positions
 * based on the waypoint types and available seed information:
 * - Joint-to-Joint: Target position assigned
 * - Joint-to-Cartesian: Start position assigned
 * - Cartesian-to-Joint: Target position assigned
 * - Cartesian-to-Cartesian: Seed position assigned
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
