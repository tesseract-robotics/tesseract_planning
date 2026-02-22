/**
 * @file simple_planner_lvs_assign_move_unit.cpp
 * @brief Unit tests for SimplePlannerLVSAssignMoveProfile
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

#include <tesseract_common/types.h>
#include <tesseract_common/unit_test_utils.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/simple/cereal_serialization.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_assign_move_profile.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_kinematics/core/kinematic_group.h>

using namespace tesseract::motion_planners;
using namespace tesseract::command_language;

/**
 * @brief Test fixture for SimplePlannerLVSAssignMoveProfile unit tests
 *
 * This test suite validates the SimplePlannerLVSAssignMoveProfile class, which generates
 * a variable number of intermediate waypoints based on longest valid segment (LVS) criteria
 * while assigning the target joint position to all intermediate waypoints. This combines
 * adaptive step sizing with position assignment behavior.
 */
class TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit : public TesseractPlanningSimplePlannerUnit
{
};

TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, Serialization)  // NOLINT
{
  auto profile = std::make_shared<SimplePlannerLVSAssignMoveProfile>(3.14, 0.5, 1.57, 5);
  // Serialization
  tesseract::common::testSerializationDerivedClass<tesseract::common::Profile, SimplePlannerLVSAssignMoveProfile>(
      profile,
      "Simple"
      "Planne"
      "rLVSAs"
      "signMo"
      "veProf"
      "ile");
}

/**
 * @brief Test Joint-to-Joint movement with freespace motion type
 *
 * This test verifies that when both start and end waypoints are joint waypoints
 * and the motion type is FREESPACE, all intermediate waypoints are assigned
 * the target joint position with step count determined by LVS criteria.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, JointJoint_AssignJointPosition_Freespace)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSAssignMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For freespace joint-to-joint moves, all intermediate waypoints should be joint waypoints
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
 * @brief Test Joint-to-Joint movement with linear motion type
 *
 * This test verifies that when both start and end waypoints are joint waypoints
 * and the motion type is LINEAR, intermediate waypoints are converted to Cartesian
 * waypoints with the target joint position used as the seed for IK solving.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, JointJoint_AssignJointPosition_Linear)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSAssignMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For linear moves, intermediate waypoints should be Cartesian with joint seed
  // The joint position from the target is used as the seed for all intermediate waypoints
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
 * @brief Test Joint-to-Cartesian movement with freespace motion type
 *
 * This test verifies that when the start waypoint is a joint waypoint and the end
 * waypoint is a Cartesian waypoint with freespace motion, the IK solution for the
 * Cartesian target is assigned to all intermediate joint waypoints.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, JointCart_AssignJointPosition_Freespace)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSAssignMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // Solve IK for the Cartesian waypoint to get expected joint position
  // This demonstrates how the planner converts Cartesian targets to joint positions
  auto kin_group = env_->getKinematicGroup(manip_info_.manipulator);
  tesseract::kinematics::KinGroupIKInput ik_input(wp2.getTransform(), manip_info_.working_frame, manip_info_.tcp_frame);
  const Eigen::VectorXd& seed = wp1.getPosition();
  tesseract::kinematics::IKSolutions ik_solutions = kin_group->calcInvKin({ ik_input }, seed);
  EXPECT_FALSE(ik_solutions.empty());
  const Eigen::VectorXd& expected_joint_position = ik_solutions[0];

  // For freespace moves, intermediate waypoints are joint waypoints with the IK solution
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(expected_joint_position.isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
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

  // The final waypoint should be the original Cartesian waypoint with the IK solution as seed
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(expected_joint_position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test Joint-to-Cartesian movement with linear motion type
 *
 * This test verifies that when the start waypoint is a joint waypoint and the end
 * waypoint is a Cartesian waypoint with linear motion, intermediate waypoints are
 * Cartesian waypoints with the IK solution as the seed.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, JointCart_AssignJointPosition_Linear)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSAssignMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // Solve IK for the Cartesian waypoint to get expected joint position
  auto kin_group = env_->getKinematicGroup(manip_info_.manipulator);
  tesseract::kinematics::KinGroupIKInput ik_input(wp2.getTransform(), manip_info_.working_frame, manip_info_.tcp_frame);
  const Eigen::VectorXd& seed = wp1.getPosition();
  tesseract::kinematics::IKSolutions ik_solutions = kin_group->calcInvKin({ ik_input }, seed);
  EXPECT_FALSE(ik_solutions.empty());
  const Eigen::VectorXd& expected_joint_position = ik_solutions[0];

  // For linear moves, intermediate waypoints should be Cartesian with joint seed
  // The IK solution is used as the seed for all intermediate Cartesian waypoints
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(
        expected_joint_position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
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

  // The final waypoint should be the original Cartesian waypoint with the IK solution as seed
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(expected_joint_position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test Cartesian-to-Joint movement with freespace motion type
 *
 * This test verifies that when the start waypoint is a Cartesian waypoint and the end
 * waypoint is a joint waypoint with freespace motion, the target joint position is
 * assigned to all intermediate joint waypoints.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, CartJoint_AssignJointPosition_Freespace)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSAssignMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For freespace moves with joint target, intermediate waypoints are joint waypoints
  // with the target joint position assigned directly
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
 * @brief Test Cartesian-to-Joint movement with linear motion type
 *
 * This test verifies that when the start waypoint is a Cartesian waypoint and the end
 * waypoint is a joint waypoint with linear motion, intermediate waypoints are Cartesian
 * waypoints with the target joint position used as the seed.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, CartJoint_AssignJointPosition_Linear)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSAssignMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // For linear moves, intermediate waypoints should be Cartesian with joint seed
  // The target joint position is used as the seed for all intermediate Cartesian waypoints
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
 * @brief Test Cartesian-to-Cartesian movement with freespace motion type
 *
 * This test verifies that when both start and end waypoints are Cartesian waypoints
 * with freespace motion, the IK solution for the target Cartesian waypoint is assigned
 * to all intermediate joint waypoints.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, CartCart_AssignJointPosition_Freespace)  // NOLINT
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

  SimplePlannerLVSAssignMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // Solve IK for the Cartesian waypoint to get expected joint position
  // This demonstrates the conversion from Cartesian target to joint assignment
  auto kin_group = env_->getKinematicGroup(manip_info_.manipulator);
  tesseract::kinematics::KinGroupIKInput ik_input(wp2.getTransform(), manip_info_.working_frame, manip_info_.tcp_frame);
  Eigen::VectorXd seed = env_->getCurrentJointValues(joint_names_);
  tesseract::kinematics::IKSolutions ik_solutions = kin_group->calcInvKin({ ik_input }, seed);
  EXPECT_FALSE(ik_solutions.empty());
  const Eigen::VectorXd& expected_joint_position = ik_solutions[0];

  // For freespace moves, intermediate waypoints are joint waypoints with the IK solution
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(expected_joint_position.isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
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

  // The final waypoint should be the original Cartesian waypoint with the IK solution as seed
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(expected_joint_position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test Cartesian-to-Cartesian movement with linear motion type
 *
 * This test verifies that when both start and end waypoints are Cartesian waypoints
 * with linear motion, intermediate waypoints are Cartesian waypoints with the IK solution
 * for the target as the seed. The poses are interpolated linearly between start and end.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, CartCart_AssignJointPosition_Linear)  // NOLINT
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

  SimplePlannerLVSAssignMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // Solve IK for the Cartesian waypoint to get expected joint position
  auto kin_group = env_->getKinematicGroup(manip_info_.manipulator);
  tesseract::kinematics::KinGroupIKInput ik_input(wp2.getTransform(), manip_info_.working_frame, manip_info_.tcp_frame);
  Eigen::VectorXd seed = env_->getCurrentJointValues(joint_names_);
  tesseract::kinematics::IKSolutions ik_solutions = kin_group->calcInvKin({ ik_input }, seed);
  EXPECT_FALSE(ik_solutions.empty());
  const Eigen::VectorXd& expected_joint_position = ik_solutions[0];

  // For linear moves, intermediate waypoints should be Cartesian with joint seed
  // The IK solution is used as the seed for all intermediate Cartesian waypoints
  // The poses should be interpolated between start and end
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(
        expected_joint_position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));

    // Check that poses are interpolated between start and end
    const Eigen::Isometry3d& pose = mi.getWaypoint().as<CartesianWaypointPoly>().getTransform();
    double alpha = static_cast<double>(i + 1) / static_cast<double>(move_instructions.size());
    Eigen::Vector3d expected_translation =
        (1.0 - alpha) * wp1.getTransform().translation() + alpha * wp2.getTransform().translation();
    EXPECT_TRUE(expected_translation.isApprox(pose.translation(), 1e-5));

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

  // The final waypoint should be the original Cartesian waypoint with the IK solution as seed
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(expected_joint_position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  EXPECT_TRUE(wp2.getTransform().isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getTransform(), 1e-5));
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test state segment length validation for joint distance
 *
 * This test verifies that the state_longest_valid_segment_length parameter
 * correctly controls the number of steps based on joint distance between waypoints.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, StateSegmentLength_Validation)  // NOLINT
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
  SimplePlannerLVSAssignMoveProfile profile_large(6.28, 10.0, 6.28, min_steps);
  auto move_instructions_large =
      profile_large.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions_large.size(), min_steps);

  // Test with small state segment length (should create more steps)
  double small_state_length = 0.05;
  SimplePlannerLVSAssignMoveProfile profile_small(small_state_length, 10.0, 6.28, min_steps);
  auto move_instructions_small =
      profile_small.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Calculate expected steps based on joint distance
  double joint_dist = (wp2.getPosition() - wp1.getPosition()).norm();
  int expected_steps = static_cast<int>(joint_dist / small_state_length) + 1;
  expected_steps = std::max(expected_steps, min_steps);

  EXPECT_GT(static_cast<int>(move_instructions_small.size()), min_steps);
  EXPECT_EQ(move_instructions_small.size(), expected_steps);

  // Verify that all intermediate waypoints still have the assigned target position
  for (std::size_t i = 0; i < move_instructions_small.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions_small.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  }
}

/**
 * @brief Test translation segment length validation for linear moves
 *
 * This test verifies that the translation_longest_valid_segment_length parameter
 * correctly controls the number of steps based on translation distance in linear moves.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, TranslationSegmentLength_Validation)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Test with large translation segment length (should use min_steps)
  int min_steps = 5;
  SimplePlannerLVSAssignMoveProfile profile_large(6.28, 10.0, 6.28, min_steps);
  auto move_instructions_large =
      profile_large.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions_large.size(), min_steps);

  // Test with small translation segment length (should create more steps)
  double small_trans_length = 0.01;
  SimplePlannerLVSAssignMoveProfile profile_small(6.28, small_trans_length, 6.28, min_steps);
  auto move_instructions_small =
      profile_small.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Calculate expected steps based on translation distance
  Eigen::Isometry3d p1 = joint_group->calcFwdKin(wp1.getPosition()).at(manip_info_.tcp_frame);
  Eigen::Isometry3d p2 = joint_group->calcFwdKin(wp2.getPosition()).at(manip_info_.tcp_frame);
  double trans_dist = (p2.translation() - p1.translation()).norm();
  int expected_steps = static_cast<int>(trans_dist / small_trans_length) + 1;
  expected_steps = std::max(expected_steps, min_steps);

  EXPECT_GT(static_cast<int>(move_instructions_small.size()), min_steps);
  EXPECT_EQ(move_instructions_small.size(), expected_steps);

  // Verify that all intermediate waypoints are Cartesian with the assigned target joint seed
  for (std::size_t i = 0; i < move_instructions_small.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions_small.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  }
}

/**
 * @brief Test rotation segment length validation for linear moves
 *
 * This test verifies that the rotation_longest_valid_segment_length parameter
 * correctly controls the number of steps based on rotation distance in linear moves.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, RotationSegmentLength_Validation)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Test with large rotation segment length (should use min_steps)
  int min_steps = 5;
  SimplePlannerLVSAssignMoveProfile profile_large(6.28, 10.0, 6.28, min_steps);
  auto move_instructions_large =
      profile_large.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions_large.size(), min_steps);

  // Test with small rotation segment length (should create more steps)
  double small_rot_length = 0.01;
  SimplePlannerLVSAssignMoveProfile profile_small(6.28, 10.0, small_rot_length, min_steps);
  auto move_instructions_small =
      profile_small.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Calculate expected steps based on rotation distance
  Eigen::Isometry3d p1 = joint_group->calcFwdKin(wp1.getPosition()).at(manip_info_.tcp_frame);
  Eigen::Isometry3d p2 = joint_group->calcFwdKin(wp2.getPosition()).at(manip_info_.tcp_frame);
  double rot_dist = Eigen::Quaterniond(p1.linear()).angularDistance(Eigen::Quaterniond(p2.linear()));
  int expected_steps = static_cast<int>(rot_dist / small_rot_length) + 1;
  expected_steps = std::max(expected_steps, min_steps);

  EXPECT_GT(static_cast<int>(move_instructions_small.size()), min_steps);
  EXPECT_EQ(move_instructions_small.size(), expected_steps);

  // Verify that all intermediate waypoints are Cartesian with the assigned target joint seed
  for (std::size_t i = 0; i < move_instructions_small.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions_small.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  }
}

/**
 * @brief Test minimum steps constraint validation
 *
 * This test verifies that the min_steps parameter is always respected,
 * even when segment lengths would suggest fewer steps are needed.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, MinSteps_Validation)  // NOLINT
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
  SimplePlannerLVSAssignMoveProfile profile(10.0, 10.0, 10.0, min_steps);
  auto move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should always respect min_steps regardless of segment lengths
  EXPECT_EQ(move_instructions.size(), min_steps);

  // Verify all intermediate waypoints have the assigned target position
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  }
}

/**
 * @brief Test maximum steps constraint validation
 *
 * This test verifies that the max_steps parameter limits the number of steps
 * even when segment lengths would suggest more steps are needed.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, MaxSteps_Validation)  // NOLINT
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
  SimplePlannerLVSAssignMoveProfile profile(0.001, 0.001, 0.001, min_steps, max_steps);
  auto move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should be limited by max_steps
  EXPECT_EQ(move_instructions.size(), max_steps);

  // Verify all intermediate waypoints have the assigned target position
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  }
}

/**
 * @brief Test Cartesian-to-Cartesian movement with explicit seed
 *
 * This test verifies that when a Cartesian waypoint has an explicit seed provided,
 * that seed is used instead of solving IK. This demonstrates the priority of
 * explicit seeds over IK solutions.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, CartCart_WithSeed_AssignJointPosition)  // NOLINT
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

  SimplePlannerLVSAssignMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should have at least min_steps
  EXPECT_GE(move_instructions.size(), 5);

  // When seed is provided, it should use the seed joint position instead of IK solving
  // This demonstrates the priority of explicit seeds over computed IK solutions
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
 * @brief Test edge case with large parameters (should use min_steps)
 *
 * This test verifies that when all segment length parameters are set to very large values,
 * the profile falls back to using the minimum number of steps.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, LargeParameters_MinSteps)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Use very large segment lengths - should default to min_steps
  int min_steps = 8;
  SimplePlannerLVSAssignMoveProfile profile(100.0, 100.0, 100.0, min_steps);
  auto move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should use exactly min_steps
  EXPECT_EQ(move_instructions.size(), min_steps);

  // Verify all intermediate waypoints have the assigned target position
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  }
}

/**
 * @brief Test edge case with very small parameters (should be limited by max_steps)
 *
 * This test verifies that when all segment length parameters are set to very small values,
 * the profile is limited by the maximum number of steps.
 */
TEST_F(TesseractPlanningSimplePlannerLVSAssignMoveProfileUnit, SmallParameters_MaxSteps)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  // Use very small segment lengths with limited max_steps
  int min_steps = 5;
  int max_steps = 12;
  SimplePlannerLVSAssignMoveProfile profile(0.0001, 0.0001, 0.0001, min_steps, max_steps);
  auto move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should be limited by max_steps
  EXPECT_EQ(move_instructions.size(), max_steps);

  // Verify all intermediate waypoints have the assigned target position
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
    EXPECT_FALSE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  }
}

/**
 * @brief Main test runner
 *
 * Initializes Google Test framework and runs all test cases.
 * This comprehensive test suite validates the SimplePlannerLVSAssignMoveProfile
 * across all waypoint type combinations, motion types, and parameter validation scenarios.
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
