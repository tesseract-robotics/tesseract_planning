/**
 * @file simple_planner_fixed_size_assign_move_unit.cpp
 * @brief Unit tests for SimplePlannerFixedSizeAssignMoveProfile
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
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_move_profile.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_kinematics/core/kinematic_group.h>

using namespace tesseract_planning;

/**
 * @brief Test fixture for SimplePlannerFixedSizeAssignMoveProfile unit tests
 *
 * This test suite validates the SimplePlannerFixedSizeAssignMoveProfile class, which generates
 * a fixed number of intermediate waypoints. For freespace moves, it assigns the target joint
 * position to all intermediate waypoints. For linear moves, it interpolates poses in Cartesian
 * space while assigning the target joint position as the seed for all intermediate waypoints.
 */
class TesseractPlanningSimplePlannerFixedSizeAssignMoveProfileUnit : public TesseractPlanningSimplePlannerUnit
{
};

TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignMoveProfileUnit, Serialization)  // NOLINT
{
  auto profile = std::make_shared<SimplePlannerFixedSizeAssignMoveProfile>(10, 10);
  // Serialization
  tesseract_common::testSerializationDerivedClass<tesseract_common::Profile, SimplePlannerFixedSizeAssignMoveProfile>(
      profile, "SimplePlannerFixedSizeAssignMoveProfile");
}

/**
 * @brief Test Joint-to-Joint movement with freespace motion type
 *
 * This test verifies that when both start and end waypoints are joint waypoints
 * and the motion type is FREESPACE, all intermediate waypoints are assigned
 * the target joint position (wp2) directly without interpolation.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignMoveProfileUnit,
       JointJoint_AssignJointPosition_Freespace)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

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
 * waypoints with poses interpolated between start and end positions, and the target
 * joint position is used as the seed for all intermediate waypoints.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignMoveProfileUnit, JointJoint_AssignJointPosition_Linear)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // For linear moves, intermediate waypoints should be Cartesian with joint seed
  // The poses are interpolated between start and end, and the target joint position
  // is used as the seed for all intermediate waypoints
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
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignMoveProfileUnit, JointCart_AssignJointPosition_Freespace)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // Solve IK for the Cartesian waypoint to get expected joint position
  // This demonstrates how the planner converts Cartesian targets to joint positions
  auto kin_group = env_->getKinematicGroup(manip_info_.manipulator);
  tesseract_kinematics::KinGroupIKInput ik_input(wp2.getTransform(), manip_info_.working_frame, manip_info_.tcp_frame);
  const Eigen::VectorXd& seed = wp1.getPosition();
  tesseract_kinematics::IKSolutions ik_solutions = kin_group->calcInvKin({ ik_input }, seed);
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
 * Cartesian waypoints with poses interpolated between start and end positions, and
 * the IK solution for the target is used as the seed for all intermediate waypoints.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignMoveProfileUnit, JointCart_AssignJointPosition_Linear)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // Solve IK for the Cartesian waypoint to get expected joint position
  auto kin_group = env_->getKinematicGroup(manip_info_.manipulator);
  tesseract_kinematics::KinGroupIKInput ik_input(wp2.getTransform(), manip_info_.working_frame, manip_info_.tcp_frame);
  const Eigen::VectorXd& seed = wp1.getPosition();
  tesseract_kinematics::IKSolutions ik_solutions = kin_group->calcInvKin({ ik_input }, seed);
  EXPECT_FALSE(ik_solutions.empty());
  const Eigen::VectorXd& expected_joint_position = ik_solutions[0];

  // For linear moves, intermediate waypoints should be Cartesian with joint seed
  // The poses are interpolated between start and end, and the IK solution is used
  // as the seed for all intermediate Cartesian waypoints
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
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignMoveProfileUnit, CartJoint_AssignJointPosition_Freespace)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

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
 * waypoints with poses interpolated between start and end positions, and the target
 * joint position is used as the seed for all intermediate waypoints.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignMoveProfileUnit, CartJoint_AssignJointPosition_Linear)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // For linear moves, intermediate waypoints should be Cartesian with joint seed
  // The poses are interpolated between start and end, and the target joint position
  // is used as the seed for all intermediate Cartesian waypoints
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
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignMoveProfileUnit, CartCart_AssignJointPosition_Freespace)  // NOLINT
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

  SimplePlannerFixedSizeAssignMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // Solve IK for the Cartesian waypoint to get expected joint position
  // This demonstrates the conversion from Cartesian target to joint assignment
  auto kin_group = env_->getKinematicGroup(manip_info_.manipulator);
  tesseract_kinematics::KinGroupIKInput ik_input(wp2.getTransform(), manip_info_.working_frame, manip_info_.tcp_frame);
  Eigen::VectorXd seed = env_->getCurrentJointValues(joint_names_);
  tesseract_kinematics::IKSolutions ik_solutions = kin_group->calcInvKin({ ik_input }, seed);
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
 * with linear motion, intermediate waypoints are Cartesian waypoints with poses
 * interpolated between start and end positions, and the IK solution for the target
 * is used as the seed for all intermediate waypoints.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignMoveProfileUnit, CartCart_AssignJointPosition_Linear)  // NOLINT
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

  SimplePlannerFixedSizeAssignMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // Solve IK for the Cartesian waypoint to get expected joint position
  auto kin_group = env_->getKinematicGroup(manip_info_.manipulator);
  tesseract_kinematics::KinGroupIKInput ik_input(wp2.getTransform(), manip_info_.working_frame, manip_info_.tcp_frame);
  Eigen::VectorXd seed = env_->getCurrentJointValues(joint_names_);
  tesseract_kinematics::IKSolutions ik_solutions = kin_group->calcInvKin({ ik_input }, seed);
  EXPECT_FALSE(ik_solutions.empty());
  const Eigen::VectorXd& expected_joint_position = ik_solutions[0];

  // For linear moves, intermediate waypoints should be Cartesian with joint seed
  // The poses are interpolated between start and end, and the IK solution is used
  // as the seed for all intermediate Cartesian waypoints
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
 * @brief Test Cartesian-to-Cartesian movement with explicit seed
 *
 * This test verifies that when a Cartesian waypoint has an explicit seed provided,
 * that seed is used instead of solving IK. This demonstrates the priority of
 * explicit seeds over IK solutions.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignMoveProfileUnit, CartCart_WithSeed_AssignJointPosition)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, -0.1, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0.1, 1);
  // Add a seed to the waypoint
  tesseract_common::JointState joint_seed;
  joint_seed.joint_names = joint_names_;
  joint_seed.position = Eigen::VectorXd::Ones(7) * 0.5;
  wp2.setSeed(joint_seed);
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

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
 * @brief Test Cartesian-to-Cartesian linear movement with pose interpolation
 *
 * This test verifies that for linear moves between Cartesian waypoints,
 * the intermediate waypoints have poses interpolated between start and end,
 * while maintaining the same joint seed throughout the trajectory.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignMoveProfileUnit, CartCart_Linear_CartesianInterpolation)  // NOLINT
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

  SimplePlannerFixedSizeAssignMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // Solve IK for the Cartesian waypoint to get expected joint position
  auto kin_group = env_->getKinematicGroup(manip_info_.manipulator);
  tesseract_kinematics::KinGroupIKInput ik_input(wp2.getTransform(), manip_info_.working_frame, manip_info_.tcp_frame);
  Eigen::VectorXd seed = env_->getCurrentJointValues(joint_names_);
  tesseract_kinematics::IKSolutions ik_solutions = kin_group->calcInvKin({ ik_input }, seed);
  EXPECT_FALSE(ik_solutions.empty());
  const Eigen::VectorXd& expected_joint_position = ik_solutions[0];

  // For linear moves, intermediate waypoints should be Cartesian with joint seed
  // The poses should be interpolated between start and end, but all should have the same joint seed
  // This demonstrates both pose interpolation and joint seed assignment
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(
        expected_joint_position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));

    // Check that poses are interpolated between start and end
    // This validates that linear motion creates a smooth trajectory in Cartesian space
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
  // The final waypoint should match the target pose exactly with the IK solution as seed
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_TRUE(expected_joint_position.isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position, 1e-5));
  EXPECT_TRUE(wp2.getTransform().isApprox(mi.getWaypoint().as<CartesianWaypointPoly>().getTransform(), 1e-5));
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
}

/**
 * @brief Test different step counts for freespace vs linear moves
 *
 * This test verifies that the planner correctly uses different step counts
 * for freespace and linear moves, demonstrating the flexibility of the
 * SimplePlannerFixedSizeAssignMoveProfile configuration.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeAssignMoveProfileUnit, DifferentStepCounts)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };

  // Test freespace with different step count (15 freespace steps vs 10 linear steps)
  // This shows that freespace_steps parameter is used for FREESPACE moves
  MoveInstruction instr1_freespace(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1_freespace };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));
  MoveInstruction instr2_freespace(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  InstructionPoly instr3;

  SimplePlannerFixedSizeAssignMoveProfile profile_freespace(15, 10);
  std::vector<MoveInstructionPoly> move_instructions_freespace = profile_freespace.generate(
      instr1_freespace, instr1_seed, instr2_freespace, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions_freespace.size(), 15);  // Should use freespace_steps

  // Test linear with different step count (15 freespace steps vs 20 linear steps)
  // This shows that linear_steps parameter is used for LINEAR moves
  MoveInstruction instr1_linear(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr2_linear(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  SimplePlannerFixedSizeAssignMoveProfile profile_linear(15, 20);
  std::vector<MoveInstructionPoly> move_instructions_linear = profile_linear.generate(
      instr1_linear, instr1_seed, instr2_linear, instr3, env_, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(move_instructions_linear.size(), 20);  // Should use linear_steps
}

/**
 * @brief Main test runner
 *
 * Initializes Google Test framework and runs all test cases.
 * This comprehensive test suite validates the SimplePlannerFixedSizeAssignMoveProfile
 * across all waypoint type combinations and motion types.
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
