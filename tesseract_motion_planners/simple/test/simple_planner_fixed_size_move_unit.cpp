/**
 * @file simple_planner_fixed_size_move_unit.cpp
 * @brief Unit tests for SimplePlannerFixedSizeMoveProfile
 *
 * @author Matthew Powelson
 * @date July 23, 2020
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
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_move_profile.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>

using namespace tesseract::motion_planners;
using namespace tesseract::command_language;

/**
 * @brief Test fixture for SimplePlannerFixedSizeMoveProfile unit tests
 *
 * This test suite validates the SimplePlannerFixedSizeMoveProfile class, which generates
 * a fixed number of intermediate waypoints between start and end waypoints using interpolation.
 * Unlike assign-based profiles that copy target positions, this profile creates smooth
 * trajectories by interpolating between waypoints. The profile supports both freespace and
 * linear motion types across all waypoint combinations (Joint-to-Joint, Joint-to-Cartesian,
 * Cartesian-to-Joint, and Cartesian-to-Cartesian).
 */
class TesseractPlanningSimplePlannerFixedSizeMoveProfileUnit : public TesseractPlanningSimplePlannerUnit
{
};

TEST_F(TesseractPlanningSimplePlannerFixedSizeMoveProfileUnit, Serialization)  // NOLINT
{
  auto profile = std::make_shared<SimplePlannerFixedSizeMoveProfile>(10, 10);
  // Serialization
  tesseract::common::testSerializationDerivedClass<tesseract::common::Profile, SimplePlannerFixedSizeMoveProfile>(
      profile,
      "Simple"
      "Planne"
      "rFixed"
      "SizeMo"
      "veProf"
      "ile");
}

/**
 * @brief Test Joint-to-Joint movement with freespace interpolation
 *
 * This test verifies that when both start and end waypoints are joint waypoints
 * with freespace motion type, the profile generates intermediate joint waypoints
 * by interpolating between the start and end joint positions. This creates a
 * smooth trajectory in joint space.
 */

TEST_F(TesseractPlanningSimplePlannerFixedSizeMoveProfileUnit, JointJoint_JointInterpolation)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);
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
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
  EXPECT_TRUE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
}

/**
 * @brief Test Joint-to-Cartesian movement with freespace interpolation
 *
 * This test verifies that when the start waypoint is a joint waypoint and the end
 * waypoint is a Cartesian waypoint with freespace motion type, the profile generates
 * intermediate joint waypoints by interpolating between the start joint position and
 * the IK solution of the target Cartesian pose. This creates a smooth trajectory in
 * joint space towards the desired Cartesian target.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeMoveProfileUnit, JointCart_JointInterpolation)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);
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
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());

  const Eigen::VectorXd& last_position = mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position;
  auto manip = env_->getJointGroup(manip_info_.manipulator);
  Eigen::Isometry3d final_pose = manip->calcFwdKin(last_position).at(manip_info_.tcp_frame);
  EXPECT_TRUE(wp2.getTransform().isApprox(final_pose, 1e-3));
}

/**
 * @brief Test Cartesian-to-Joint movement with freespace interpolation
 *
 * This test verifies that when the start waypoint is a Cartesian waypoint and the end
 * waypoint is a joint waypoint with freespace motion type, the profile generates
 * intermediate joint waypoints by interpolating between the seed joint position and
 * the target joint position. This creates a smooth trajectory in joint space from
 * the seed towards the target.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeMoveProfileUnit, CartJoint_JointInterpolation)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);
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
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
  EXPECT_TRUE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
}

/**
 * @brief Test Cartesian-to-Joint movement with linear interpolation
 *
 * This test verifies that when the start waypoint is a Cartesian waypoint and the end
 * waypoint is a joint waypoint with linear motion type, the profile generates intermediate
 * Cartesian waypoints by interpolating poses between the start Cartesian pose and the
 * forward kinematics solution of the target joint position. This creates a smooth linear
 * trajectory in Cartesian space towards the target joint configuration.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeMoveProfileUnit, CartToJoint_Interpolate_Linear)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // For linear cart-to-joint moves, intermediate waypoints should be Cartesian
  // with interpolated poses between start and end
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  }
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
  EXPECT_TRUE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
}

/**
 * @brief Test Cartesian-to-Cartesian movement with freespace interpolation
 *
 * This test verifies that when both start and end waypoints are Cartesian waypoints
 * with freespace motion type, the profile generates intermediate joint waypoints by
 * interpolating between the IK solutions of the start and end Cartesian poses.
 * This creates a smooth trajectory in joint space between the two Cartesian targets.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeMoveProfileUnit, CartCart_JointInterpolation)  // NOLINT
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

  SimplePlannerFixedSizeMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);
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
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
  const Eigen::VectorXd& last_position = mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position;
  auto manip = env_->getJointGroup(manip_info_.manipulator);
  Eigen::Isometry3d final_pose = manip->calcFwdKin(last_position).at(manip_info_.tcp_frame);
  EXPECT_TRUE(wp2.getTransform().isApprox(final_pose, 1e-3));
}

/**
 * @brief Test Joint-to-Joint movement with linear interpolation
 *
 * This test verifies that when both start and end waypoints are joint waypoints
 * with linear motion type, the profile generates intermediate Cartesian waypoints
 * by interpolating poses between the forward kinematics solutions of the start
 * and end joint positions. This creates a smooth linear trajectory in Cartesian
 * space between the two joint configurations.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeMoveProfileUnit, JointToJoint_Interpolate_Linear)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // For linear joint-to-joint moves, intermediate waypoints should be Cartesian
  // with interpolated poses between start and end joint positions
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  }
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
  EXPECT_TRUE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
}

/**
 * @brief Test Joint-to-Cartesian movement with linear interpolation
 *
 * This test verifies that when the start waypoint is a joint waypoint and the end
 * waypoint is a Cartesian waypoint with linear motion type, the profile generates
 * intermediate Cartesian waypoints by interpolating poses between the forward
 * kinematics solution of the start joint position and the target Cartesian pose.
 * This creates a smooth linear trajectory in Cartesian space towards the target.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeMoveProfileUnit, JointToCart_Interpolate_Linear)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // For linear joint-to-cart moves, intermediate waypoints should be Cartesian
  // with interpolated poses between start and end
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  }
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());

  const Eigen::VectorXd& last_position = mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position;
  auto manip = env_->getJointGroup(manip_info_.manipulator);
  Eigen::Isometry3d final_pose = manip->calcFwdKin(last_position).at(manip_info_.tcp_frame);
  EXPECT_TRUE(wp2.getTransform().isApprox(final_pose, 1e-3));
}

/**
 * @brief Test Cartesian-to-Cartesian movement with linear interpolation
 *
 * This test verifies that when both start and end waypoints are Cartesian waypoints
 * with linear motion type, the profile generates intermediate Cartesian waypoints
 * by interpolating poses between the start and end Cartesian poses. This creates
 * a smooth linear trajectory in Cartesian space between the two targets.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeMoveProfileUnit, CartToCart_Interpolate_Linear)  // NOLINT
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

  SimplePlannerFixedSizeMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // For linear cart-to-cart moves, intermediate waypoints should be Cartesian
  // with interpolated poses between start and end
  for (std::size_t i = 0; i < move_instructions.size() - 1; ++i)
  {
    const MoveInstructionPoly& mi = move_instructions.at(i);
    EXPECT_TRUE(mi.getWaypoint().isCartesianWaypoint());
  }
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
  const Eigen::VectorXd& last_position = mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position;
  auto manip = env_->getJointGroup(manip_info_.manipulator);
  Eigen::Isometry3d final_pose = manip->calcFwdKin(last_position).at(manip_info_.tcp_frame);
  EXPECT_TRUE(wp2.getTransform().isApprox(final_pose, 1e-3));
}

/**
 * @brief Test Cartesian-to-Cartesian movement with explicit seed and freespace interpolation
 *
 * This test verifies that when a Cartesian waypoint has an explicit seed provided
 * and freespace motion type is used, that seed is used for the generated waypoints
 * instead of solving IK. This demonstrates the profile's ability to handle explicit
 * joint seeds and prioritize them over IK solutions, ensuring predictable joint
 * configurations in the trajectory.
 */
TEST_F(TesseractPlanningSimplePlannerFixedSizeMoveProfileUnit, WithExplicitSeed_Interpolate)  // NOLINT
{
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, -0.1, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0.1, 1);
  // Set explicit seed for the target waypoint
  Eigen::VectorXd explicit_seed = Eigen::VectorXd::Ones(7) * 0.5;
  tesseract::common::JointState joint_seed;
  joint_seed.joint_names = joint_names_;
  joint_seed.position = explicit_seed;
  wp2.setSeed(joint_seed);
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizeMoveProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(move_instructions.size(), 10);

  // Verify that the explicit seed is used in the final waypoint
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
  const Eigen::VectorXd& last_position = mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position;
  EXPECT_TRUE(explicit_seed.isApprox(last_position, 1e-5));
}

/**
 * @brief Main test runner
 *
 * Initializes Google Test framework and runs all test cases.
 * This comprehensive test suite validates the SimplePlannerFixedSizeMoveProfile
 * across all waypoint type combinations (Joint-to-Joint, Joint-to-Cartesian,
 * Cartesian-to-Joint, Cartesian-to-Cartesian) and motion types (FREESPACE, LINEAR).
 * The profile generates smooth interpolated trajectories with a fixed number of
 * intermediate waypoints, supporting both joint space and Cartesian space interpolation
 * depending on the waypoint types and motion requirements.
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
