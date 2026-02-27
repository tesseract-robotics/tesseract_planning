/**
 * @file simple_planner_lvs_move_unit.cpp
 * @brief Unit tests for SimplePlannerLVSMoveProfile
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

#include <tesseract/common/types.h>
#include <tesseract/common/unit_test_utils.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/simple/cereal_serialization.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_move_profile.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>

using namespace tesseract::motion_planners;
using namespace tesseract::command_language;

/**
 * @brief Test fixture for SimplePlannerLVSMoveProfile unit tests
 *
 * This test suite validates the SimplePlannerLVSMoveProfile class, which generates
 * a variable number of intermediate waypoints based on longest valid segment (LVS) criteria
 * while using interpolation between waypoints. This planner combines adaptive step sizing
 * with smooth interpolation behavior, making it suitable for motion planning where the
 * number of waypoints should adapt to the distance and complexity of the motion.
 *
 * The LVS profile uses the following parameters:
 * - state_longest_valid_segment_length: Controls step size based on joint space distance
 * - translation_longest_valid_segment_length: Controls step size based on translation distance
 * - rotation_longest_valid_segment_length: Controls step size based on rotation distance
 * - min_steps: Minimum number of intermediate waypoints to generate
 */
class TesseractPlanningSimplePlannerLVSMoveProfileUnit : public TesseractPlanningSimplePlannerUnit
{
};

TEST_F(TesseractPlanningSimplePlannerLVSMoveProfileUnit, Serialization)  // NOLINT
{
  auto profile = std::make_shared<SimplePlannerLVSMoveProfile>(3.14, 0.5, 1.57, 5);
  // Serialization
  tesseract::common::testSerializationDerivedClass<tesseract::common::Profile, SimplePlannerLVSMoveProfile>(profile,
                                                                                                            "SimplePlan"
                                                                                                            "ne"
                                                                                                            "rLVSMovePr"
                                                                                                            "of"
                                                                                                            "ile");
}

/**
 * @brief Test Joint-to-Joint movement with freespace motion type and LVS interpolation
 *
 * This test verifies that when both start and end waypoints are joint waypoints
 * and the motion type is FREESPACE, the profile generates intermediate joint waypoints
 * by interpolating between the start and end positions. The number of waypoints adapts
 * based on the joint space distance and the state_longest_valid_segment_length parameter.
 */
TEST_F(TesseractPlanningSimplePlannerLVSMoveProfileUnit, InterpolateStateWaypoint_JointJoint_Freespace)  // NOLINT
{
  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSMoveProfile profile(3.14, 0.5, 1.57, 5);
  auto move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
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

  // Test with large segment length parameters - should use min_steps
  // When all LVS parameters are large, the distance-based calculations will result
  // in fewer steps than min_steps, so min_steps should be used
  int min_steps = 5;
  SimplePlannerLVSMoveProfile cs_profile(6.28, 0.5, 1.57, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Test with small state_longest_valid_segment_length - should generate more steps
  // When state_longest_valid_segment_length is small relative to joint space distance,
  // the number of steps should increase to satisfy the segment length constraint
  double longest_valid_segment_length = 0.05;
  SimplePlannerLVSMoveProfile cl_profile(longest_valid_segment_length, 10, 6.28, min_steps);
  auto cl = cl_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  double dist = (wp1.getPosition() - wp2.getPosition()).norm();
  int steps = int(dist / longest_valid_segment_length) + 1;
  EXPECT_GT(static_cast<int>(cl.size()), min_steps);
  EXPECT_EQ(cl.size(), steps);
}

/**
 * @brief Test Joint-to-Joint movement with linear motion type and LVS interpolation
 *
 * This test verifies that when both start and end waypoints are joint waypoints
 * and the motion type is LINEAR, the profile generates intermediate Cartesian waypoints
 * by interpolating between the forward kinematics solutions of the start and end joint
 * positions. The number of waypoints adapts based on translation and rotation distances.
 */
TEST_F(TesseractPlanningSimplePlannerLVSMoveProfileUnit, InterpolateStateWaypoint_JointJoint_Linear)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
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
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
  EXPECT_TRUE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));

  // Test with large segment length parameters - should use min_steps
  // When all LVS parameters are large, the distance-based calculations will result
  // in fewer steps than min_steps, so min_steps should be used
  int min_steps = 5;
  SimplePlannerLVSMoveProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Test with small translation_longest_valid_segment_length - should generate more steps
  // When translation_longest_valid_segment_length is small relative to Cartesian distance,
  // the number of steps should increase to satisfy the translation segment length constraint
  double translation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSMoveProfile ctl_profile(6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  auto ctl = ctl_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  Eigen::Isometry3d p1 = joint_group->calcFwdKin(wp1.getPosition()).at(manip_info_.tcp_frame);
  Eigen::Isometry3d p2 = joint_group->calcFwdKin(wp2.getPosition()).at(manip_info_.tcp_frame);
  double trans_dist = (p2.translation() - p1.translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_GT(static_cast<int>(ctl.size()), min_steps);
  EXPECT_EQ(ctl.size(), trans_steps);

  // Test with small rotation_longest_valid_segment_length - should generate more steps
  // When rotation_longest_valid_segment_length is small relative to angular distance,
  // the number of steps should increase to satisfy the rotation segment length constraint
  double rotation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSMoveProfile crl_profile(6.28, 10, rotation_longest_valid_segment_length, min_steps);
  auto crl = crl_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  double rot_dist = Eigen::Quaterniond(p1.linear()).angularDistance(Eigen::Quaterniond(p2.linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_GT(static_cast<int>(crl.size()), min_steps);
  EXPECT_EQ(crl.size(), rot_steps);
}

/**
 * @brief Test Joint-to-Cartesian movement with freespace motion type and LVS interpolation
 *
 * This test verifies that when the start waypoint is a joint waypoint and the end
 * waypoint is a Cartesian waypoint with freespace motion, the profile generates
 * intermediate joint waypoints by interpolating between the start joint position
 * and the IK solution for the Cartesian target. The number of waypoints adapts
 * based on the joint space distance.
 */
TEST_F(TesseractPlanningSimplePlannerLVSMoveProfileUnit, InterpolateStateWaypoint_JointCart_Freespace)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };

  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
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
  Eigen::Isometry3d final_pose = joint_group->calcFwdKin(last_position).at(manip_info_.tcp_frame);
  EXPECT_TRUE(wp2.getTransform().isApprox(final_pose, 1e-3));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSMoveProfile cs_profile(6.28, 0.5, 1.57, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.01;
  SimplePlannerLVSMoveProfile cl_profile(longest_valid_segment_length, 10, 6.28, min_steps);
  auto cl = cl_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_GT(static_cast<int>(cl.size()), min_steps);
}

/**
 * @brief Test Joint-to-Cartesian movement with linear motion type and LVS interpolation
 *
 * This test verifies that when the start waypoint is a joint waypoint and the end
 * waypoint is a Cartesian waypoint with linear motion, the profile generates
 * intermediate Cartesian waypoints by interpolating between the forward kinematics
 * solution of the start joint position and the target Cartesian pose. The number
 * of waypoints adapts based on translation and rotation distances.
 */
TEST_F(TesseractPlanningSimplePlannerLVSMoveProfileUnit, InterpolateStateWaypoint_JointCart_Linear)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
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
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
  const Eigen::VectorXd& last_position = mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position;
  Eigen::Isometry3d final_pose = joint_group->calcFwdKin(last_position).at(manip_info_.tcp_frame);
  EXPECT_TRUE(wp2.getTransform().isApprox(final_pose, 1e-3));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSMoveProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used
  double translation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSMoveProfile ctl_profile(6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  auto ctl = ctl_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  Eigen::Isometry3d p1 = joint_group->calcFwdKin(wp1.getPosition()).at(manip_info_.tcp_frame);
  double trans_dist = (wp2.getTransform().translation() - p1.translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_GT(static_cast<int>(ctl.size()), min_steps);
  EXPECT_EQ(ctl.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSMoveProfile crl_profile(6.28, 10, rotation_longest_valid_segment_length, min_steps);
  auto crl = crl_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  double rot_dist = Eigen::Quaterniond(p1.linear()).angularDistance(Eigen::Quaterniond(wp2.getTransform().linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_GT(static_cast<int>(crl.size()), min_steps);
  EXPECT_EQ(crl.size(), rot_steps);
}

/**
 * @brief Test Cartesian-to-Joint movement with freespace motion type and LVS interpolation
 *
 * This test verifies that when the start waypoint is a Cartesian waypoint and the end
 * waypoint is a joint waypoint with freespace motion, the profile generates
 * intermediate joint waypoints by interpolating between the IK solution for the
 * Cartesian start position and the target joint position. The number of waypoints
 * adapts based on the joint space distance.
 */
TEST_F(TesseractPlanningSimplePlannerLVSMoveProfileUnit, InterpolateStateWaypoint_CartJoint_Freespace)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  CartesianWaypoint wp1{ joint_group->calcFwdKin(Eigen::VectorXd::Zero(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
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

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSMoveProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.01;
  SimplePlannerLVSMoveProfile cl_profile(longest_valid_segment_length, 10, 6.28, min_steps);
  auto cl = cl_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_GT(static_cast<int>(cl.size()), min_steps);
}

/**
 * @brief Test Cartesian-to-Joint movement with linear motion type and LVS interpolation
 *
 * This test verifies that when the start waypoint is a Cartesian waypoint and the end
 * waypoint is a joint waypoint with linear motion, the profile generates intermediate
 * Cartesian waypoints by interpolating between the start Cartesian pose and the
 * forward kinematics solution of the target joint position. The number of waypoints
 * adapts based on translation and rotation distances.
 */
TEST_F(TesseractPlanningSimplePlannerLVSMoveProfileUnit, InterpolateStateWaypoint_CartJoint_Linear)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  CartesianWaypoint wp1{ joint_group->calcFwdKin(Eigen::VectorXd::Zero(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
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
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
  EXPECT_TRUE(mi.getWaypoint().as<JointWaypointPoly>().isConstrained());
  EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSMoveProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used
  double translation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSMoveProfile ctl_profile(6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  auto ctl = ctl_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  Eigen::Isometry3d p2 = joint_group->calcFwdKin(wp2.getPosition()).at(manip_info_.tcp_frame);
  double trans_dist = (p2.translation() - wp1.getTransform().translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_GT(static_cast<int>(ctl.size()), min_steps);
  EXPECT_EQ(ctl.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSMoveProfile crl_profile(6.28, 10, rotation_longest_valid_segment_length, min_steps);
  auto crl = crl_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  double rot_dist = Eigen::Quaterniond(wp1.getTransform().linear()).angularDistance(Eigen::Quaterniond(p2.linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_GT(static_cast<int>(crl.size()), min_steps);
  EXPECT_EQ(crl.size(), rot_steps);
}

/**
 * @brief Test Cartesian-to-Cartesian movement with freespace motion type and LVS interpolation
 *
 * This test verifies that when both start and end waypoints are Cartesian waypoints
 * with freespace motion, the profile generates intermediate joint waypoints by
 * interpolating between the IK solutions for the start and end Cartesian poses.
 * The number of waypoints adapts based on the joint space distance between IK solutions.
 */
TEST_F(TesseractPlanningSimplePlannerLVSMoveProfileUnit, InterpolateStateWaypoint_CartCart_Freespace)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  CartesianWaypoint wp1{ joint_group->calcFwdKin(Eigen::VectorXd::Zero(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
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
  Eigen::Isometry3d final_pose = joint_group->calcFwdKin(last_position).at(manip_info_.tcp_frame);
  EXPECT_TRUE(wp2.getTransform().isApprox(final_pose, 1e-3));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSMoveProfile cs_profile(6.28, 0.5, 1.57, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.01;
  SimplePlannerLVSMoveProfile cl_profile(longest_valid_segment_length, 10, 6.28, min_steps);
  auto cl = cl_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_GT(static_cast<int>(cl.size()), min_steps);
}

/**
 * @brief Test Cartesian-to-Cartesian movement with linear motion type and LVS interpolation
 *
 * This test verifies that when both start and end waypoints are Cartesian waypoints
 * with linear motion, the profile generates intermediate Cartesian waypoints by
 * interpolating between the start and end poses. The number of waypoints adapts
 * based on translation and rotation distances, with IK solutions used as seeds.
 */
TEST_F(TesseractPlanningSimplePlannerLVSMoveProfileUnit, InterpolateStateWaypoint_CartCart_Linear)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  CartesianWaypoint wp1{ joint_group->calcFwdKin(Eigen::VectorXd::Zero(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  CartesianWaypoint wp2{ joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
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
  const MoveInstructionPoly& mi = move_instructions.back();
  EXPECT_EQ(mi.getProfile(), instr2.getProfile());
  EXPECT_EQ(mi.getPathProfile(), instr2.getPathProfile());
  const Eigen::VectorXd& last_position = mi.getWaypoint().as<CartesianWaypointPoly>().getSeed().position;
  Eigen::Isometry3d final_pose = joint_group->calcFwdKin(last_position).at(manip_info_.tcp_frame);
  EXPECT_TRUE(wp2.getTransform().isApprox(final_pose, 1e-3));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSMoveProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used
  double translation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSMoveProfile ctl_profile(6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  auto ctl = ctl_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  double trans_dist = (wp2.getTransform().translation() - wp1.getTransform().translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_GT(static_cast<int>(ctl.size()), min_steps);
  EXPECT_EQ(ctl.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSMoveProfile crl_profile(6.28, 10, rotation_longest_valid_segment_length, min_steps);
  auto crl = crl_profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  double rot_dist =
      Eigen::Quaterniond(wp1.getTransform().linear()).angularDistance(Eigen::Quaterniond(wp2.getTransform().linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_GT(static_cast<int>(crl.size()), min_steps);
  EXPECT_EQ(crl.size(), rot_steps);
}

/**
 * @brief Test empty instruction handling with LVS interpolation
 *
 * This test verifies that the profile properly handles cases where start and end
 * waypoints are identical (zero distance), ensuring it still generates the minimum
 * required number of steps. This tests the robustness of the LVS algorithm when
 * dealing with edge cases where distance calculations might return zero.
 */
TEST_F(TesseractPlanningSimplePlannerLVSMoveProfileUnit,
       InterpolateStateWaypoint_EmptyInstructions_Freespace)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Zero(7) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Zero(7) };  // Same as start
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should generate minimum number of steps for identical waypoints
  EXPECT_EQ(move_instructions.size(), 5);

  // All waypoints should be joint waypoints with the target position
  for (const auto& mi : move_instructions)
  {
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
  }
}

/**
 * @brief Test identical waypoints handling with LVS interpolation
 *
 * This test verifies that the profile correctly handles cases where the start and
 * end waypoints are identical but non-zero, ensuring consistent behavior with the
 * min_steps parameter. This validates that the LVS profile maintains predictable
 * behavior regardless of the specific joint values when distance is zero.
 */
TEST_F(TesseractPlanningSimplePlannerLVSMoveProfileUnit,
       InterpolateStateWaypoint_IdenticalWaypoints_Freespace)  // NOLINT
{
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  JointWaypoint wp1{ joint_names_, Eigen::VectorXd::Ones(7) * 0.5 };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.getWaypoint() = JointWaypoint(joint_names_, env_->getCurrentJointValues(joint_names_));

  JointWaypoint wp2{ joint_names_, Eigen::VectorXd::Ones(7) * 0.5 };  // Identical to start
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());

  // Should generate minimum number of steps even with identical waypoints
  EXPECT_EQ(move_instructions.size(), 5);

  // All waypoints should be joint waypoints with the target position
  for (const auto& mi : move_instructions)
  {
    EXPECT_TRUE(mi.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(wp2.getPosition().isApprox(mi.getWaypoint().as<JointWaypointPoly>().getPosition(), 1e-5));
  }
}

/**
 * @brief Test Cartesian-to-Cartesian movement with explicit seed and LVS interpolation
 *
 * This test verifies that when a Cartesian waypoint has an explicit seed provided,
 * that seed is used instead of solving IK for joint positions. This demonstrates
 * the priority of explicit seeds over computed IK solutions and validates that
 * the LVS profile properly handles user-provided seed information.
 */
TEST_F(TesseractPlanningSimplePlannerLVSMoveProfileUnit, WithExplicitSeed_Interpolate)  // NOLINT
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

  SimplePlannerLVSMoveProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, env_, tesseract::common::ManipulatorInfo());
  EXPECT_GT(move_instructions.size(), 0);

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
 * This comprehensive test suite validates the SimplePlannerLVSMoveProfile
 * across all waypoint type combinations (Joint-Joint, Joint-Cartesian,
 * Cartesian-Joint, Cartesian-Cartesian), motion types (FREESPACE, LINEAR),
 * and edge cases (identical waypoints, explicit seeds).
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
