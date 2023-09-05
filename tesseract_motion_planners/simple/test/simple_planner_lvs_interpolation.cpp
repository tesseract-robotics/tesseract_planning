/**
 * @file simple_planner_fixed_size_interpolation.cpp
 * @brief
 *
 * @author Matthew Powelson
 * @date July 23, 2020
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

using namespace tesseract_environment;
using namespace tesseract_planning;

class TesseractPlanningSimplePlannerLVSInterpolationUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;
  tesseract_common::ManipulatorInfo manip_info_;
  std::vector<std::string> joint_names_;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract_common::TesseractSupportResourceLocator>();
    Environment::Ptr env = std::make_shared<Environment>();
    tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator));
    env_ = env;

    manip_info_.manipulator = "manipulator";
    manip_info_.tcp_frame = "tool0";
    manip_info_.working_frame = "base_link";
    joint_names_ = env_->getJointGroup("manipulator")->getJointNames();
  }
};

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_JointJoint_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();

  JointWaypointPoly wp1{ JointWaypoint(joint_names_, Eigen::VectorXd::Zero(7)) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.assignJointWaypoint(JointWaypoint(joint_names_, request.env_state.getJointValues(joint_names_)));

  JointWaypointPoly wp2{ JointWaypoint(joint_names_, Eigen::VectorXd::Ones(7)) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  auto move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
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
  SimplePlannerLVSPlanProfile cs_profile(6.28, 0.5, 1.57, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.05;
  SimplePlannerLVSPlanProfile cl_profile(longest_valid_segment_length, 10, 6.28, min_steps);
  auto cl = cl_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  double dist = (wp1.getPosition() - wp2.getPosition()).norm();
  int steps = int(dist / longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(cl.size()) > min_steps);
  EXPECT_EQ(cl.size(), steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_JointJoint_Linear)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  JointWaypointPoly wp1{ JointWaypoint(joint_names_, Eigen::VectorXd::Zero(7)) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.assignJointWaypoint(JointWaypoint(joint_names_, request.env_state.getJointValues(joint_names_)));

  JointWaypointPoly wp2{ JointWaypoint(joint_names_, Eigen::VectorXd::Ones(7)) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
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
  SimplePlannerLVSPlanProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used when large motion given
  double translation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile ctl_profile(6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  auto ctl = ctl_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  Eigen::Isometry3d p1 = joint_group->calcFwdKin(wp1.getPosition()).at(manip_info_.tcp_frame);
  Eigen::Isometry3d p2 = joint_group->calcFwdKin(wp2.getPosition()).at(manip_info_.tcp_frame);
  double trans_dist = (p2.translation() - p1.translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(ctl.size()) > min_steps);
  EXPECT_EQ(ctl.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile crl_profile(6.28, 10, rotation_longest_valid_segment_length, min_steps);
  auto crl = crl_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  double rot_dist = Eigen::Quaterniond(p1.linear()).angularDistance(Eigen::Quaterniond(p2.linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(crl.size()) > min_steps);
  EXPECT_EQ(crl.size(), rot_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_JointCart_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();
  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  JointWaypointPoly wp1{ JointWaypoint(joint_names_, Eigen::VectorXd::Zero(7)) };

  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.assignJointWaypoint(JointWaypoint(joint_names_, request.env_state.getJointValues(joint_names_)));

  CartesianWaypointPoly wp2{ CartesianWaypoint(
      joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame)) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
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
  SimplePlannerLVSPlanProfile cs_profile(6.28, 0.5, 1.57, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile cl_profile(longest_valid_segment_length, 10, 6.28, min_steps);
  auto cl = cl_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  EXPECT_TRUE(static_cast<int>(cl.size()) > min_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_JointCart_Linear)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();
  auto joint_group = request.env->getJointGroup(manip_info_.manipulator);

  JointWaypointPoly wp1{ JointWaypoint(joint_names_, Eigen::VectorXd::Zero(7)) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.assignJointWaypoint(JointWaypoint(joint_names_, request.env_state.getJointValues(joint_names_)));

  CartesianWaypointPoly wp2{ CartesianWaypoint(
      joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame)) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
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
  SimplePlannerLVSPlanProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used
  double translation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile ctl_profile(6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  auto ctl = ctl_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  Eigen::Isometry3d p1 = joint_group->calcFwdKin(wp1.getPosition()).at(manip_info_.tcp_frame);
  double trans_dist = (wp2.getTransform().translation() - p1.translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(ctl.size()) > min_steps);
  EXPECT_EQ(ctl.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile crl_profile(6.28, 10, rotation_longest_valid_segment_length, min_steps);
  auto crl = crl_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  double rot_dist = Eigen::Quaterniond(p1.linear()).angularDistance(Eigen::Quaterniond(wp2.getTransform().linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(crl.size()) > min_steps);
  EXPECT_EQ(crl.size(), rot_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_CartJoint_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();

  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  CartesianWaypointPoly wp1{ CartesianWaypoint(
      joint_group->calcFwdKin(Eigen::VectorXd::Zero(7)).at(manip_info_.tcp_frame)) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.assignJointWaypoint(JointWaypoint(joint_names_, request.env_state.getJointValues(joint_names_)));

  JointWaypointPoly wp2{ JointWaypoint(joint_names_, Eigen::VectorXd::Ones(7)) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
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
  SimplePlannerLVSPlanProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile cl_profile(longest_valid_segment_length, 10, 6.28, min_steps);
  auto cl = cl_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  EXPECT_TRUE(static_cast<int>(cl.size()) > min_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_CartJoint_Linear)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();

  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  CartesianWaypointPoly wp1{ CartesianWaypoint(
      joint_group->calcFwdKin(Eigen::VectorXd::Zero(7)).at(manip_info_.tcp_frame)) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.assignJointWaypoint(JointWaypoint(joint_names_, request.env_state.getJointValues(joint_names_)));

  JointWaypointPoly wp2{ JointWaypoint(joint_names_, Eigen::VectorXd::Ones(7)) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
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
  SimplePlannerLVSPlanProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used
  double translation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile ctl_profile(6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  auto ctl = ctl_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  Eigen::Isometry3d p2 = joint_group->calcFwdKin(wp2.getPosition()).at(manip_info_.tcp_frame);
  double trans_dist = (p2.translation() - wp1.getTransform().translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(ctl.size()) > min_steps);
  EXPECT_EQ(ctl.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile crl_profile(6.28, 10, rotation_longest_valid_segment_length, min_steps);
  auto crl = crl_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  double rot_dist = Eigen::Quaterniond(wp1.getTransform().linear()).angularDistance(Eigen::Quaterniond(p2.linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(crl.size()) > min_steps);
  EXPECT_EQ(crl.size(), rot_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_CartCart_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();

  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  CartesianWaypointPoly wp1{ CartesianWaypoint(
      joint_group->calcFwdKin(Eigen::VectorXd::Zero(7)).at(manip_info_.tcp_frame)) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.assignJointWaypoint(JointWaypoint(joint_names_, request.env_state.getJointValues(joint_names_)));

  CartesianWaypointPoly wp2{ CartesianWaypoint(
      joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame)) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
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
  SimplePlannerLVSPlanProfile cs_profile(6.28, 0.5, 1.57, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile cl_profile(longest_valid_segment_length, 10, 6.28, min_steps);
  auto cl = cl_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  EXPECT_TRUE(static_cast<int>(cl.size()) > min_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_CartCart_Linear)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();

  auto joint_group = env_->getJointGroup(manip_info_.manipulator);

  CartesianWaypointPoly wp1{ CartesianWaypoint(
      joint_group->calcFwdKin(Eigen::VectorXd::Zero(7)).at(manip_info_.tcp_frame)) };
  MoveInstruction instr1(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.assignJointWaypoint(JointWaypoint(joint_names_, request.env_state.getJointValues(joint_names_)));

  CartesianWaypointPoly wp2{ CartesianWaypoint(
      joint_group->calcFwdKin(Eigen::VectorXd::Ones(7)).at(manip_info_.tcp_frame)) };
  MoveInstruction instr2(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
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
  SimplePlannerLVSPlanProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used
  double translation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile ctl_profile(6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  auto ctl = ctl_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  double trans_dist = (wp2.getTransform().translation() - wp1.getTransform().translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(ctl.size()) > min_steps);
  EXPECT_EQ(ctl.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile crl_profile(6.28, 10, rotation_longest_valid_segment_length, min_steps);
  auto crl = crl_profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
  double rot_dist =
      Eigen::Quaterniond(wp1.getTransform().linear()).angularDistance(Eigen::Quaterniond(wp2.getTransform().linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(crl.size()) > min_steps);
  EXPECT_EQ(crl.size(), rot_steps);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
