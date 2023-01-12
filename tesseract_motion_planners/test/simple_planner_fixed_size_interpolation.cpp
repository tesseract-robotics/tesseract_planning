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
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_plan_profile.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

using namespace tesseract_environment;
using namespace tesseract_planning;

class TesseractPlanningSimplePlannerFixedSizeInterpolationUnit : public ::testing::Test
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

    manip_info_.tcp_frame = "tool0";
    manip_info_.manipulator = "manipulator";
    manip_info_.working_frame = "base_link";
    joint_names_ = env_->getJointGroup("manipulator")->getJointNames();
  }
};

TEST_F(TesseractPlanningSimplePlannerFixedSizeInterpolationUnit, JointJoint_JointInterpolation)  // NOLINT
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

  SimplePlannerFixedSizePlanProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
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

TEST_F(TesseractPlanningSimplePlannerFixedSizeInterpolationUnit, JointCart_JointInterpolation)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();
  JointWaypointPoly wp1{ JointWaypoint(joint_names_, Eigen::VectorXd::Zero(7)) };
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.assignJointWaypoint(JointWaypoint(joint_names_, request.env_state.getJointValues(joint_names_)));

  CartesianWaypointPoly wp2{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizePlanProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
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

TEST_F(TesseractPlanningSimplePlannerFixedSizeInterpolationUnit, CartJoint_JointInterpolation)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();
  CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, 0, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.assignJointWaypoint(JointWaypoint(joint_names_, request.env_state.getJointValues(joint_names_)));

  JointWaypointPoly wp2{ JointWaypoint(joint_names_, Eigen::VectorXd::Zero(7)) };
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizePlanProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
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

TEST_F(TesseractPlanningSimplePlannerFixedSizeInterpolationUnit, CartCart_JointInterpolation)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getState();
  CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
  wp1.getTransform().translation() = Eigen::Vector3d(0.25, -0.1, 1);
  MoveInstruction instr1(wp1, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);
  MoveInstruction instr1_seed{ instr1 };
  instr1_seed.assignJointWaypoint(JointWaypoint(joint_names_, request.env_state.getJointValues(joint_names_)));

  CartesianWaypointPoly wp2{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
  wp2.getTransform().translation() = Eigen::Vector3d(0.25, 0.1, 1);
  MoveInstruction instr2(wp2, MoveInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  InstructionPoly instr3;

  SimplePlannerFixedSizePlanProfile profile(10, 10);
  std::vector<MoveInstructionPoly> move_instructions =
      profile.generate(instr1, instr1_seed, instr2, instr3, request, tesseract_common::ManipulatorInfo());
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
