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
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>

using namespace tesseract_environment;
using namespace tesseract_planning;

bool DEBUG = false;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

class TesseractPlanningSimplePlannerLVSInterpolationUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;
  ManipulatorInfo manip_info_;
  std::vector<std::string> joint_names_;

  void SetUp() override
  {
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    Environment::Ptr env = std::make_shared<Environment>();
    tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    EXPECT_TRUE(env->init<OFKTStateSolver>(urdf_path, srdf_path, locator));
    env_ = env;

    manip_info_.manipulator = "manipulator";
    joint_names_ = env_->getManipulatorManager()->getFwdKinematicSolver("manipulator")->getJointNames();
  }
};

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_JointJoint_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getCurrentState();

  JointWaypoint wp1(joint_names_, Eigen::VectorXd::Zero(7));
  JointWaypoint wp2(joint_names_, Eigen::VectorXd::Ones(7));
  PlanInstruction instr1(wp1, PlanInstructionType::START, "TEST_PROFILE", manip_info_);
  PlanInstruction instr2(wp2, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  auto composite = profile.generate(instr1, instr2, request, ManipulatorInfo());
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.as<MoveInstruction>().getWaypoint()));
    EXPECT_EQ(c.as<MoveInstruction>().getProfile(), instr2.getProfile());
  }
  const auto& mi = composite.back().as<MoveInstruction>();
  EXPECT_TRUE(wp2.isApprox(mi.getWaypoint().as<StateWaypoint>().position, 1e-5));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSPlanProfile cs_profile(6.28, 0.5, 1.57, min_steps);
  auto cs = cs_profile.generate(instr1, instr2, request, ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.05;
  SimplePlannerLVSPlanProfile cl_profile(longest_valid_segment_length, 10, 6.28, min_steps);
  auto cl = cl_profile.generate(instr1, instr2, request, ManipulatorInfo());
  double dist = (wp1 - wp2).norm();
  int steps = int(dist / longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(cl.size()) > min_steps);
  EXPECT_EQ(cl.size(), steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_JointJoint_Linear)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getCurrentState();
  auto fwd_kin = env_->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  JointWaypoint wp1(joint_names_, Eigen::VectorXd::Zero(7));
  JointWaypoint wp2(joint_names_, Eigen::VectorXd::Ones(7));
  PlanInstruction instr1(wp1, PlanInstructionType::START, "TEST_PROFILE", manip_info_);
  PlanInstruction instr2(wp2, PlanInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  auto composite = profile.generate(instr1, instr2, request, ManipulatorInfo());
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.as<MoveInstruction>().getWaypoint()));
    EXPECT_EQ(c.as<MoveInstruction>().getProfile(), instr2.getProfile());
  }
  const auto& mi = composite.back().as<MoveInstruction>();
  EXPECT_TRUE(wp2.isApprox(mi.getWaypoint().as<StateWaypoint>().position, 1e-5));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSPlanProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr2, request, ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used when large motion given
  double translation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile ctl_profile(6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  auto ctl = ctl_profile.generate(instr1, instr2, request, ManipulatorInfo());
  Eigen::Isometry3d p1 = fwd_kin->calcFwdKin(wp1);
  Eigen::Isometry3d p2 = fwd_kin->calcFwdKin(wp2);
  double trans_dist = (p2.translation() - p1.translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(ctl.size()) > min_steps);
  EXPECT_EQ(ctl.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile crl_profile(6.28, 10, rotation_longest_valid_segment_length, min_steps);
  auto crl = crl_profile.generate(instr1, instr2, request, ManipulatorInfo());
  double rot_dist = Eigen::Quaterniond(p1.linear()).angularDistance(Eigen::Quaterniond(p2.linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(crl.size()) > min_steps);
  EXPECT_EQ(crl.size(), rot_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_JointCart_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getCurrentState();
  auto fwd_kin = env_->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  JointWaypoint wp1(joint_names_, Eigen::VectorXd::Zero(7));
  CartesianWaypoint wp2 = fwd_kin->calcFwdKin(Eigen::VectorXd::Ones(7));
  PlanInstruction instr1(wp1, PlanInstructionType::START, "TEST_PROFILE", manip_info_);
  PlanInstruction instr2(wp2, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  auto composite = profile.generate(instr1, instr2, request, ManipulatorInfo());
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.as<MoveInstruction>().getWaypoint()));
    EXPECT_EQ(c.as<MoveInstruction>().getProfile(), instr2.getProfile());
  }
  const auto& mi = composite.back().as<MoveInstruction>();
  const Eigen::VectorXd& last_position = mi.getWaypoint().as<StateWaypoint>().position;
  Eigen::Isometry3d final_pose = fwd_kin->calcFwdKin(last_position);
  EXPECT_TRUE(wp2.isApprox(final_pose, 1e-3));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSPlanProfile cs_profile(6.28, 0.5, 1.57, min_steps);
  auto cs = cs_profile.generate(instr1, instr2, request, ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile cl_profile(longest_valid_segment_length, 10, 6.28, min_steps);
  auto cl = cl_profile.generate(instr1, instr2, request, ManipulatorInfo());
  EXPECT_TRUE(static_cast<int>(cl.size()) > min_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_JointCart_Linear)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getCurrentState();
  auto fwd_kin = request.env->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  JointWaypoint wp1(joint_names_, Eigen::VectorXd::Zero(7));
  CartesianWaypoint wp2 = fwd_kin->calcFwdKin(Eigen::VectorXd::Ones(7));

  PlanInstruction instr1(wp1, PlanInstructionType::START, "TEST_PROFILE", manip_info_);
  PlanInstruction instr2(wp2, PlanInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  auto composite = profile.generate(instr1, instr2, request, ManipulatorInfo());
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.as<MoveInstruction>().getWaypoint()));
    EXPECT_EQ(c.as<MoveInstruction>().getProfile(), instr2.getProfile());
  }
  const auto& mi = composite.back().as<MoveInstruction>();
  const Eigen::VectorXd& last_position = mi.getWaypoint().as<StateWaypoint>().position;
  Eigen::Isometry3d final_pose = fwd_kin->calcFwdKin(last_position);
  EXPECT_TRUE(wp2.isApprox(final_pose, 1e-3));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSPlanProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr2, request, ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used
  double translation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile ctl_profile(6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  auto ctl = ctl_profile.generate(instr1, instr2, request, ManipulatorInfo());
  Eigen::Isometry3d p1 = fwd_kin->calcFwdKin(wp1);
  double trans_dist = (wp2.translation() - p1.translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(ctl.size()) > min_steps);
  EXPECT_EQ(ctl.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile crl_profile(6.28, 10, rotation_longest_valid_segment_length, min_steps);
  auto crl = crl_profile.generate(instr1, instr2, request, ManipulatorInfo());
  double rot_dist = Eigen::Quaterniond(p1.linear()).angularDistance(Eigen::Quaterniond(wp2.linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(crl.size()) > min_steps);
  EXPECT_EQ(crl.size(), rot_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_CartJoint_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getCurrentState();

  auto fwd_kin = env_->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  CartesianWaypoint wp1 = fwd_kin->calcFwdKin(Eigen::VectorXd::Zero(7));
  JointWaypoint wp2(joint_names_, Eigen::VectorXd::Ones(7));
  PlanInstruction instr1(wp1, PlanInstructionType::START, "TEST_PROFILE", manip_info_);
  PlanInstruction instr2(wp2, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  auto composite = profile.generate(instr1, instr2, request, ManipulatorInfo());
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.as<MoveInstruction>().getWaypoint()));
    EXPECT_EQ(c.as<MoveInstruction>().getProfile(), instr2.getProfile());
  }
  const auto& mi = composite.back().as<MoveInstruction>();
  EXPECT_TRUE(wp2.isApprox(mi.getWaypoint().as<StateWaypoint>().position, 1e-5));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSPlanProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr2, request, ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile cl_profile(longest_valid_segment_length, 10, 6.28, min_steps);
  auto cl = cl_profile.generate(instr1, instr2, request, ManipulatorInfo());
  EXPECT_TRUE(static_cast<int>(cl.size()) > min_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_CartJoint_Linear)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getCurrentState();

  auto fwd_kin = env_->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  CartesianWaypoint wp1 = fwd_kin->calcFwdKin(Eigen::VectorXd::Zero(7));
  JointWaypoint wp2(joint_names_, Eigen::VectorXd::Ones(7));
  PlanInstruction instr1(wp1, PlanInstructionType::START, "TEST_PROFILE", manip_info_);
  PlanInstruction instr2(wp2, PlanInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  auto composite = profile.generate(instr1, instr2, request, ManipulatorInfo());
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.as<MoveInstruction>().getWaypoint()));
    EXPECT_EQ(c.as<MoveInstruction>().getProfile(), instr2.getProfile());
  }
  const auto& mi = composite.back().as<MoveInstruction>();
  EXPECT_TRUE(wp2.isApprox(mi.getWaypoint().as<StateWaypoint>().position, 1e-5));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSPlanProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr2, request, ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used
  double translation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile ctl_profile(6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  auto ctl = ctl_profile.generate(instr1, instr2, request, ManipulatorInfo());
  Eigen::Isometry3d p2 = fwd_kin->calcFwdKin(wp2);
  double trans_dist = (p2.translation() - wp1.translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(ctl.size()) > min_steps);
  EXPECT_EQ(ctl.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile crl_profile(6.28, 10, rotation_longest_valid_segment_length, min_steps);
  auto crl = crl_profile.generate(instr1, instr2, request, ManipulatorInfo());
  double rot_dist = Eigen::Quaterniond(wp1.linear()).angularDistance(Eigen::Quaterniond(p2.linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(crl.size()) > min_steps);
  EXPECT_EQ(crl.size(), rot_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_CartCart_Freespace)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getCurrentState();

  auto fwd_kin = env_->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  CartesianWaypoint wp1 = fwd_kin->calcFwdKin(Eigen::VectorXd::Zero(7));
  CartesianWaypoint wp2 = fwd_kin->calcFwdKin(Eigen::VectorXd::Ones(7));
  PlanInstruction instr1(wp1, PlanInstructionType::START, "TEST_PROFILE", manip_info_);
  PlanInstruction instr2(wp2, PlanInstructionType::FREESPACE, "TEST_PROFILE", manip_info_);

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  auto composite = profile.generate(instr1, instr2, request, ManipulatorInfo());
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.as<MoveInstruction>().getWaypoint()));
    EXPECT_EQ(c.as<MoveInstruction>().getProfile(), instr2.getProfile());
  }
  const auto& mi = composite.back().as<MoveInstruction>();
  const Eigen::VectorXd& last_position = mi.getWaypoint().as<StateWaypoint>().position;
  Eigen::Isometry3d final_pose = fwd_kin->calcFwdKin(last_position);
  EXPECT_TRUE(wp2.isApprox(final_pose, 1e-3));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSPlanProfile cs_profile(6.28, 0.5, 1.57, min_steps);
  auto cs = cs_profile.generate(instr1, instr2, request, ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure state_longest_valid_segment_length is used
  double longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile cl_profile(longest_valid_segment_length, 10, 6.28, min_steps);
  auto cl = cl_profile.generate(instr1, instr2, request, ManipulatorInfo());
  EXPECT_TRUE(static_cast<int>(cl.size()) > min_steps);
}

TEST_F(TesseractPlanningSimplePlannerLVSInterpolationUnit, InterpolateStateWaypoint_CartCart_Linear)  // NOLINT
{
  PlannerRequest request;
  request.env = env_;
  request.env_state = env_->getCurrentState();

  auto fwd_kin = env_->getManipulatorManager()->getFwdKinematicSolver(manip_info_.manipulator);

  CartesianWaypoint wp1 = fwd_kin->calcFwdKin(Eigen::VectorXd::Zero(7));
  CartesianWaypoint wp2 = fwd_kin->calcFwdKin(Eigen::VectorXd::Ones(7));
  PlanInstruction instr1(wp1, PlanInstructionType::START, "TEST_PROFILE", manip_info_);
  PlanInstruction instr2(wp2, PlanInstructionType::LINEAR, "TEST_PROFILE", manip_info_);

  SimplePlannerLVSPlanProfile profile(3.14, 0.5, 1.57, 5);
  auto composite = profile.generate(instr1, instr2, request, ManipulatorInfo());
  for (const auto& c : composite)
  {
    EXPECT_TRUE(isMoveInstruction(c));
    EXPECT_TRUE(isStateWaypoint(c.as<MoveInstruction>().getWaypoint()));
    EXPECT_EQ(c.as<MoveInstruction>().getProfile(), instr2.getProfile());
  }
  const auto& mi = composite.back().as<MoveInstruction>();
  const Eigen::VectorXd& last_position = mi.getWaypoint().as<StateWaypoint>().position;
  Eigen::Isometry3d final_pose = fwd_kin->calcFwdKin(last_position);
  EXPECT_TRUE(wp2.isApprox(final_pose, 1e-3));

  // Ensure equal to minimum number steps when all params set large
  int min_steps = 5;
  SimplePlannerLVSPlanProfile cs_profile(6.28, 10, 6.28, min_steps);
  auto cs = cs_profile.generate(instr1, instr2, request, ManipulatorInfo());
  EXPECT_EQ(cs.size(), min_steps);

  // Ensure translation_longest_valid_segment_length is used
  double translation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile ctl_profile(6.28, translation_longest_valid_segment_length, 6.28, min_steps);
  auto ctl = ctl_profile.generate(instr1, instr2, request, ManipulatorInfo());
  double trans_dist = (wp2.translation() - wp1.translation()).norm();
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(ctl.size()) > min_steps);
  EXPECT_EQ(ctl.size(), trans_steps);

  // Ensure rotation_longest_valid_segment_length is used
  double rotation_longest_valid_segment_length = 0.01;
  SimplePlannerLVSPlanProfile crl_profile(6.28, 10, rotation_longest_valid_segment_length, min_steps);
  auto crl = crl_profile.generate(instr1, instr2, request, ManipulatorInfo());
  double rot_dist = Eigen::Quaterniond(wp1.linear()).angularDistance(Eigen::Quaterniond(wp2.linear()));
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  EXPECT_TRUE(static_cast<int>(crl.size()) > min_steps);
  EXPECT_EQ(crl.size(), rot_steps);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
