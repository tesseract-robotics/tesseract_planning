/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Ken Anderson
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ken Anderson */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization_profiles.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

using namespace tesseract_planning;

// Initialize one-joint, 3 points exactly the same.
CompositeInstruction createRepeatedPointTrajectory()
{
  const int num = 3;
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

  CompositeInstruction program;
  for (int i = 0; i < num; i++)
  {
    StateWaypoint swp{ joint_names, Eigen::VectorXd::Zero(6) };
    swp.getPosition()[0] = 1;
    program.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  return program;
}

// Initialize one-joint, straight-line trajectory
CompositeInstruction createStraightTrajectory()
{
  const int num = 10;
  const double max = 2.0;

  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

  CompositeInstruction program;
  for (int i = 0; i < num; i++)
  {
    StateWaypoint swp{ joint_names, Eigen::VectorXd::Zero(6) };
    swp.getPosition()[0] = i * max / num;
    program.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  // leave final velocity/acceleration unset
  StateWaypoint swp{ joint_names, Eigen::VectorXd::Zero(6) };
  swp.getPosition()[0] = max;
  program.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE));

  return program;
}

class IterativeSplineParameterizationUnit : public ::testing::Test
{
protected:
  std::string name_{ "IterativeSplineParameterizationUnit" };
  tesseract_common::GeneralResourceLocator::Ptr locator_;
  tesseract_environment::Environment::Ptr env_;
  tesseract_common::ManipulatorInfo manip_;

  void SetUp() override
  {
    locator_ = std::make_shared<tesseract_common::GeneralResourceLocator>();
    auto env = std::make_shared<tesseract_environment::Environment>();

    std::filesystem::path urdf_path(
        locator_->locateResource("package://tesseract_support/urdf/abb_irb2400.urdf")->getFilePath());
    std::filesystem::path srdf_path(
        locator_->locateResource("package://tesseract_support/urdf/abb_irb2400.srdf")->getFilePath());
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator_));
    env_ = env;

    manip_.manipulator = "manipulator";
    manip_.working_frame = "base_link";
    manip_.tcp_frame = "tool0";
  }
};

TEST_F(IterativeSplineParameterizationUnit, Solve)  // NOLINT
{
  EXPECT_TRUE(true);
}

TEST_F(IterativeSplineParameterizationUnit, TestIterativeSpline)  // NOLINT
{
  CompositeInstruction program = createStraightTrajectory();
  program.setManipulatorInfo(manip_);

  // Profile
  auto profile = std::make_shared<IterativeSplineParameterizationCompositeProfile>();
  profile->add_points = false;
  profile->override_limits = true;
  profile->velocity_limits = Eigen::MatrixX2d(6, 2);
  profile->velocity_limits.col(0) << -2.088, -2.082, -3.27, -3.6, -3.3, -3.078;
  profile->velocity_limits.col(1) << 2.088, 2.082, 3.27, 3.6, 3.3, 3.078;
  profile->acceleration_limits = Eigen::MatrixX2d(6, 2);
  profile->acceleration_limits.col(0) = -1 * Eigen::VectorXd::Ones(6);
  profile->acceleration_limits.col(1) = Eigen::VectorXd::Ones(6);

  // Profile Dictionary
  tesseract_common::ProfileDictionary profiles;
  ;
  profiles.addProfile(name_, DEFAULT_PROFILE_KEY, profile);

  // Solve
  IterativeSplineParameterization time_parameterization(name_);
  EXPECT_TRUE(time_parameterization.compute(program, *env_, profiles));
  ASSERT_LT(program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime(), 5.0);
}

TEST_F(IterativeSplineParameterizationUnit, TestIterativeSplineAddPoints)  // NOLINT
{
  CompositeInstruction program = createStraightTrajectory();
  program.setManipulatorInfo(manip_);

  // Profile
  auto profile = std::make_shared<IterativeSplineParameterizationCompositeProfile>();
  profile->add_points = true;
  profile->override_limits = true;
  profile->velocity_limits = Eigen::MatrixX2d(6, 2);
  profile->velocity_limits.col(0) << -2.088, -2.082, -3.27, -3.6, -3.3, -3.078;
  profile->velocity_limits.col(1) << 2.088, 2.082, 3.27, 3.6, 3.3, 3.078;
  profile->acceleration_limits = Eigen::MatrixX2d(6, 2);
  profile->acceleration_limits.col(0) = -1 * Eigen::VectorXd::Ones(6);
  profile->acceleration_limits.col(1) = Eigen::VectorXd::Ones(6);

  // Profile Dictionary
  tesseract_common::ProfileDictionary profiles;
  ;
  profiles.addProfile(name_, DEFAULT_PROFILE_KEY, profile);

  // Solve
  IterativeSplineParameterization time_parameterization(name_);
  EXPECT_TRUE(time_parameterization.compute(program, *env_, profiles));
  ASSERT_LT(program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime(), 5.0);
}

TEST_F(IterativeSplineParameterizationUnit, TestIterativeSplineDynamicParams)  // NOLINT
{
  CompositeInstruction program = createStraightTrajectory();
  program.setManipulatorInfo(manip_);

  // Profile
  auto profile = std::make_shared<IterativeSplineParameterizationCompositeProfile>();
  profile->add_points = false;
  profile->override_limits = true;
  profile->velocity_limits = Eigen::MatrixX2d(6, 2);
  profile->velocity_limits.col(0) << -2.088, -2.082, -3.27, -3.6, -3.3, -3.078;
  profile->velocity_limits.col(1) << 2.088, 2.082, 3.27, 3.6, 3.3, 3.078;
  profile->acceleration_limits = Eigen::MatrixX2d(6, 2);
  profile->acceleration_limits.col(0) = -1 * Eigen::VectorXd::Ones(6);
  profile->acceleration_limits.col(1) = Eigen::VectorXd::Ones(6);

  auto move_profile = std::make_shared<IterativeSplineParameterizationMoveProfile>();
  move_profile->max_velocity_scaling_factor = 0.5;
  move_profile->max_acceleration_scaling_factor = 0.5;

  // Profile Dictionary
  tesseract_common::ProfileDictionary profiles;
  ;
  profiles.addProfile(name_, DEFAULT_PROFILE_KEY, profile);
  profiles.addProfile(name_, "FIRST_MOVE_KEY", profile);

  IterativeSplineParameterization time_parameterization(name_);
  EXPECT_TRUE(time_parameterization.compute(program, *env_, profiles));
  EXPECT_LT(program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime(), 5.0);

  // Solve
  program = createStraightTrajectory();
  program.setManipulatorInfo(manip_);
  program.getFirstMoveInstruction()->setProfile("FIRST_MOVE_KEY");
  EXPECT_TRUE(time_parameterization.compute(program, *env_, profiles));
}

TEST_F(IterativeSplineParameterizationUnit, TestRepeatedPoint)  // NOLINT
{
  CompositeInstruction program = createRepeatedPointTrajectory();
  program.setManipulatorInfo(manip_);

  // Profile
  auto profile = std::make_shared<IterativeSplineParameterizationCompositeProfile>();
  profile->add_points = true;
  profile->override_limits = true;
  profile->velocity_limits = Eigen::MatrixX2d(6, 2);
  profile->velocity_limits.col(0) << -2.088, -2.082, -3.27, -3.6, -3.3, -3.078;
  profile->velocity_limits.col(1) << 2.088, 2.082, 3.27, 3.6, 3.3, 3.078;
  profile->acceleration_limits = Eigen::MatrixX2d(6, 2);
  profile->acceleration_limits.col(0) = -1 * Eigen::VectorXd::Ones(6);
  profile->acceleration_limits.col(1) = Eigen::VectorXd::Ones(6);

  // Profile Dictionary
  tesseract_common::ProfileDictionary profiles;
  ;
  profiles.addProfile(name_, DEFAULT_PROFILE_KEY, profile);

  // Solve
  IterativeSplineParameterization time_parameterization(name_);
  EXPECT_TRUE(time_parameterization.compute(program, *env_, profiles));
  ASSERT_LT(program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime(), 0.001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  return RUN_ALL_TESTS();
}
