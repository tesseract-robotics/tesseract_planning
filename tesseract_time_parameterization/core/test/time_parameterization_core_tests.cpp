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
#include <tesseract_common/unit_test_utils.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

using namespace tesseract::time_parameterization;
using namespace tesseract::command_language;

// Initialize one-joint, straight-line trajectory
CompositeInstruction createStraightTrajectory()
{
  const int num = 10;
  const double pos_max = 2.0;
  const double vel_max = 3.0;
  const double acc_max = 5.0;

  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

  CompositeInstruction program;
  for (int i = 0; i < num; i++)
  {
    StateWaypoint swp{
      joint_names, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6), static_cast<double>(i)
    };
    swp.getPosition()[0] = i * pos_max / num;
    swp.getVelocity()[0] = i * vel_max / num;
    swp.getAcceleration()[0] = i * acc_max / num;
    program.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  // leave final velocity/acceleration unset
  StateWaypoint swp{
    joint_names, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6), static_cast<double>(num)
  };
  swp.getPosition()[0] = pos_max;
  swp.getVelocity()[0] = vel_max;
  swp.getAcceleration()[0] = acc_max;
  program.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE));

  return program;
}

TEST(TimeParameterizationCoreUnit, instructionsTrajectoryTests)  // NOLINT
{
  static double epsilon = std::numeric_limits<float>::epsilon();

  CompositeInstruction program = createStraightTrajectory();
  auto flattened = program.flatten(moveFilter);
  InstructionsTrajectory traj(flattened);
  EXPECT_FALSE(traj.empty());
  for (Eigen::Index i = 0; i < traj.size(); i++)
  {
    const auto& wp =
        program.at(static_cast<std::size_t>(i)).as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
    EXPECT_TRUE(tesseract::common::almostEqualRelativeAndAbs(traj.getPosition(i), wp.getPosition(), epsilon));
    EXPECT_TRUE(tesseract::common::almostEqualRelativeAndAbs(traj.getVelocity(i), wp.getVelocity(), epsilon));
    EXPECT_TRUE(tesseract::common::almostEqualRelativeAndAbs(traj.getAcceleration(i), wp.getAcceleration(), epsilon));
    EXPECT_TRUE(tesseract::common::almostEqualRelativeAndAbs(traj.getTimeFromStart(i), wp.getTime(), epsilon));

    EXPECT_TRUE(
        tesseract::common::almostEqualRelativeAndAbs(std::as_const(traj).getPosition(i), wp.getPosition(), epsilon));
    EXPECT_TRUE(
        tesseract::common::almostEqualRelativeAndAbs(std::as_const(traj).getVelocity(i), wp.getVelocity(), epsilon));
    EXPECT_TRUE(tesseract::common::almostEqualRelativeAndAbs(
        std::as_const(traj).getAcceleration(i), wp.getAcceleration(), epsilon));
    EXPECT_TRUE(
        tesseract::common::almostEqualRelativeAndAbs(std::as_const(traj).getTimeFromStart(i), wp.getTime(), epsilon));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
