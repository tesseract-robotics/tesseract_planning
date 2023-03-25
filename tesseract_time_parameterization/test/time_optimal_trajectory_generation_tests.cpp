/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

using tesseract_planning::CompositeInstruction;
using tesseract_planning::InstructionsTrajectory;
using tesseract_planning::MoveInstruction;
using tesseract_planning::MoveInstructionPoly;
using tesseract_planning::MoveInstructionType;
using tesseract_planning::StateWaypoint;
using tesseract_planning::StateWaypointPoly;
using tesseract_planning::TimeOptimalTrajectoryGeneration;
using tesseract_planning::TrajectoryContainer;
using tesseract_planning::totg::Path;
using tesseract_planning::totg::PathData;
using tesseract_planning::totg::Trajectory;

TEST(time_optimal_trajectory_generation, test1)  // NOLINT
{
  Eigen::VectorXd waypoint(4);
  std::list<Eigen::VectorXd> waypoints;

  waypoint << 1424.0, 984.999694824219, 2126.0, 0.0;
  waypoints.push_back(waypoint);
  waypoint << 1423.0, 985.000244140625, 2126.0, 0.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd max_velocities(4);
  max_velocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd max_accelerations(4);
  max_accelerations << 0.00249, 0.00249, 0.00249, 0.00249;

  Trajectory trajectory(Path(waypoints, 100.0), max_velocities, max_accelerations, 10.0);
  EXPECT_TRUE(trajectory.isValid());
  EXPECT_DOUBLE_EQ(40.080256821829849, trajectory.getDuration());

  // Test start matches
  PathData path_data = trajectory.getPathData(0);
  Eigen::VectorXd position = trajectory.getPosition(path_data);
  EXPECT_DOUBLE_EQ(1424.0, position[0]);
  EXPECT_DOUBLE_EQ(984.999694824219, position[1]);
  EXPECT_DOUBLE_EQ(2126.0, position[2]);
  EXPECT_DOUBLE_EQ(0.0, position[3]);

  // Test end matches
  path_data = trajectory.getPathData(trajectory.getDuration());
  position = trajectory.getPosition(path_data);
  EXPECT_DOUBLE_EQ(1423.0, position[0]);
  EXPECT_DOUBLE_EQ(985.000244140625, position[1]);
  EXPECT_DOUBLE_EQ(2126.0, position[2]);
  EXPECT_DOUBLE_EQ(0.0, position[3]);
}

TEST(time_optimal_trajectory_generation, test2)  // NOLINT
{
  Eigen::VectorXd waypoint(4);
  std::list<Eigen::VectorXd> waypoints;

  waypoint << 1427.0, 368.0, 690.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 1427.0, 368.0, 790.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 952.499938964844, 433.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 951.0, 90.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd max_velocities(4);
  max_velocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd max_accelerations(4);
  max_accelerations << 0.002, 0.002, 0.002, 0.002;

  Trajectory trajectory(Path(waypoints, 100.0), max_velocities, max_accelerations, 10.0);
  EXPECT_TRUE(trajectory.isValid());
  EXPECT_DOUBLE_EQ(1922.1418427445944, trajectory.getDuration());

  // Test start matches
  PathData path_data = trajectory.getPathData(0);
  Eigen::VectorXd position = trajectory.getPosition(path_data);
  EXPECT_DOUBLE_EQ(1427.0, position[0]);
  EXPECT_DOUBLE_EQ(368.0, position[1]);
  EXPECT_DOUBLE_EQ(690.0, position[2]);
  EXPECT_DOUBLE_EQ(90.0, position[3]);

  // Test end matches
  path_data = trajectory.getPathData(trajectory.getDuration());
  position = trajectory.getPosition(path_data);
  EXPECT_DOUBLE_EQ(452.5, position[0]);
  EXPECT_DOUBLE_EQ(533.0, position[1]);
  EXPECT_DOUBLE_EQ(951.0, position[2]);
  EXPECT_DOUBLE_EQ(90.0, position[3]);
}

TEST(time_optimal_trajectory_generation, test3)  // NOLINT
{
  Eigen::VectorXd waypoint(4);
  std::list<Eigen::VectorXd> waypoints;

  waypoint << 1427.0, 368.0, 690.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 1427.0, 368.0, 790.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 952.499938964844, 433.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 951.0, 90.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd max_velocities(4);
  max_velocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd max_accelerations(4);
  max_accelerations << 0.002, 0.002, 0.002, 0.002;

  Trajectory trajectory(Path(waypoints, 100.0), max_velocities, max_accelerations);
  EXPECT_TRUE(trajectory.isValid());
  EXPECT_DOUBLE_EQ(1919.5597888812974, trajectory.getDuration());

  // Test start matches
  PathData path_data = trajectory.getPathData(0);
  Eigen::VectorXd position = trajectory.getPosition(path_data);
  EXPECT_DOUBLE_EQ(1427.0, position[0]);
  EXPECT_DOUBLE_EQ(368.0, position[1]);
  EXPECT_DOUBLE_EQ(690.0, position[2]);
  EXPECT_DOUBLE_EQ(90.0, position[3]);

  // Test end matches
  path_data = trajectory.getPathData(trajectory.getDuration());
  position = trajectory.getPosition(path_data);
  EXPECT_DOUBLE_EQ(452.5, position[0]);
  EXPECT_DOUBLE_EQ(533.0, position[1]);
  EXPECT_DOUBLE_EQ(951.0, position[2]);
  EXPECT_DOUBLE_EQ(90.0, position[3]);
}

// Test that totg algorithm doesn't give large acceleration
TEST(time_optimal_trajectory_generation, testLargeAccel)  // NOLINT
{
  double path_tolerance = 0.1;
  double resample_dt = 0.1;
  Eigen::VectorXd waypoint(6);
  std::list<Eigen::VectorXd> waypoints;
  Eigen::VectorXd max_velocities(6);
  Eigen::VectorXd max_accelerations(6);

  // Waypoints
  // clang-format off
  waypoint << 1.6113056281076339,
             -0.21400163389235427,
             -1.974502599739185,
              9.9653618690354051e-12,
             -1.3810916877429624,
              1.5293902838041467;
  waypoints.push_back(waypoint);

  waypoint << 1.6088016187976597,
             -0.21792862470933924,
             -1.9758628799742952,
              0.00010424017303217738,
             -1.3835690515335755,
              1.5279972853269816;
  waypoints.push_back(waypoint);

  waypoint << 1.5887695443178671,
             -0.24934455124521923,
             -1.9867451218551782,
              0.00093816147756670078,
             -1.4033879618584812,
              1.5168532975096607;
  waypoints.push_back(waypoint);

  waypoint << 1.1647412393815282,
             -0.91434018564402375,
             -2.2170946337498498,
              0.018590164397622583,
             -1.8229041212673529,
              1.2809632867583278;
  waypoints.push_back(waypoint);

  // Max velocities
  max_velocities << 0.89535390627300004,
                    0.89535390627300004,
                    0.79587013890930003,
                    0.92022484811399996,
                    0.82074108075029995,
                    1.3927727430915;
  // Max accelerations
  max_accelerations << 0.82673490883799994,
                       0.78539816339699997,
                       0.60883578557700002,
                       3.2074759432319997,
                       1.4398966328939999,
                       4.7292792634680003;
  // clang-format on

  Trajectory parameterized(Path(waypoints, path_tolerance), max_velocities, max_accelerations, 0.001);

  ASSERT_TRUE(parameterized.isValid());

  auto sample_count = static_cast<size_t>(std::ceil(parameterized.getDuration() / resample_dt));
  for (size_t sample = 0; sample <= sample_count; ++sample)
  {
    // always sample the end of the trajectory as well
    double t = std::min(parameterized.getDuration(), static_cast<double>(sample) * resample_dt);
    PathData path_data = parameterized.getPathData(t);
    Eigen::VectorXd acceleration = parameterized.getAcceleration(path_data);

    ASSERT_EQ(acceleration.size(), 6);
    for (std::size_t i = 0; i < 6; ++i)
      EXPECT_NEAR(acceleration(static_cast<Eigen::Index>(i)), 0.0, 100.0)
          << "Invalid acceleration at position " << sample_count << "\n";
  }
}

// Caues the issue below. Workaround is applied when using computeTimestamps interface like in
// testCommandLanguageInterface https://github.com/ros-industrial-consortium/tesseract_planning/issues/27
// TEST(time_optimal_trajectory_generation, test_return_home)
//{
//  Eigen::VectorXd waypoint(6);
//  std::list<Eigen::VectorXd> waypoints;

//  waypoint << 0, 0.7, -2.1, 0, -0.25, 0;
//  waypoints.push_back(waypoint);
//  waypoint << 0, 0, 0, 0, 0, 0;
//  waypoints.push_back(waypoint);
//  waypoint << 0, 0.70001, -2.1, 0, -0.25, 0;
//  waypoints.push_back(waypoint);
//  waypoint << 0, 0, 0, 0, 0, 0.1;
//  waypoints.push_back(waypoint);

//  Eigen::VectorXd max_velocities(6);
//  max_velocities.setOnes();
//  Eigen::VectorXd max_accelerations(6);
//  max_accelerations.setOnes();

//  Trajectory trajectory(Path(waypoints, 0.001), max_velocities, max_accelerations, 0.001);
//  EXPECT_TRUE(trajectory.isValid());
//}

TEST(time_optimal_trajectory_generation, testCommandLanguageInterface)  // NOLINT
{
  tesseract_planning::CompositeInstruction program;
  Eigen::VectorXd waypoint(6);
  std::list<Eigen::VectorXd> waypoints;

  std::vector<std::string> joint_names = { "j1", "j2", "j3", "j4", "j5", "j6" };

  {
    waypoint << 0, 0.7, -2.1, 0, -0.25, 0;
    StateWaypointPoly wp{ StateWaypoint(joint_names, waypoint) };
    MoveInstruction plan_f0(wp, MoveInstructionType::FREESPACE, "profile_name");
    plan_f0.setDescription("freespace_motion");
    program.appendMoveInstruction(plan_f0);
  }
  {
    waypoint << 0, 0, 0, 0, 0, 0;
    StateWaypointPoly wp{ StateWaypoint(joint_names, waypoint) };
    MoveInstruction plan_f0(wp, MoveInstructionType::FREESPACE, "profile_name");
    plan_f0.setDescription("freespace_motion");
    program.appendMoveInstruction(plan_f0);
  }
  {
    waypoint << 0, 0.70001, -2.1, 0, -0.25, 0;
    StateWaypointPoly wp{ StateWaypoint(joint_names, waypoint) };
    MoveInstruction plan_f0(wp, MoveInstructionType::FREESPACE, "profile_name");
    plan_f0.setDescription("freespace_motion");
    program.appendMoveInstruction(plan_f0);
  }
  {
    waypoint << 0, 0, 0, 0, 0, 0.1;
    StateWaypointPoly wp{ StateWaypoint(joint_names, waypoint) };
    MoveInstruction plan_f0(wp, MoveInstructionType::FREESPACE, "profile_name");
    plan_f0.setDescription("freespace_motion");
    program.appendMoveInstruction(plan_f0);
  }

  Eigen::VectorXd max_velocities(6);
  max_velocities.setOnes();
  Eigen::VectorXd max_accelerations(6);
  max_accelerations.setOnes();

  TimeOptimalTrajectoryGeneration solver(0.001, 1e-3);
  InstructionsTrajectory traj_wrapper(program);
  EXPECT_TRUE(solver.computeTimeStamps(traj_wrapper, max_velocities, max_accelerations, 1.0, 1.0));
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
    StateWaypointPoly swp{ StateWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
    swp.getPosition()[0] = i * max / num;
    program.appendMoveInstruction(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  // leave final velocity/acceleration unset
  StateWaypointPoly swp{ StateWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
  swp.getPosition()[0] = max;
  program.appendMoveInstruction(MoveInstruction(swp, MoveInstructionType::FREESPACE));

  return program;
}

void runTrajectoryContainerInterfaceTest(double path_tolerance)
{
  TimeOptimalTrajectoryGeneration solver(path_tolerance, 1e-3);
  CompositeInstruction program = createStraightTrajectory();
  Eigen::VectorXd max_velocity(6);
  max_velocity << 2.088, 2.082, 3.27, 3.6, 3.3, 3.078;
  Eigen::VectorXd max_acceleration(6);
  max_acceleration << 1, 1, 1, 1, 1, 1;
  TrajectoryContainer::Ptr trajectory = std::make_shared<InstructionsTrajectory>(program);
  EXPECT_TRUE(solver.computeTimeStamps(*trajectory, max_velocity, max_acceleration));
  ASSERT_LT(
      program.back().as<tesseract_planning::MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime(),
      5.0);
}

TEST(time_optimal_trajectory_generation, testTrajectoryContainerInterface)  // NOLINT
{
  runTrajectoryContainerInterfaceTest(0.001);
  runTrajectoryContainerInterfaceTest(0.01);
  runTrajectoryContainerInterfaceTest(0.0001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
