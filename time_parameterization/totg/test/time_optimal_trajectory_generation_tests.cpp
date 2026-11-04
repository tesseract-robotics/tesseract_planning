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

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <gtest/gtest.h>
#include <tesseract/time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract/time_parameterization/totg/time_optimal_trajectory_generation_profiles.h>
#include <tesseract/time_parameterization/totg/cereal_serialization.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/profile_dictionary.h>
#include <tesseract/common/unit_test_utils.h>
#include <tesseract/common/serialization.h>
#include <tesseract/environment/environment.h>
#include <tesseract/command_language/poly/state_waypoint_poly.h>
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/state_waypoint.h>
#include <tesseract/command_language/move_instruction.h>
#include <tesseract/command_language/set_digital_instruction.h>
#include <tesseract/time_parameterization/instructions_trajectory.h>
#include "totg_test_oracles.h"

using namespace tesseract::time_parameterization;
using namespace tesseract::command_language;

using tesseract::time_parameterization::totg::Path;
using tesseract::time_parameterization::totg::PathData;
using tesseract::time_parameterization::totg::Trajectory;

class TimeOptimalTrajectoryGenerationUnit : public ::testing::Test
{
protected:
  std::string name_{ "TimeOptimalTrajectoryGenerationUnit" };
  tesseract::common::GeneralResourceLocator::Ptr locator_;
  tesseract::environment::Environment::Ptr env_;
  tesseract::common::ManipulatorInfo manip_;

  void SetUp() override
  {
    locator_ = std::make_shared<tesseract::common::GeneralResourceLocator>();
    auto env = std::make_shared<tesseract::environment::Environment>();

    std::filesystem::path urdf_path(
        locator_->locateResource("package://tesseract/support/urdf/abb_irb2400.urdf")->getFilePath());
    std::filesystem::path srdf_path(
        locator_->locateResource("package://tesseract/support/urdf/abb_irb2400.srdf")->getFilePath());
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator_));
    env_ = env;

    manip_.manipulator = "manipulator";
    manip_.working_frame = "base_link";
    manip_.tcp_frame = "tool0";
  }

  /** @brief Expect compute to throw a message containing expected, for the override profile after change */
  void expectComputeError(const std::function<void(TimeOptimalTrajectoryGenerationCompositeProfile&)>& change,
                          const std::string& expected) const;

  /** @brief Run compute on the program with the override profile, for the fixture's manipulator */
  bool computeWithOverrideProfile(CompositeInstruction& program) const;
};

TEST_F(TimeOptimalTrajectoryGenerationUnit, test1)  // NOLINT
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

TEST_F(TimeOptimalTrajectoryGenerationUnit, test2)  // NOLINT
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

TEST_F(TimeOptimalTrajectoryGenerationUnit, test3)  // NOLINT
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
TEST_F(TimeOptimalTrajectoryGenerationUnit, testLargeAccel)  // NOLINT
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

TEST_F(TimeOptimalTrajectoryGenerationUnit, profileConstructor)  // NOLINT
{
  TimeOptimalTrajectoryGenerationCompositeProfile profile(3, 5, 0.001, 0.1);
  EXPECT_NEAR(profile.max_velocity_scaling_factor, 3, 1e-6);
  EXPECT_NEAR(profile.max_acceleration_scaling_factor, 5, 1e-6);
  EXPECT_NEAR(profile.path_tolerance, 0.001, 1e-6);
  EXPECT_NEAR(profile.min_angle_change, 0.1, 1e-6);
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, testCommandLanguageInterface)  // NOLINT
{
  CompositeInstruction program;
  program.setManipulatorInfo(manip_);

  Eigen::VectorXd waypoint(6);
  std::list<Eigen::VectorXd> waypoints;

  std::vector<tesseract::common::JointId> joint_ids = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
  };

  {
    waypoint << 0, 0.7, -2.1, 0, -0.25, 0;
    StateWaypoint wp{ joint_ids, waypoint };
    MoveInstruction plan_f0(wp, MoveInstructionType::FREESPACE, "profile_name");
    plan_f0.setDescription("freespace_motion");
    program.push_back(plan_f0);
  }
  {
    waypoint << 0, 0, 0, 0, 0, 0;
    StateWaypoint wp{ joint_ids, waypoint };
    MoveInstruction plan_f0(wp, MoveInstructionType::FREESPACE, "profile_name");
    plan_f0.setDescription("freespace_motion");
    program.push_back(plan_f0);
  }
  {
    waypoint << 0, 0.70001, -2.1, 0, -0.25, 0;
    StateWaypoint wp{ joint_ids, waypoint };
    MoveInstruction plan_f0(wp, MoveInstructionType::FREESPACE, "profile_name");
    plan_f0.setDescription("freespace_motion");
    program.push_back(plan_f0);
  }
  {
    waypoint << 0, 0, 0, 0, 0, 0.1;
    StateWaypoint wp{ joint_ids, waypoint };
    MoveInstruction plan_f0(wp, MoveInstructionType::FREESPACE, "profile_name");
    plan_f0.setDescription("freespace_motion");
    program.push_back(plan_f0);
  }

  // Profile
  auto profile = std::make_shared<TimeOptimalTrajectoryGenerationCompositeProfile>();
  profile->path_tolerance = 0.001;
  profile->min_angle_change = 1e-3;
  profile->override_limits = true;
  profile->velocity_limits = Eigen::MatrixX2d(6, 2);
  profile->velocity_limits.col(0) = -1 * Eigen::VectorXd::Ones(6);
  profile->velocity_limits.col(1) = Eigen::VectorXd::Ones(6);
  profile->acceleration_limits = Eigen::MatrixX2d(6, 2);
  profile->acceleration_limits.col(0) = -1 * Eigen::VectorXd::Ones(6);
  profile->acceleration_limits.col(1) = Eigen::VectorXd::Ones(6);

  // Serialization
  tesseract::common::testSerializationDerivedClass<tesseract::common::Profile,
                                                   TimeOptimalTrajectoryGenerationCompositeProfile>(profile,
                                                                                                    "testCommandLanguag"
                                                                                                    "eInterface");

  // Profile Dictionary
  tesseract::common::ProfileDictionary profiles;
  profiles.addProfile(name_, DEFAULT_PROFILE_KEY, profile);

  TimeOptimalTrajectoryGeneration solver(name_);
  EXPECT_EQ(solver.getName(), name_);
  EXPECT_TRUE(solver.compute(program, *env_, profiles));
  test_suite::expectTimesStrictlyIncreasing(program);
  test_suite::expectVelocitiesWithinLimits(program, profile->velocity_limits.col(1));

  // Error
  std::string empty_name;
  EXPECT_ANY_THROW(TimeOptimalTrajectoryGeneration(std::move(empty_name)));  // NOLINT
}

/** @brief A straight trajectory in the first joint, with the given joint ids and number of joints per waypoint */
CompositeInstruction createStraightTrajectory(const std::vector<tesseract::common::JointId>& joint_ids,
                                              Eigen::Index num_joints)
{
  CompositeInstruction program;
  for (int i = 0; i <= 10; i++)
  {
    // Set through the setters: the constructor rejects ids that do not match the positions in size
    StateWaypoint swp;
    swp.setJointIds(joint_ids);
    Eigen::VectorXd position = Eigen::VectorXd::Zero(num_joints);
    position[0] = i * 2.0 / 10;
    swp.setPosition(position);
    program.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }
  return program;
}

// Initialize one-joint, straight-line trajectory
CompositeInstruction createStraightTrajectory()
{
  return createStraightTrajectory({ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" }, 6);
}

void runTrajectoryContainerInterfaceTest(const std::string& name,
                                         const tesseract::environment::Environment& env,
                                         const tesseract::common::ManipulatorInfo& manip_info,
                                         double path_tolerance)
{
  CompositeInstruction program = createStraightTrajectory();
  program.setManipulatorInfo(manip_info);

  // Profile
  auto profile = std::make_shared<TimeOptimalTrajectoryGenerationCompositeProfile>();
  profile->path_tolerance = path_tolerance;
  profile->min_angle_change = 1e-3;
  profile->override_limits = true;
  profile->velocity_limits = Eigen::MatrixX2d(6, 2);
  profile->velocity_limits.col(0) << -2.088, -2.082, -3.27, -3.6, -3.3, -3.078;
  profile->velocity_limits.col(1) << 2.088, 2.082, 3.27, 3.6, 3.3, 3.078;
  profile->acceleration_limits = Eigen::MatrixX2d(6, 2);
  profile->acceleration_limits.col(0) = -1 * Eigen::VectorXd::Ones(6);
  profile->acceleration_limits.col(1) = Eigen::VectorXd::Ones(6);

  // Profile Dictionary
  tesseract::common::ProfileDictionary profiles;
  ;
  profiles.addProfile(name, DEFAULT_PROFILE_KEY, profile);

  // Solve
  TimeOptimalTrajectoryGeneration solver(name);
  EXPECT_TRUE(solver.compute(program, env, profiles));
  test_suite::expectTimesStrictlyIncreasing(program);
  test_suite::expectVelocitiesWithinLimits(program, profile->velocity_limits.col(1));
  ASSERT_LT(program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getTime(), 5.0);
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, testTrajectoryContainerInterface)  // NOLINT
{
  runTrajectoryContainerInterfaceTest(name_, *env_, manip_, 0.001);
  runTrajectoryContainerInterfaceTest(name_, *env_, manip_, 0.01);
  runTrajectoryContainerInterfaceTest(name_, *env_, manip_, 0.0001);
}

/** @brief Run compute and return the message of the exception it throws, or an empty string if it does not throw */
std::string computeErrorMessage(const std::string& name,
                                const tesseract::environment::Environment& env,
                                const tesseract::common::ManipulatorInfo& manip_info,
                                const TimeOptimalTrajectoryGenerationCompositeProfile::Ptr& profile)
{
  CompositeInstruction program = createStraightTrajectory();
  program.setManipulatorInfo(manip_info);
  tesseract::common::ProfileDictionary profiles;
  profiles.addProfile(name, DEFAULT_PROFILE_KEY, profile);
  TimeOptimalTrajectoryGeneration solver(name);
  try
  {
    solver.compute(program, env, profiles);
  }
  catch (const std::exception& e)
  {
    return e.what();
  }
  return {};
}

/** @brief A profile overriding the limits with symmetric velocity and acceleration limits for six joints */
TimeOptimalTrajectoryGenerationCompositeProfile::Ptr createOverrideProfile()
{
  auto profile = std::make_shared<TimeOptimalTrajectoryGenerationCompositeProfile>();
  profile->override_limits = true;
  profile->velocity_limits = Eigen::RowVector2d(-1, 1).replicate(6, 1);
  profile->acceleration_limits = Eigen::RowVector2d(-1, 1).replicate(6, 1);
  return profile;
}

void TimeOptimalTrajectoryGenerationUnit::expectComputeError(
    const std::function<void(TimeOptimalTrajectoryGenerationCompositeProfile&)>& change,
    const std::string& expected) const
{
  auto profile = createOverrideProfile();
  change(*profile);
  const std::string message = computeErrorMessage(name_, *env_, manip_, profile);
  EXPECT_NE(message.find(expected), std::string::npos) << "expected '" << expected << "', got '" << message << "'";
}

bool TimeOptimalTrajectoryGenerationUnit::computeWithOverrideProfile(CompositeInstruction& program) const
{
  program.setManipulatorInfo(manip_);
  tesseract::common::ProfileDictionary profiles;
  profiles.addProfile(name_, DEFAULT_PROFILE_KEY, createOverrideProfile());
  const TimeOptimalTrajectoryGeneration solver(name_);
  return solver.compute(program, *env_, profiles);
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, scalingFactorErrorsNameTheirClassAndFactor)  // NOLINT
{
  expectComputeError([](auto& p) { p.max_velocity_scaling_factor = 0.0; },
                     "TimeOptimalTrajectoryGeneration, velocity scale factor");
  expectComputeError([](auto& p) { p.max_acceleration_scaling_factor = 1.5; },
                     "TimeOptimalTrajectoryGeneration, acceleration scale factor");
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, limitRowsMustMatchTheJointCount)  // NOLINT
{
  // Valid signs, so only the row count is wrong
  expectComputeError([](auto& p) { p.velocity_limits = Eigen::RowVector2d(-1, 1).replicate(5, 1); },
                     "velocity limits must have one row per joint");
  expectComputeError([](auto& p) { p.acceleration_limits = Eigen::RowVector2d(-1, 1).replicate(7, 1); },
                     "acceleration limits must have one row per joint");
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, nonPositivePathToleranceIsRejected)  // NOLINT
{
  for (double path_tolerance : { 0.0, -0.1, std::numeric_limits<double>::quiet_NaN() })
    expectComputeError([path_tolerance](auto& p) { p.path_tolerance = path_tolerance; }, "path tolerance");
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, minAngleChangeMustBeFiniteAndNotNegative)  // NOLINT
{
  for (double min_angle_change :
       { -0.001, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() })
    expectComputeError([min_angle_change](auto& p) { p.min_angle_change = min_angle_change; }, "min angle change");

  // Zero merges only exact duplicates
  auto profile = createOverrideProfile();
  profile->min_angle_change = 0.0;
  EXPECT_EQ(computeErrorMessage(name_, *env_, manip_, profile), "");
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, AccelerationLimitIsRespected)  // NOLINT
{
  const double resample_dt = 0.01;
  const Path path({ Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(1, 1, 0) }, 0.001);
  const Eigen::VectorXd max_accelerations = Eigen::VectorXd::Constant(3, 0.5);
  const Trajectory parameterized(path, Eigen::VectorXd::Constant(3, 0.1), max_accelerations, 0.001);
  ASSERT_TRUE(parameterized.isValid());

  const auto sample_count = static_cast<std::size_t>(std::ceil(parameterized.getDuration() / resample_dt));
  Eigen::VectorXd previous_velocity = Eigen::VectorXd::Zero(3);
  for (std::size_t sample = 0; sample <= sample_count; ++sample)
  {
    const double t = std::min(parameterized.getDuration(), static_cast<double>(sample) * resample_dt);
    const Eigen::VectorXd velocity = parameterized.getVelocity(parameterized.getPathData(t));
    const double acceleration = (velocity - previous_velocity).norm() / resample_dt;
    EXPECT_LT(acceleration, max_accelerations.norm() + 1e-3) << "t = " << t;
    previous_velocity = velocity;
  }
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, signInvertedLimitsAreRejected)  // NOLINT
{
  expectComputeError([](auto& p) { p.velocity_limits(2, 0) = 0.5; }, "velocity limits must have lower <= 0 <= upper");
  expectComputeError([](auto& p) { p.velocity_limits(2, 1) = -0.5; }, "velocity limits must have lower <= 0 <= upper");
  expectComputeError([](auto& p) { p.acceleration_limits(4, 0) = std::numeric_limits<double>::quiet_NaN(); },
                     "acceleration limits must have lower <= 0 <= upper");
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, infiniteLimitsAreRejected)  // NOLINT
{
  // An infinite side is rejected even where the other side would make the joint's limit finite
  expectComputeError([](auto& p) { p.velocity_limits(2, 0) = -std::numeric_limits<double>::infinity(); },
                     "velocity limits must be finite");
  expectComputeError([](auto& p) { p.acceleration_limits(4, 1) = std::numeric_limits<double>::infinity(); },
                     "acceleration limits must be finite");
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, asymmetricLimitsUseTheTighterSide)  // NOLINT
{
  // The straight trajectory moves joint_1 in the positive direction only, so only a tighter lower
  // limit tells the two sides apart.
  auto profile = createOverrideProfile();
  profile->velocity_limits(0, 0) = -0.5;

  CompositeInstruction program = createStraightTrajectory();
  program.setManipulatorInfo(manip_);
  tesseract::common::ProfileDictionary profiles;
  profiles.addProfile(name_, DEFAULT_PROFILE_KEY, profile);
  TimeOptimalTrajectoryGeneration solver(name_);
  ASSERT_TRUE(solver.compute(program, *env_, profiles));

  Eigen::VectorXd max_velocity = Eigen::VectorXd::Ones(6);
  max_velocity[0] = 0.5;
  test_suite::expectTimesStrictlyIncreasing(program);
  test_suite::expectVelocitiesWithinLimits(program, max_velocity);
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, waypointJointsMustMatchTheGroup)  // NOLINT
{
  auto run = [this](CompositeInstruction program) {
    program.setManipulatorInfo(manip_);
    tesseract::common::ProfileDictionary profiles;
    profiles.addProfile(name_, DEFAULT_PROFILE_KEY, createOverrideProfile());
    TimeOptimalTrajectoryGeneration solver(name_);
    return solver.compute(program, *env_, profiles);
  };

  EXPECT_TRUE(run(createStraightTrajectory()));

  // Waypoints without ids are checked on their size alone
  EXPECT_TRUE(run(createStraightTrajectory({}, 6)));
  EXPECT_FALSE(run(createStraightTrajectory({}, 5)));

  EXPECT_FALSE(run(createStraightTrajectory({ "j1", "j2", "j3", "j4", "j5", "j6" }, 6)));
  EXPECT_FALSE(run(createStraightTrajectory({ "joint_2", "joint_1", "joint_3", "joint_4", "joint_5", "joint_6" }, 6)));
  EXPECT_FALSE(run(createStraightTrajectory({ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5" }, 5)));
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, testRelevantZeroMaxAccelerationsInvalidateTrajectory)  // NOLINT
{
  const Eigen::Vector2d max_velocity(1, 1);
  const Path path({ Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1) }, 0.1);

  EXPECT_FALSE(Trajectory(path, max_velocity, Eigen::Vector2d(0, 1)).isValid());
  EXPECT_FALSE(Trajectory(path, max_velocity, Eigen::Vector2d(1, 0)).isValid());
  EXPECT_FALSE(Trajectory(path, max_velocity, Eigen::Vector2d(0, 0)).isValid());
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, testIrrelevantZeroMaxAccelerationsDontInvalidateTrajectory)  // NOLINT
{
  const Eigen::Vector2d max_velocity(1, 1);

  EXPECT_TRUE(
      Trajectory(Path({ Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 1) }, 0.1), max_velocity, Eigen::Vector2d(0, 1))
          .isValid());
  EXPECT_TRUE(
      Trajectory(Path({ Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0) }, 0.1), max_velocity, Eigen::Vector2d(1, 0))
          .isValid());
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, zeroLimitsOnMovingJointsAreRejected)  // NOLINT
{
  // createStraightTrajectory moves joint_1 only. The limits are valid; it is this program that moves a joint they lock,
  // so compute fails without throwing
  const auto compute = [this](const std::function<void(TimeOptimalTrajectoryGenerationCompositeProfile&)>& change,
                              CompositeInstruction& program) {
    auto profile = createOverrideProfile();
    change(*profile);
    tesseract::common::ProfileDictionary profiles;
    profiles.addProfile(name_, DEFAULT_PROFILE_KEY, profile);
    bool ok{ false };
    EXPECT_NO_THROW(ok = TimeOptimalTrajectoryGeneration(name_).compute(program, *env_, profiles));  // NOLINT
    return ok;
  };

  CompositeInstruction program = createStraightTrajectory();
  program.setManipulatorInfo(manip_);
  const CompositeInstruction input = program;
  EXPECT_FALSE(compute([](auto& p) { p.acceleration_limits.row(0).setZero(); }, program));
  EXPECT_EQ(program, input);
  EXPECT_FALSE(compute([](auto& p) { p.velocity_limits.row(0).setZero(); }, program));
  EXPECT_EQ(program, input);

  // Zero limits on a joint the program does not move
  EXPECT_TRUE(compute(
      [](auto& p) {
        p.velocity_limits.row(3).setZero();
        p.acceleration_limits.row(3).setZero();
      },
      program));
  test_suite::expectTimesStrictlyIncreasing(program);
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, relevantZeroMaxVelocityInvalidatesTrajectory)  // NOLINT
{
  // The second joint moves on the first segment, so its zero limit caps the path velocity at zero there
  const Path path({ Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1), Eigen::Vector2d(2, 1) }, 0.1);
  EXPECT_FALSE(Trajectory(path, Eigen::Vector2d(1, 0), Eigen::Vector2d(1, 1)).isValid());

  // A zero limit on a joint the path never moves leaves the trajectory valid
  const Path straight({ Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0) }, 0.1);
  const Trajectory trajectory(straight, Eigen::Vector2d(1, 0), Eigen::Vector2d(1, 1));
  ASSERT_TRUE(trajectory.isValid());
  EXPECT_TRUE(std::isfinite(trajectory.getDuration()));
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, testTimeStepZeroMakesTrajectoryInvalid)  // NOLINT
{
  const Path path({ Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1) }, 0.1);
  for (double time_step : { 0.0, -0.001 })
    EXPECT_FALSE(Trajectory(path, Eigen::Vector2d(1, 1), Eigen::Vector2d(1, 1), time_step).isValid()) << time_step;
}

namespace
{
/** @brief Expect the same bit patterns, so NaN results compare too */
void expectSameBits(const Eigen::VectorXd& actual, const Eigen::VectorXd& expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  EXPECT_EQ(std::memcmp(actual.data(), expected.data(), sizeof(double) * static_cast<std::size_t>(actual.size())), 0)
      << actual.transpose() << " vs " << expected.transpose();
}

void expectSameBits(double actual, double expected)
{
  expectSameBits(Eigen::VectorXd::Constant(1, actual), Eigen::VectorXd::Constant(1, expected));
}

void expectSameBits(const PathData& actual, const PathData& expected)
{
  const auto fields = [](const PathData& d) {
    return (Eigen::VectorXd(6) << d.prev_path_pos, d.prev_path_vel, d.prev_time, d.path_pos, d.path_vel, d.time)
        .finished();
  };
  expectSameBits(fields(actual), fields(expected));
}

/** @brief Each value with its neighbouring doubles on both sides */
std::vector<double> withNeighbours(const std::vector<double>& values)
{
  std::vector<double> out;
  for (double v : values)
  {
    out.push_back(std::nextafter(v, -std::numeric_limits<double>::infinity()));
    out.push_back(v);
    out.push_back(std::nextafter(v, std::numeric_limits<double>::infinity()));
  }
  return out;
}

/** @brief Sort, then alternate between the smallest and the largest left, so every query jumps far forward or back */
std::vector<double> alternateEnds(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  std::vector<double> out;
  std::size_t lo = 0;
  std::size_t hi = values.size();
  while (lo < hi)
  {
    out.push_back(values[lo++]);
    if (lo < hi)
      out.push_back(values[--hi]);
  }
  return out;
}

// The first three waypoints are collinear, so the blend at the second has zero length and two segments start at one
// position
const std::list<Eigen::VectorXd> cursor_waypoints{ Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0),
                                                   Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(2, 1, 0),
                                                   Eigen::Vector3d(2, 1, 1), Eigen::Vector3d(0, 0, 0) };
const Eigen::Vector3d cursor_max_velocity(1, 1, 1);
const Eigen::Vector3d cursor_max_acceleration(1, 2, 3);
}  // namespace

TEST_F(TimeOptimalTrajectoryGenerationUnit, pathLookupsInAnyOrderMatchAFreshPath)  // NOLINT
{
  const Path path(cursor_waypoints, 0.1);
  std::vector<double> boundaries{ 0.0, path.getLength() };
  for (const auto& switching_point : path.getSwitchingPoints())
    boundaries.push_back(switching_point.first);
  for (double s : path.getMapping())
    boundaries.push_back(s);

  for (double s : alternateEnds(withNeighbours(boundaries)))
  {
    const Path fresh(cursor_waypoints, 0.1);
    expectSameBits(path.getConfig(s), fresh.getConfig(s));
    expectSameBits(path.getTangent(s), fresh.getTangent(s));
    expectSameBits(path.getCurvature(s), fresh.getCurvature(s));
  }

  // The segments that share a start, approached from below and from above
  for (double before : { 0.5, path.getLength() })
  {
    const Path fresh(cursor_waypoints, 0.1);
    path.getConfig(before);
    expectSameBits(path.getConfig(1.0), fresh.getConfig(1.0));
    expectSameBits(path.getTangent(1.0), fresh.getTangent(1.0));
  }

  // A NaN position finds the first segment
  path.getConfig(path.getLength());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  expectSameBits(path.getTangent(nan), Path(cursor_waypoints, 0.1).getTangent(nan));
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, trajectoryLookupsInAnyOrderMatchAFreshTrajectory)  // NOLINT
{
  const Path path(cursor_waypoints, 0.1);
  const Trajectory trajectory(path, cursor_max_velocity, cursor_max_acceleration);
  ASSERT_TRUE(trajectory.isValid());
  const double duration = trajectory.getDuration();

  // The step keys, read off the steps that samples land in
  std::vector<double> times{ 0.0, duration };
  std::vector<double> positions{ 0.0, path.getLength() };
  for (int i = 1; i < 40; ++i)
  {
    const PathData data = trajectory.getPathData(duration * i / 40.0);
    times.push_back(data.prev_time);
    positions.push_back(data.prev_path_pos);
  }
  for (double s : path.getMapping())
    positions.push_back(s);

  // Each lookup is compared with one in a copy of a trajectory never queried, so it starts from the first step
  const Trajectory unqueried(path, cursor_max_velocity, cursor_max_acceleration);
  for (double t : alternateEnds(withNeighbours(times)))
  {
    if (t < 0.0)
      continue;
    expectSameBits(trajectory.getPathData(t), Trajectory(unqueried).getPathData(t));
  }
  for (double s : alternateEnds(withNeighbours(positions)))
  {
    if (s < 0.0 || s > path.getLength())
      continue;
    expectSameBits(trajectory.getTime(s), Trajectory(unqueried).getTime(s));
  }

  // A NaN position matches no step
  trajectory.getTime(path.getLength() * 0.9);
  EXPECT_THROW(trajectory.getTime(std::numeric_limits<double>::quiet_NaN()), std::runtime_error);  // NOLINT

  // A failed lookup leaves the next one unaffected
  expectSameBits(trajectory.getTime(path.getLength() * 0.95), Trajectory(unqueried).getTime(path.getLength() * 0.95));
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, copiedTrajectoryLooksUpInItsOwnSteps)  // NOLINT
{
  const Path path(cursor_waypoints, 0.1);
  auto original = std::make_unique<Trajectory>(path, cursor_max_velocity, cursor_max_acceleration);
  ASSERT_TRUE(original->isValid());
  const double duration = original->getDuration();
  original->getPathData(0.5 * duration);
  original->getTime(0.5 * path.getLength());

  const Trajectory fresh(path, cursor_max_velocity, cursor_max_acceleration);
  const PathData expected_data = fresh.getPathData(0.75 * duration);
  const double expected_time = fresh.getTime(0.75 * path.getLength());

  // A copy that kept the original's cursors would read freed steps here, which only a sanitizer reliably reports
  const Trajectory copy(*original);
  original.reset();

  // Later than the original's last queries, so a lookup resumes rather than restarts
  expectSameBits(copy.getPathData(0.75 * duration), expected_data);
  expectSameBits(copy.getTime(0.75 * path.getLength()), expected_time);
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, lookupsOutsideTheTrajectoryReadItsEnds)  // NOLINT
{
  const Path path(cursor_waypoints, 0.1);
  const Trajectory trajectory(path, cursor_max_velocity, cursor_max_acceleration);
  ASSERT_TRUE(trajectory.isValid());
  const double nan = std::numeric_limits<double>::quiet_NaN();

  // Before the start, on the first lookup and on one after a later time
  const auto expect_first_step = [](const PathData& data) {
    EXPECT_EQ(data.prev_time, 0.0);
    EXPECT_EQ(data.prev_path_pos, 0.0);
    EXPECT_TRUE(std::isfinite(data.path_pos));
  };
  expect_first_step(trajectory.getPathData(-0.1));
  trajectory.getPathData(0.5 * trajectory.getDuration());
  expect_first_step(trajectory.getPathData(-0.1));

  const PathData at_nan = trajectory.getPathData(nan);
  EXPECT_EQ(at_nan.prev_time, 0.0);
  EXPECT_TRUE(std::isnan(at_nan.path_pos));
  EXPECT_TRUE(std::isnan(at_nan.path_vel));

  EXPECT_DEBUG_DEATH(EXPECT_EQ(trajectory.getTime(-1e-12), 0.0), "");  // NOLINT
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, pathNeedsTwoWaypoints)  // NOLINT
{
  EXPECT_THROW(Path(std::list<Eigen::VectorXd>{}, 0.1), std::invalid_argument);                         // NOLINT
  EXPECT_THROW(Path(std::list<Eigen::VectorXd>{ Eigen::Vector2d(0, 0) }, 0.1), std::invalid_argument);  // NOLINT
}

// A temporary path's switching points come back as a copy, so that a loop over them outlives the path
static_assert(std::is_same_v<decltype(std::declval<Path>().getSwitchingPoints()), std::list<std::pair<double, bool>>>);
static_assert(std::is_same_v<decltype(std::declval<const Path&>().getSwitchingPoints()),
                             const std::list<std::pair<double, bool>>&>);

TEST_F(TimeOptimalTrajectoryGenerationUnit, getTimeInvertsGetPathData)  // NOLINT
{
  // Long enough to cruise, and too short to; the low acceleration rounds the first steps' 2 a d to zero next to the
  // start
  for (double length : { 10.0, 0.5 })
  {
    const Path path({ Eigen::Vector2d(0, 0), Eigen::Vector2d(length, 0) }, 0.1);
    const Trajectory trajectory(path, Eigen::Vector2d(1, 1), Eigen::Vector2d(0.25, 0.25));
    ASSERT_TRUE(trajectory.isValid());

    double t_prev = 0.0;
    for (int i = 1; i <= 200; ++i)
    {
      const double s = path.getLength() * i / 201.0;
      const double t = trajectory.getTime(s);
      ASSERT_TRUE(std::isfinite(t)) << "length " << length << " s " << s;
      EXPECT_GT(t, t_prev) << "length " << length << " s " << s;
      EXPECT_NEAR(trajectory.getPathData(t).path_pos, s, 1e-9) << "length " << length;
      t_prev = t;
    }

    const double t_first = trajectory.getTime(std::nextafter(0.0, 1.0));
    EXPECT_GE(t_first, 0.0);
    EXPECT_LT(t_first, 1e-100);
  }
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, cruiseWaypointsAreTimedAtTheCruiseVelocity)  // NOLINT
{
  CompositeInstruction program = createStraightTrajectory();
  ASSERT_TRUE(computeWithOverrideProfile(program));

  test_suite::expectTimesStrictlyIncreasing(program);
  test_suite::expectDisplacementsWithinLimits(program, Eigen::VectorXd::Ones(6));

  // Between two waypoints at the same velocity the motion cruises, so it takes the distance over that velocity
  const auto moves = program.flatten(moveFilter);
  int cruising{ 0 };
  for (std::size_t i = 1; i < moves.size(); ++i)
  {
    const auto& prev = moves[i - 1].get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
    const auto& curr = moves[i].get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
    const double v = curr.getVelocity()[0];
    if (!(v > 0.0) || std::abs(prev.getVelocity()[0] - v) > 1e-9)
      continue;
    ++cruising;
    EXPECT_NEAR(curr.getTime() - prev.getTime(), (curr.getPosition()[0] - prev.getPosition()[0]) / v, 1e-6)
        << "waypoints " << i - 1 << " to " << i;
  }
  EXPECT_GT(cruising, 0);
}

namespace
{
MoveInstruction joint1Move(double joint_1, double joint_2 = 0.0)
{
  StateWaypoint swp({ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" },
                    (Eigen::VectorXd(6) << joint_1, joint_2, 0, 0, 0, 0).finished());
  return MoveInstruction(swp, MoveInstructionType::FREESPACE);
}

CompositeInstruction joint1Program(const std::vector<double>& joint_1)
{
  CompositeInstruction program;
  for (double q : joint_1)
    program.push_back(joint1Move(q));
  return program;
}

std::vector<double> joint1Positions(const CompositeInstruction& program)
{
  std::vector<double> positions;
  for (const auto& move : program.flatten(moveFilter))
    positions.push_back(move.get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getPosition()[0]);
  return positions;
}

/** @brief Expect strictly increasing times, motion within the velocity limit, and rest at the last waypoint */
void expectTimedFromRestToRest(const CompositeInstruction& program)
{
  test_suite::expectTimesStrictlyIncreasing(program);
  test_suite::expectDisplacementsWithinLimits(program, Eigen::VectorXd::Ones(6));
  const auto moves = program.flatten(moveFilter);
  const auto& last = moves.back().get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
  EXPECT_TRUE(last.getVelocity().isZero(0.0)) << last.getVelocity().transpose();
}
}  // namespace

TEST_F(TimeOptimalTrajectoryGenerationUnit, repeatedWaypointsLeaveTheOutput)  // NOLINT
{
  const auto run = [this](CompositeInstruction program) {
    EXPECT_TRUE(computeWithOverrideProfile(program));
    return program;
  };

  // A repeat inside the program goes; a final run keeps its last waypoint, the goal
  const std::vector<std::pair<std::vector<double>, std::vector<double>>> cases{
    { { 0, 0.5, 0.5, 1.0 }, { 0, 0.5, 1.0 } },
    { { 0, 1, 1.0005 }, { 0, 1.0005 } },
    // Moving the last point onto the goal can make it repeat the point before, which then goes
    { { 0, 0.01, 0.0111, 0.0102 }, { 0, 0.0102 } },
    // A motion within min_angle_change of the start is still timed
    { { 0, 0.0005, 0.0009 }, { 0, 0.0009 } },
    { { 0, 0.0011, 0.0003 }, { 0, 0.0003 } },
  };
  for (const auto& [input, expected] : cases)
  {
    const CompositeInstruction program = run(joint1Program(input));
    EXPECT_EQ(joint1Positions(program), expected);
    expectTimedFromRestToRest(program);
  }

  // Nothing moves: the first waypoint stays, at rest at time zero
  for (const std::vector<double>& input : { std::vector<double>{ 0, 0, 0 }, std::vector<double>{ 0, 0.0005, 0 } })
  {
    CompositeInstruction input_program = joint1Program(input);
    const boost::uuids::uuid first = input_program.front().getUUID();
    const CompositeInstruction program = run(input_program);
    ASSERT_EQ(joint1Positions(program), std::vector<double>{ 0 });
    EXPECT_EQ(program.front().getUUID(), first);
    const auto& swp = program.front().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
    EXPECT_EQ(swp.getTime(), 0.0);
    EXPECT_TRUE(swp.getVelocity().isZero(0.0));
    EXPECT_TRUE(swp.getAcceleration().isZero(0.0));
  }

  // A repeat is one within min_angle_change in every joint
  CompositeInstruction program;
  program.push_back(joint1Move(0, 0));
  program.push_back(joint1Move(1, 0));
  program.push_back(joint1Move(1.0005, 0.002));
  EXPECT_EQ(joint1Positions(run(program)).size(), 3);
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, repeatedWaypointsLeaveOtherInstructionsInPlace)  // NOLINT
{
  const auto run = [this](CompositeInstruction& program) {
    EXPECT_TRUE(computeWithOverrideProfile(program));
    expectTimedFromRestToRest(program);
  };

  // I/O after the run's first waypoint keeps it, so the I/O still follows the motion
  CompositeInstruction blocked = joint1Program({ 0, 1 });
  blocked.push_back(SetDigitalInstruction("io", 0, true));
  blocked.push_back(joint1Move(1.0005));
  run(blocked);
  ASSERT_EQ(blocked.size(), 3);
  EXPECT_TRUE(blocked[1].isMoveInstruction());
  EXPECT_FALSE(blocked[2].isMoveInstruction());
  EXPECT_EQ(joint1Positions(blocked), (std::vector<double>{ 0, 1 }));

  // I/O before the run is not crossed, so the goal is kept
  CompositeInstruction free;
  free.push_back(joint1Move(0));
  free.push_back(SetDigitalInstruction("io", 0, true));
  free.push_back(joint1Move(1));
  free.push_back(joint1Move(1.0005));
  run(free);
  ASSERT_EQ(free.size(), 3);
  EXPECT_FALSE(free[1].isMoveInstruction());
  EXPECT_EQ(joint1Positions(free), (std::vector<double>{ 0, 1.0005 }));

  // When nothing moves, I/O still follows the one waypoint kept
  CompositeInstruction still = joint1Program({ 0 });
  still.push_back(SetDigitalInstruction("io", 0, true));
  still.push_back(joint1Move(0));
  run(still);
  ASSERT_EQ(still.size(), 2);
  EXPECT_TRUE(still[0].isMoveInstruction());
  EXPECT_FALSE(still[1].isMoveInstruction());
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, nonFiniteWaypointsAreRejected)  // NOLINT
{
  for (double bad : { std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity() })
  {
    CompositeInstruction program = joint1Program({ 0, bad, 1 });
    EXPECT_FALSE(computeWithOverrideProfile(program)) << bad;
  }
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, repeatedWaypointsLeaveNestedComposites)  // NOLINT
{
  // The second composite only repeats the first's end, so it is left empty
  CompositeInstruction program;
  program.push_back(joint1Program({ 0, 1 }));
  program.push_back(joint1Program({ 1 }));
  program.push_back(joint1Program({ 2, 2.0005 }));
  ASSERT_TRUE(computeWithOverrideProfile(program));
  expectTimedFromRestToRest(program);

  ASSERT_EQ(program.size(), 3);
  EXPECT_EQ(joint1Positions(program[0].as<CompositeInstruction>()), (std::vector<double>{ 0, 1 }));
  EXPECT_TRUE(program[1].as<CompositeInstruction>().empty());
  EXPECT_EQ(joint1Positions(program[2].as<CompositeInstruction>()), (std::vector<double>{ 2.0005 }));

  // The goal is kept across composites
  CompositeInstruction split;
  split.push_back(joint1Program({ 0, 1 }));
  split.push_back(joint1Program({ 1.0005 }));
  ASSERT_TRUE(computeWithOverrideProfile(split));
  expectTimedFromRestToRest(split);
  EXPECT_EQ(joint1Positions(split[0].as<CompositeInstruction>()), (std::vector<double>{ 0 }));
  EXPECT_EQ(joint1Positions(split[1].as<CompositeInstruction>()), (std::vector<double>{ 1.0005 }));

  // ...unless I/O in between would then come ahead of the motion
  CompositeInstruction blocked;
  blocked.push_back(joint1Program({ 0, 1 }));
  CompositeInstruction tail;
  tail.push_back(SetDigitalInstruction("io", 0, true));
  tail.push_back(joint1Move(1.0005));
  blocked.push_back(tail);
  ASSERT_TRUE(computeWithOverrideProfile(blocked));
  expectTimedFromRestToRest(blocked);
  EXPECT_EQ(joint1Positions(blocked[0].as<CompositeInstruction>()), (std::vector<double>{ 0, 1 }));
  ASSERT_EQ(blocked[1].as<CompositeInstruction>().size(), 1);
  EXPECT_FALSE(blocked[1].as<CompositeInstruction>().front().isMoveInstruction());
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, assignDataEndsAtTheDuration)  // NOLINT
{
  const Path path({ Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0), Eigen::Vector2d(1, 1) }, 0.1);
  const Trajectory trajectory(path, Eigen::Vector2d(1, 1), Eigen::Vector2d(1, 1));
  ASSERT_TRUE(trajectory.isValid());

  CompositeInstruction program;
  for (const Eigen::Vector2d& q : { Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0), Eigen::Vector2d(1, 1) })
    program.push_back(MoveInstruction(StateWaypoint({ "a", "b" }, q), MoveInstructionType::FREESPACE));
  auto flattened = program.flatten(moveFilter);
  InstructionsTrajectory waypoints(flattened);
  ASSERT_TRUE(trajectory.assignData(waypoints, { 0, 1, 2 }));

  const auto& last = program.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
  expectSameBits(last.getTime(), trajectory.getDuration());
  EXPECT_TRUE(last.getVelocity().isZero(0.0));
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, backwardIntegrationJoinsOnlyWhereTheCurvesMeet)  // NOLINT
{
  // Random spread waypoints on which the backward pass runs parallel to the forward one at a lower velocity before
  // they meet
  const std::vector<Eigen::VectorXd> waypoints{
    (Eigen::VectorXd(6) << 1.1213423098059754,
     -1.8270202485653966,
     -2.1115264439877963,
     -1.178873514726793,
     2.9135461657166584,
     1.7323036160661867)
        .finished(),
    (Eigen::VectorXd(6) << -1.1454937611120264,
     -2.5032361045916769,
     1.9364201984221188,
     0.3267322496586893,
     1.4191849223443147,
     -2.4105806185316228)
        .finished(),
    (Eigen::VectorXd(6) << 1.3605620581907094,
     0.095353218408931212,
     0.77502715593802729,
     -0.70538392462050759,
     -1.9867113174244126,
     -2.3284962511470577)
        .finished(),
    (Eigen::VectorXd(6) << -1.3680341014163291,
     2.6167991708821035,
     -1.7780856705029602,
     -1.4992164434709061,
     -1.8515878373920043,
     2.266750287460356)
        .finished(),
    (Eigen::VectorXd(6) << -0.22115632306905625,
     0.87588399033494491,
     -1.778853099442514,
     -2.8566033866830502,
     -1.2597628722968797,
     -1.4219644067766404)
        .finished(),
    (Eigen::VectorXd(6) << -0.38431399455717374,
     -0.57086953194929724,
     2.7135592918582105,
     1.55180329251955,
     2.099705867820608,
     2.8092408837593061)
        .finished(),
    (Eigen::VectorXd(6) << -1.9555497995594846,
     0.45639667844460541,
     1.5996155673720573,
     0.79573041402864342,
     -0.57319681469746409,
     1.3140653657649966)
        .finished(),
    (Eigen::VectorXd(6) << -1.4328977585036182,
     -2.3046038828311586,
     2.3972112560408552,
     2.5643709507550057,
     -3.1178839565328684,
     2.165761425910409)
        .finished(),
    (Eigen::VectorXd(6) << 2.7412005867968441,
     2.4069363031799988,
     2.7238951157986273,
     -3.0500152421319044,
     2.6947696937135657,
     1.2755949334580954)
        .finished(),
    (Eigen::VectorXd(6) << -2.6586838440394382,
     -2.2769338080220241,
     2.2731119440543219,
     1.6680595092481609,
     1.144903273721982,
     0.18195354245145889)
        .finished(),
    (Eigen::VectorXd(6) << 2.2361880377707575,
     3.0346906129403139,
     -1.2191869931382406,
     1.0440635543862449,
     1.6382321489391569,
     0.26426931181037361)
        .finished(),
    (Eigen::VectorXd(6) << -0.56762994425974012,
     2.638488029667553,
     1.3564924426803704,
     -3.1041618976431797,
     0.10578888414066892,
     -1.005679041362)
        .finished(),
    (Eigen::VectorXd(6) << -1.9660630456170782,
     1.3119306511643707,
     1.8921762155611801,
     -0.85079774110918416,
     -0.23490887970475213,
     -0.90347848041846501)
        .finished(),
    (Eigen::VectorXd(6) << -1.0758990383236249,
     -3.0638961559257445,
     2.4371079401331173,
     3.1407333369838697,
     -0.46904770306857546,
     1.5975395924379059)
        .finished(),
    (Eigen::VectorXd(6) << 2.1924629271808569,
     -2.2450722537625505,
     -0.68916516839495523,
     0.04959141294594268,
     1.7169495976988696,
     -1.7063518749754301)
        .finished(),
    (Eigen::VectorXd(6) << -0.93573365511926376,
     -0.49253317882523445,
     -2.9432504694152,
     1.0751370141946628,
     2.9871609033520206,
     -1.844694597511261)
        .finished(),
    (Eigen::VectorXd(6) << -0.72538316656880664,
     -1.8513933335242994,
     -2.399441867032523,
     -0.10726072392947683,
     -1.2930704723638751,
     0.043058849081623024)
        .finished(),
    (Eigen::VectorXd(6) << -0.91942226932579663,
     -0.32295373638974634,
     -0.12970006456214189,
     -2.3737669805606312,
     -0.45953216371983979,
     1.7498226442590368)
        .finished(),
    (Eigen::VectorXd(6) << 2.854811095393897,
     0.63022076324725063,
     -2.4413404941013908,
     -1.9069195720802634,
     -0.056998551518364783,
     1.5117766679736366)
        .finished(),
    (Eigen::VectorXd(6) << -1.4064216607332223,
     -2.003255255516128,
     -1.0408974916472498,
     2.0136190117802908,
     1.6775733291861927,
     -0.10968078958787987)
        .finished(),
  };
  const Eigen::VectorXd max_velocity = (Eigen::VectorXd(6) << 1.0882213037012505,
                                        0.79924855376902015,
                                        0.72776121821854389,
                                        1.224873844212915,
                                        1.4939258653889675,
                                        0.76866658201779936)
                                           .finished();
  const Eigen::VectorXd max_acceleration = (Eigen::VectorXd(6) << 1.2228100229153809,
                                            1.7253687545527456,
                                            0.94640508866629269,
                                            0.86390107576955011,
                                            1.183816859093048,
                                            0.84991742129126979)
                                               .finished();

  const Path path(std::list<Eigen::VectorXd>(waypoints.begin(), waypoints.end()), 0.1);
  const Trajectory trajectory(path, max_velocity, max_acceleration);
  ASSERT_TRUE(trajectory.isValid());

  // No step in the reported velocity is larger than the acceleration limit allows
  const double h = 1e-3;
  Eigen::VectorXd v_prev = trajectory.getVelocity(trajectory.getPathData(0.0));
  double worst{ 0 };
  for (int i = 1; i * h <= trajectory.getDuration(); ++i)
  {
    const double t = i * h;
    const Eigen::VectorXd v = trajectory.getVelocity(trajectory.getPathData(t));
    worst = std::max(worst, ((v - v_prev) / h).cwiseAbs().cwiseQuotient(max_acceleration).maxCoeff());
    v_prev = v;
  }
  EXPECT_LT(worst, 1.1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
