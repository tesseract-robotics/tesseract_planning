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
#include <tesseract/kinematics/joint_group.h>
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
  EXPECT_DOUBLE_EQ(1922.1436537997095, trajectory.getDuration());

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
  EXPECT_DOUBLE_EQ(1919.5597913722672, trajectory.getDuration());

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

TEST_F(TimeOptimalTrajectoryGenerationUnit, coreRejectsAReversal)  // NOLINT
{
  // The second waypoint turns the path back towards the first
  std::list<Eigen::VectorXd> waypoints;
  Eigen::VectorXd waypoint(6);
  waypoint << 0, 0.7, -2.1, 0, -0.25, 0;
  waypoints.push_back(waypoint);
  waypoint << 0, 0, 0, 0, 0, 0;
  waypoints.push_back(waypoint);
  waypoint << 0, 0.70001, -2.1, 0, -0.25, 0;
  waypoints.push_back(waypoint);
  waypoint << 0, 0, 0, 0, 0, 0.1;
  waypoints.push_back(waypoint);

  const Path path(waypoints, 0.001);
  EXPECT_TRUE(path.hasReversal());
  EXPECT_FALSE(Trajectory(path, Eigen::VectorXd::Ones(6), Eigen::VectorXd::Ones(6), 0.001).isValid());
}

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
  profile->use_start_velocity = true;
  TimeOptimalTrajectoryGenerationCompositeProfile at_rest(*profile);
  at_rest.use_start_velocity = false;
  EXPECT_FALSE(*profile == at_rest);
  profile->use_end_velocity = true;
  TimeOptimalTrajectoryGenerationCompositeProfile ending_at_rest(*profile);
  ending_at_rest.use_end_velocity = false;
  EXPECT_FALSE(*profile == ending_at_rest);

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
  test_suite::expectAccelerationsWithinLimits(program, profile->acceleration_limits.col(1));

  // The second waypoint turns the path back, so the motion stops there
  EXPECT_TRUE(program[1].as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getVelocity().isZero(0.0));

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
  test_suite::expectAccelerationsWithinLimits(program, profile->acceleration_limits.col(1));
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

TEST_F(TimeOptimalTrajectoryGenerationUnit, minAngleChangeHasAFloor)  // NOLINT
{
  for (double min_angle_change :
       { 0.0, -0.001, 1e-6, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() })
    expectComputeError([min_angle_change](auto& p) { p.min_angle_change = min_angle_change; }, "min angle change");

  auto profile = createOverrideProfile();
  profile->min_angle_change = 1e-5;
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
  test_suite::expectAccelerationsWithinLimits(
      program, (-profile->acceleration_limits.col(0)).cwiseMin(profile->acceleration_limits.col(1)));
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
  test_suite::expectAccelerationsWithinLimits(program, Eigen::VectorXd::Ones(6));
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

TEST_F(TimeOptimalTrajectoryGenerationUnit, hugeVelocityLimitsLeaveOnlyTheAccelerationLimits)  // NOLINT
{
  // A velocity limit far above what the acceleration limits allow does not bind, so the trajectory takes as long as
  // under a moderate one. Scaled down, the path's first blend lies below 0.1, where adding and then subtracting the
  // velocity switching-point scan's step does not always give back the position
  for (const double scale : { 1.0, 0.01 })
  {
    const Path path({ Eigen::Vector3d(0, 0, 0) * scale,
                      Eigen::Vector3d(1, 0.5, -0.3) * scale,
                      Eigen::Vector3d(1.4, -0.2, 0.6) * scale,
                      Eigen::Vector3d(0.5, 0.9, 1.2) * scale },
                    0.1 * scale);
    const Trajectory moderate(path, Eigen::VectorXd::Constant(3, 1e3), Eigen::VectorXd::Ones(3));
    ASSERT_TRUE(moderate.isValid()) << scale;
    const double duration = moderate.getDuration();
    for (const double limit : { 1e8, 1e300, std::numeric_limits<double>::max() })
    {
      const Trajectory trajectory(path, Eigen::VectorXd::Constant(3, limit), Eigen::VectorXd::Ones(3));
      EXPECT_TRUE(trajectory.isValid()) << scale << " " << limit;
      EXPECT_NEAR(trajectory.getDuration(), duration, 1e-9 * duration) << scale << " " << limit;
    }
  }
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

namespace
{
/**
 * @brief The worst exact joint acceleration T(s) s'' + K(s) s'^2 inside any integration step, divided by the limit
 * @details Recovers the steps from the lookups at a probe interval well below time_step
 */
double worstAccelerationInSteps(const Path& path,
                                const Trajectory& trajectory,
                                const Eigen::VectorXd& max_acceleration,
                                double time_step)
{
  struct Step
  {
    double t, s, v;
  };
  const double probe = 2e-3 * time_step;
  std::vector<Step> steps;
  for (int k = 0; k * probe < trajectory.getDuration(); ++k)
  {
    const PathData data = trajectory.getPathData(k * probe);
    if (steps.empty() || data.prev_time > steps.back().t)
      steps.push_back({ data.prev_time, data.prev_path_pos, data.prev_path_vel });
  }
  steps.push_back({ trajectory.getDuration(), path.getLength(), 0.0 });

  double worst{ 0 };
  for (std::size_t i = 0; i + 1 < steps.size(); ++i)
  {
    const double dt = steps[i + 1].t - steps[i].t;
    const double ds = steps[i + 1].s - steps[i].s;
    const double sdd = 2.0 * (ds - dt * steps[i].v) / (dt * dt);
    for (int f = 0; f <= 8; ++f)
    {
      const double fraction = std::min(std::max(f / 8.0, 1e-9), 1.0 - 1e-9);
      const double s = steps[i].s + (fraction * ds);
      const double v = steps[i].v + (fraction * dt * sdd);
      const Eigen::VectorXd a = path.getTangent(s) * sdd + path.getCurvature(s) * v * v;
      worst = std::max(worst, a.cwiseAbs().cwiseQuotient(max_acceleration).maxCoeff());
    }
  }
  return worst;
}
}  // namespace

TEST_F(TimeOptimalTrajectoryGenerationUnit, integrationStepsKeepTheAccelerationLimit)  // NOLINT
{
  // Dense random waypoints, whose backward pass decelerates from lines into blends
  const std::vector<Eigen::VectorXd> waypoints{
    (Eigen::VectorXd(6) << -0.65579606047800976,
     1.6305341103863198,
     0.88375613919118434,
     1.3792520549974243,
     1.7297410090512955,
     0.49512407542683956)
        .finished(),
    (Eigen::VectorXd(6) << -0.64119901666227186,
     1.6171914013989865,
     0.88087381761232797,
     1.373922123016432,
     1.7322346799208579,
     0.49661933108330031)
        .finished(),
    (Eigen::VectorXd(6) << -0.62846303314618301,
     1.6043420662099721,
     0.88337914954116203,
     1.3659640475914243,
     1.7467055010530308,
     0.48624214889569922)
        .finished(),
    (Eigen::VectorXd(6) << -0.62674036993575144,
     1.5902582854829386,
     0.88939754148485806,
     1.3735005545326844,
     1.7562925309373523,
     0.50485352857185495)
        .finished(),
    (Eigen::VectorXd(6) << -0.62790049349228894,
     1.5899317389336214,
     0.88357651861160258,
     1.3541639371062943,
     1.7638962782998822,
     0.52365205819066363)
        .finished(),
    (Eigen::VectorXd(6) << -0.62230409257841468,
     1.5710729818676559,
     0.90100805426902197,
     1.3374934735558532,
     1.7526912796796321,
     0.53914844988344968)
        .finished(),
    (Eigen::VectorXd(6) << -0.62036092706377577,
     1.5704705181362941,
     0.89267366807771853,
     1.3367129393389077,
     1.7404099866802252,
     0.52827886967159676)
        .finished(),
    (Eigen::VectorXd(6) << -0.629175976179559,
     1.5604246662952908,
     0.88072285814687479,
     1.328817460482524,
     1.7547908940320118,
     0.52237968327292916)
        .finished(),
    (Eigen::VectorXd(6) << -0.61769069575602742,
     1.5496255033077242,
     0.86241156413931175,
     1.3353013049389177,
     1.7745271002221175,
     0.52816798281456512)
        .finished(),
    (Eigen::VectorXd(6) << -0.6292428489010391,
     1.5331990750473943,
     0.87150836396966769,
     1.3222487345524307,
     1.7854025044701614,
     0.51124295647672846)
        .finished(),
    (Eigen::VectorXd(6) << -0.64062901316444643,
     1.5178383253400425,
     0.85376323612123861,
     1.3276177453455889,
     1.792132509368207,
     0.51217561637122655)
        .finished(),
    (Eigen::VectorXd(6) << -0.64487808431708737,
     1.4998035041951547,
     0.85853576990716163,
     1.310636852061787,
     1.7975128238223164,
     0.5208116143878917)
        .finished(),
    (Eigen::VectorXd(6) << -0.65437034658230919,
     1.4992395874462705,
     0.86914992842342953,
     1.3015772651362238,
     1.7938840767297486,
     0.51670620752799978)
        .finished(),
    (Eigen::VectorXd(6) << -0.65414527312580628,
     1.5171831264698663,
     0.88127619123390699,
     1.2965729109513411,
     1.792054540624221,
     0.52693620830256582)
        .finished(),
    (Eigen::VectorXd(6) << -0.66551281452830213,
     1.5332401926037307,
     0.89455470417305916,
     1.312521809471616,
     1.7844037148605607,
     0.52446100892156)
        .finished(),
    (Eigen::VectorXd(6) << -0.66622953226728421,
     1.5432965727050212,
     0.89346645327788454,
     1.2935351176906522,
     1.7670554413385506,
     0.50847057964665465)
        .finished(),
    (Eigen::VectorXd(6) << -0.67351162523156705,
     1.5487164714846922,
     0.89654305428978187,
     1.3077168078161017,
     1.7550246475502032,
     0.49900248878812331)
        .finished(),
  };
  const Eigen::VectorXd max_velocity = (Eigen::VectorXd(6) << 0.7483295128489762,
                                        1.2746354603674193,
                                        1.8479081329738245,
                                        1.1446014158050404,
                                        1.7600521322113427,
                                        0.77593825559627749)
                                           .finished();
  const Eigen::VectorXd max_acceleration = (Eigen::VectorXd(6) << 0.89230822956140776,
                                            1.5009400983009549,
                                            0.82768002176751199,
                                            1.2866478274695268,
                                            1.2981213905675786,
                                            0.5019603210569894)
                                               .finished();

  const Path path(std::list<Eigen::VectorXd>(waypoints.begin(), waypoints.end()), 0.1);
  const Trajectory trajectory(path, max_velocity, max_acceleration);
  ASSERT_TRUE(trajectory.isValid());

  const double worst = worstAccelerationInSteps(path, trajectory, max_acceleration, 0.001);

  // Euler integration at a 1 ms step overshoots by a few percent
  EXPECT_LT(worst, 1.05);
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, integrationStepsKeepTheAccelerationLimitAtAShortTimeStep)  // NOLINT
{
  // Dense random waypoints at a 0.1 ms step, where the steps the backward pass joins on are about a micrometre long
  const std::vector<Eigen::VectorXd> waypoints{
    (Eigen::VectorXd(6) << -1.455322847590911,
     0.50525778821539047,
     0.23264999276890228,
     1.7714755634773203,
     1.566289489071391,
     0.82108015937536061)
        .finished(),
    (Eigen::VectorXd(6) << -1.468114301895409,
     0.49170694322841291,
     0.22116046409975954,
     1.7628267471880603,
     1.5820015112149644,
     0.83459527551800883)
        .finished(),
    (Eigen::VectorXd(6) << -1.4716180204294467,
     0.50757648081553242,
     0.20188162559545372,
     1.7670009027569999,
     1.6008960783385933,
     0.83939118947661706)
        .finished(),
    (Eigen::VectorXd(6) << -1.4592272893992455,
     0.52318032747455889,
     0.18959011492151157,
     1.7806688467783947,
     1.5915172298896336,
     0.82177647961391886)
        .finished(),
    (Eigen::VectorXd(6) << -1.4636813589794251,
     0.50991192956984965,
     0.1935269385267292,
     1.7734962475037777,
     1.5746876963475285,
     0.8173578919593778)
        .finished(),
    (Eigen::VectorXd(6) << -1.4594245963457617,
     0.5049869411472484,
     0.17751435435067348,
     1.7745279555664284,
     1.5574768684678866,
     0.81731855387076469)
        .finished(),
    (Eigen::VectorXd(6) << -1.4692812871564962,
     0.49080640190413993,
     0.18031005624290389,
     1.7724429968276021,
     1.5431081714569619,
     0.83288273995660933)
        .finished(),
    (Eigen::VectorXd(6) << -1.4598782937288273,
     0.4859333595387953,
     0.20023724108557328,
     1.7647034291342314,
     1.535783814313459,
     0.84373772096290256)
        .finished(),
    (Eigen::VectorXd(6) << -1.4686913194060287,
     0.48477696353371807,
     0.21884981592511923,
     1.7458557273130624,
     1.5243135837157367,
     0.84648736829216165)
        .finished(),
    (Eigen::VectorXd(6) << -1.472888820857402,
     0.46645995554245673,
     0.21683995536191505,
     1.7522652816591384,
     1.54139045824504,
     0.84422853567769185)
        .finished(),
    (Eigen::VectorXd(6) << -1.4552847342368953,
     0.44968902303028507,
     0.21720948587840014,
     1.7444579651770713,
     1.5305134029524148,
     0.83633402162290271)
        .finished(),
    (Eigen::VectorXd(6) << -1.460711981323632,
     0.44156783940585165,
     0.2003089737269331,
     1.7425862022902827,
     1.5267279149743942,
     0.81947340745943176)
        .finished(),
    (Eigen::VectorXd(6) << -1.4507153504568644,
     0.45395177470007941,
     0.20629195753247764,
     1.750087124143739,
     1.5210892313763111,
     0.80810405429539156)
        .finished(),
  };
  const Eigen::VectorXd max_velocity = (Eigen::VectorXd(6) << 1.8897585866447471,
                                        0.55774425781799186,
                                        0.95906699437263798,
                                        0.60132892891696166,
                                        1.281202330663112,
                                        1.9243510379229172)
                                           .finished();
  const Eigen::VectorXd max_acceleration = (Eigen::VectorXd(6) << 0.97079115837935548,
                                            1.3194753024447889,
                                            0.78470208935848373,
                                            1.0849480333274344,
                                            1.1733674956226832,
                                            0.72072924811328787)
                                               .finished();

  const Path path(std::list<Eigen::VectorXd>(waypoints.begin(), waypoints.end()), 0.1);
  const Trajectory trajectory(path, max_velocity, max_acceleration, 1e-4);
  ASSERT_TRUE(trajectory.isValid());
  // The Euler integration's overshoot shrinks with the step
  EXPECT_LT(worstAccelerationInSteps(path, trajectory, max_acceleration, 1e-4), 1.01);
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, slowStopsKeepTheAccelerationLimit)  // NOLINT
{
  // So slow that the stop's last steps are shorter than the core's position tolerance; how the stop falls on the
  // step grid depends on the length
  const Eigen::Vector2d direction = Eigen::Vector2d(3, 1).normalized();
  for (const double length : { 0.05, 0.1, 0.316, 1.0 })
  {
    for (const auto& [velocity, acceleration] : { std::pair{ 0.0015, 0.1 }, std::pair{ 0.0026, 1.3 } })
    {
      const Path path({ Eigen::Vector2d::Zero(), Eigen::Vector2d(length * direction) }, 0.1);
      const Eigen::Vector2d max_acceleration = Eigen::Vector2d::Constant(acceleration);
      const Trajectory trajectory(path, Eigen::Vector2d::Constant(velocity), max_acceleration);
      ASSERT_TRUE(trajectory.isValid()) << length << " " << velocity;
      double worst{ 0 };
      for (int k = 0; k * 1e-4 < trajectory.getDuration(); ++k)
        worst = std::max(worst, trajectory.getAcceleration(trajectory.getPathData(k * 1e-4)).cwiseAbs().maxCoeff());
      worst = std::max(
          worst, trajectory.getAcceleration(trajectory.getPathData(trajectory.getDuration())).cwiseAbs().maxCoeff());
      EXPECT_LE(worst, acceleration * (1.0 + 1e-6)) << length << " " << velocity;
    }
  }
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, slowMotionEndsWithinTheAccelerationLimits)  // NOLINT
{
  // The URDF limits, scaled down to a crawl
  auto profile = std::make_shared<TimeOptimalTrajectoryGenerationCompositeProfile>();
  profile->max_velocity_scaling_factor = 0.001;
  CompositeInstruction program;
  program.setManipulatorInfo(manip_);
  program.push_back(joint1Move(0, 0));
  program.push_back(joint1Move(0.3, 0.1));

  tesseract::common::ProfileDictionary profiles;
  profiles.addProfile(name_, DEFAULT_PROFILE_KEY, profile);
  ASSERT_TRUE(TimeOptimalTrajectoryGeneration(name_).compute(program, *env_, profiles));
  test_suite::expectAccelerationsWithinLimits(
      program, env_->getJointGroup(manip_.manipulator)->getLimits().acceleration_limits.col(1));
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, backwardPassMeetsTheForwardPassPastItsEnd)  // NOLINT
{
  // The forward pass stops just short of the limit curve, and the backward pass from the next switching point meets
  // its line in that gap: past the forward pass's last step, and here past the backward pass's own start too
  const Path corner({ Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0), Eigen::Vector2d(1, 1) }, 0.1);
  const Trajectory corner_trajectory(corner, Eigen::Vector2d::Constant(0.4913476), Eigen::Vector2d::Ones());
  ASSERT_TRUE(corner_trajectory.isValid());
  EXPECT_LT(worstAccelerationInSteps(corner, corner_trajectory, Eigen::Vector2d::Ones(), 0.001), 1.05);

  const std::vector<Eigen::VectorXd> waypoints{
    (Eigen::VectorXd(6) << -1.4081554911517529,
     -1.8043029710580933,
     0.91977775831180519,
     0.79130680885445914,
     0.56739349488577195,
     -1.0513272140246777)
        .finished(),
    (Eigen::VectorXd(6) << -1.4078990700666134,
     -1.8052591319171054,
     0.92069733244027718,
     0.79195461539826928,
     0.56813864550675219,
     -1.0516678791310725)
        .finished(),
    (Eigen::VectorXd(6) << -1.4079810688644787,
     -1.8051215829482243,
     0.91976933280897455,
     0.79250200168584339,
     0.56792268237523069,
     -1.0502433208001487)
        .finished(),
    (Eigen::VectorXd(6) << -1.4078162224064195,
     -1.8048041826540628,
     0.9199442547658897,
     0.79286677339688838,
     0.56704483385039872,
     -1.0501270261026985)
        .finished(),
    (Eigen::VectorXd(6) << -1.4077181133417265,
     -1.8035636381569178,
     0.91873093440957909,
     0.79427089068349899,
     0.56556687297402819,
     -1.0494742221189644)
        .finished(),
    (Eigen::VectorXd(6) << -1.4077872127459414,
     -1.8043505779996694,
     0.91777000492435412,
     0.79460580649355661,
     0.56479976908039464,
     -1.0494285692615297)
        .finished(),
    (Eigen::VectorXd(6) << -1.4092114720337137,
     -1.8032311412075346,
     0.91674293269325824,
     0.79536026145733929,
     0.56612809090020011,
     -1.0501839333563214)
        .finished(),
    (Eigen::VectorXd(6) << -1.4102208843809108,
     -1.8017712521327274,
     0.91637376953969163,
     0.79584596158853171,
     0.56738154846407374,
     -1.0512447134306266)
        .finished(),
    (Eigen::VectorXd(6) << -1.4110741375998486,
     -1.8019143725218374,
     0.91545801999896137,
     0.79654509082884473,
     0.56593025861744239,
     -1.0506958573024314)
        .finished(),
    (Eigen::VectorXd(6) << -1.4100624297029938,
     -1.8032753213112978,
     0.91476026739528493,
     0.79745594889596771,
     0.56511518737768185,
     -1.051821035215176)
        .finished(),
    (Eigen::VectorXd(6) << -1.4110604978080736,
     -1.8045544481358804,
     0.91507785375039907,
     0.79844194297576243,
     0.56650703545317704,
     -1.0527401622652839)
        .finished(),
    (Eigen::VectorXd(6) << -1.4101478071303422,
     -1.8034857222221989,
     0.91553566879852244,
     0.79988422322767483,
     0.56532780435690577,
     -1.0528973816009179)
        .finished(),
  };
  const Eigen::VectorXd max_velocity = (Eigen::VectorXd(6) << 1.3551854663497644,
                                        0.89581542174147022,
                                        1.8133583493077112,
                                        0.62636411627240562,
                                        0.64084573254981514,
                                        1.2094044284346199)
                                           .finished();
  const Eigen::VectorXd max_acceleration = (Eigen::VectorXd(6) << 0.64306331237214454,
                                            0.84418233362658501,
                                            1.236686502320351,
                                            0.65925237480723775,
                                            1.532753762121885,
                                            1.3764998609414565)
                                               .finished();
  const Path path(std::list<Eigen::VectorXd>(waypoints.begin(), waypoints.end()), 0.1);
  const Trajectory trajectory(path, max_velocity, max_acceleration);
  ASSERT_TRUE(trajectory.isValid());

  // The Euler integration overshoots by up to 7% where this path's deceleration limit changes fast before a segment end
  EXPECT_LT(worstAccelerationInSteps(path, trajectory, max_acceleration, 0.001), 1.1);
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, reportedAccelerationsFollowTheProfile)  // NOLINT
{
  // Dense random waypoints; sampled every millisecond, many samples land within rounding of an integration step's start
  const std::vector<Eigen::VectorXd> waypoints{
    (Eigen::VectorXd(6) << 1.1527481255201792,
     2.4778545783982735,
     0.39805022606117779,
     0.30752038635737333,
     -1.2468101667282403,
     0.30785189430654714)
        .finished(),
    (Eigen::VectorXd(6) << 1.1464036611492012,
     2.4909805068210518,
     0.41097238462529373,
     0.30924966160534401,
     -1.2581055239522116,
     0.3001943293424339)
        .finished(),
    (Eigen::VectorXd(6) << 1.1367663646321557,
     2.4739886282915426,
     0.42008499996441989,
     0.31984134436365175,
     -1.2744913717179533,
     0.28417343570792553)
        .finished(),
    (Eigen::VectorXd(6) << 1.124437429231131,
     2.4605596305985098,
     0.42602274226660991,
     0.33759919631686841,
     -1.2570365212201005,
     0.30394649940247054)
        .finished(),
    (Eigen::VectorXd(6) << 1.1052077531021138,
     2.4782218830875906,
     0.43031578658094083,
     0.34390091022355207,
     -1.2654336237048578,
     0.28807540332793091)
        .finished(),
    (Eigen::VectorXd(6) << 1.0963362296304924,
     2.4750262390834967,
     0.42174875766997955,
     0.34325305855541954,
     -1.2797754253280167,
     0.28088909715921612)
        .finished(),
    (Eigen::VectorXd(6) << 1.0954643803658908,
     2.4726033213113423,
     0.42577793650601276,
     0.33717512675686767,
     -1.2790226923964845,
     0.2646151257032579)
        .finished(),
    (Eigen::VectorXd(6) << 1.0997053306623745,
     2.4728585182303013,
     0.43212708979127867,
     0.32348833490549106,
     -1.2646388057955611,
     0.27660279535650384)
        .finished(),
    (Eigen::VectorXd(6) << 1.1011512108892363,
     2.4610899896317036,
     0.44847706169685647,
     0.32012336723525353,
     -1.2540057829150661,
     0.25786654794921854)
        .finished(),
    (Eigen::VectorXd(6) << 1.0828371415913773,
     2.4649827948660641,
     0.45600193514513571,
     0.31204750228887757,
     -1.2500617418293454,
     0.27237147889151458)
        .finished(),
    (Eigen::VectorXd(6) << 1.0704178445375758,
     2.464737883383437,
     0.43770942350636166,
     0.31879310559672758,
     -1.247759098197426,
     0.28010624561612885)
        .finished(),
  };
  const Eigen::VectorXd max_velocity = (Eigen::VectorXd(6) << 1.1555103964761071,
                                        1.1930992499937916,
                                        1.1613936958961411,
                                        1.0114525775956422,
                                        1.9610162098264725,
                                        1.4527014311498441)
                                           .finished();
  const Eigen::VectorXd max_acceleration = (Eigen::VectorXd(6) << 1.1238693172413976,
                                            1.803423252441716,
                                            1.1706228638524405,
                                            0.51805638339175875,
                                            1.916070845494902,
                                            1.9068486895287216)
                                               .finished();

  const Path path(std::list<Eigen::VectorXd>(waypoints.begin(), waypoints.end()), 0.1);
  const Trajectory trajectory(path, max_velocity, max_acceleration);
  ASSERT_TRUE(trajectory.isValid());

  double worst{ 0 };
  for (int i = 0; i * 1e-3 <= trajectory.getDuration(); ++i)
  {
    const PathData data = trajectory.getPathData(i * 1e-3);
    worst = std::max(worst, trajectory.getAcceleration(data).cwiseAbs().cwiseQuotient(max_acceleration).maxCoeff());
  }
  EXPECT_LT(worst, 1.05);

  // The motion starts at rest, accelerating
  const PathData start = trajectory.getPathData(0.0);
  EXPECT_TRUE(trajectory.getVelocity(start).isZero(0.0));
  const double h = 1e-6;
  const Eigen::VectorXd difference = trajectory.getVelocity(trajectory.getPathData(h)) / h;
  EXPECT_TRUE(trajectory.getAcceleration(start).isApprox(difference, 1e-4))
      << trajectory.getAcceleration(start).transpose() << " vs " << difference.transpose();
}

namespace
{
CompositeInstruction programOf(const std::vector<Eigen::VectorXd>& positions)
{
  CompositeInstruction program;
  for (const auto& q : positions)
    program.push_back(
        MoveInstruction(StateWaypoint({ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" }, q),
                        MoveInstructionType::FREESPACE));
  return program;
}

Eigen::VectorXd joints(double q1, double q2 = 0, double q3 = 0)
{
  return (Eigen::VectorXd(6) << q1, q2, q3, 0, 0, 0).finished();
}

const StateWaypointPoly& waypointAt(const CompositeInstruction& program, std::size_t i)
{
  return program.flatten(moveFilter)[i].get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
}

/** @brief Time positions through compute, expecting success, increasing times and motion within the limits */
CompositeInstruction runWithinLimits(const std::vector<Eigen::VectorXd>& positions,
                                     const TimeOptimalTrajectoryGenerationCompositeProfile::Ptr& profile,
                                     const std::string& name,
                                     const tesseract::environment::Environment& env,
                                     const tesseract::common::ManipulatorInfo& manip)
{
  tesseract::common::ProfileDictionary profiles;
  profiles.addProfile(name, DEFAULT_PROFILE_KEY, profile);
  CompositeInstruction program = programOf(positions);
  program.setManipulatorInfo(manip);
  EXPECT_TRUE(TimeOptimalTrajectoryGeneration(name).compute(program, env, profiles));
  test_suite::expectTimesStrictlyIncreasing(program);
  test_suite::expectVelocitiesWithinLimits(program, Eigen::VectorXd::Ones(6));
  test_suite::expectAccelerationsWithinLimits(program, Eigen::VectorXd::Ones(6));
  return program;
}

}  // namespace

TEST_F(TimeOptimalTrajectoryGenerationUnit, reversalsAreTimedWithAStop)  // NOLINT
{
  const auto profile = createOverrideProfile();
  const auto run = [&](const std::vector<Eigen::VectorXd>& positions) {
    return runWithinLimits(positions, profile, name_, *env_, manip_);
  };
  const auto leg = [&](const Eigen::VectorXd& from, const Eigen::VectorXd& to) {
    return Trajectory(Path({ from, to }, profile->path_tolerance), Eigen::VectorXd::Ones(6), Eigen::VectorXd::Ones(6))
        .getDuration();
  };

  // Out and back, on one joint and on three: the turn is passed at rest, after the motion out
  for (const Eigen::VectorXd& out : { joints(1), joints(1, -0.5, 0.25) })
  {
    const CompositeInstruction program = run({ joints(0), out, joints(0) });
    const auto& turn = waypointAt(program, 1);
    EXPECT_TRUE(turn.getVelocity().isZero(0.0));
    expectSameBits(turn.getTime(), leg(joints(0), out));
  }

  // Several turns in a row, each after the legs before it
  const CompositeInstruction back_and_forth = run({ joints(0), joints(1), joints(0), joints(1) });
  EXPECT_TRUE(waypointAt(back_and_forth, 1).getVelocity().isZero(0.0));
  EXPECT_TRUE(waypointAt(back_and_forth, 2).getVelocity().isZero(0.0));
  expectSameBits(waypointAt(back_and_forth, 2).getTime(), leg(joints(0), joints(1)) + leg(joints(1), joints(0)));

  // Turns among collinear waypoints, which go straight on
  const CompositeInstruction mixed = run({ joints(0), joints(0.5), joints(1), joints(0.5), joints(0), joints(0.5) });
  EXPECT_GT(waypointAt(mixed, 1).getVelocity().norm(), 0.0);
  EXPECT_TRUE(waypointAt(mixed, 2).getVelocity().isZero(0.0));
  EXPECT_GT(waypointAt(mixed, 3).getVelocity().norm(), 0.0);
  EXPECT_TRUE(waypointAt(mixed, 4).getVelocity().isZero(0.0));

  // A turn next to the final run, which moves the last point onto the goal
  const CompositeInstruction near_goal = run({ joints(0), joints(1), joints(0), joints(0.0005) });
  ASSERT_EQ(near_goal.size(), 3);
  EXPECT_TRUE(waypointAt(near_goal, 1).getVelocity().isZero(0.0));

  // A goal a hair away from the start, within min_angle_change, still timed
  const CompositeInstruction hair = run({ joints(0), joints(1e-12) });
  ASSERT_EQ(hair.size(), 2);
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, sharpCornersBlendOrStop)  // NOLINT
{
  auto profile = createOverrideProfile();
  const auto run = [&](const std::vector<Eigen::VectorXd>& positions) {
    return runWithinLimits(positions, profile, name_, *env_, manip_);
  };

  // A sharp corner on long segments still blends, so the motion goes on through it
  const CompositeInstruction blended = run({ joints(0), joints(1), joints(0, 0.15) });
  EXPECT_GT(waypointAt(blended, 1).getVelocity().norm(), 0.0);

  // The same corner on segments too short for a blend the integration resolves stops instead
  const CompositeInstruction stopped = run({ joints(0), joints(0.002), joints(0, 0.0003) });
  EXPECT_TRUE(waypointAt(stopped, 1).getVelocity().isZero(0.0));

  // Just short of a reversal: a turn of pi - 0.006, blended on long segments, stopped on short ones
  const double turn = M_PI - 0.006;
  const Eigen::VectorXd back = joints(std::cos(turn), std::sin(turn));
  EXPECT_GT(waypointAt(run({ joints(0), joints(1), joints(1) + back }), 1).getVelocity().norm(), 0.0);
  EXPECT_TRUE(waypointAt(run({ joints(0), joints(0.01), joints(0.01) + (0.01 * back) }), 1).getVelocity().isZero(0.0));

  // Straight on, and nearly straight on, at deflections from 1e-6 to 1e-3 rad
  for (double deflection : { 0.0, 1e-6, 1e-5, 1e-4, 1e-3 })
    run({ joints(0), joints(0.5), joints(1.0, 0.5 * deflection) });

  // A dense path, with segments shorter than the integration's switching-point scan
  profile->min_angle_change = 1e-5;
  std::vector<Eigen::VectorXd> dense;
  for (int i = 0; i <= 600; ++i)
    dense.push_back(joints(0.3 * std::sin(i * 0.0015), 0.3 * (1.0 - std::cos(i * 0.0015)), 0.0002 * i));
  run(dense);
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, blendedCornersMoveOntoTheBlend)  // NOLINT
{
  auto profile = createOverrideProfile();
  const auto run = [&](const std::vector<Eigen::VectorXd>& positions, double path_tolerance) {
    profile->path_tolerance = path_tolerance;
    return runWithinLimits(positions, profile, name_, *env_, manip_);
  };
  const auto expectAt = [](const CompositeInstruction& program, std::size_t i, const Eigen::VectorXd& expected) {
    EXPECT_TRUE(waypointAt(program, i).getPosition().isApprox(expected, 1e-12))
        << "waypoint " << i << ": " << waypointAt(program, i).getPosition().transpose();
  };
  const auto expectUnchanged = [](const CompositeInstruction& program, std::size_t i, const Eigen::VectorXd& input) {
    expectSameBits(waypointAt(program, i).getPosition(), input);
  };

  // A right angle at tolerance 0.1: the corner moves onto the middle of its blend, 0.1 from it
  const double cut = 0.1 / std::sqrt(2.0);
  const CompositeInstruction corner = run({ joints(0), joints(1), joints(1, 1) }, 0.1);
  expectUnchanged(corner, 0, joints(0));
  expectAt(corner, 1, joints(1 - cut, cut));
  expectUnchanged(corner, 2, joints(1, 1));

  // Two corners whose blends meet, with no line between them
  const double half = 0.5 / std::sqrt(2.0);
  const CompositeInstruction meeting = run({ joints(0), joints(1), joints(1, 1), joints(2, 1) }, 0.5);
  expectAt(meeting, 1, joints(0.5 + half, 0.5 - half));
  expectAt(meeting, 2, joints(1.5 - half, 0.5 + half));

  // Waypoints the path runs straight through stay as they were, in any direction
  const std::vector<Eigen::VectorXd> line{
    joints(0), joints(0.3, 0.2, -0.1), joints(0.6, 0.4, -0.2), joints(0.9, 0.6, -0.3)
  };
  const CompositeInstruction straight = run(line, 0.1);
  for (std::size_t i = 0; i < line.size(); ++i)
    expectUnchanged(straight, i, line[i]);

  // A stop where the path turns back keeps its waypoint; the corner before it, in the same piece, moves
  const CompositeInstruction turn = run({ joints(0), joints(1), joints(1, 1), joints(1) }, 0.1);
  expectAt(turn, 1, joints(1 - cut, cut));
  expectUnchanged(turn, 2, joints(1, 1));
  expectUnchanged(turn, 3, joints(1));

  // With a repeat removed before it, the corner's own waypoint moves
  const CompositeInstruction repeat = run({ joints(0), joints(0), joints(1), joints(1, 1) }, 0.1);
  ASSERT_EQ(repeat.size(), 3);
  expectAt(repeat, 1, joints(1 - cut, cut));

  // The moved waypoint's velocity runs along the path there, at 45 degrees through the right angle
  const Eigen::VectorXd v = waypointAt(corner, 1).getVelocity();
  EXPECT_NEAR(v[0], v[1], 1e-9 * v.norm()) << v.transpose();
  EXPECT_GT(v[0], 0.0);
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, pathTellsWhichWaypointsAreBlended)  // NOLINT
{
  const std::list<Eigen::VectorXd> waypoints{
    Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0), Eigen::Vector2d(2, 0), Eigen::Vector2d(2, 1)
  };
  const Path blended(waypoints, 0.1);
  ASSERT_EQ(blended.getMapping().size(), waypoints.size());
  EXPECT_FALSE(blended.isBlended(0));
  EXPECT_FALSE(blended.isBlended(1));  // straight on
  EXPECT_TRUE(blended.isBlended(2));
  EXPECT_FALSE(blended.isBlended(3));

  const Path unblended(waypoints, 0.0);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    EXPECT_FALSE(unblended.isBlended(i)) << i;
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, coreStartsAtAStartVelocity)  // NOLINT
{
  // A unit line, with an acceleration limit of 1 and room to brake from up to sqrt(2) by its end
  const Path line({ Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0) }, 0.1);
  const Eigen::Vector2d fast(10, 10);
  const Eigen::Vector2d unit(1, 1);

  const Trajectory moving(line, fast, unit, 0.001, 1.0);
  ASSERT_TRUE(moving.isValid());
  expectSameBits(moving.getPathData(0.0).path_vel, 1.0);
  EXPECT_LT(moving.getDuration(), Trajectory(line, fast, unit).getDuration());
  for (int i = 0; i * 1e-3 <= moving.getDuration(); ++i)
  {
    const Eigen::VectorXd a = moving.getAcceleration(moving.getPathData(i * 1e-3));
    EXPECT_LE(a.cwiseAbs().maxCoeff(), 1.05) << i;
  }

  // Braking room
  EXPECT_TRUE(Trajectory(line, fast, unit, 0.001, std::sqrt(2.0) * 0.999).isValid());
  EXPECT_FALSE(Trajectory(line, fast, unit, 0.001, std::sqrt(2.0) * 1.001).isValid());

  // The velocity limit, which a start exactly at it meets, also where the limit is not exactly representable
  EXPECT_TRUE(Trajectory(line, unit, unit, 0.001, 1.0).isValid());
  const Path diagonal({ Eigen::Vector2d(0, 0), Eigen::Vector2d(0.3, 0.7) }, 0.1);
  EXPECT_TRUE(Trajectory(diagonal, unit, unit, 0.001, Eigen::Vector2d(0.3, 0.7).norm() / 0.7).isValid());
  EXPECT_FALSE(Trajectory(line, unit, unit, 0.001, 1.5).isValid());
  for (double bad : { -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity() })
    EXPECT_FALSE(Trajectory(line, fast, unit, 0.001, bad).isValid()) << bad;

  // A straight lead-in to a tight corner, which slows the path to a crawl: braking from 1.2 takes 0.72 of the 0.5 there
  // is, from 0.6 only 0.18
  const Path lead_in({ Eigen::Vector2d(0, 0), Eigen::Vector2d(0.5, 0), Eigen::Vector2d(0.5, 1) }, 0.001);
  EXPECT_FALSE(Trajectory(lead_in, fast, unit, 0.001, 1.2).isValid());
  EXPECT_TRUE(Trajectory(lead_in, fast, unit, 0.001, 0.6).isValid());
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, computeStartsAtTheFirstWaypointsVelocity)  // NOLINT
{
  auto profile = createOverrideProfile();
  tesseract::common::ProfileDictionary profiles;
  profiles.addProfile(name_, DEFAULT_PROFILE_KEY, profile);
  const TimeOptimalTrajectoryGeneration solver(name_);
  const auto program = [&](const std::vector<Eigen::VectorXd>& positions, const Eigen::VectorXd& start_velocity) {
    CompositeInstruction p = programOf(positions);
    p.setManipulatorInfo(manip_);
    p.front().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().setVelocity(start_velocity);
    return p;
  };
  const std::vector<Eigen::VectorXd> line{ joints(0), joints(1), joints(2) };
  const Eigen::VectorXd along = joints(0.5);

  // Ignored unless the profile asks for it
  CompositeInstruction ignored = program(line, along);
  CompositeInstruction at_rest = programOf(line);
  at_rest.setManipulatorInfo(manip_);
  ASSERT_TRUE(solver.compute(ignored, *env_, profiles));
  ASSERT_TRUE(solver.compute(at_rest, *env_, profiles));
  for (std::size_t i = 0; i < line.size(); ++i)
    expectSameBits(waypointAt(ignored, i).getTime(), waypointAt(at_rest, i).getTime());

  profile->use_start_velocity = true;
  CompositeInstruction moving = program(line, along);
  ASSERT_TRUE(solver.compute(moving, *env_, profiles));
  EXPECT_TRUE(waypointAt(moving, 0).getVelocity().isApprox(along, 1e-12)) << waypointAt(moving, 0).getVelocity();
  test_suite::expectTimesStrictlyIncreasing(moving);
  test_suite::expectVelocitiesWithinLimits(moving, Eigen::VectorXd::Ones(6));
  test_suite::expectAccelerationsWithinLimits(moving, Eigen::VectorXd::Ones(6));
  EXPECT_LT(waypointAt(moving, 2).getTime(), waypointAt(at_rest, 2).getTime());

  // Across a stop where the path turns back, only the motion out starts moving
  CompositeInstruction turn = program({ joints(0), joints(1), joints(0) }, along);
  ASSERT_TRUE(solver.compute(turn, *env_, profiles));
  EXPECT_TRUE(waypointAt(turn, 1).getVelocity().isZero(0.0));

  // A goal within min_angle_change of the start, or of the last point kept, sets the direction itself
  CompositeInstruction hair = program({ joints(0), joints(0.0005) }, joints(0.01));
  EXPECT_TRUE(solver.compute(hair, *env_, profiles));
  CompositeInstruction near_goal = program({ joints(0), joints(1), joints(1.0005) }, along);
  ASSERT_TRUE(solver.compute(near_goal, *env_, profiles));
  EXPECT_TRUE(waypointAt(near_goal, 0).getVelocity().isApprox(along, 1e-12));

  // A slightly tilted start velocity is taken along the path
  const double tilt = 5e-4;
  CompositeInstruction tilted = program(line, joints(0.5 * std::cos(tilt), 0.5 * std::sin(tilt)));
  ASSERT_TRUE(solver.compute(tilted, *env_, profiles));
  EXPECT_NEAR(waypointAt(tilted, 0).getVelocity()[1], 0.0, 1e-12);
  EXPECT_NEAR(waypointAt(tilted, 0).getVelocity()[0], 0.5 * std::cos(tilt), 1e-12);

  // Backwards, sideways, too fast, malformed, or with nowhere to go
  const double off = 1e-2;
  for (const Eigen::VectorXd& bad : { joints(-0.5),
                                      Eigen::VectorXd(joints(0.5 * std::cos(off), 0.5 * std::sin(off))),
                                      joints(1.5),
                                      Eigen::VectorXd(Eigen::VectorXd::Constant(5, 0.1)),
                                      Eigen::VectorXd(Eigen::VectorXd::Zero(5)),
                                      joints(std::numeric_limits<double>::quiet_NaN()) })
  {
    CompositeInstruction rejected = program(line, bad);
    EXPECT_FALSE(solver.compute(rejected, *env_, profiles)) << bad.transpose();
  }
  CompositeInstruction still = program({ joints(0), joints(0) }, along);
  EXPECT_FALSE(solver.compute(still, *env_, profiles));
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, coreEndsAtAnEndVelocity)  // NOLINT
{
  // A unit line, with an acceleration limit of 1 and room to reach up to sqrt(2) by its end
  const Path line({ Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0) }, 0.1);
  const Eigen::Vector2d fast(10, 10);
  const Eigen::Vector2d unit(1, 1);

  const Trajectory moving(line, fast, unit, 0.001, 0.0, 1.0);
  ASSERT_TRUE(moving.isValid());
  EXPECT_NEAR(moving.getPathData(moving.getDuration()).path_vel, 1.0, 1e-9);
  EXPECT_LT(moving.getDuration(), Trajectory(line, fast, unit).getDuration());
  for (int i = 0; i * 1e-3 <= moving.getDuration(); ++i)
  {
    const Eigen::VectorXd a = moving.getAcceleration(moving.getPathData(i * 1e-3));
    EXPECT_LE(a.cwiseAbs().maxCoeff(), 1.05) << i;
  }

  // Room to reach it; exactly sqrt(2) is not, as the integration's chords lie below the curve
  EXPECT_TRUE(Trajectory(line, fast, unit, 0.001, 0.0, std::sqrt(2.0) * 0.999).isValid());
  EXPECT_FALSE(Trajectory(line, fast, unit, 0.001, 0.0, std::sqrt(2.0) * 1.001).isValid());

  // The velocity limit, which an end exactly at it meets, also where the limit is not exactly representable
  EXPECT_TRUE(Trajectory(line, unit, unit, 0.001, 0.0, 1.0).isValid());
  const Path diagonal({ Eigen::Vector2d(0, 0), Eigen::Vector2d(0.3, 0.7) }, 0.1);
  EXPECT_TRUE(Trajectory(diagonal, unit, unit, 0.001, 0.0, Eigen::Vector2d(0.3, 0.7).norm() / 0.7).isValid());
  EXPECT_FALSE(Trajectory(line, unit, unit, 0.001, 0.0, 1.5).isValid());
  for (double bad : { -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity() })
    EXPECT_FALSE(Trajectory(line, fast, unit, 0.001, 0.0, bad).isValid()) << bad;

  // Ending at the cruise velocity saves the braking, half a second here; starting at it too saves the acceleration
  const Path long_line({ Eigen::Vector2d(0, 0), Eigen::Vector2d(5, 0) }, 0.1);
  const Trajectory cruising(long_line, unit, unit, 0.001, 0.0, 1.0);
  ASSERT_TRUE(cruising.isValid());
  EXPECT_NEAR(cruising.getDuration(), 5.5, 1e-6);
  EXPECT_NEAR(Trajectory(long_line, unit, unit, 0.001, 1.0, 1.0).getDuration(), 5.0, 1e-6);

  // The cruise holds to the end, at the limit and without the braking the last steps would otherwise read
  const double end = cruising.getDuration();
  for (int i = 0; i <= 100; ++i)
  {
    const PathData data = cruising.getPathData(end - (i * 1e-4));
    EXPECT_NEAR(data.path_vel, 1.0, 1e-9) << i;
  }
  EXPECT_NEAR(cruising.getAcceleration(cruising.getPathData(end)).cwiseAbs().maxCoeff(), 0.0, 1e-9);

  // Past a blend the backward pass from an end velocity out of reach must not leave the limit curve; within reach it
  // keeps the limit
  const Path corner({ Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1), Eigen::Vector2d(1, 1.3) }, 0.1);
  EXPECT_FALSE(Trajectory(corner, unit, unit, 0.001, 0.0, 1.0).isValid());
  const Trajectory reachable(corner, unit, unit, 0.001, 0.0, 0.6);
  ASSERT_TRUE(reachable.isValid());
  EXPECT_NEAR(reachable.getPathData(reachable.getDuration()).path_vel, 0.6, 1e-9);
  for (int i = 0; i * 1e-4 <= reachable.getDuration(); ++i)
  {
    const PathData data = reachable.getPathData(i * 1e-4);
    EXPECT_LE(reachable.getAcceleration(data).cwiseAbs().maxCoeff(), 1.05) << i;
    EXPECT_LE(reachable.getVelocity(data).cwiseAbs().maxCoeff(), 1.0 + 1e-6) << i;
  }
}

TEST_F(TimeOptimalTrajectoryGenerationUnit, computeEndsAtTheLastWaypointsVelocity)  // NOLINT
{
  auto profile = createOverrideProfile();
  tesseract::common::ProfileDictionary profiles;
  profiles.addProfile(name_, DEFAULT_PROFILE_KEY, profile);
  const TimeOptimalTrajectoryGeneration solver(name_);
  const auto program = [&](const std::vector<Eigen::VectorXd>& positions, const Eigen::VectorXd& end_velocity) {
    CompositeInstruction p = programOf(positions);
    p.setManipulatorInfo(manip_);
    p.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().setVelocity(end_velocity);
    return p;
  };
  const std::vector<Eigen::VectorXd> line{ joints(0), joints(1), joints(2) };
  const Eigen::VectorXd along = joints(0.5);

  // Ignored unless the profile asks for it
  CompositeInstruction ignored = program(line, along);
  CompositeInstruction at_rest = programOf(line);
  at_rest.setManipulatorInfo(manip_);
  ASSERT_TRUE(solver.compute(ignored, *env_, profiles));
  ASSERT_TRUE(solver.compute(at_rest, *env_, profiles));
  for (std::size_t i = 0; i < line.size(); ++i)
    expectSameBits(waypointAt(ignored, i).getTime(), waypointAt(at_rest, i).getTime());
  EXPECT_TRUE(waypointAt(ignored, 2).getVelocity().isZero(0.0));

  profile->use_end_velocity = true;
  CompositeInstruction moving = program(line, along);
  ASSERT_TRUE(solver.compute(moving, *env_, profiles));
  expectSameBits(waypointAt(moving, 2).getVelocity(), along);
  test_suite::expectTimesStrictlyIncreasing(moving);
  test_suite::expectVelocitiesWithinLimits(moving, Eigen::VectorXd::Ones(6));
  test_suite::expectAccelerationsWithinLimits(moving, Eigen::VectorXd::Ones(6));
  EXPECT_LT(waypointAt(moving, 2).getTime(), waypointAt(at_rest, 2).getTime());

  // A zero end velocity ends at rest, as without the flag
  CompositeInstruction zero = program(line, joints(0));
  ASSERT_TRUE(solver.compute(zero, *env_, profiles));
  for (std::size_t i = 0; i < line.size(); ++i)
    expectSameBits(waypointAt(zero, i).getTime(), waypointAt(at_rest, i).getTime());

  // A slightly tilted end velocity is taken along the path
  const double tilt = 5e-4;
  CompositeInstruction tilted = program(line, joints(0.5 * std::cos(tilt), 0.5 * std::sin(tilt)));
  ASSERT_TRUE(solver.compute(tilted, *env_, profiles));
  EXPECT_NEAR(waypointAt(tilted, 2).getVelocity()[1], 0.0, 1e-12);
  EXPECT_NEAR(waypointAt(tilted, 2).getVelocity()[0], 0.5 * std::cos(tilt), 1e-12);

  // Across a stop where the path turns back, only the motion back ends moving
  CompositeInstruction turn = program({ joints(0), joints(1), joints(0) }, joints(-0.5));
  ASSERT_TRUE(solver.compute(turn, *env_, profiles));
  EXPECT_TRUE(waypointAt(turn, 1).getVelocity().isZero(0.0));
  EXPECT_TRUE(waypointAt(turn, 2).getVelocity().isApprox(joints(-0.5), 1e-12));

  // Starting and ending moving at once
  profile->use_start_velocity = true;
  CompositeInstruction both = program(line, along);
  both.front().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().setVelocity(along);
  ASSERT_TRUE(solver.compute(both, *env_, profiles));
  EXPECT_TRUE(waypointAt(both, 0).getVelocity().isApprox(along, 1e-12));
  EXPECT_TRUE(waypointAt(both, 2).getVelocity().isApprox(along, 1e-12));
  profile->use_start_velocity = false;

  // When I/O keeps the goal's run from moving its last point onto the goal, the goal's velocity still ends the motion
  CompositeInstruction blocked = programOf({ joints(0), joints(1) });
  blocked.setManipulatorInfo(manip_);
  blocked.push_back(SetDigitalInstruction("io", 0, true));
  blocked.push_back(MoveInstruction(
      StateWaypoint({ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" }, joints(1.0005)),
      MoveInstructionType::FREESPACE));
  blocked.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().setVelocity(along);
  ASSERT_TRUE(solver.compute(blocked, *env_, profiles));
  ASSERT_EQ(blocked.flatten(moveFilter).size(), 2);
  EXPECT_TRUE(waypointAt(blocked, 1).getVelocity().isApprox(along, 1e-12)) << waypointAt(blocked, 1).getVelocity();

  // Out of reach: at the limit right after a corner, or after a short last piece past a stop
  CompositeInstruction corner = program({ joints(0, 0), joints(1, 1), joints(1, 1.3) }, joints(0, 1));
  EXPECT_FALSE(solver.compute(corner, *env_, profiles));
  CompositeInstruction short_return = program({ joints(0), joints(1), joints(0.9) }, joints(-1));
  EXPECT_FALSE(solver.compute(short_return, *env_, profiles));

  // Backwards, sideways, too fast, malformed, or with nowhere to go
  const double off = 1e-2;
  for (const Eigen::VectorXd& bad : { joints(-0.5),
                                      Eigen::VectorXd(joints(0.5 * std::cos(off), 0.5 * std::sin(off))),
                                      joints(1.5),
                                      Eigen::VectorXd(Eigen::VectorXd::Constant(5, 0.1)),
                                      Eigen::VectorXd(Eigen::VectorXd::Zero(5)),
                                      joints(std::numeric_limits<double>::quiet_NaN()) })
  {
    CompositeInstruction rejected = program(line, bad);
    EXPECT_FALSE(solver.compute(rejected, *env_, profiles)) << bad.transpose();
  }
  CompositeInstruction still = program({ joints(0), joints(0) }, along);
  EXPECT_FALSE(solver.compute(still, *env_, profiles));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
