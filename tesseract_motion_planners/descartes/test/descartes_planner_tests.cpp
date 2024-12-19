/**
 * @file descartes_planner_tests.cpp
 * @brief This contains unit test for the tesseract descartes planner
 *
 * @author Levi Armstrong
 * @date September 16, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <tesseract_kinematics/core/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

#include <tesseract_kinematics/core/kinematic_group.h>

#include <tesseract_environment/environment.h>

#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_ladder_graph_solver_profile.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_common/resource_locator.h>

using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_planning;
using namespace tesseract_kinematics;
using namespace descartes_light;

static const bool DEBUG = false;
static const std::string DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask";

class TesseractPlanningDescartesUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;
  tesseract_common::ManipulatorInfo manip;

  void SetUp() override
  {
    auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    Environment::Ptr env = std::make_shared<Environment>();
    tesseract_common::fs::path urdf_path(
        locator->locateResource("package://tesseract_support/urdf/abb_irb2400.urdf")->getFilePath());
    tesseract_common::fs::path srdf_path(
        locator->locateResource("package://tesseract_support/urdf/abb_irb2400.srdf")->getFilePath());
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator));
    env_ = env;

    manip.tcp_frame = "tool0";
    manip.manipulator = "manipulator";
    manip.manipulator_ik_solver = "OPWInvKin";
    manip.working_frame = "base_link";
  }
};

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerFixedPoses)  // NOLINT
{
  // Specify a start waypoint
  CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.20, 0.8) *
                                               Eigen::Quaterniond(0, 0, -1.0, 0)) };

  // Specify a end waypoint
  CartesianWaypointPoly wp2{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .20, 0.8) *
                                               Eigen::Quaterniond(0, 0, -1.0, 0)) };

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip);

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip);

  // Create a program
  CompositeInstruction program;
  program.setManipulatorInfo(manip);
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program = generateInterpolatedProgram(program, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto solver_profile = std::make_shared<DescartesLadderGraphSolverProfileD>();
  solver_profile->num_threads = 1;

  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile(DESCARTES_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);
  profiles->addProfile(DESCARTES_DEFAULT_NAMESPACE, "TEST_PROFILE", solver_profile);

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner(DESCARTES_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.profiles = profiles;

  PlannerResponse single_planner_response = single_descartes_planner.solve(request);
  EXPECT_TRUE(&single_planner_response);

  CompositeInstruction official_results = single_planner_response.results;

  for (int i = 0; i < 10; ++i)
  {
    DescartesMotionPlannerD descartes_planner(DESCARTES_DEFAULT_NAMESPACE);
    solver_profile->num_threads = 4;

    PlannerResponse planner_response = descartes_planner.solve(request);

    if (DEBUG)
    {
      std::cout << "Request Instructions:" << std::endl;
      request.instructions.print();
      std::cout << "Single Planner Response Results:" << std::endl;
      single_planner_response.results.print();
      std::cout << "Threaded Planner Response Results:" << std::endl;
      planner_response.results.print();
    }

    EXPECT_TRUE(&planner_response);
    EXPECT_EQ(official_results.size(), planner_response.results.size());
    for (CompositeInstruction::size_type j = 0; j < official_results.size(); ++j)
    {
      if (official_results[j].isCompositeInstruction())  //
      {
        const auto& sub_official = official_results[j].as<CompositeInstruction>();
        const auto& sub = interpolated_program[j].as<CompositeInstruction>();
        for (std::size_t k = 0; k < sub.size(); ++k)
        {
          if (sub_official[k].isCompositeInstruction())
          {
            EXPECT_TRUE(false);
          }
          else if (sub_official[k].isMoveInstruction())
          {
            const auto& mv_official = sub_official[k].as<MoveInstructionPoly>();
            const auto& mv = sub[k].as<MoveInstructionPoly>();
            EXPECT_TRUE(getJointPosition(mv_official.getWaypoint()).isApprox(getJointPosition(mv.getWaypoint()), 1e-5));
          }
        }
      }
      else if (official_results[j].isMoveInstruction())
      {
        const auto& mv_official = official_results[j].as<MoveInstructionPoly>();
        const auto& mv = planner_response.results[j].as<MoveInstructionPoly>();
        EXPECT_TRUE(getJointPosition(mv_official.getWaypoint()).isApprox(getJointPosition(mv.getWaypoint()), 1e-5));
      }
    }
  }
}

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerAxialSymetric)  // NOLINT
{
  // Specify a start waypoint
  CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.20, 0.8) *
                                               Eigen::Quaterniond(0, 0, -1.0, 0)) };

  // Specify a end waypoint
  CartesianWaypointPoly wp2{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .20, 0.8) *
                                               Eigen::Quaterniond(0, 0, -1.0, 0)) };

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip);

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip);

  // Create a program
  CompositeInstruction program;
  program.setManipulatorInfo(manip);
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program = generateInterpolatedProgram(program, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto solver_profile = std::make_shared<DescartesLadderGraphSolverProfileD>();
  solver_profile->num_threads = 1;

  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();

  // Make this a tool z-axis free sampler
  plan_profile->target_pose_fixed = false;
  plan_profile->target_pose_sample_axis = Eigen::Vector3d(0, 0, 1);
  plan_profile->target_pose_sample_resolution = M_PI_4;

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile(DESCARTES_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);
  profiles->addProfile(DESCARTES_DEFAULT_NAMESPACE, "TEST_PROFILE", solver_profile);

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner(DESCARTES_DEFAULT_NAMESPACE);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.profiles = profiles;

  PlannerResponse single_planner_response = single_descartes_planner.solve(request);
  EXPECT_TRUE(&single_planner_response);

  CompositeInstruction official_results = single_planner_response.results;

  for (int i = 0; i < 10; ++i)
  {
    DescartesMotionPlannerD descartes_planner(DESCARTES_DEFAULT_NAMESPACE);
    solver_profile->num_threads = 4;

    PlannerResponse planner_response = descartes_planner.solve(request);
    EXPECT_TRUE(&planner_response);
    EXPECT_TRUE(official_results.size() == planner_response.results.size());
    for (CompositeInstruction::size_type j = 0; j < official_results.size(); ++j)
    {
      if (official_results[j].isCompositeInstruction())
      {
        const auto& sub_official = official_results[j].as<CompositeInstruction>();
        const auto& sub = planner_response.results[j].as<CompositeInstruction>();
        for (std::size_t k = 0; k < sub.size(); ++k)
        {
          if (sub_official[k].isCompositeInstruction())
          {
            EXPECT_TRUE(false);
          }
          else if (sub_official[k].isMoveInstruction())
          {
            const auto& mv_official = sub_official[k].as<MoveInstructionPoly>();
            const auto& mv = sub[k].as<MoveInstructionPoly>();
            EXPECT_TRUE(getJointPosition(mv_official.getWaypoint()).isApprox(getJointPosition(mv.getWaypoint()), 1e-5));
          }
        }
      }
      else if (official_results[j].isMoveInstruction())
      {
        const auto& mv_official = official_results[j].as<MoveInstructionPoly>();
        const auto& mv = planner_response.results[j].as<MoveInstructionPoly>();
        EXPECT_TRUE(getJointPosition(mv_official.getWaypoint()).isApprox(getJointPosition(mv.getWaypoint()), 1e-5));
      }
    }
  }
}

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerCollisionEdgeEvaluator)  // NOLINT
{
  // Specify a start waypoint
  CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.10, 0.8) *
                                               Eigen::Quaterniond(0, 0, -1.0, 0)) };

  // Specify a end waypoint
  CartesianWaypointPoly wp2{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .10, 0.8) *
                                               Eigen::Quaterniond(0, 0, -1.0, 0)) };

  // Define Start Instruction
  MoveInstruction start_instruction(wp1, MoveInstructionType::LINEAR, "TEST_PROFILE", manip);

  // Define Plan Instructions
  MoveInstruction plan_f1(wp2, MoveInstructionType::LINEAR, "TEST_PROFILE", manip);

  // Create a program
  CompositeInstruction program;
  program.setManipulatorInfo(manip);
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f1);

  // Create a seed
  CompositeInstruction interpolated_program = generateInterpolatedProgram(program, env_, 3.14, 1.0, 3.14, 2);

  // Create Profiles
  auto solver_profile = std::make_shared<DescartesLadderGraphSolverProfileD>();
  solver_profile->num_threads = 1;

  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();

  // Make this a tool z-axis free sampler
  plan_profile->target_pose_fixed = false;
  plan_profile->target_pose_sample_axis = Eigen::Vector3d(0, 0, 1);
  plan_profile->target_pose_sample_resolution = 60 * M_PI * 180.0;

  // Add collision edge evaluator
  plan_profile->enable_edge_collision = true;

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile(DESCARTES_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.profiles = profiles;

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner(DESCARTES_DEFAULT_NAMESPACE);

  PlannerResponse single_planner_response = single_descartes_planner.solve(request);
  EXPECT_TRUE(&single_planner_response);

  CompositeInstruction official_results = single_planner_response.results;

  for (int i = 0; i < 10; ++i)
  {
    DescartesMotionPlannerD descartes_planner(DESCARTES_DEFAULT_NAMESPACE);
    solver_profile->num_threads = 4;

    PlannerResponse planner_response = descartes_planner.solve(request);
    EXPECT_TRUE(&planner_response);
    EXPECT_TRUE(official_results.size() == planner_response.results.size());
    for (CompositeInstruction::size_type j = 0; j < official_results.size(); ++j)
    {
      if (official_results[j].isCompositeInstruction())
      {
        const auto& sub_official = official_results[j].as<CompositeInstruction>();
        const auto& sub = planner_response.results[j].as<CompositeInstruction>();
        for (std::size_t k = 0; k < sub.size(); ++k)
        {
          if (sub_official[k].isCompositeInstruction())
          {
            EXPECT_TRUE(false);
          }
          else if (sub_official[k].isMoveInstruction())
          {
            const auto& mv_official = sub_official[k].as<MoveInstructionPoly>();
            const auto& mv = sub[k].as<MoveInstructionPoly>();
            EXPECT_TRUE(getJointPosition(mv_official.getWaypoint()).isApprox(getJointPosition(mv.getWaypoint()), 1e-5));
          }
        }
      }
      else if (official_results[j].isMoveInstruction())
      {
        const auto& mv_official = official_results[j].as<MoveInstructionPoly>();
        const auto& mv = planner_response.results[j].as<MoveInstructionPoly>();
        EXPECT_TRUE(getJointPosition(mv_official.getWaypoint()).isApprox(getJointPosition(mv.getWaypoint()), 1e-5));
      }
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
