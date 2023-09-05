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

#include <tesseract_environment/environment.h>

#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/serialize.h>
#include <tesseract_motion_planners/descartes/deserialize.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

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
    auto locator = std::make_shared<tesseract_common::TesseractSupportResourceLocator>();
    Environment::Ptr env = std::make_shared<Environment>();
    tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
    tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
    EXPECT_TRUE(env->init(urdf_path, srdf_path, locator));
    env_ = env;

    manip.tcp_frame = "tool0";
    manip.manipulator = "manipulator";
    manip.manipulator_ik_solver = "OPWInvKin";
    manip.working_frame = "base_link";
  }
};

TEST(TesseractPlanningDescartesSerializeUnit, SerializeDescartesDefaultPlanToXml)  // NOLINT
{
  // Write program to file
  DescartesDefaultPlanProfile<double> plan_profile;
  plan_profile.enable_edge_collision = true;

  EXPECT_TRUE(toXMLFile(plan_profile, tesseract_common::getTempPath() + "descartes_default_plan_example_input.xml"));

  // Import file
  DescartesDefaultPlanProfile<double> imported_plan_profile =
      descartesPlanFromXMLFile(tesseract_common::getTempPath() + "descartes_default_plan_example_input.xml");

  // Re-write file and compare changed from default term
  EXPECT_TRUE(
      toXMLFile(imported_plan_profile, tesseract_common::getTempPath() + "descartes_default_plan_example_input2.xml"));
  EXPECT_TRUE(plan_profile.enable_edge_collision == imported_plan_profile.enable_edge_collision);
}

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerFixedPoses)  // NOLINT
{
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  tesseract_kinematics::KinematicGroup::Ptr kin_group =
      env_->getKinematicGroup(manip.manipulator, manip.manipulator_ik_solver);
  auto cur_state = env_->getState();

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
  CompositeInstruction interpolated_program =
      generateInterpolatedProgram(program, cur_state, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<DescartesPlanProfile<double>>(DESCARTES_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner(DESCARTES_DEFAULT_NAMESPACE);
  plan_profile->num_threads = 1;

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.env_state = cur_state;
  request.profiles = profiles;

  PlannerResponse single_planner_response = single_descartes_planner.solve(request);
  EXPECT_TRUE(&single_planner_response);

  CompositeInstruction official_results = single_planner_response.results;

  for (int i = 0; i < 10; ++i)
  {
    // Test the problem generator
    {
      auto problem = single_descartes_planner.createProblem(request);
      EXPECT_EQ(problem->samplers.size(), 11);
      EXPECT_EQ(problem->edge_evaluators.size(), 10);
    }

    DescartesMotionPlannerD descartes_planner(DESCARTES_DEFAULT_NAMESPACE);
    plan_profile->num_threads = 4;

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
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  tesseract_kinematics::KinematicGroup::Ptr kin_group =
      env_->getKinematicGroup(manip.manipulator, manip.manipulator_ik_solver);
  auto cur_state = env_->getState();

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
  CompositeInstruction interpolated_program =
      generateInterpolatedProgram(program, cur_state, env_, 3.14, 1.0, 3.14, 10);

  // Create Profiles
  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();
  // Make this a tool z-axis free sampler
  plan_profile->target_pose_sampler = [](const Eigen::Isometry3d& tool_pose) {
    return tesseract_planning::sampleToolAxis(tool_pose, M_PI_4, Eigen::Vector3d(0, 0, 1));
  };

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<DescartesPlanProfile<double>>(DESCARTES_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner(DESCARTES_DEFAULT_NAMESPACE);
  plan_profile->num_threads = 1;

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.env_state = cur_state;
  request.profiles = profiles;

  auto problem = single_descartes_planner.createProblem(request);
  problem->num_threads = 1;
  EXPECT_EQ(problem->samplers.size(), 11);
  EXPECT_EQ(problem->edge_evaluators.size(), 10);

  PlannerResponse single_planner_response = single_descartes_planner.solve(request);
  EXPECT_TRUE(&single_planner_response);

  CompositeInstruction official_results = single_planner_response.results;

  for (int i = 0; i < 10; ++i)
  {
    DescartesMotionPlannerD descartes_planner(DESCARTES_DEFAULT_NAMESPACE);
    plan_profile->num_threads = 4;

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
  // Create the planner and the responses that will store the results
  PlannerResponse planning_response;

  tesseract_kinematics::KinematicGroup::Ptr kin_group =
      env_->getKinematicGroup(manip.manipulator, manip.manipulator_ik_solver);
  auto cur_state = env_->getState();

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
  CompositeInstruction interpolated_program = generateInterpolatedProgram(program, cur_state, env_, 3.14, 1.0, 3.14, 2);

  // Create Profiles
  auto plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();
  // Make this a tool z-axis free sampler
  plan_profile->target_pose_sampler = [](const Eigen::Isometry3d& tool_pose) {
    return tesseract_planning::sampleToolAxis(tool_pose, 60 * M_PI * 180.0, Eigen::Vector3d(0, 0, 1));
  };
  plan_profile->enable_edge_collision = true;  // Add collision edge evaluator

  // Profile Dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<DescartesPlanProfile<double>>(DESCARTES_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile);

  // Create Planning Request
  PlannerRequest request;
  request.instructions = interpolated_program;
  request.env = env_;
  request.env_state = cur_state;
  request.profiles = profiles;

  // Create Planner
  DescartesMotionPlannerD single_descartes_planner(DESCARTES_DEFAULT_NAMESPACE);
  plan_profile->num_threads = 1;

  // Test Problem size - TODO: Make dedicated unit test for DefaultDescartesProblemGenerator
  auto problem = single_descartes_planner.createProblem(request);
  EXPECT_EQ(problem->samplers.size(), 3);
  EXPECT_EQ(problem->edge_evaluators.size(), 2);
  EXPECT_EQ(problem->num_threads, 1);

  PlannerResponse single_planner_response = single_descartes_planner.solve(request);
  EXPECT_TRUE(&single_planner_response);

  CompositeInstruction official_results = single_planner_response.results;

  for (int i = 0; i < 10; ++i)
  {
    DescartesMotionPlannerD descartes_planner(DESCARTES_DEFAULT_NAMESPACE);
    plan_profile->num_threads = 4;

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
