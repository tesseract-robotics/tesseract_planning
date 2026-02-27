/**
 * @file chain_example.cpp
 * @brief Chained motion planning example
 *
 * @author Levi Armstrong
 * @date August 31, 2020
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
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/types.h>

#include <tesseract/kinematics/utils.h>

#include <tesseract/environment/environment.h>

#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_move_profile.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/simple/interpolation.h>

#include <tesseract/visualization/visualization_loader.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract/common/resource_locator.h>

using namespace tesseract::motion_planners;

const static std::string DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask";
const static std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";

int main(int /*argc*/, char** /*argv*/)
{
  try
  {
    // Setup
    auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    auto env = std::make_shared<tesseract::environment::Environment>();
    std::filesystem::path urdf_path(
        locator->locateResource("package://tesseract/support/urdf/abb_irb2400.urdf")->getFilePath());
    std::filesystem::path srdf_path(
        locator->locateResource("package://tesseract/support/urdf/abb_irb2400.srdf")->getFilePath());
    env->init(urdf_path, srdf_path, locator);

    // Dynamically load ignition visualizer if exist
    tesseract::visualization::VisualizationLoader loader;
    auto plotter = loader.get();

    if (plotter != nullptr)
    {
      plotter->waitForConnection();
      plotter->plotEnvironment(*env);
    }

    tesseract::common::ManipulatorInfo manip;
    manip.tcp_frame = "tool0";
    manip.working_frame = "base_link";
    manip.manipulator = "manipulator";
    manip.manipulator_ik_solver = "OPWInvKin";

    auto state_solver = env->getStateSolver();
    auto kin_group = env->getKinematicGroup(manip.manipulator, manip.manipulator_ik_solver);
    auto cur_state = env->getState();

    // Specify start location
    JointWaypoint wp0{ kin_group->getJointNames(), Eigen::VectorXd::Zero(6) };

    // Specify raster 1 start waypoint and end waypoint
    CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.20, 0.8) *
                           Eigen::Quaterniond(0, 0, -1.0, 0) };
    CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, .20, 0.8) *
                           Eigen::Quaterniond(0, 0, -1.0, 0) };

    // Specify raster 2 start waypoint and end waypoint
    CartesianWaypoint wp3{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.9, -.20, 0.8) *
                           Eigen::Quaterniond(0, 0, -1.0, 0) };
    CartesianWaypoint wp4{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.9, .20, 0.8) *
                           Eigen::Quaterniond(0, 0, -1.0, 0) };

    // Specify raster 4 start waypoint and end waypoint
    CartesianWaypoint wp5{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(1.0, -.20, 0.8) *
                           Eigen::Quaterniond(0, 0, -1.0, 0) };
    CartesianWaypoint wp6{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(1.0, .20, 0.8) *
                           Eigen::Quaterniond(0, 0, -1.0, 0) };

    // Define Plan Instructions
    MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "DEFAULT");
    MoveInstruction plan_f1(wp1, MoveInstructionType::FREESPACE, "DEFAULT");
    MoveInstruction plan_c1(wp2, MoveInstructionType::LINEAR, "DEFAULT");
    MoveInstruction plan_c2(wp3, MoveInstructionType::LINEAR, "DEFAULT");
    MoveInstruction plan_c3(wp4, MoveInstructionType::LINEAR, "DEFAULT");
    MoveInstruction plan_c4(wp5, MoveInstructionType::LINEAR, "DEFAULT");
    MoveInstruction plan_c5(wp6, MoveInstructionType::LINEAR, "DEFAULT");
    MoveInstruction plan_f3(wp0, MoveInstructionType::FREESPACE, "DEFAULT");

    // Create program
    CompositeInstruction program;
    program.setManipulatorInfo(manip);
    program.push_back(start_instruction);
    program.push_back(plan_f1);
    program.push_back(plan_c1);
    program.push_back(plan_c2);
    program.push_back(plan_c3);
    program.push_back(plan_c4);
    program.push_back(plan_c5);
    program.push_back(plan_f3);

    // Plot Program
    if (plotter)
    {
    }

    // Create Profiles
    auto descartes_move_profile = std::make_shared<DescartesDefaultMoveProfileD>();
    auto trajopt_move_profile = std::make_shared<TrajOptDefaultMoveProfile>();
    auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
    auto trajopt_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();

    // Create a interpolated program
    CompositeInstruction interpolated_program = generateInterpolatedProgram(program, cur_state, env);

    // Profile Dictionary
    auto profiles = std::make_shared<tesseract::common::ProfileDictionary>();
    profiles->addProfile(DESCARTES_DEFAULT_NAMESPACE, "DEFAULT", descartes_move_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_move_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_solver_profile);

    // Create Planning Request
    PlannerRequest request;
    request.instructions = interpolated_program;
    request.env = env;
    request.env_state = cur_state;
    request.profiles = profiles;

    // Solve Descartes Plan
    DescartesMotionPlannerD descartes_planner(DESCARTES_DEFAULT_NAMESPACE);
    PlannerResponse descartes_response = descartes_planner.solve(request);
    assert(descartes_response);

    // Plot Descartes Trajectory
    if (plotter)
    {
      plotter->waitForInput();
      plotter->plotTrajectory(toJointTrajectory(descartes_response.results), *state_solver);
    }

    // Update program
    request.instructions = descartes_response.results;

    // Solve TrajOpt Plan
    TrajOptMotionPlanner trajopt_planner(TRAJOPT_DEFAULT_NAMESPACE);
    PlannerResponse trajopt_response = trajopt_planner.solve(request);
    assert(trajopt_response);

    if (plotter)
    {
      plotter->waitForInput();
      plotter->plotTrajectory(toJointTrajectory(trajopt_response.results), *state_solver);
    }
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Example failed with message: %s", e.what());
    return -1;
  }
}
