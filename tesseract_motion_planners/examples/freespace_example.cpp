/**
 * @file freespace_example.cpp
 * @brief Freespace motion planning example
 *
 * @author Levi Armstrong
 * @date August 31, 2020
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
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

#include <tesseract_kinematics/core/utils.h>

#include <tesseract_environment/environment.h>

#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_move_profile.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/simple/interpolation.h>

#include <tesseract_visualization/visualization_loader.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_common/resource_locator.h>

using namespace tesseract_planning;

const static std::string OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask";
const static std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";

int main(int /*argc*/, char** /*argv*/)
{
  try
  {
    // Setup
    auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    auto env = std::make_shared<tesseract_environment::Environment>();
    std::filesystem::path urdf_path(
        locator->locateResource("package://tesseract_support/urdf/abb_irb2400.urdf")->getFilePath());
    std::filesystem::path srdf_path(
        locator->locateResource("package://tesseract_support/urdf/abb_irb2400.srdf")->getFilePath());
    env->init(urdf_path, srdf_path, locator);

    // Dynamically load ignition visualizer if exist
    tesseract_visualization::VisualizationLoader loader;
    auto plotter = loader.get();

    if (plotter != nullptr)
    {
      plotter->waitForConnection();
      plotter->plotEnvironment(*env);
    }

    tesseract_common::ManipulatorInfo manip;
    manip.tcp_frame = "tool0";
    manip.working_frame = "base_link";
    manip.manipulator = "manipulator";
    manip.manipulator_ik_solver = "OPWInvKin";

    auto state_solver = env->getStateSolver();
    auto kin_group = env->getKinematicGroup(manip.manipulator, manip.manipulator_ik_solver);
    auto cur_state = env->getState();

    // Specify start location
    StateWaypoint wp0{ kin_group->getJointNames(), Eigen::VectorXd::Zero(6) };

    // Specify freespace start waypoint
    CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.20, 0.8) *
                           Eigen::Quaterniond(0, 0, -1.0, 0) };

    // Define Plan Instructions
    MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "DEFAULT");
    MoveInstruction plan_f1(wp1, MoveInstructionType::FREESPACE, "DEFAULT");

    // Create program
    CompositeInstruction program;
    program.setManipulatorInfo(manip);
    program.push_back(start_instruction);
    program.push_back(plan_f1);

    // Plot Program
    if (plotter)
    {
    }

    // Create Profiles
    auto ompl_move_profile = std::make_shared<OMPLRealVectorMoveProfile>();
    auto trajopt_move_profile = std::make_shared<TrajOptDefaultMoveProfile>();
    auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
    auto trajopt_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();

    // Create a interpolated program
    CompositeInstruction interpolated_program = generateInterpolatedProgram(program, cur_state, env);

    // Profile Dictionary
    auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
    profiles->addProfile(OMPL_DEFAULT_NAMESPACE, "DEFAULT", ompl_move_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_move_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_solver_profile);

    // Create Planning Request
    PlannerRequest request;
    request.instructions = interpolated_program;
    request.env = env;
    request.env_state = cur_state;
    request.profiles = profiles;

    // Solve OMPL Plan
    OMPLMotionPlanner ompl_planner(OMPL_DEFAULT_NAMESPACE);
    PlannerResponse ompl_response = ompl_planner.solve(request);
    assert(ompl_response);

    // Plot OMPL Trajectory
    if (plotter)
    {
      plotter->waitForInput();
      plotter->plotTrajectory(toJointTrajectory(ompl_response.results), *state_solver);
    }

    // Update Seed
    request.instructions = ompl_response.results;

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
