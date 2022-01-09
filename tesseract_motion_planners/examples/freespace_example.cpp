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
#include <tesseract_visualization/visualization_loader.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/interface_utils.h>
// OMPL
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/profile/ompl_planner_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_composite_profile_rvss.h>
#include <tesseract_motion_planners/ompl/profile/ompl_waypoint_profile.h>
// TrajOpt
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>

using namespace tesseract_planning;
using namespace tesseract_planning::profile_ns;

static const std::string PROFILE_NAME = "DEFAULT";

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

int main(int argc, char** argv)
{
  try
  {
    // Setup
    auto locator = std::make_shared<tesseract_common::SimpleResourceLocator>(locateResource);
    auto env = std::make_shared<tesseract_environment::Environment>();
    tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
    tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
    env->init(urdf_path, srdf_path, locator);

    // Dynamically load ignition visualizer if it exists
    tesseract_visualization::VisualizationLoader loader;
    const std::string plugin_name = argc < 2 ? "" : argv[1];
    auto plotter = loader.get(plugin_name);

    if (plotter != nullptr)
    {
      plotter->waitForConnection();
      plotter->plotEnvironment(*env);
    }

    ManipulatorInfo manip;
    manip.tcp_frame = "tool0";
    manip.working_frame = "base_link";
    manip.manipulator = "manipulator";
    manip.manipulator_ik_solver = "OPWInvKin";

    auto state_solver = env->getStateSolver();
    auto kin_group = env->getKinematicGroup(manip.manipulator, manip.manipulator_ik_solver);
    auto cur_state = env->getState();

    // Specify start location
    StateWaypoint wp0(kin_group->getJointNames(), Eigen::VectorXd::Zero(6));

    // Specify freespace start waypoint
    CartesianWaypoint wp1 =
        Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -.20, 0.8) * Eigen::Quaterniond(0, 0, -1.0, 0);

    // Define Plan Instructions
    PlanInstruction start_instruction(wp0, PlanInstructionType::START, PROFILE_NAME, manip);
    PlanInstruction plan_f1(wp1, PlanInstructionType::FREESPACE, PROFILE_NAME, manip);
    PlanInstruction plan_f2(wp0, PlanInstructionType::FREESPACE, PROFILE_NAME, manip);

    // Create program
    CompositeInstruction program;
    program.setStartInstruction(start_instruction);
    program.setManipulatorInfo(manip);
    program.push_back(plan_f1);
    program.push_back(plan_f2);

    // Profile Dictionary
    auto profiles = std::make_shared<ProfileDictionary>();
    profiles->planner_profiles[OMPL_DEFAULT_NAMESPACE][PROFILE_NAME] = std::make_shared<OMPLPlannerProfile>();
    profiles->composite_profiles[OMPL_DEFAULT_NAMESPACE][PROFILE_NAME] = std::make_shared<OMPLCompositeProfileRVSS>();
    profiles->waypoint_profiles[OMPL_DEFAULT_NAMESPACE][PROFILE_NAME] = std::make_shared<OMPLWaypointProfile>();

    // Create a seed
    CompositeInstruction seed = generateSeed(program, cur_state, env);

    // Create Planning Request
    PlannerRequest request;
    request.seed = seed;
    request.instructions = program;
    request.env = env;
    request.env_state = cur_state;
    request.profiles = profiles;

    // Solve OMPL Plan
    PlannerResponse ompl_response;
    OMPLMotionPlanner ompl_planner;
    auto ompl_status = ompl_planner.solve(request, ompl_response);
    assert(ompl_status);

    // Plot OMPL Trajectory
    if (plotter != nullptr)
    {
      plotter->waitForInput();
      plotter->plotTrajectory(toJointTrajectory(ompl_response.results), *state_solver);
    }

    // Create the TrajOpt profiles
    auto trajopt_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
    auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

    // Add the TrajOpt profiles to the dictionary
    profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, PROFILE_NAME, trajopt_plan_profile);
    profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, PROFILE_NAME, trajopt_composite_profile);

    // Update the seed to be the OMPL trajecory
    request.seed = ompl_response.results;

    // Solve TrajOpt Plan
    PlannerResponse trajopt_response;
    TrajOptMotionPlanner trajopt_planner;
    auto trajopt_status = trajopt_planner.solve(request, trajopt_response, true);
    assert(trajopt_status);

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
