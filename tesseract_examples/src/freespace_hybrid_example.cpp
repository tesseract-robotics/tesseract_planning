/**
 * @file freespace_ompl_example.cpp
 * @brief An example of a feespace motion planning with OMPL.
 *
 * @author Levi Armstrong
 * @date March 16, 2020
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

#include <tesseract_examples/freespace_hybrid_example.h>

#include <trajopt_common/collision_types.h>

#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>

#include <tesseract_state_solver/state_solver.h>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_environment/commands/add_link_command.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tesseract_geometry/impl/sphere.h>

using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

static const std::string OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask";
static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";

namespace tesseract_examples
{
FreespaceHybridExample::FreespaceHybridExample(std::shared_ptr<tesseract_environment::Environment> env,
                                               std::shared_ptr<tesseract_visualization::Visualization> plotter,
                                               bool ifopt,
                                               bool debug,
                                               double range,
                                               double planning_time)
  : Example(std::move(env), std::move(plotter))
  , ifopt_(ifopt)
  , debug_(debug)
  , range_(range)
  , planning_time_(planning_time)
{
}

Command::Ptr addSphere()
{
  // Add sphere to environment
  Link link_sphere("sphere_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(0.5, 0, 0.55);
  visual->geometry = std::make_shared<tesseract_geometry::Sphere>(0.15);
  link_sphere.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_sphere.collision.push_back(collision);

  Joint joint_sphere("joint_sphere_attached");
  joint_sphere.parent_link_name = "base_link";
  joint_sphere.child_link_name = link_sphere.getName();
  joint_sphere.type = JointType::FIXED;

  return std::make_shared<tesseract_environment::AddLinkCommand>(link_sphere, joint_sphere);
}

bool FreespaceHybridExample::run()
{
  if (debug_)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  // Add sphere to environment
  Command::Ptr cmd = addSphere();
  if (!env_->applyCommand(cmd))
    return false;

  if (plotter_ != nullptr)
    plotter_->waitForConnection();

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.emplace_back("joint_a1");
  joint_names.emplace_back("joint_a2");
  joint_names.emplace_back("joint_a3");
  joint_names.emplace_back("joint_a4");
  joint_names.emplace_back("joint_a5");
  joint_names.emplace_back("joint_a6");
  joint_names.emplace_back("joint_a7");

  Eigen::VectorXd joint_start_pos(7);
  joint_start_pos(0) = -0.4;
  joint_start_pos(1) = 0.2762;
  joint_start_pos(2) = 0.0;
  joint_start_pos(3) = -1.3348;
  joint_start_pos(4) = 0.0;
  joint_start_pos(5) = 1.4959;
  joint_start_pos(6) = 0.0;

  Eigen::VectorXd joint_end_pos(7);
  joint_end_pos(0) = 0.4;
  joint_end_pos(1) = 0.2762;
  joint_end_pos(2) = 0.0;
  joint_end_pos(3) = -1.3348;
  joint_end_pos(4) = 0.0;
  joint_end_pos(5) = 1.4959;
  joint_end_pos(6) = 0.0;

  env_->setState(joint_names, joint_start_pos);

  // Create Task Composer Plugin Factory
  const std::string share_dir(TESSERACT_TASK_COMPOSER_DIR);
  tesseract_common::fs::path config_path(share_dir + "/config/task_composer_plugins.yaml");
  TaskComposerPluginFactory factory(config_path);

  // Create Program
  CompositeInstruction program(
      "FREESPACE", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool0"));

  // Start and End Joint Position for the program
  StateWaypointPoly wp0{ StateWaypoint(joint_names, joint_start_pos) };
  StateWaypointPoly wp1{ StateWaypoint(joint_names, joint_end_pos) };

  MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "FREESPACE");
  start_instruction.setDescription("Start Instruction");

  // Plan freespace from start
  MoveInstruction plan_f0(wp1, MoveInstructionType::FREESPACE, "FREESPACE");
  plan_f0.setDescription("freespace_plan");

  // Add Instructions to program
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f0);

  // Print Diagnostics
  program.print("Program: ");

  CONSOLE_BRIDGE_logInform("freespace hybrid plan example");

  // Create Executor
  auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");

  // Create profile dictionary
  auto profiles = std::make_shared<ProfileDictionary>();

  // Create OMPL Profile
  auto ompl_profile = std::make_shared<OMPLDefaultPlanProfile>();
  auto ompl_planner_config = std::make_shared<RRTConnectConfigurator>();
  ompl_planner_config->range = range_;
  ompl_profile->planning_time = planning_time_;
  ompl_profile->planners = { ompl_planner_config, ompl_planner_config };

  profiles->addProfile(OMPL_DEFAULT_NAMESPACE, "FREESPACE", ompl_profile);

  if (ifopt_)
  {
    // Create TrajOpt_Ifopt Profile
    auto trajopt_ifopt_composite_profile = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
    trajopt_ifopt_composite_profile->collision_constraint_config->contact_manager_config =
        tesseract_collision::ContactManagerConfig(0.025);
    trajopt_ifopt_composite_profile->collision_cost_config->contact_manager_config =
        tesseract_collision::ContactManagerConfig(0.025);
    trajopt_ifopt_composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
    trajopt_ifopt_composite_profile->acceleration_coeff = Eigen::VectorXd::Ones(1);
    trajopt_ifopt_composite_profile->jerk_coeff = Eigen::VectorXd::Ones(1);

    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_ifopt_composite_profile);
  }

  // Create task
  const std::string task_name = (ifopt_) ? "FreespaceIfoptPipeline" : "FreespacePipeline";
  TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
  const std::string output_key = task->getOutputKeys().get("program");

  // Create Task Composer Data Storage
  auto data = std::make_unique<tesseract_planning::TaskComposerDataStorage>();
  data->setData("planning_input", program);
  data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
  data->setData("profiles", profiles);

  // Solve task
  TaskComposerFuture::UPtr future = executor->run(*task, std::move(data));
  future->wait();

  // Plot Process Trajectory
  if (plotter_ != nullptr && plotter_->isConnected())
  {
    plotter_->waitForInput();
    auto ci = future->context->data_storage->getData(output_key).as<CompositeInstruction>();
    tesseract_common::Toolpath toolpath = toToolpath(ci, *env_);
    tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
    auto state_solver = env_->getStateSolver();
    plotter_->plotMarker(ToolpathMarker(toolpath));
    plotter_->plotTrajectory(trajectory, *state_solver);
  }

  CONSOLE_BRIDGE_logInform("Final trajectory is collision free");
  return future->context->isSuccessful();
}
}  // namespace tesseract_examples
