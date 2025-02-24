/**
 * @file puzzle_piece_auxillary_axes_example.cpp
 * @brief Puzzle piece auxillary axes implementation
 *
 * @author Levi Armstrong
 * @date July 22, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <fstream>
#include <trajopt_common/collision_types.h>
#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/puzzle_piece_auxillary_axes_example.h>

#include <tesseract_collision/core/types.h>

#include <tesseract_state_solver/state_solver.h>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/stopwatch.h>

#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <trajopt_sco/osqp_interface.hpp>

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;

static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";

namespace tesseract_examples
{
tesseract_common::VectorIsometry3d makePuzzleToolPoses(const tesseract_common::ResourceLocator::ConstPtr& locator)
{
  tesseract_common::VectorIsometry3d path;  // results
  std::ifstream indata;                     // input file

  // You could load your parts from anywhere, but we are transporting them with
  // the git repo
  auto resource = locator->locateResource("package://tesseract_support/urdf/puzzle_bent.csv");

  // In a non-trivial app, you'll of course want to check that calls like 'open'
  // succeeded
  indata.open(resource->getFilePath());

  std::string line;
  int lnum = 0;
  while (std::getline(indata, line))
  {
    ++lnum;
    if (lnum < 3)
      continue;

    std::stringstream lineStream(line);
    std::string cell;
    Eigen::Matrix<double, 6, 1> xyzijk;
    xyzijk.setZero();
    int i = -2;
    while (std::getline(lineStream, cell, ','))
    {
      ++i;
      if (i == -1)
        continue;

      xyzijk(i) = std::stod(cell);
    }

    Eigen::Vector3d pos = xyzijk.head<3>();
    pos = pos / 1000.0;  // Most things in ROS use meters as the unit of length.
                         // Our part was exported in mm.
    Eigen::Vector3d norm = xyzijk.tail<3>();
    norm.normalize();

    // This code computes two extra directions to turn the normal direction into
    // a full defined frame. Descartes
    // will search around this frame for extra poses, so the exact values do not
    // matter as long they are valid.
    Eigen::Vector3d temp_x = (-1 * pos).normalized();
    Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
    Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();
    Eigen::Isometry3d pose;
    pose.matrix().col(0).head<3>() = x_axis;
    pose.matrix().col(1).head<3>() = y_axis;
    pose.matrix().col(2).head<3>() = norm;
    pose.matrix().col(3).head<3>() = pos;

    path.push_back(pose);
  }
  indata.close();

  return path;
}

PuzzlePieceAuxillaryAxesExample::PuzzlePieceAuxillaryAxesExample(
    std::shared_ptr<tesseract_environment::Environment> env,
    std::shared_ptr<tesseract_visualization::Visualization> plotter,
    bool ifopt,
    bool debug)
  : Example(std::move(env), std::move(plotter)), ifopt_(ifopt), debug_(debug)
{
}

bool PuzzlePieceAuxillaryAxesExample::run()
{
  if (debug_)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

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
  joint_names.emplace_back("joint_aux1");
  joint_names.emplace_back("joint_aux2");

  Eigen::VectorXd joint_pos(9);
  joint_pos(0) = -0.785398;
  joint_pos(1) = 0.4;
  joint_pos(2) = 0.0;
  joint_pos(3) = -1.9;
  joint_pos(4) = 0.0;
  joint_pos(5) = 1.0;
  joint_pos(6) = 0.0;
  joint_pos(7) = 0.0;
  joint_pos(8) = 0.0;

  env_->setState(joint_names, joint_pos);

  // Get Tool Poses
  tesseract_common::VectorIsometry3d tool_poses = makePuzzleToolPoses(env_->getResourceLocator());

  // Create manipulator information for program
  tesseract_common::ManipulatorInfo mi;
  mi.manipulator = "manipulator_aux";
  mi.working_frame = "part";
  mi.tcp_frame = "grinder_frame";

  // Create Task Composer Plugin Factory
  std::shared_ptr<const tesseract_common::ResourceLocator> locator = env_->getResourceLocator();
  std::filesystem::path config_path(
      locator->locateResource("package://tesseract_task_composer/config/task_composer_plugins.yaml")->getFilePath());
  TaskComposerPluginFactory factory(config_path, *env_->getResourceLocator());

  // Create Program
  CompositeInstruction program("DEFAULT", mi);

  // Create cartesian waypoint
  for (std::size_t i = 0; i < tool_poses.size(); ++i)
  {
    CartesianWaypointPoly wp{ CartesianWaypoint(tool_poses[i]) };
    MoveInstruction plan_instruction(wp, MoveInstructionType::LINEAR, "CARTESIAN");
    plan_instruction.setDescription("waypoint_" + std::to_string(i));
    program.appendMoveInstruction(plan_instruction);
  }

  // Assign the current state as the seed for cartesian waypoints
  assignCurrentStateAsSeed(program, *env_);

  // Print Diagnostics
  program.print("Program: ");

  // Create Executor
  auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");

  // Create profile dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  if (ifopt_)
  {
    // Create TrajOpt_Ifopt Profile
    auto trajopt_ifopt_plan_profile = std::make_shared<TrajOptIfoptDefaultPlanProfile>();
    trajopt_ifopt_plan_profile->joint_cost_config.enabled = false;
    trajopt_ifopt_plan_profile->cartesian_cost_config.enabled = false;
    trajopt_ifopt_plan_profile->cartesian_constraint_config.enabled = true;
    trajopt_ifopt_plan_profile->cartesian_constraint_config.coeff = Eigen::VectorXd::Constant(6, 1, 5);
    trajopt_ifopt_plan_profile->cartesian_constraint_config.coeff(3) = 2;
    trajopt_ifopt_plan_profile->cartesian_constraint_config.coeff(4) = 2;
    trajopt_ifopt_plan_profile->cartesian_constraint_config.coeff(5) = 0;

    auto trajopt_ifopt_composite_profile = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
    trajopt_ifopt_composite_profile->collision_constraint_config = nullptr;
    trajopt_ifopt_composite_profile->collision_cost_config->type =
        tesseract_collision::CollisionEvaluatorType::DISCRETE;
    trajopt_ifopt_composite_profile->collision_cost_config->contact_manager_config =
        tesseract_collision::ContactManagerConfig(0.025);
    trajopt_ifopt_composite_profile->collision_cost_config->collision_margin_buffer = 0.05;
    trajopt_ifopt_composite_profile->collision_cost_config->collision_coeff_data =
        trajopt_common::CollisionCoeffData(2);
    trajopt_ifopt_composite_profile->smooth_velocities = false;
    trajopt_ifopt_composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
    trajopt_ifopt_composite_profile->smooth_accelerations = true;
    trajopt_ifopt_composite_profile->acceleration_coeff = Eigen::VectorXd::Ones(1);
    trajopt_ifopt_composite_profile->smooth_jerks = true;
    trajopt_ifopt_composite_profile->jerk_coeff = Eigen::VectorXd::Ones(1);

    auto trajopt_ifopt_solver_profile = std::make_shared<TrajOptIfoptOSQPSolverProfile>();
    // trajopt_ifopt_solver_profile->convex_solver_settings.adaptive_rho = 0;
    trajopt_ifopt_solver_profile->opt_params.max_iterations = 200;
    trajopt_ifopt_solver_profile->opt_params.min_approx_improve = 1e-3;
    trajopt_ifopt_solver_profile->opt_params.min_trust_box_size = 1e-3;

    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_ifopt_plan_profile);
    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_ifopt_composite_profile);
    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_ifopt_solver_profile);
  }
  else
  {
    // Create TrajOpt Profile
    auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
    trajopt_plan_profile->joint_cost_config.enabled = false;
    trajopt_plan_profile->cartesian_cost_config.enabled = false;
    trajopt_plan_profile->cartesian_constraint_config.enabled = true;
    trajopt_plan_profile->cartesian_constraint_config.coeff = Eigen::VectorXd::Constant(6, 1, 5);
    trajopt_plan_profile->cartesian_constraint_config.coeff(3) = 2;
    trajopt_plan_profile->cartesian_constraint_config.coeff(4) = 2;
    trajopt_plan_profile->cartesian_constraint_config.coeff(5) = 0;

    auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
    trajopt_composite_profile->collision_constraint_config.enabled = false;
    trajopt_composite_profile->collision_cost_config.enabled = true;
    trajopt_composite_profile->collision_cost_config.safety_margin = 0.025;
    trajopt_composite_profile->collision_cost_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
    trajopt_composite_profile->collision_cost_config.coeff = 1;

    auto trajopt_solver_profile = std::make_shared<TrajOptOSQPSolverProfile>();
    trajopt_solver_profile->settings.adaptive_rho = 0;
    trajopt_solver_profile->opt_params.max_iter = 200;
    trajopt_solver_profile->opt_params.min_approx_improve = 1e-3;
    trajopt_solver_profile->opt_params.min_trust_box_size = 1e-3;

    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_plan_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_solver_profile);
  }

  // Create task
  const std::string task_name = (ifopt_) ? "TrajOptIfoptPipeline" : "TrajOptPipeline";
  TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
  const std::string output_key = task->getOutputKeys().get("program");

  if (plotter_ != nullptr)
    plotter_->waitForInput();

  // Create Task Composer Data Storage
  auto data = std::make_unique<tesseract_planning::TaskComposerDataStorage>();
  data->setData("planning_input", program);
  data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
  data->setData("profiles", profiles);

  // Solve task
  tesseract_common::Stopwatch stopwatch;
  stopwatch.start();
  TaskComposerFuture::UPtr future = executor->run(*task, std::move(data));
  future->wait();

  stopwatch.stop();
  CONSOLE_BRIDGE_logInform("Planning took %f seconds.", stopwatch.elapsedSeconds());

  // Plot Process Trajectory
  if (plotter_ != nullptr && plotter_->isConnected())
  {
    plotter_->waitForInput();
    auto ci = future->context->data_storage->getData(output_key).as<CompositeInstruction>();
    tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
    auto state_solver = env_->getStateSolver();
    plotter_->plotTrajectory(trajectory, *state_solver);
  }

  CONSOLE_BRIDGE_logInform("Final trajectory is collision free");
  return future->context->isSuccessful();
}
}  // namespace tesseract_examples
