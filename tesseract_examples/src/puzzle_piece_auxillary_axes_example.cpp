/**
 * @file puzzle_piece_auxillary_axes_example.cpp
 * @brief Puzzle piece auxillary axes implementation
 *
 * @author Levi Armstrong
 * @date July 22, 2019
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
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <fstream>
#include <trajopt_common/collision_types.h>
#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/puzzle_piece_auxillary_axes_example.h>

#include <tesseract/collision/types.h>

#include <tesseract/state_solver/state_solver.h>

#include <tesseract/environment/environment.h>
#include <tesseract/environment/utils.h>

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/stopwatch.h>
#include <tesseract/common/profile_dictionary.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

#include <tesseract/visualization/visualization.h>
#include <tesseract/visualization/markers/toolpath_marker.h>

#include <trajopt_sco/osqp_interface.hpp>

using namespace trajopt;
using namespace tesseract::environment;
using namespace tesseract::scene_graph;
using namespace tesseract::collision;
using namespace tesseract::visualization;
using namespace tesseract::task_composer;
using namespace tesseract::command_language;
using namespace tesseract::motion_planners;

static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";

namespace tesseract::examples
{
tesseract::common::VectorIsometry3d makePuzzleToolPoses(const tesseract::common::ResourceLocator::ConstPtr& locator)
{
  tesseract::common::VectorIsometry3d path;  // results
  std::ifstream indata;                      // input file

  // You could load your parts from anywhere, but we are transporting them with
  // the git repo
  auto resource = locator->locateResource("package://tesseract/support/urdf/puzzle_bent.csv");

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
    std::shared_ptr<tesseract::environment::Environment> env,
    std::shared_ptr<tesseract::visualization::Visualization> plotter,
    bool ifopt,
    bool debug,
    bool benchmark)
  : Example(std::move(env), std::move(plotter)), ifopt_(ifopt), debug_(debug), benchmark_(benchmark)
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
  tesseract::common::VectorIsometry3d tool_poses = makePuzzleToolPoses(env_->getResourceLocator());

  // Create manipulator information for program
  tesseract::common::ManipulatorInfo mi;
  mi.manipulator = "manipulator_aux";
  mi.working_frame = "part";
  mi.tcp_frame = "grinder_frame";

  // Create Task Composer Plugin Factory
  std::shared_ptr<const tesseract::common::ResourceLocator> locator = env_->getResourceLocator();
  std::filesystem::path config_path(
      locator->locateResource("package://tesseract_task_composer/config/task_composer_plugins.yaml")->getFilePath());
  TaskComposerPluginFactory factory(config_path, *env_->getResourceLocator());

  // Create Program
  CompositeInstruction program("DEFAULT", mi);

  // Create cartesian waypoint
  for (std::size_t i = 0; i < tool_poses.size(); ++i)
  {
    CartesianWaypoint wp{ tool_poses[i] };
    MoveInstruction plan_instruction(wp, MoveInstructionType::LINEAR, "CARTESIAN");
    plan_instruction.setDescription("waypoint_" + std::to_string(i));
    program.push_back(plan_instruction);
  }

  // Assign the current state as the seed for cartesian waypoints
  assignCurrentStateAsSeed(program, *env_);

  // Print Diagnostics
  if (debug_)
    program.print("Program: ");

  // Create profile dictionary
  auto profiles = std::make_shared<tesseract::common::ProfileDictionary>();
  if (ifopt_)
  {
    // Create TrajOpt_Ifopt Profile
    auto trajopt_ifopt_move_profile = std::make_shared<TrajOptIfoptDefaultMoveProfile>();
    trajopt_ifopt_move_profile->joint_cost_config.enabled = false;
    trajopt_ifopt_move_profile->cartesian_cost_config.enabled = false;
    trajopt_ifopt_move_profile->cartesian_constraint_config.enabled = true;
    trajopt_ifopt_move_profile->cartesian_constraint_config.coeff = Eigen::VectorXd::Constant(6, 1, 5);
    trajopt_ifopt_move_profile->cartesian_constraint_config.coeff(3) = 2;
    trajopt_ifopt_move_profile->cartesian_constraint_config.coeff(4) = 2;
    trajopt_ifopt_move_profile->cartesian_constraint_config.coeff(5) = 0;

    auto trajopt_ifopt_composite_profile = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
    trajopt_ifopt_composite_profile->collision_constraint_config.enabled = false;
    trajopt_ifopt_composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.025, 1);
    trajopt_ifopt_composite_profile->collision_cost_config.collision_check_config.type =
        tesseract::collision::CollisionEvaluatorType::DISCRETE;

    auto trajopt_ifopt_solver_profile = std::make_shared<TrajOptIfoptOSQPSolverProfile>();
    // trajopt_ifopt_solver_profile->convex_solver_settings.adaptive_rho = 0;
    trajopt_ifopt_solver_profile->opt_params.max_iterations = 200;
    trajopt_ifopt_solver_profile->opt_params.min_approx_improve = 1e-3;
    trajopt_ifopt_solver_profile->opt_params.min_trust_box_size = 1e-3;

    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_ifopt_move_profile);
    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_ifopt_composite_profile);
    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_ifopt_solver_profile);
  }
  else
  {
    // Create TrajOpt Profile
    auto trajopt_move_profile = std::make_shared<TrajOptDefaultMoveProfile>();
    trajopt_move_profile->joint_cost_config.enabled = false;
    trajopt_move_profile->cartesian_cost_config.enabled = false;
    trajopt_move_profile->cartesian_constraint_config.enabled = true;
    trajopt_move_profile->cartesian_constraint_config.coeff = Eigen::VectorXd::Constant(6, 1, 5);
    trajopt_move_profile->cartesian_constraint_config.coeff(3) = 2;
    trajopt_move_profile->cartesian_constraint_config.coeff(4) = 2;
    trajopt_move_profile->cartesian_constraint_config.coeff(5) = 0;

    auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
    trajopt_composite_profile->collision_constraint_config.enabled = false;
    trajopt_composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.025, 1);
    trajopt_composite_profile->collision_cost_config.collision_check_config.type =
        tesseract::collision::CollisionEvaluatorType::DISCRETE;

    auto trajopt_solver_profile = std::make_shared<TrajOptOSQPSolverProfile>();
    trajopt_solver_profile->settings.adaptive_rho = 0;
    trajopt_solver_profile->opt_params.max_iter = 200;
    trajopt_solver_profile->opt_params.min_approx_improve = 1e-3;
    trajopt_solver_profile->opt_params.min_trust_box_size = 1e-3;

    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_move_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_solver_profile);
  }

  // Create task
  const std::string task_name = (ifopt_) ? "TrajOptIfoptPipeline" : "TrajOptPipeline";
  TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
  const std::string output_key = task->getOutputKeys().get("program");

  if (plotter_ != nullptr)
    plotter_->waitForInput();

  // Solve task
  TaskComposerFuture::UPtr future;
  if (!benchmark_)
  {
    // Create Executor
    auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");

    // Create Task Composer Data Storage
    auto data = std::make_unique<TaskComposerDataStorage>();
    data->setData("planning_input", program);
    data->setData("environment", std::shared_ptr<const tesseract::environment::Environment>(env_));
    data->setData("profiles", profiles);

    tesseract::common::Stopwatch stopwatch;
    stopwatch.start();
    auto context = std::make_shared<TaskComposerContext>(task->getName(), std::move(data));
    future = executor->run(*task, std::move(context));
    future->wait();
    stopwatch.stop();
    CONSOLE_BRIDGE_logInform("Planning took %f seconds.", stopwatch.elapsedSeconds());
  }
  else
  {
    const int cnt{ 10 };
    std::vector<std::string> contact_managers{ "BulletDiscreteBVHManager", "BulletDiscreteSimpleManager" };

    for (const auto& contact_manager : contact_managers)
    {
      // Create Executor
      auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");

      double initial_planning_time{ 0 };
      double accumulated_time{ 0 };
      for (int i = 0; i < cnt; ++i)
      {
        // Set the active contact manager
        env_->setActiveDiscreteContactManager(contact_manager);

        // Create Task Composer Data Storage
        auto data = std::make_unique<TaskComposerDataStorage>();
        data->setData("planning_input", program);
        data->setData("environment", std::shared_ptr<const tesseract::environment::Environment>(env_));
        data->setData("profiles", profiles);

        tesseract::common::Stopwatch stopwatch;
        stopwatch.start();
        auto context = std::make_shared<TaskComposerContext>(task->getName(), std::move(data));
        future = executor->run(*task, std::move(context));
        future->wait();
        stopwatch.stop();
        if (i == 0)
          initial_planning_time = stopwatch.elapsedSeconds();
        else
          accumulated_time += stopwatch.elapsedSeconds();
      }
      if (ifopt_)
      {
        CONSOLE_BRIDGE_logInform("PuzzlePieceAuxillaryAxesExample(Ifopt), %s, %f, %f, %d",
                                 contact_manager.c_str(),
                                 initial_planning_time,
                                 accumulated_time / (cnt - 1),
                                 (cnt - 1));
      }
      else
      {
        CONSOLE_BRIDGE_logInform("PuzzlePieceAuxillaryAxesExample, %s, %f, %f, %d",
                                 contact_manager.c_str(),
                                 initial_planning_time,
                                 accumulated_time / (cnt - 1),
                                 (cnt - 1));
      }
    }
  }

  // Plot Process Trajectory
  if (plotter_ != nullptr && plotter_->isConnected())
  {
    plotter_->waitForInput();
    auto ci = future->context->data_storage->getData(output_key).as<CompositeInstruction>();
    tesseract::common::JointTrajectory trajectory = toJointTrajectory(ci);
    auto state_solver = env_->getStateSolver();
    plotter_->plotTrajectory(trajectory, *state_solver);
  }

  return future->context->isSuccessful();
}
}  // namespace tesseract::examples
