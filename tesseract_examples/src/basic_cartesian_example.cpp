/**
 * @file basic_cartesian_plan.cpp
 * @brief Basic cartesian example implementation
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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <console_bridge/console.h>
#include <trajopt_common/collision_types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/basic_cartesian_example.h>

#include <tesseract_common/stopwatch.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/profile_dictionary.h>

#include <tesseract_collision/core/types.h>

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
#include <tesseract_command_language/utils.h>

#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

#include <tesseract_geometry/impl/octree.h>
#include <tesseract_geometry/impl/octree_utils.h>

using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";

namespace tesseract_examples
{
Command::Ptr addPointCloud()
{
  // Create octomap and add it to the local environment
  pcl::PointCloud<pcl::PointXYZ> full_cloud;
  double delta = 0.05;
  auto length = static_cast<int>(1 / delta);

  for (int x = 0; x < length; ++x)
    for (int y = 0; y < length; ++y)
      for (int z = 0; z < length; ++z)
        full_cloud.push_back(pcl::PointXYZ(-0.5F + static_cast<float>(x * delta),
                                           -0.5F + static_cast<float>(y * delta),
                                           -0.5F + static_cast<float>(z * delta)));

  //  sensor_msgs::PointCloud2 pointcloud_msg;
  //  pcl::toROSMsg(full_cloud, pointcloud_msg);

  //  octomap::Pointcloud octomap_data;
  //  octomap::pointCloud2ToOctomap(pointcloud_msg, octomap_data);
  //  std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(2 * delta);
  //  octree->insertPointCloud(octomap_data, octomap::point3d(0, 0, 0));

  // Add octomap to environment
  Link link_octomap("octomap_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(1, 0, 0);
  auto ot = tesseract_geometry::createOctree(full_cloud, 2 * delta, false, true);
  visual->geometry =
      std::make_shared<tesseract_geometry::Octree>(std::move(ot), tesseract_geometry::OctreeSubType::BOX, false, true);
  link_octomap.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_octomap.collision.push_back(collision);

  Joint joint_octomap("joint_octomap_attached");
  joint_octomap.parent_link_name = "base_link";
  joint_octomap.child_link_name = link_octomap.getName();
  joint_octomap.type = JointType::FIXED;

  return std::make_shared<tesseract_environment::AddLinkCommand>(link_octomap, joint_octomap);
}

BasicCartesianExample::BasicCartesianExample(std::shared_ptr<tesseract_environment::Environment> env,
                                             std::shared_ptr<tesseract_visualization::Visualization> plotter,
                                             bool ifopt,
                                             bool debug,
                                             bool benchmark)
  : Example(std::move(env), std::move(plotter)), ifopt_(ifopt), debug_(debug), benchmark_(benchmark)
{
}

bool BasicCartesianExample::run()
{
  // Create octomap and add it to the local environment
  Command::Ptr cmd = addPointCloud();
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

  Eigen::VectorXd joint_pos(7);
  joint_pos(0) = -0.4;
  joint_pos(1) = 0.2762;
  joint_pos(2) = 0.0;
  joint_pos(3) = -1.3348;
  joint_pos(4) = 0.0;
  joint_pos(5) = 1.4959;
  joint_pos(6) = 0.0;

  env_->setState(joint_names, joint_pos);

  if (debug_)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  // Create Task Composer Plugin Factory
  std::shared_ptr<const tesseract_common::ResourceLocator> locator = env_->getResourceLocator();
  std::filesystem::path config_path(
      locator->locateResource("package://tesseract_task_composer/config/task_composer_plugins.yaml")->getFilePath());
  TaskComposerPluginFactory factory(config_path, *env_->getResourceLocator());

  // Create Program
  CompositeInstruction program("cartesian_program", ManipulatorInfo("manipulator", "base_link", "tool0"));

  // Start Joint Position for the program
  StateWaypoint wp0{ joint_names, joint_pos };
  MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "freespace_profile");
  start_instruction.setDescription("Start Instruction");

  // Create cartesian waypoint
  CartesianWaypoint wp1{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.5, -0.2, 0.62) *
                         Eigen::Quaterniond(0, 0, 1.0, 0) };

  CartesianWaypoint wp2{ Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.5, 0.3, 0.62) *
                         Eigen::Quaterniond(0, 0, 1.0, 0) };

  // Plan freespace from start
  MoveInstruction plan_f0(wp1, MoveInstructionType::FREESPACE, "freespace_profile");
  plan_f0.setDescription("from_start_plan");

  // Plan linear move
  MoveInstruction plan_c0(wp2, MoveInstructionType::LINEAR, "RASTER");

  // Plan freespace to end
  MoveInstruction plan_f1(wp0, MoveInstructionType::FREESPACE, "freespace_profile");
  plan_f1.setDescription("to_end_plan");

  // Add Instructions to program
  program.push_back(start_instruction);
  program.push_back(plan_f0);
  program.push_back(plan_c0);
  program.push_back(plan_f1);

  // Print Diagnostics
  if (debug_)
    program.print("Program: ");

  // Create profile dictionary
  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
  if (ifopt_)
  {
    auto composite_profile = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
    composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.025, 20);
    composite_profile->collision_cost_config.collision_check_config.type =
        tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    composite_profile->collision_constraint_config = trajopt_common::TrajOptCollisionConfig(0.0, 20);
    composite_profile->collision_constraint_config.collision_check_config.type =
        tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;

    composite_profile->smooth_velocities = true;
    composite_profile->smooth_accelerations = false;
    composite_profile->smooth_jerks = false;
    composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "cartesian_program", composite_profile);

    auto move_profile = std::make_shared<TrajOptIfoptDefaultMoveProfile>();
    move_profile->cartesian_cost_config.enabled = false;
    move_profile->cartesian_constraint_config.enabled = true;
    move_profile->cartesian_constraint_config.coeff = Eigen::VectorXd::Ones(6);
    move_profile->joint_cost_config.enabled = false;
    move_profile->joint_constraint_config.enabled = true;
    move_profile->joint_constraint_config.coeff = Eigen::VectorXd::Ones(7);
    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "RASTER", move_profile);
    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "freespace_profile", move_profile);
  }
  else
  {
    auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
    composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.025, 20);
    composite_profile->collision_cost_config.collision_check_config.type =
        tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    composite_profile->collision_constraint_config = trajopt_common::TrajOptCollisionConfig(0.0, 20);
    composite_profile->collision_constraint_config.collision_check_config.type =
        tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    composite_profile->smooth_velocities = true;
    composite_profile->smooth_accelerations = false;
    composite_profile->smooth_jerks = false;
    composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "cartesian_program", composite_profile);

    auto move_profile = std::make_shared<TrajOptDefaultMoveProfile>();
    move_profile->cartesian_cost_config.enabled = false;
    move_profile->cartesian_constraint_config.enabled = true;
    move_profile->cartesian_constraint_config.coeff = Eigen::VectorXd::Ones(6);
    move_profile->joint_cost_config.enabled = false;
    move_profile->joint_constraint_config.enabled = true;
    move_profile->joint_constraint_config.coeff = Eigen::VectorXd::Ones(7);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "RASTER", move_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "freespace_profile", move_profile);
  }

  // Create task
  const std::string task_name = (ifopt_) ? "TrajOptIfoptPipeline" : "TrajOptPipeline";
  TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
  const std::string output_key = task->getOutputKeys().get("program");

  if (plotter_ != nullptr && plotter_->isConnected())
    plotter_->waitForInput("Hit Enter to solve for trajectory.");

  // Solve task
  TaskComposerFuture::UPtr future;
  if (!benchmark_)
  {
    // Create Executor
    auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");

    // Create Task Composer Data Storage
    auto data = std::make_unique<tesseract_planning::TaskComposerDataStorage>();
    data->setData("planning_input", program);
    data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    data->setData("profiles", profiles);

    tesseract_common::Stopwatch stopwatch;
    stopwatch.start();
    future = executor->run(*task, std::move(data));
    future->wait();
    stopwatch.stop();
    CONSOLE_BRIDGE_logInform("Planning took %f seconds.", stopwatch.elapsedSeconds());
  }
  else
  {
    const int cnt{ 100 };
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
        auto data = std::make_unique<tesseract_planning::TaskComposerDataStorage>();
        data->setData("planning_input", program);
        data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
        data->setData("profiles", profiles);

        tesseract_common::Stopwatch stopwatch;
        stopwatch.start();
        future = executor->run(*task, std::move(data));
        future->wait();
        stopwatch.stop();
        if (i == 0)
          initial_planning_time = stopwatch.elapsedSeconds();
        else
          accumulated_time += stopwatch.elapsedSeconds();
      }
      CONSOLE_BRIDGE_logInform("BasicCartesianExample, %s, %f, %f, %d",
                               contact_manager.c_str(),
                               initial_planning_time,
                               accumulated_time / (cnt - 1),
                               (cnt - 1));
    }
  }

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

  return future->context->isSuccessful();
}

}  // namespace tesseract_examples
