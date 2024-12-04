/**
 * @file car_seat_example.cpp
 * @brief Car seat example implementation
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
#include <trajopt_common/collision_types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/car_seat_example.h>

#include <tesseract_common/allowed_collision_matrix.h>

#include <tesseract_collision/bullet/convex_hull_utils.h>

#include <tesseract_kinematics/core/joint_group.h>

#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>

#include <tesseract_state_solver/state_solver.h>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_environment/commands/move_link_command.h>
#include <tesseract_environment/commands/modify_allowed_collisions_command.h>

#include <tesseract_geometry/mesh_parser.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_solver_profile.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
namespace tesseract_examples
{
Commands addSeats(const tesseract_common::ResourceLocator::ConstPtr& locator)
{
  Commands cmds;

  for (int i = 0; i < 3; ++i)
  {
    Link link_seat("seat_" + std::to_string(i + 1));

    Visual::Ptr visual = std::make_shared<Visual>();
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry =
        tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(locator->locateResource("package://"
                                                                                                     "tesseract_"
                                                                                                     "support/meshes/"
                                                                                                     "car_seat/visual/"
                                                                                                     "seat.dae"),
                                                                             Eigen::Vector3d(1, 1, 1),
                                                                             true)[0];
    link_seat.visual.push_back(visual);

    for (int m = 1; m <= 10; ++m)
    {
      std::vector<tesseract_geometry::Mesh::Ptr> meshes =
          tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(
              locator->locateResource("package://tesseract_support/"
                                      "meshes/car_seat/collision/seat_" +
                                      std::to_string(m) + ".stl"));
      for (auto& mesh : meshes)
      {
        Collision::Ptr collision = std::make_shared<Collision>();
        collision->origin = visual->origin;
        collision->geometry = makeConvexMesh(*mesh);
        link_seat.collision.push_back(collision);
      }
    }

    Joint joint_seat("joint_seat_" + std::to_string(i + 1));
    joint_seat.parent_link_name = "world";
    joint_seat.child_link_name = link_seat.getName();
    joint_seat.type = JointType::FIXED;
    joint_seat.parent_to_joint_origin_transform = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                                                  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                                  Eigen::AngleAxisd(3.14159, Eigen::Vector3d::UnitZ());
    joint_seat.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0.5 + i, 2.15, 0.45);

    cmds.push_back(std::make_shared<tesseract_environment::AddLinkCommand>(link_seat, joint_seat));
  }
  return cmds;
}

CarSeatExample::CarSeatExample(std::shared_ptr<tesseract_environment::Environment> env,
                               std::shared_ptr<tesseract_visualization::Visualization> plotter,
                               bool ifopt,
                               bool debug)
  : Example(std::move(env), std::move(plotter)), ifopt_(ifopt), debug_(debug)
{
}

std::unordered_map<std::string, std::unordered_map<std::string, double>> getPredefinedPosition()
{
  std::unordered_map<std::string, std::unordered_map<std::string, double>> result;

  std::unordered_map<std::string, double> default_pos;
  default_pos["carriage_rail"] = 1.0;
  default_pos["joint_b"] = 0.0;
  default_pos["joint_e"] = 0.0;
  default_pos["joint_l"] = 0.0;
  default_pos["joint_r"] = 0.0;
  default_pos["joint_s"] = -1.5707;
  default_pos["joint_t"] = 0.0;
  default_pos["joint_u"] = -1.5707;
  result["Default"] = default_pos;

  std::unordered_map<std::string, double> pick1;
  pick1["carriage_rail"] = 2.22;
  pick1["joint_b"] = 0.39;
  pick1["joint_e"] = 0.0;
  pick1["joint_l"] = 0.5;
  pick1["joint_r"] = 0.0;
  pick1["joint_s"] = -3.14;
  pick1["joint_t"] = -0.29;
  pick1["joint_u"] = -1.45;
  result["Pick1"] = pick1;

  std::unordered_map<std::string, double> pick2;
  pick2["carriage_rail"] = 1.22;
  pick1["joint_b"] = 0.39;
  pick1["joint_e"] = 0.0;
  pick1["joint_l"] = 0.5;
  pick1["joint_r"] = 0.0;
  pick1["joint_s"] = -3.14;
  pick1["joint_t"] = -0.29;
  pick1["joint_u"] = -1.45;
  result["Pick2"] = pick2;

  std::unordered_map<std::string, double> pick3;
  pick3["carriage_rail"] = 0.22;
  pick1["joint_b"] = 0.39;
  pick1["joint_e"] = 0.0;
  pick1["joint_l"] = 0.5;
  pick1["joint_r"] = 0.0;
  pick1["joint_s"] = -3.14;
  pick1["joint_t"] = -0.29;
  pick1["joint_u"] = -1.45;
  result["Pick3"] = pick3;

  std::unordered_map<std::string, double> place1;
  place1["carriage_rail"] = 4.15466;
  place1["joint_b"] = 0.537218;
  place1["joint_e"] = 0.0189056;
  place1["joint_l"] = 0.801223;
  place1["joint_r"] = 0.0580309;
  place1["joint_s"] = -0.0481182;
  place1["joint_t"] = -0.325783;
  place1["joint_u"] = -1.2813;
  result["Place1"] = place1;

  std::unordered_map<std::string, double> home;
  home["carriage_rail"] = 0.0;
  home["joint_b"] = 0.0;
  home["joint_e"] = 0.0;
  home["joint_l"] = 0.0;
  home["joint_r"] = 0.0;
  home["joint_s"] = 0.0;
  home["joint_t"] = 0.0;
  home["joint_u"] = 0.0;
  result["Home"] = home;

  return result;
}

std::vector<double> getPositionVector(const JointGroup& joint_group, const std::unordered_map<std::string, double>& pos)
{
  std::vector<double> result;
  result.reserve(static_cast<std::size_t>(joint_group.numJoints()));
  for (const auto& joint_name : joint_group.getJointNames())
    result.push_back(pos.at(joint_name));

  return result;
}

Eigen::VectorXd getPositionVectorXd(const JointGroup& joint_group, const std::unordered_map<std::string, double>& pos)
{
  Eigen::VectorXd result;
  result.resize(joint_group.numJoints());
  int cnt = 0;
  for (const auto& joint_name : joint_group.getJointNames())
    result[cnt++] = pos.at(joint_name);

  return result;
}

bool CarSeatExample::run()
{
  if (debug_)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  // Create Task Composer Plugin Factory
  const std::string share_dir(TESSERACT_TASK_COMPOSER_DIR);
  tesseract_common::fs::path config_path(share_dir + "/config/task_composer_plugins.yaml");
  TaskComposerPluginFactory factory(config_path);

  // Get manipulator
  JointGroup::UPtr joint_group = env_->getJointGroup("manipulator");

  // Create seats and add it to the local environment
  Commands cmds = addSeats(env_->getResourceLocator());
  if (!env_->applyCommands(cmds))
    return false;

  if (plotter_ != nullptr)
    plotter_->waitForConnection();

  if (plotter_ != nullptr && plotter_->isConnected())
    plotter_->waitForInput();

  // Get predefined positions
  saved_positions_ = getPredefinedPosition();

  // Move to home position
  env_->setState(saved_positions_["Home"]);

  // Create Executor
  auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");

  // Create profile dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  if (ifopt_)
  {
    // Create TrajOpt_Ifopt Profile
    auto trajopt_ifopt_composite_profile = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
    trajopt_ifopt_composite_profile->collision_constraint_config->type =
        tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    trajopt_ifopt_composite_profile->collision_constraint_config->contact_manager_config =
        tesseract_collision::ContactManagerConfig(0.00);
    trajopt_ifopt_composite_profile->collision_constraint_config->collision_margin_buffer = 0.005;
    trajopt_ifopt_composite_profile->collision_constraint_config->collision_coeff_data =
        trajopt_common::CollisionCoeffData(1);
    trajopt_ifopt_composite_profile->collision_cost_config->type =
        tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
    trajopt_ifopt_composite_profile->collision_cost_config->contact_manager_config =
        tesseract_collision::ContactManagerConfig(0.005);
    trajopt_ifopt_composite_profile->collision_cost_config->collision_margin_buffer = 0.01;
    trajopt_ifopt_composite_profile->collision_cost_config->collision_coeff_data =
        trajopt_common::CollisionCoeffData(50);
    trajopt_ifopt_composite_profile->smooth_velocities = false;
    trajopt_ifopt_composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
    trajopt_ifopt_composite_profile->smooth_accelerations = true;
    trajopt_ifopt_composite_profile->acceleration_coeff = Eigen::VectorXd::Ones(1);
    trajopt_ifopt_composite_profile->smooth_jerks = true;
    trajopt_ifopt_composite_profile->jerk_coeff = Eigen::VectorXd::Ones(1);
    trajopt_ifopt_composite_profile->longest_valid_segment_length = 0.1;

    auto trajopt_ifopt_plan_profile = std::make_shared<TrajOptIfoptDefaultPlanProfile>();
    trajopt_ifopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Ones(6);
    trajopt_ifopt_plan_profile->joint_coeff = Eigen::VectorXd::Ones(8);

    auto trajopt_ifopt_solver_profile = std::make_shared<TrajOptIfoptDefaultSolverProfile>();
    trajopt_ifopt_solver_profile->opt_info.max_iterations = 200;
    trajopt_ifopt_solver_profile->opt_info.min_approx_improve = 1e-3;
    trajopt_ifopt_solver_profile->opt_info.min_trust_box_size = 1e-3;

    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_ifopt_composite_profile);
    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_ifopt_plan_profile);
    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_ifopt_solver_profile);
  }
  else
  {
    // Create TrajOpt Profile
    auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
    trajopt_composite_profile->collision_constraint_config.enabled = true;
    trajopt_composite_profile->collision_constraint_config.safety_margin = 0.00;
    trajopt_composite_profile->collision_constraint_config.safety_margin_buffer = 0.005;
    trajopt_composite_profile->collision_constraint_config.coeff = 10;
    trajopt_composite_profile->collision_cost_config.safety_margin = 0.005;
    trajopt_composite_profile->collision_cost_config.safety_margin_buffer = 0.01;
    trajopt_composite_profile->collision_cost_config.coeff = 50;

    auto trajopt_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();
    trajopt_solver_profile->opt_info.max_iter = 200;
    trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;
    trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;

    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_composite_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_solver_profile);
  }

  // Solve Trajectory
  CONSOLE_BRIDGE_logInform("Car Seat Demo Started");

  {  // Create Program to pick up first seat
    CompositeInstruction program(
        "FREESPACE", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "world", "end_effector"));
    program.setDescription("Pick up the first seat!");

    // Start and End Joint Position for the program
    Eigen::VectorXd start_pos = getPositionVectorXd(*joint_group, saved_positions_["Home"]);
    Eigen::VectorXd pick_pose = getPositionVectorXd(*joint_group, saved_positions_["Pick1"]);
    StateWaypointPoly wp0{ StateWaypoint(joint_group->getJointNames(), start_pos) };
    StateWaypointPoly wp1{ StateWaypoint(joint_group->getJointNames(), pick_pose) };

    // Start Joint Position for the program
    MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "FREESPACE");
    start_instruction.setDescription("Start Instruction");

    // Plan freespace from start
    MoveInstruction plan_f0(wp1, MoveInstructionType::FREESPACE, "FREESPACE");
    plan_f0.setDescription("Freespace pick seat 1");

    // Add Instructions to program
    program.appendMoveInstruction(start_instruction);
    program.appendMoveInstruction(plan_f0);

    // Print Diagnostics
    program.print("Program: ");

    CONSOLE_BRIDGE_logInform("Freespace plan to pick seat 1 example");

    // Create task
    const std::string task_name = (ifopt_) ? "TrajOptIfoptPipeline" : "TrajOptPipeline";
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

    if (!future->context->isSuccessful())
      return false;

    // Plot Process Trajectory
    if (plotter_ != nullptr && plotter_->isConnected())
    {
      auto ci = future->context->data_storage->getData(output_key).as<CompositeInstruction>();
      tesseract_common::Toolpath toolpath = toToolpath(ci, *env_);
      tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
      auto state_solver = env_->getStateSolver();
      plotter_->plotMarker(ToolpathMarker(toolpath));
      plotter_->plotTrajectory(trajectory, *state_solver);
      plotter_->waitForInput();
    }
  }

  // Get the state at the end of pick 1 trajectory
  SceneState state = env_->getState(saved_positions_["Pick1"]);

  // Now we to detach seat_1 and attach it to the robot end_effector
  Joint joint_seat_1_robot("joint_seat_1_robot");
  joint_seat_1_robot.parent_link_name = "end_effector";
  joint_seat_1_robot.child_link_name = "seat_1";
  joint_seat_1_robot.type = JointType::FIXED;
  joint_seat_1_robot.parent_to_joint_origin_transform =
      state.link_transforms.at("end_effector").inverse() * state.link_transforms.at("seat_1");

  cmds.clear();
  cmds.push_back(std::make_shared<MoveLinkCommand>(joint_seat_1_robot));

  tesseract_common::AllowedCollisionMatrix add_ac;
  add_ac.addAllowedCollision("seat_1", "end_effector", "Adjacent");
  add_ac.addAllowedCollision("seat_1", "cell_logo", "Never");
  add_ac.addAllowedCollision("seat_1", "fence", "Never");
  add_ac.addAllowedCollision("seat_1", "link_b", "Never");
  add_ac.addAllowedCollision("seat_1", "link_r", "Never");
  add_ac.addAllowedCollision("seat_1", "link_t", "Never");
  cmds.push_back(std::make_shared<tesseract_environment::ModifyAllowedCollisionsCommand>(
      add_ac, tesseract_environment::ModifyAllowedCollisionsType::ADD));

  // Apply the commands to the environment
  if (!env_->applyCommands(cmds))
    return false;

  {  // Create Program to place first seat
    CompositeInstruction program(
        "FREESPACE", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "world", "end_effector"));
    program.setDescription("Place the first seat!");

    // Start and End Joint Position for the program
    Eigen::VectorXd start_pos = getPositionVectorXd(*joint_group, saved_positions_["Pick1"]);
    Eigen::VectorXd pick_pose = getPositionVectorXd(*joint_group, saved_positions_["Place1"]);
    StateWaypointPoly wp0{ StateWaypoint(joint_group->getJointNames(), start_pos) };
    StateWaypointPoly wp1{ StateWaypoint(joint_group->getJointNames(), pick_pose) };

    // Start Joint Position for the program
    MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "FREESPACE");
    start_instruction.setDescription("Start Instruction");

    // Plan freespace from start
    MoveInstruction plan_f0(wp1, MoveInstructionType::FREESPACE, "FREESPACE");
    plan_f0.setDescription("Freespace pick seat 1");

    // Add Instructions to program
    program.appendMoveInstruction(start_instruction);
    program.appendMoveInstruction(plan_f0);

    // Print Diagnostics
    program.print("Program: ");

    CONSOLE_BRIDGE_logInform("Freespace plan to pick seat 1 example");

    // Create task
    const std::string task_name = (ifopt_) ? "TrajOptIfoptPipeline" : "TrajOptPipeline";
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

    if (!future->context->isSuccessful())
      return false;

    // Plot Process Trajectory
    if (plotter_ != nullptr && plotter_->isConnected())
    {
      auto ci = future->context->data_storage->getData(output_key).as<CompositeInstruction>();
      tesseract_common::Toolpath toolpath = toToolpath(ci, *env_);
      tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
      auto state_solver = env_->getStateSolver();
      plotter_->plotMarker(ToolpathMarker(toolpath));
      plotter_->plotTrajectory(trajectory, *state_solver);
      plotter_->waitForInput();
    }
  }

  return true;
}

}  // namespace tesseract_examples
