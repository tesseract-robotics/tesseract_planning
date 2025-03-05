/**
 * @file pick_and_place_example.cpp
 * @brief Pick and place implementation
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

#include <tesseract_examples/pick_and_place_example.h>

#include <tesseract_common/resource_locator.h>

#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>

#include <tesseract_state_solver/state_solver.h>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_environment/commands/change_collision_margins_command.h>
#include <tesseract_environment/commands/modify_allowed_collisions_command.h>
#include <tesseract_environment/commands/move_link_command.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tesseract_geometry/impl/box.h>

using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

const double OFFSET = 0.005;

const std::string LINK_BOX_NAME = "box";
const std::string LINK_BASE_NAME = "world";
const std::string LINK_END_EFFECTOR_NAME = "iiwa_tool0";
const std::string DISCRETE_CONTACT_CHECK_TASK_NAME = "DiscreteContactCheckTask";
static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
namespace tesseract_examples
{
PickAndPlaceExample::PickAndPlaceExample(std::shared_ptr<tesseract_environment::Environment> env,
                                         std::shared_ptr<tesseract_visualization::Visualization> plotter,
                                         bool ifopt,
                                         bool debug,
                                         double box_size,
                                         std::array<double, 2> box_position)
  : Example(std::move(env), std::move(plotter))
  , ifopt_(ifopt)
  , debug_(debug)
  , box_size_(box_size)
  , box_position_(box_position)
{
}

Command::Ptr addBox(double box_x, double box_y, double box_side)
{
  Link link_box(LINK_BOX_NAME);

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->geometry = std::make_shared<tesseract_geometry::Box>(box_side, box_side, box_side);
  link_box.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_box.collision.push_back(collision);

  Joint joint_box("joint_box");
  joint_box.parent_link_name = "workcell_base";
  joint_box.child_link_name = LINK_BOX_NAME;
  joint_box.type = JointType::FIXED;
  joint_box.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
  joint_box.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(box_x, box_y, (box_side / 2.0) + OFFSET);

  return std::make_shared<tesseract_environment::AddLinkCommand>(link_box, joint_box);
}

bool PickAndPlaceExample::run()
{
  /////////////
  /// SETUP ///
  /////////////

  if (debug_)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  // Set default contact distance
  Command::Ptr cmd_default_dist = std::make_shared<tesseract_environment::ChangeCollisionMarginsCommand>(0.005);
  if (!env_->applyCommand(cmd_default_dist))
    return false;

  if (plotter_ != nullptr)
    plotter_->waitForConnection();

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.emplace_back("iiwa_joint_a1");
  joint_names.emplace_back("iiwa_joint_a2");
  joint_names.emplace_back("iiwa_joint_a3");
  joint_names.emplace_back("iiwa_joint_a4");
  joint_names.emplace_back("iiwa_joint_a5");
  joint_names.emplace_back("iiwa_joint_a6");
  joint_names.emplace_back("iiwa_joint_a7");

  Eigen::VectorXd joint_pos(7);
  joint_pos(0) = 0.0;
  joint_pos(1) = 0.0;
  joint_pos(2) = 0.0;
  joint_pos(3) = -1.57;
  joint_pos(4) = 0.0;
  joint_pos(5) = 0.0;
  joint_pos(6) = 0.0;

  env_->setState(joint_names, joint_pos);

  // Add simulated box to environment
  Command::Ptr cmd = addBox(box_position_[0], box_position_[1], box_size_);
  if (!env_->applyCommand(cmd))
    return false;

  ////////////
  /// PICK ///
  ////////////
  if (plotter_ != nullptr)
    plotter_->waitForInput();

  // Create Task Composer Plugin Factory
  std::shared_ptr<const tesseract_common::ResourceLocator> locator = env_->getResourceLocator();
  std::filesystem::path config_path(
      locator->locateResource("package://tesseract_task_composer/config/task_composer_plugins.yaml")->getFilePath());
  TaskComposerPluginFactory factory(config_path, *env_->getResourceLocator());

  // Create Program
  CompositeInstruction pick_program("DEFAULT", ManipulatorInfo("manipulator", LINK_BASE_NAME, LINK_END_EFFECTOR_NAME));

  StateWaypoint pick_swp{ joint_names, joint_pos };
  MoveInstruction start_instruction(pick_swp, MoveInstructionType::FREESPACE, "FREESPACE");
  start_instruction.setDescription("Start Instruction");

  // Define the final pose (on top of the box)
  Eigen::Isometry3d pick_final_pose;
  pick_final_pose.linear() = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0).matrix();
  pick_final_pose.translation() =
      Eigen::Vector3d(box_position_[0], box_position_[1], box_size_ + 0.772 + OFFSET);  // Offset for the table
  CartesianWaypoint pick_wp1{ pick_final_pose };

  // Define the approach pose
  Eigen::Isometry3d pick_approach_pose = pick_final_pose;
  pick_approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.15);
  CartesianWaypoint pick_wp0{ pick_approach_pose };

  // Plan freespace from start
  MoveInstruction pick_plan_a0(pick_wp0, MoveInstructionType::FREESPACE, "FREESPACE");
  pick_plan_a0.setDescription("From start to pick Approach");

  // Plan cartesian approach
  MoveInstruction pick_plan_a1(pick_wp1, MoveInstructionType::LINEAR, "CARTESIAN");
  pick_plan_a1.setDescription("Pick Approach");

  // Add Instructions to program
  pick_program.push_back(start_instruction);
  pick_program.push_back(pick_plan_a0);
  pick_program.push_back(pick_plan_a1);

  // Print Diagnostics
  pick_program.print("Program: ");

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
    trajopt_ifopt_plan_profile->cartesian_constraint_config.coeff = Eigen::VectorXd::Constant(6, 1, 10);

    auto trajopt_ifopt_composite_profile = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
    trajopt_ifopt_composite_profile->collision_constraint_config->type =
        tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    trajopt_ifopt_composite_profile->collision_constraint_config->contact_manager_config =
        tesseract_collision::ContactManagerConfig(0.00);
    trajopt_ifopt_composite_profile->collision_constraint_config->collision_margin_buffer = 0.005;
    trajopt_ifopt_composite_profile->collision_constraint_config->collision_coeff_data =
        trajopt_common::CollisionCoeffData(10);
    trajopt_ifopt_composite_profile->collision_cost_config->type =
        tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    trajopt_ifopt_composite_profile->collision_cost_config->contact_manager_config =
        tesseract_collision::ContactManagerConfig(0.005);
    trajopt_ifopt_composite_profile->collision_cost_config->collision_margin_buffer = 0.01;
    trajopt_ifopt_composite_profile->collision_cost_config->collision_coeff_data =
        trajopt_common::CollisionCoeffData(50);
    trajopt_ifopt_composite_profile->smooth_velocities = true;
    trajopt_ifopt_composite_profile->velocity_coeff = 0.1 * Eigen::VectorXd::Ones(1);
    trajopt_ifopt_composite_profile->smooth_accelerations = true;
    trajopt_ifopt_composite_profile->acceleration_coeff = Eigen::VectorXd::Ones(1);
    trajopt_ifopt_composite_profile->smooth_jerks = true;
    trajopt_ifopt_composite_profile->jerk_coeff = Eigen::VectorXd::Ones(1);
    trajopt_ifopt_composite_profile->longest_valid_segment_length = 0.05;

    auto trajopt_ifopt_solver_profile = std::make_shared<TrajOptIfoptOSQPSolverProfile>();
    trajopt_ifopt_solver_profile->opt_params.max_iterations = 100;

    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_ifopt_plan_profile);
    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_ifopt_composite_profile);
    profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_ifopt_solver_profile);
  }
  else
  {
    // Create TrajOpt Profile
    auto trajopt_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
    trajopt_plan_profile->joint_cost_config.enabled = false;
    trajopt_plan_profile->cartesian_cost_config.enabled = false;
    trajopt_plan_profile->cartesian_constraint_config.enabled = true;
    trajopt_plan_profile->cartesian_constraint_config.coeff = Eigen::VectorXd::Constant(6, 1, 10);

    auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
    trajopt_composite_profile->longest_valid_segment_length = 0.05;
    trajopt_composite_profile->collision_constraint_config.enabled = true;
    trajopt_composite_profile->collision_constraint_config.safety_margin = 0.0;
    trajopt_composite_profile->collision_constraint_config.safety_margin_buffer = 0.005;
    trajopt_composite_profile->collision_constraint_config.coeff = 10;
    trajopt_composite_profile->collision_cost_config.safety_margin = 0.005;
    trajopt_composite_profile->collision_cost_config.safety_margin_buffer = 0.01;
    trajopt_composite_profile->collision_cost_config.coeff = 50;

    auto trajopt_solver_profile = std::make_shared<TrajOptOSQPSolverProfile>();
    trajopt_solver_profile->opt_params.max_iter = 100;

    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_plan_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile);
    profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_solver_profile);
  }

  auto post_check_profile = std::make_shared<ContactCheckProfile>();

  profiles->addProfile(DISCRETE_CONTACT_CHECK_TASK_NAME, "CARTESIAN", post_check_profile);
  profiles->addProfile(DISCRETE_CONTACT_CHECK_TASK_NAME, "DEFAULT", post_check_profile);

  CONSOLE_BRIDGE_logInform("Pick plan");

  // Create task
  const std::string task_name = (ifopt_) ? "TrajOptIfoptPipeline" : "TrajOptPipeline";
  TaskComposerNode::UPtr pick_task = factory.createTaskComposerNode(task_name);
  const std::string pick_output_key = pick_task->getOutputKeys().get("program");

  // Create Task Composer Data Storage
  auto pick_data = std::make_unique<tesseract_planning::TaskComposerDataStorage>();
  pick_data->setData("planning_input", pick_program);
  pick_data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
  pick_data->setData("profiles", profiles);

  // Solve task
  TaskComposerFuture::UPtr pick_future = executor->run(*pick_task, std::move(pick_data));
  pick_future->wait();

  if (!pick_future->context->isSuccessful())
    return false;

  // Plot Process Trajectory
  if (plotter_ != nullptr && plotter_->isConnected())
  {
    plotter_->waitForInput();
    auto ci = pick_future->context->data_storage->getData(pick_output_key).as<CompositeInstruction>();
    tesseract_common::Toolpath toolpath = toToolpath(ci, *env_);
    tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
    auto state_solver = env_->getStateSolver();
    plotter_->plotMarker(ToolpathMarker(toolpath));
    plotter_->plotTrajectory(trajectory, *state_solver);
  }

  /////////////
  /// PLACE ///
  /////////////

  if (plotter_ != nullptr)
    plotter_->waitForInput();

  // Detach the simulated box from the world and attach to the end effector
  tesseract_environment::Commands cmds;
  Joint joint_box2("joint_box2");
  joint_box2.parent_link_name = LINK_END_EFFECTOR_NAME;
  joint_box2.child_link_name = LINK_BOX_NAME;
  joint_box2.type = JointType::FIXED;
  joint_box2.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
  joint_box2.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(0, 0, box_size_ / 2.0);
  cmds.push_back(std::make_shared<tesseract_environment::MoveLinkCommand>(joint_box2));
  tesseract_common::AllowedCollisionMatrix add_ac;
  add_ac.addAllowedCollision(LINK_BOX_NAME, LINK_END_EFFECTOR_NAME, "Never");
  add_ac.addAllowedCollision(LINK_BOX_NAME, "iiwa_link_7", "Never");
  add_ac.addAllowedCollision(LINK_BOX_NAME, "iiwa_link_6", "Never");
  cmds.push_back(std::make_shared<tesseract_environment::ModifyAllowedCollisionsCommand>(
      add_ac, tesseract_environment::ModifyAllowedCollisionsType::ADD));
  env_->applyCommands(cmds);

  // Get the last move instruction
  CompositeInstruction pick_composite =
      pick_future->context->data_storage->getData(pick_output_key).as<CompositeInstruction>();
  const MoveInstructionPoly* pick_final_state = pick_composite.getLastMoveInstruction();

  // Retreat to the approach pose
  Eigen::Isometry3d retreat_pose = pick_approach_pose;

  // Define some place locations.
  Eigen::Isometry3d bottom_right_shelf, bottom_left_shelf, middle_right_shelf, middle_left_shelf, top_right_shelf,
      top_left_shelf;
  bottom_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  bottom_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 0.906);
  bottom_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  bottom_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 0.906);
  middle_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  middle_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 1.16);
  middle_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  middle_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 1.16);
  top_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  top_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 1.414);
  top_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  top_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 1.414);

  // Set the target pose to middle_left_shelf
  Eigen::Isometry3d place_pose = middle_left_shelf;

  // Setup approach for place move
  Eigen::Isometry3d place_approach_pose = place_pose;
  place_approach_pose.translation() += Eigen::Vector3d(0.0, -0.25, 0);

  // Create Program
  CompositeInstruction place_program("DEFAULT", ManipulatorInfo("manipulator", LINK_BASE_NAME, LINK_END_EFFECTOR_NAME));

  // Define the approach pose
  CartesianWaypoint place_wp0{ retreat_pose };

  // Define the final pose approach
  CartesianWaypoint place_wp1{ place_approach_pose };

  // Define the final pose
  CartesianWaypoint place_wp2{ place_pose };

  // Plan cartesian retraction from picking up the box
  MoveInstruction place_plan_a0(place_wp0, MoveInstructionType::LINEAR, "CARTESIAN");
  place_plan_a0.setDescription("Place retraction");

  // Plan freespace to approach for box drop off
  MoveInstruction place_plan_a1(place_wp1, MoveInstructionType::FREESPACE, "FREESPACE");
  place_plan_a1.setDescription("Place Freespace");

  // Plan cartesian approach to box drop location
  MoveInstruction place_plan_a2(place_wp2, MoveInstructionType::LINEAR, "CARTESIAN");
  place_plan_a2.setDescription("Place approach");

  // Add Instructions to program
  place_program.push_back(*pick_final_state);
  place_program.push_back(place_plan_a0);
  place_program.push_back(place_plan_a1);
  place_program.push_back(place_plan_a2);

  // Print Diagnostics
  place_program.print("Program: ");

  // Create task
  TaskComposerNode::UPtr place_task = factory.createTaskComposerNode(task_name);
  const std::string place_output_key = pick_task->getOutputKeys().get("program");

  // Create Task Composer Data Storage
  auto place_data = std::make_unique<tesseract_planning::TaskComposerDataStorage>();
  place_data->setData("planning_input", place_program);
  place_data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
  place_data->setData("profiles", profiles);

  // Solve task
  TaskComposerFuture::UPtr place_future = executor->run(*place_task, std::move(place_data));
  place_future->wait();

  if (!place_future->context->isSuccessful())
    return false;

  // Plot Process Trajectory
  if (plotter_ != nullptr && plotter_->isConnected())
  {
    plotter_->waitForInput();
    auto ci = place_future->context->data_storage->getData(place_output_key).as<CompositeInstruction>();
    tesseract_common::Toolpath toolpath = toToolpath(ci, *env_);
    tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
    auto state_solver = env_->getStateSolver();
    plotter_->plotMarker(ToolpathMarker(toolpath));
    plotter_->plotTrajectory(trajectory, *state_solver);
  }

  if (plotter_ != nullptr)
    plotter_->waitForInput();

  CONSOLE_BRIDGE_logInform("Done");
  return true;
}
}  // namespace tesseract_examples
