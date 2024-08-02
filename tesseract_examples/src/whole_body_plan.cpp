#include <iostream>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/whole_body_plan.h>


#include <tesseract_common/timer.h>


#include <trajopt_common/collision_types.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>

#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_kinematics/core/kinematics_plugin_factory.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/validate.h>

#include <tesseract_state_solver/state_solver.h>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_environment/commands/add_kinematics_information_command.h>
#include <tesseract_collision/core/types.h>


#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_command_language/profile_dictionary.h>

#include <tesseract_visualization/markers/toolpath_marker.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/visualization_loader.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_solver_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>

#include <tesseract_support/tesseract_support_resource_locator.h>
#include <tesseract_geometry/impl/sphere.h>

#include <tesseract_srdf/kinematics_information.h>

using namespace tesseract_planning;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace trajopt;


using tesseract_common::ManipulatorInfo;

static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
const static std::string OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask";

namespace tesseract_examples
{

WholeBodyPlan::WholeBodyPlan(std::shared_ptr<tesseract_environment::Environment> env,
                                             std::shared_ptr<tesseract_visualization::Visualization> plotter,
                                             bool ifopt,
                                             bool debug)
  : Example(std::move(env), std::move(plotter)), ifopt_(ifopt), debug_(debug)
{
    tesseract_common::PluginInfo pi;
    tesseract_srdf::ChainGroup group;
    tesseract_srdf::KinematicsInformation kin_info;
    std::string group_id = "manipulator";
    pi.class_name = "KDLInvKinChainLMAFactory";
    pi.config["base_link"] = "world";
    pi.config["tip_link"] = "ee_link";
    group.push_back(std::make_pair("world", "ee_link"));
    kin_info.kinematics_plugin_info.inv_plugin_infos[group_id].plugins = {
    std::make_pair("KDLFInvKinLMA", pi)};
    kin_info.addChainGroup(group_id, group);
    auto cmd = std::make_shared<AddKinematicsInformationCommand>(kin_info);
    env_->applyCommand(cmd);
    }

Command::Ptr addSphere1()
{
  // Add sphere to environment
  Link link_sphere("sphere_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(1.5, 0, 0.55);
  visual->geometry = std::make_shared<tesseract_geometry::Sphere>(0.45);
  link_sphere.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_sphere.collision.push_back(collision);

  Joint joint_sphere("joint_sphere_attached");
  joint_sphere.parent_link_name = "world";
  joint_sphere.child_link_name = link_sphere.getName();
  joint_sphere.type = JointType::FIXED;

  return std::make_shared<tesseract_environment::AddLinkCommand>(link_sphere, joint_sphere);
}

bool WholeBodyPlan::run()
{

  // tesseract_visualization::VisualizationLoader loader;
  // auto plotter_ = loader.get();
  if (plotter_ != nullptr)
    plotter_->waitForConnection();

  Command::Ptr cmd = addSphere1();
  if (!env_->applyCommand(cmd))
    return false;

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.emplace_back("base_y_base_x");
  joint_names.emplace_back("base_theta_base_y");
  joint_names.emplace_back("base_link_base_theta");
  joint_names.emplace_back("shoulder_pan_joint");
  joint_names.emplace_back("shoulder_lift_joint");
  joint_names.emplace_back("elbow_joint");
  joint_names.emplace_back("wrist_1_joint");
  joint_names.emplace_back("wrist_2_joint");
  joint_names.emplace_back("wrist_3_joint");

  Eigen::VectorXd joint_start_pos(9);
  joint_start_pos(0) = 0;
  joint_start_pos(1) = 0;
  joint_start_pos(2) = 0.0;
  joint_start_pos(3) = 0.25037;
  joint_start_pos(4) = 0.6096;
  joint_start_pos(5) = -0.0425;
  joint_start_pos(6) = 1.061965;
  joint_start_pos(7) = 1.321897;
  joint_start_pos(8) = -0.000931;

  Eigen::VectorXd joint_end_pos(9);
  joint_end_pos(0) = 3;
  joint_end_pos(1) = 0;
  joint_end_pos(2) = 0.0;
  joint_end_pos(3) = 0.06;
  joint_end_pos(4) = 1.26;
  joint_end_pos(5) = 0.0;
  joint_end_pos(6) = -1.26;
  joint_end_pos(7) = 1.63;
  joint_end_pos(8) = 0;
  std::cout << "+++++++++++++++++++++++++++++++" <<std::endl;
  auto s = env_->getGroupJointNames("manipulator");
  for(auto ss:s) {
    std::cout<< ss << "         ";
  }
  env_->setState(joint_names, joint_start_pos);

  if (debug_)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  tesseract_common::ManipulatorInfo manip;
  manip.tcp_frame = "ee_link";
  manip.working_frame = "world";
  manip.manipulator = "manipulator";



  auto state_solver = env_->getStateSolver();
  //auto kin_group = env_->getKinematicGroup(manip.manipulator, manip.manipulator_ik_solver);
  auto cur_state = env_->getState();

  
  StateWaypointPoly wp0{ StateWaypoint(joint_names, joint_start_pos) };
  StateWaypointPoly wpend{ StateWaypoint(joint_names, joint_end_pos) };

  MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "DEFAULT");
  MoveInstruction plan_f1(wpend, MoveInstructionType::FREESPACE, "DEFAULT");

      // Create program
  CompositeInstruction program;
  program.setManipulatorInfo(manip); 
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f1);

  // Print Diagnostics
  program.print("Program: ");

  CONSOLE_BRIDGE_logInform("whole body plan example");

   // Create Profiles
    auto ompl_plan_profile = std::make_shared<OMPLDefaultPlanProfile>();
    auto ompl_planner_config = std::make_shared<RRTConnectConfigurator>();
    ompl_plan_profile->planning_time = 10;
    ompl_plan_profile->planners = { ompl_planner_config, ompl_planner_config };

    auto trajopt_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
    auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
    trajopt_composite_profile->collision_constraint_config.enabled = true;
    trajopt_composite_profile->collision_constraint_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
    trajopt_composite_profile->collision_constraint_config.safety_margin = 0.001;
    trajopt_composite_profile->collision_constraint_config.coeff = 10;
    trajopt_composite_profile->collision_cost_config.enabled = false;
    trajopt_composite_profile->velocity_coeff.setConstant(9, 5);
    trajopt_composite_profile->acceleration_coeff.setConstant(9, 5);

    trajopt_composite_profile->smooth_accelerations = true;
    trajopt_composite_profile->smooth_velocities = true;
    trajopt_composite_profile->smooth_jerks = false;
    
    auto trajopt_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();
    trajopt_solver_profile->opt_info.max_iter = 100;


    // Create a interpolated program
    CompositeInstruction interpolated_program = generateInterpolatedProgram(program, cur_state, env_);

    // Profile Dictionary
    auto profiles = std::make_shared<ProfileDictionary>();
    profiles->addProfile<OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, "DEFAULT", ompl_plan_profile);
    profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_plan_profile);
    profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile);
    profiles->addProfile<TrajOptSolverProfile>(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_solver_profile);

    // Create Planning Request
    PlannerRequest request;
    request.instructions = interpolated_program;
    request.env = env_;
    request.env_state = cur_state;
    request.profiles = profiles;

    if (plotter_ != nullptr && plotter_->isConnected())
      plotter_->waitForInput("Hit Enter to solve for trajectory.");    

    tesseract_common::Timer stopwatch;
    stopwatch.start();
    // Solve OMPL Plan
    OMPLMotionPlanner ompl_planner(OMPL_DEFAULT_NAMESPACE);
    PlannerResponse ompl_response = ompl_planner.solve(request);
    assert(ompl_response);
    
    stopwatch.stop();
    CONSOLE_BRIDGE_logInform("ompl Planning took %f seconds.", stopwatch.elapsedSeconds());

    // Plot OMPL Trajectory
    if (plotter_)
    {
      plotter_->waitForInput();
      tesseract_common::JointTrajectory trajectory = toJointTrajectory(ompl_response.results);
      plotter_->plotTrajectory(trajectory, *state_solver);
    }

    if (plotter_ != nullptr && plotter_->isConnected())
      plotter_->waitForInput("Hit Enter to solve for trajectory.");    

      // Plot Process Trajectory
    // Update Seed
    request.instructions = ompl_response.results;
    
    stopwatch.start();
    // Solve TrajOpt Plan
    TrajOptMotionPlanner trajopt_planner(TRAJOPT_DEFAULT_NAMESPACE);
    PlannerResponse trajopt_response = trajopt_planner.solve(request);
    assert(trajopt_response);
    stopwatch.stop();
    CONSOLE_BRIDGE_logInform("trajopt Planning took %f seconds.", stopwatch.elapsedSeconds());
    if (plotter_)
    {
      plotter_->waitForInput();
      plotter_->plotTrajectory(toJointTrajectory(trajopt_response.results), *state_solver);
    }


    CONSOLE_BRIDGE_logInform("Final trajectory is collision free");
    return true;
  }


}  // namespace tesseract_examples
