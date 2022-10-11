/**
 * @file raster_example.cpp
 * @brief Raster motion planning example
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
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

#include <tesseract_kinematics/core/utils.h>

#include <tesseract_environment/environment.h>

#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/interface_utils.h>

#include <tesseract_visualization/visualization_loader.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

using namespace tesseract_planning;

const static std::string DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask";
const static std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";

int main(int /*argc*/, char** /*argv*/)
{
  try
  {
    // Setup
    auto locator = std::make_shared<tesseract_common::TesseractSupportResourceLocator>();
    auto env = std::make_shared<tesseract_environment::Environment>();
    tesseract_common::fs::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
    tesseract_common::fs::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
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

    auto kin_group = env->getKinematicGroup(manip.manipulator, manip.manipulator_ik_solver);
    auto cur_state = env->getState();

    CompositeInstruction program(DEFAULT_PROFILE_KEY, CompositeInstructionOrder::ORDERED, manip);

    // Start Joint Position for the program
    StateWaypointPoly wp0{ StateWaypoint(kin_group->getJointNames(), Eigen::VectorXd::Zero(6)) };
    MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE);
    start_instruction.setDescription("Start");

    // Define raster poses
    CartesianWaypointPoly wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.2, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.1, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.0, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.1, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.2, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.3, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));

    // Define raster move instruction
    MoveInstruction plan_c0(wp2, MoveInstructionType::LINEAR);
    MoveInstruction plan_c1(wp3, MoveInstructionType::LINEAR);
    MoveInstruction plan_c2(wp4, MoveInstructionType::LINEAR);
    MoveInstruction plan_c3(wp5, MoveInstructionType::LINEAR);
    MoveInstruction plan_c4(wp6, MoveInstructionType::LINEAR);
    MoveInstruction plan_c5(wp7, MoveInstructionType::LINEAR);

    MoveInstruction plan_f0(wp1, MoveInstructionType::FREESPACE);
    plan_f0.setDescription("from_start_plan");
    CompositeInstruction from_start;
    from_start.setDescription("from_start");
    from_start.appendMoveInstruction(start_instruction);
    from_start.appendMoveInstruction(plan_f0);
    program.push_back(from_start);

    {
      CompositeInstruction raster_segment;
      raster_segment.setDescription("raster_segment");
      raster_segment.appendMoveInstruction(plan_c0);
      raster_segment.appendMoveInstruction(plan_c1);
      raster_segment.appendMoveInstruction(plan_c2);
      raster_segment.appendMoveInstruction(plan_c3);
      raster_segment.appendMoveInstruction(plan_c4);
      raster_segment.appendMoveInstruction(plan_c5);
      program.push_back(raster_segment);
    }

    {
      MoveInstruction plan_f1(wp1, MoveInstructionType::FREESPACE);
      plan_f1.setDescription("transition_from_end_plan");
      CompositeInstruction transition_from_end;
      transition_from_end.setDescription("transition_from_end");
      transition_from_end.appendMoveInstruction(plan_f1);
      CompositeInstruction transition_from_start;
      transition_from_start.setDescription("transition_from_start");
      transition_from_start.appendMoveInstruction(plan_f1);

      CompositeInstruction transitions("DEFAULT", CompositeInstructionOrder::UNORDERED);
      transitions.setDescription("transitions");
      transitions.push_back(transition_from_start);
      transitions.push_back(transition_from_end);
      program.push_back(transitions);
    }

    {
      CompositeInstruction raster_segment;
      raster_segment.setDescription("raster_segment");
      raster_segment.appendMoveInstruction(plan_c0);
      raster_segment.appendMoveInstruction(plan_c1);
      raster_segment.appendMoveInstruction(plan_c2);
      raster_segment.appendMoveInstruction(plan_c3);
      raster_segment.appendMoveInstruction(plan_c4);
      raster_segment.appendMoveInstruction(plan_c5);
      program.push_back(raster_segment);
    }

    {
      MoveInstruction plan_f1(wp1, MoveInstructionType::FREESPACE);
      plan_f1.setDescription("transition_from_end_plan");
      CompositeInstruction transition_from_end;
      transition_from_end.setDescription("transition_from_end");
      transition_from_end.appendMoveInstruction(plan_f1);
      CompositeInstruction transition_from_start;
      transition_from_start.setDescription("transition_from_start");
      transition_from_start.appendMoveInstruction(plan_f1);

      CompositeInstruction transitions("DEFAULT", CompositeInstructionOrder::UNORDERED);
      transitions.setDescription("transitions");
      transitions.push_back(transition_from_start);
      transitions.push_back(transition_from_end);
      program.push_back(transitions);
    }

    {
      CompositeInstruction raster_segment;
      raster_segment.setDescription("raster_segment");
      raster_segment.appendMoveInstruction(plan_c0);
      raster_segment.appendMoveInstruction(plan_c1);
      raster_segment.appendMoveInstruction(plan_c2);
      raster_segment.appendMoveInstruction(plan_c3);
      raster_segment.appendMoveInstruction(plan_c4);
      raster_segment.appendMoveInstruction(plan_c5);
      program.push_back(raster_segment);
    }

    MoveInstruction plan_f2(wp1, MoveInstructionType::FREESPACE);
    plan_f2.setDescription("to_end_plan");
    CompositeInstruction to_end;
    to_end.setDescription("to_end");
    to_end.appendMoveInstruction(plan_f2);
    program.push_back(to_end);

    // Plot Program
    auto state_solver = env->getStateSolver();
    if (plotter)
    {
    }

    // Create Profiles
    auto descartes_plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();
    auto trajopt_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
    auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
    auto trajopt_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();

    // Create a interpolated program
    CompositeInstruction interpolated_program = generateInterpolatedProgram(program, cur_state, env);

    // Profile Dictionary
    auto profiles = std::make_shared<ProfileDictionary>();
    profiles->addProfile<DescartesPlanProfile<double>>(
        DESCARTES_DEFAULT_NAMESPACE, DEFAULT_PROFILE_KEY, descartes_plan_profile);
    profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, DEFAULT_PROFILE_KEY, trajopt_plan_profile);
    profiles->addProfile<TrajOptCompositeProfile>(
        TRAJOPT_DEFAULT_NAMESPACE, DEFAULT_PROFILE_KEY, trajopt_composite_profile);
    profiles->addProfile<TrajOptSolverProfile>(TRAJOPT_DEFAULT_NAMESPACE, DEFAULT_PROFILE_KEY, trajopt_solver_profile);

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

    //  // *************************************
    //  // Create Motion Plan for home to raster
    //  // *************************************

    //  // Get raster solution last position
    //  JointWaypoint last_position;  // TODO: Get last move instruction position

    //  // Update last plan instruction
    //  p1.setEndWaypoint(last_position);

    //  // Create a seed
    //  CompositeInstruction p1_seed = generateSeed(p1, cur_state, fwd_kin, inv_kin);

    //  // Use planners

    //  // *************************************
    //  // Create Motion Plan for raster to home
    //  // *************************************

    //  // Get raster solution last position
    //  JointWaypoint first_position;  // TODO: Get first waypoint

    //  // Update end waypoint
    //  p1.setStartWaypoint(first_position);

    //  // Create a seed
    //  CompositeInstruction p2_seed = generateSeed(p2, cur_state, fwd_kin, inv_kin);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Example failed with message: %s", e.what());
    return -1;
  }
}
