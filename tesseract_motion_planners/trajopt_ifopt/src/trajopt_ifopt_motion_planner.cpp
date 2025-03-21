/**
 * @file trajopt_planner.cpp
 * @brief Tesseract ROS Trajopt planner
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/planner_utils.h>

#include <tesseract_common/joint_state.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/utils.h>

constexpr auto SOLUTION_FOUND{ "Found valid solution" };
constexpr auto ERROR_INVALID_INPUT{ "Failed invalid input: " };
constexpr auto ERROR_FAILED_TO_FIND_VALID_SOLUTION{ "Failed to find valid solution" };

namespace tesseract_planning
{
TrajOptIfoptMotionPlanner::TrajOptIfoptMotionPlanner(std::string name) : MotionPlanner(std::move(name)) {}

bool TrajOptIfoptMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

void TrajOptIfoptMotionPlanner::clear() {}

std::unique_ptr<MotionPlanner> TrajOptIfoptMotionPlanner::clone() const
{
  return std::make_unique<TrajOptIfoptMotionPlanner>(name_);
}

PlannerResponse TrajOptIfoptMotionPlanner::solve(const PlannerRequest& request) const
{
  PlannerResponse response;
  std::string reason;
  if (!checkRequest(request, reason))
  {
    response.successful = false;
    response.message = std::string(ERROR_INVALID_INPUT) + reason;
    return response;
  }

  // Get composite manip info
  const tesseract_common::ManipulatorInfo& composite_mi = request.instructions.getManipulatorInfo();
  if (composite_mi.empty())
    throw std::runtime_error("TrajoptIfoptMotionPlanner, manipulator info is empty!");

  std::shared_ptr<trajopt_sqp::TrajOptQPProblem> nlp = std::make_shared<trajopt_sqp::TrajOptQPProblem>();

  // Flatten the input for planning
  auto move_instructions = request.instructions.flatten(&moveFilter);

  // Store fixed steps
  std::vector<int> fixed_steps;

  // ----------------
  // Translate TCL for MoveInstructions
  // ----------------
  // Transform plan instructions into trajopt cost and constraints
  std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>> vars;
  for (int i = 0; i < static_cast<int>(move_instructions.size()); ++i)
  {
    const auto& move_instruction = move_instructions[static_cast<std::size_t>(i)].get().as<MoveInstructionPoly>();

    // Get Plan Profile
    TrajOptIfoptPlanProfile::ConstPtr cur_plan_profile =
        getProfile<TrajOptIfoptPlanProfile>(name_,
                                            move_instruction.getProfile(name_),
                                            *request.profiles,
                                            std::make_shared<TrajOptIfoptDefaultPlanProfile>());
    if (!cur_plan_profile)
      throw std::runtime_error("TrajOptIfoptMotionPlanner: Invalid profile");

    // Create waypoint info
    TrajOptIfoptWaypointInfo wp_info = cur_plan_profile->create(move_instruction, composite_mi, request.env, i);

    // Update fixed steps
    if (wp_info.fixed)
      fixed_steps.push_back(i);

    // Add variable set
    vars.push_back(wp_info.var);
    nlp->addVariableSet(wp_info.var);

    // Add Waypoint Cost and Constraints
    for (const auto& cnt : wp_info.term_infos.constraints)
      nlp->addConstraintSet(cnt);

    for (const auto& absolute_cost : wp_info.term_infos.absolute_costs)
      nlp->addCostSet(absolute_cost, trajopt_sqp::CostPenaltyType::ABSOLUTE);

    for (const auto& squared_cost : wp_info.term_infos.squared_costs)
      nlp->addCostSet(squared_cost, trajopt_sqp::CostPenaltyType::SQUARED);

    for (const auto& hinge_cost : wp_info.term_infos.hinge_costs)
      nlp->addCostSet(hinge_cost, trajopt_sqp::CostPenaltyType::HINGE);
  }

  // ----------------
  // Translate TCL for CompositeInstructions
  // ----------------
  TrajOptIfoptCompositeProfile::ConstPtr cur_composite_profile =
      getProfile<TrajOptIfoptCompositeProfile>(name_,
                                               request.instructions.getProfile(name_),
                                               *request.profiles,
                                               std::make_shared<TrajOptIfoptDefaultCompositeProfile>());

  if (!cur_composite_profile)
    throw std::runtime_error("TrajOptIfoptMotionPlanner: Invalid profile");

  TrajOptIfoptTermInfos term_infos = cur_composite_profile->create(composite_mi, request.env, vars, fixed_steps);

  // Add Waypoint Cost and Constraints
  for (const auto& cnt : term_infos.constraints)
    nlp->addConstraintSet(cnt);

  for (const auto& absolute_cost : term_infos.absolute_costs)
    nlp->addCostSet(absolute_cost, trajopt_sqp::CostPenaltyType::ABSOLUTE);

  for (const auto& squared_cost : term_infos.squared_costs)
    nlp->addCostSet(squared_cost, trajopt_sqp::CostPenaltyType::SQUARED);

  for (const auto& hinge_cost : term_infos.hinge_costs)
    nlp->addCostSet(hinge_cost, trajopt_sqp::CostPenaltyType::HINGE);

  // Setup
  nlp->setup();

  // Create Solver
  TrajOptIfoptSolverProfile::ConstPtr solver_profile =
      getProfile<TrajOptIfoptSolverProfile>(name_,
                                            request.instructions.getProfile(name_),
                                            *request.profiles,
                                            std::make_shared<TrajOptIfoptOSQPSolverProfile>());

  if (!solver_profile)
    throw std::runtime_error("TrajOptIfoptMotionPlanner: Invalid profile");

  std::unique_ptr<trajopt_sqp::TrustRegionSQPSolver> solver = solver_profile->create(request.verbose);

  // Solver
  solver->solve(nlp);

  // Check success
  if (solver->getStatus() != trajopt_sqp::SQPStatus::NLP_CONVERGED)
  {
    response.successful = false;
    response.message = ERROR_FAILED_TO_FIND_VALID_SOLUTION;
    return response;
  }

  auto manip = request.env->getJointGroup(composite_mi.manipulator);
  const std::vector<std::string> joint_names = manip->getJointNames();
  const Eigen::MatrixX2d joint_limits = manip->getLimits().joint_limits;

  // Get the results - This can likely be simplified if we get rid of the traj array
  Eigen::VectorXd x = nlp->getVariableValues();
  Eigen::Map<tesseract_common::TrajArray> traj(
      x.data(), static_cast<Eigen::Index>(vars.size()), static_cast<Eigen::Index>(vars[0]->GetValues().size()));

  // Enforce limits
  for (Eigen::Index i = 0; i < traj.rows(); i++)
  {
    assert(tesseract_common::satisfiesLimits<double>(traj.row(i), joint_limits, 1e-4));
    tesseract_common::enforceLimits<double>(traj.row(i), joint_limits);
  }

  // Flatten the results to make them easier to process
  response.results = request.instructions;
  auto results_instructions = response.results.flatten(&moveFilter);
  assert(results_instructions.size() == traj.rows());
  for (std::size_t idx = 0; idx < results_instructions.size(); idx++)
  {
    auto& move_instruction = results_instructions.at(idx).get().as<MoveInstructionPoly>();
    assignSolution(
        move_instruction, joint_names, traj.row(static_cast<Eigen::Index>(idx)), request.format_result_as_input);
  }

  response.successful = true;
  response.message = SOLUTION_FOUND;
  return response;
}

}  // namespace tesseract_planning
