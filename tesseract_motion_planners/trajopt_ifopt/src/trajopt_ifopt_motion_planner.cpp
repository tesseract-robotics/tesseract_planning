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
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/sqp_callback.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <OsqpEigen/OsqpEigen.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_problem.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_solver_profile.h>
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

  std::shared_ptr<TrajOptIfoptProblem> problem;
  if (request.data)
  {
    problem = std::static_pointer_cast<TrajOptIfoptProblem>(request.data);
  }
  else
  {
    try
    {
      problem = createProblem(request);
    }
    catch (std::exception& e)
    {
      CONSOLE_BRIDGE_logError("TrajOptIfoptPlanner failed to generate problem: %s.", e.what());
      response.successful = false;
      response.message = ERROR_INVALID_INPUT;
      return response;
    }

    response.data = problem;
  }

  // Create optimizer
  /** @todo Enable solver selection (e.g. IPOPT) */
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();

  // There seems to be no way to set objects solver_->settings() (OsqpEigen::Settings)
  // or solver_->settings()->getSettings() (OSQPSettings) at once
  copyOSQPEigenSettings(*qp_solver->solver_->settings(), *problem->convex_solver_settings);
  qp_solver->solver_->settings()->setVerbosity((problem->convex_solver_settings->getSettings()->verbose != 0) ||
                                               request.verbose);

  problem->qp_solver = qp_solver;

  trajopt_sqp::TrustRegionSQPSolver solver(problem->qp_solver);
  solver.params = problem->opt_info;

  // Add all callbacks
  for (const trajopt_sqp::SQPCallback::Ptr& callback : problem->callbacks)
  {
    solver.registerCallback(callback);
  }

  // solve
  solver.verbose = request.verbose;
  solver.solve(problem->nlp);

  // Check success
  if (solver.getStatus() != trajopt_sqp::SQPStatus::NLP_CONVERGED)
  {
    response.successful = false;
    response.message = ERROR_FAILED_TO_FIND_VALID_SOLUTION;
    return response;
  }

  const std::vector<std::string> joint_names = problem->manip->getJointNames();
  const Eigen::MatrixX2d joint_limits = problem->manip->getLimits().joint_limits;

  // Get the results - This can likely be simplified if we get rid of the traj array
  Eigen::VectorXd x = problem->nlp->getVariableValues();
  Eigen::Map<tesseract_common::TrajArray> traj(x.data(),
                                               static_cast<Eigen::Index>(problem->vars.size()),
                                               static_cast<Eigen::Index>(problem->vars[0]->GetValues().size()));

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

std::shared_ptr<TrajOptIfoptProblem> TrajOptIfoptMotionPlanner::createProblem(const PlannerRequest& request) const
{
  // Store fixed steps
  std::vector<int> fixed_steps;

  // Create the problem
  auto problem = std::make_shared<TrajOptIfoptProblem>();
  problem->environment = request.env;
  problem->env_state = request.env->getState();
  problem->nlp = std::make_shared<trajopt_sqp::TrajOptQPProblem>();

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!request.instructions.getManipulatorInfo().empty());
  const tesseract_common::ManipulatorInfo& composite_mi = request.instructions.getManipulatorInfo();

  if (composite_mi.manipulator.empty())
    throw std::runtime_error("DefaultTrajoptIfoptProblemGenerator, manipulator is empty!");

  // Assign Kinematics object
  try
  {
    tesseract_kinematics::KinematicGroup::Ptr kin_group;
    std::string error_msg;
    if (composite_mi.manipulator_ik_solver.empty())
    {
      kin_group = request.env->getKinematicGroup(composite_mi.manipulator);
      error_msg = "Failed to find kinematic group for manipulator '" + composite_mi.manipulator + "'";
    }
    else
    {
      kin_group = request.env->getKinematicGroup(composite_mi.manipulator, composite_mi.manipulator_ik_solver);
      error_msg = "Failed to find kinematic group for manipulator '" + composite_mi.manipulator + "' with solver '" +
                  composite_mi.manipulator_ik_solver + "'";
    }

    if (kin_group == nullptr)
    {
      CONSOLE_BRIDGE_logError("%s", error_msg.c_str());
      throw std::runtime_error(error_msg);
    }

    problem->manip = kin_group;
  }
  catch (...)
  {
    problem->manip = request.env->getJointGroup(composite_mi.manipulator);
  }

  if (problem->manip == nullptr)
  {
    std::string error_msg = "In TrajOpt IFOPT problem generator, manipulator does not exist!";
    CONSOLE_BRIDGE_logError(error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  // Apply Solver parameters
  TrajOptIfoptSolverProfile::ConstPtr solver_profile =
      getProfile<TrajOptIfoptSolverProfile>(name_,
                                            request.instructions.getProfile(name_),
                                            *request.profiles,
                                            std::make_shared<TrajOptIfoptDefaultSolverProfile>());
  if (!solver_profile)
    throw std::runtime_error("TrajOptIfoptMotionPlanner: Invalid profile");

  solver_profile->apply(*problem);

  // Get kinematics information
  tesseract_environment::Environment::ConstPtr env = request.env;
  std::vector<std::string> active_links = problem->manip->getActiveLinkNames();
  std::vector<std::string> joint_names = problem->manip->getJointNames();
  Eigen::MatrixX2d joint_limits_eigen = problem->manip->getLimits().joint_limits;

  // Flatten the input for planning
  auto move_instructions = request.instructions.flatten(&moveFilter);

  // ----------------
  // Translate TCL for MoveInstructions
  // ----------------
  // Transform plan instructions into trajopt cost and constraints
  for (int i = 0; i < move_instructions.size(); ++i)
  {
    const auto& move_instruction = move_instructions[static_cast<std::size_t>(i)].get().as<MoveInstructionPoly>();
    // If plan instruction has manipulator information then use it over the one provided by the composite.
    tesseract_common::ManipulatorInfo mi = composite_mi.getCombined(move_instruction.getManipulatorInfo());

    if (mi.manipulator.empty())
      throw std::runtime_error("TrajOpt, manipulator is empty!");

    if (mi.tcp_frame.empty())
      throw std::runtime_error("TrajOpt, tcp_frame is empty!");

    if (mi.working_frame.empty())
      throw std::runtime_error("TrajOpt, working_frame is empty!");

    // Get Plan Profile
    TrajOptIfoptPlanProfile::ConstPtr cur_plan_profile =
        getProfile<TrajOptIfoptPlanProfile>(name_,
                                            move_instruction.getProfile(name_),
                                            *request.profiles,
                                            std::make_shared<TrajOptIfoptDefaultPlanProfile>());
    if (!cur_plan_profile)
      throw std::runtime_error("TrajOptMotionPlanner: Invalid profile");

    if (move_instruction.getWaypoint().isCartesianWaypoint())
    {
      const auto& cwp = move_instruction.getWaypoint().as<CartesianWaypointPoly>();

      // Add seed state
      Eigen::VectorXd seed_state;
      if (cwp.hasSeed())
      {
        assert(checkJointPositionFormat(joint_names, move_instruction.getWaypoint()));
        seed_state = cwp.getSeed().position;
      }
      else
      {
        seed_state = request.env->getCurrentJointValues(joint_names);
      }

      // Add variable set to problem
      auto var = std::make_shared<trajopt_ifopt::JointPosition>(
          seed_state, joint_names, "Joint_Position_" + std::to_string(i));
      var->SetBounds(joint_limits_eigen);
      problem->vars.push_back(var);
      problem->nlp->addVariableSet(var);

      // Apply profile
      cur_plan_profile->apply(*problem, cwp, move_instruction, composite_mi, active_links, i);

      /** @todo If fixed cartesian and not term_type cost add as fixed */
    }
    else if (move_instruction.getWaypoint().isJointWaypoint())
    {
      assert(checkJointPositionFormat(joint_names, move_instruction.getWaypoint()));

      const auto& jwp = move_instruction.getWaypoint().as<JointWaypointPoly>();

      // Add variable set to problem
      auto var = std::make_shared<trajopt_ifopt::JointPosition>(
          jwp.getPosition(), joint_names, "Joint_Position_" + std::to_string(i));
      var->SetBounds(joint_limits_eigen);
      problem->vars.push_back(var);
      problem->nlp->addVariableSet(var);

      // Apply profile
      if (jwp.isConstrained())
      {
        cur_plan_profile->apply(*problem, jwp, move_instruction, composite_mi, active_links, i);

        // Add to fixed indices
        if (!jwp.isToleranced()) /** @todo Should not make fixed if term_type is cost */
          fixed_steps.push_back(i);
      }
    }
    else if (move_instruction.getWaypoint().isStateWaypoint())
    {
      assert(checkJointPositionFormat(joint_names, move_instruction.getWaypoint()));
      const auto& swp = move_instruction.getWaypoint().as<StateWaypointPoly>();
      JointWaypointPoly jwp = move_instruction.createJointWaypoint();
      jwp.setNames(swp.getNames());
      jwp.setPosition(swp.getPosition());

      // Add variable set to problem
      auto var = std::make_shared<trajopt_ifopt::JointPosition>(
          swp.getPosition(), joint_names, "Joint_Position_" + std::to_string(i));
      var->SetBounds(joint_limits_eigen);
      problem->vars.push_back(var);
      problem->nlp->addVariableSet(var);

      // Apply profile
      cur_plan_profile->apply(*problem, jwp, move_instruction, composite_mi, active_links, i);

      // Add to fixed indices
      fixed_steps.push_back(i); /** @todo Should not make fixed if term_type is cost */
    }
    else
    {
      throw std::runtime_error("TrajOptMotionPlanner: unknown waypoint type");
    }
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
    throw std::runtime_error("DefaultTrajoptIfoptProblemGenerator: Invalid profile");

  cur_composite_profile->apply(
      *problem, 0, static_cast<int>(problem->vars.size()) - 1, composite_mi, active_links, fixed_steps);

  // ----------------
  // Return problem
  // ----------------
  problem->nlp->setup();
  return problem;
}

}  // namespace tesseract_planning
