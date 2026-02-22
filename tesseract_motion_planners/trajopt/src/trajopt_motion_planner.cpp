/**
 * @file trajopt_planner.cpp
 * @brief Tesseract ROS Trajopt planner
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt/utils.hpp>
#include <trajopt_common/config.hpp>
#include <trajopt_common/logging.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/sco_common.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>

#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_common/joint_state.h>
#include <tesseract_common/profile_dictionary.h>

#include <tesseract_kinematics/core/kinematic_group.h>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>

constexpr auto SOLUTION_FOUND{ "Found valid solution" };
constexpr auto ERROR_INVALID_INPUT{ "Failed invalid input: " };
constexpr auto ERROR_FAILED_TO_FIND_VALID_SOLUTION{ "Failed to find valid solution: " };

using namespace trajopt;

namespace tesseract::motion_planners
{
TrajOptMotionPlanner::TrajOptMotionPlanner(std::string name) : MotionPlanner(std::move(name)) {}

bool TrajOptMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

void TrajOptMotionPlanner::clear() {}

std::unique_ptr<MotionPlanner> TrajOptMotionPlanner::clone() const
{
  return std::make_unique<TrajOptMotionPlanner>(name_);
}

PlannerResponse TrajOptMotionPlanner::solve(const PlannerRequest& request) const
{
  PlannerResponse response;
  std::string reason;
  if (!checkRequest(request, reason))
  {
    response.successful = false;
    response.message = std::string(ERROR_INVALID_INPUT) + reason;
    return response;
  }

  std::shared_ptr<trajopt::ProblemConstructionInfo> pci;
  if (request.data)
  {
    pci = std::static_pointer_cast<trajopt::ProblemConstructionInfo>(request.data);
  }
  else
  {
    try
    {
      pci = createProblem(request);
    }
    catch (std::exception& e)
    {
      CONSOLE_BRIDGE_logError("TrajOptPlanner failed to generate problem: %s.", e.what());
      response.successful = false;
      response.message = std::string(ERROR_INVALID_INPUT) + e.what();
      return response;
    }

    response.data = pci;
  }

  // Construct Problem
  trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

  // Set Log Level
  if (request.verbose)
    trajopt_common::gLogLevel = trajopt_common::LevelInfo;
  else
    trajopt_common::gLogLevel = trajopt_common::LevelWarn;

  // Create optimizer
  sco::BasicTrustRegionSQP::Ptr opt;
  if (pci->opt_info.num_threads > 1)
    opt = std::make_shared<sco::BasicTrustRegionSQPMultiThreaded>(problem);
  else
    opt = std::make_shared<sco::BasicTrustRegionSQP>(problem);

  opt->setParameters(pci->opt_info);

  // Add all callbacks
  for (const sco::Optimizer::Callback& callback : pci->callbacks)
    opt->addCallback(callback);

  // Initialize
  opt->initialize(trajToDblVec(problem->GetInitTraj()));

  // Optimize
  opt->optimize();
  if (opt->results().status != sco::OptStatus::OPT_CONVERGED)
  {
    response.successful = false;
    response.message = std::string(ERROR_FAILED_TO_FIND_VALID_SOLUTION) + sco::statusToString(opt->results().status);
  }
  else
  {
    response.successful = true;
    response.message = SOLUTION_FOUND;
  }

  const std::vector<std::string> joint_names = problem->GetKin()->getJointNames();
  const Eigen::MatrixX2d joint_limits = problem->GetKin()->getLimits().joint_limits;

  // Get the results
  tesseract::common::TrajArray traj = getTraj(opt->x(), problem->GetVars());

  // Enforce limits
  for (Eigen::Index i = 0; i < traj.rows(); i++)
  {
    assert(tesseract::common::satisfiesLimits<double>(traj.row(i), joint_limits, 1e-4));
    tesseract::common::enforceLimits<double>(traj.row(i), joint_limits);
  }

  // Flatten the results to make them easier to process
  response.results = request.instructions;
  auto results_instructions = response.results.flatten(&tesseract::command_language::moveFilter);
  assert(static_cast<Eigen::Index>(results_instructions.size()) == traj.rows());
  for (std::size_t idx = 0; idx < results_instructions.size(); idx++)
  {
    auto& move_instruction = results_instructions.at(idx).get().as<tesseract::command_language::MoveInstructionPoly>();
    assignSolution(
        move_instruction, joint_names, traj.row(static_cast<Eigen::Index>(idx)), request.format_result_as_input);
  }

  return response;
}

std::shared_ptr<trajopt::ProblemConstructionInfo>
TrajOptMotionPlanner::createProblem(const PlannerRequest& request) const
{
  // Store fixed steps
  std::vector<int> fixed_steps;

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!request.instructions.getManipulatorInfo().empty());
  const tesseract::common::ManipulatorInfo& composite_mi = request.instructions.getManipulatorInfo();

  // Flatten the input for planning
  auto move_instructions = request.instructions.flatten(&tesseract::command_language::moveFilter);

  // Create the problem
  auto pci = std::make_shared<trajopt::ProblemConstructionInfo>(request.env);

  // Assign Kinematics object
  pci->kin = createKinematicGroup(composite_mi, *request.env);
  if (pci->kin == nullptr)
  {
    std::string error_msg = "In TrajOpt problem generator, manipulator does not exist!";
    CONSOLE_BRIDGE_logError(error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  // Setup Basic Info
  pci->basic_info.n_steps = static_cast<int>(move_instructions.size());
  pci->basic_info.manip = composite_mi.manipulator;
  pci->basic_info.use_time = false;

  // Apply Solver parameters
  TrajOptSolverProfile::ConstPtr solver_profile = request.profiles->getProfile<TrajOptSolverProfile>(
      name_, request.instructions.getProfile(name_), std::make_shared<TrajOptOSQPSolverProfile>());
  if (!solver_profile)
    throw std::runtime_error("TrajOptMotionPlanner: Invalid profile");

  pci->basic_info.convex_solver = solver_profile->getSolverType();
  pci->basic_info.convex_solver_config = solver_profile->createSolverConfig();
  pci->opt_info = solver_profile->createOptimizationParameters();
  pci->callbacks = solver_profile->createOptimizationCallbacks();

  // Get kinematics information
  std::vector<std::string> active_links = pci->kin->getActiveLinkNames();

  // Create a temp seed storage.
  std::vector<Eigen::VectorXd> seed_states;
  seed_states.reserve(move_instructions.size());

  for (int i = 0; i < static_cast<Eigen::Index>(move_instructions.size()); ++i)
  {
    const auto& move_instruction =
        move_instructions[static_cast<std::size_t>(i)].get().as<tesseract::command_language::MoveInstructionPoly>();

    // Get Plan Profile
    TrajOptMoveProfile::ConstPtr cur_move_profile = request.profiles->getProfile<TrajOptMoveProfile>(
        name_, move_instruction.getProfile(name_), std::make_shared<TrajOptDefaultMoveProfile>());
    if (!cur_move_profile)
      throw std::runtime_error("TrajOptMotionPlanner: Invalid profile");

    TrajOptWaypointInfo wp_info =
        cur_move_profile->create(move_instruction, composite_mi, request.env, active_links, i);

    if (wp_info.seed.rows() != pci->kin->numJoints())
      throw std::runtime_error("TrajOptMotionPlanner, profile returned invalid seed");

    if (!wp_info.term_infos.costs.empty())
      pci->cost_infos.insert(pci->cost_infos.end(), wp_info.term_infos.costs.begin(), wp_info.term_infos.costs.end());

    if (!wp_info.term_infos.constraints.empty())
      pci->cnt_infos.insert(
          pci->cnt_infos.end(), wp_info.term_infos.constraints.begin(), wp_info.term_infos.constraints.end());

    seed_states.push_back(wp_info.seed);
    if (wp_info.fixed)
      fixed_steps.push_back(i);
  }

  // Add the fixed timesteps. TrajOpt will constrain the optimization such that any costs applied at these timesteps
  // will be ignored. Costs applied to variables at fixed timesteps generally causes solver failures
  pci->basic_info.fixed_timesteps = fixed_steps;

  // Set TrajOpt seed
  assert(static_cast<long>(seed_states.size()) == pci->basic_info.n_steps);
  pci->init_info.type = trajopt::InitInfo::GIVEN_TRAJ;
  pci->init_info.data.resize(pci->basic_info.n_steps, pci->kin->numJoints());
  for (long i = 0; i < pci->basic_info.n_steps; ++i)
    pci->init_info.data.row(i) = seed_states[static_cast<std::size_t>(i)];

  // Get composite cost and constraints
  TrajOptCompositeProfile::ConstPtr composite_profile = request.profiles->getProfile<TrajOptCompositeProfile>(
      name_, request.instructions.getProfile(name_), std::make_shared<TrajOptDefaultCompositeProfile>());

  if (!composite_profile)
    throw std::runtime_error("TrajOptMotionPlanner: Invalid composite profile");

  TrajOptTermInfos c_term_infos =
      composite_profile->create(composite_mi, request.env, fixed_steps, 0, pci->basic_info.n_steps - 1);
  if (!c_term_infos.costs.empty())
    pci->cost_infos.insert(pci->cost_infos.end(), c_term_infos.costs.begin(), c_term_infos.costs.end());

  if (!c_term_infos.constraints.empty())
    pci->cnt_infos.insert(pci->cnt_infos.end(), c_term_infos.constraints.begin(), c_term_infos.constraints.end());

  return pci;
}
}  // namespace tesseract::motion_planners
