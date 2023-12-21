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
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_common/config.hpp>
#include <trajopt_common/logging.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <tesseract_environment/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>

#include <tesseract_command_language/utils.h>

constexpr auto SOLUTION_FOUND{ "Found valid solution" };
constexpr auto ERROR_INVALID_INPUT{ "Failed invalid input" };
constexpr auto ERROR_FAILED_TO_FIND_VALID_SOLUTION{ "Failed to find valid solution" };

using namespace trajopt;

namespace tesseract_planning
{
TrajOptMotionPlanner::TrajOptMotionPlanner(std::string name) : MotionPlanner(std::move(name)) {}

bool TrajOptMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

void TrajOptMotionPlanner::clear() {}

MotionPlanner::Ptr TrajOptMotionPlanner::clone() const { return std::make_shared<TrajOptMotionPlanner>(name_); }

PlannerResponse TrajOptMotionPlanner::solve(const PlannerRequest& request) const
{
  PlannerResponse response;
  if (!checkRequest(request))
  {
    response.successful = false;
    response.message = ERROR_INVALID_INPUT;
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
      response.message = ERROR_INVALID_INPUT;
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
    response.message = ERROR_FAILED_TO_FIND_VALID_SOLUTION;
    return response;
  }

  const std::vector<std::string> joint_names = problem->GetKin()->getJointNames();
  const Eigen::MatrixX2d joint_limits = problem->GetKin()->getLimits().joint_limits;

  // Get the results
  tesseract_common::TrajArray traj = getTraj(opt->x(), problem->GetVars());

  // Enforce limits
  for (Eigen::Index i = 0; i < traj.rows(); i++)
  {
    assert(tesseract_common::satisfiesPositionLimits<double>(traj.row(i), joint_limits, 1e-4));
    tesseract_common::enforcePositionLimits<double>(traj.row(i), joint_limits);
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

std::shared_ptr<trajopt::ProblemConstructionInfo>
TrajOptMotionPlanner::createProblem(const PlannerRequest& request) const
{
  // Store fixed steps
  std::vector<int> fixed_steps;

  // Create the problem
  auto pci = std::make_shared<trajopt::ProblemConstructionInfo>(request.env);

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!request.instructions.getManipulatorInfo().empty());
  const tesseract_common::ManipulatorInfo& composite_mi = request.instructions.getManipulatorInfo();

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

    pci->kin = kin_group;
  }
  catch (...)
  {
    pci->kin = request.env->getJointGroup(composite_mi.manipulator);
  }

  if (pci->kin == nullptr)
  {
    std::string error_msg = "In TrajOpt problem generator, manipulator does not exist!";
    CONSOLE_BRIDGE_logError(error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  // Apply Solver parameters
  std::string profile = request.instructions.getProfile();
  ProfileDictionary::ConstPtr profile_overrides = request.instructions.getProfileOverrides();
  profile = getProfileString(name_, profile, request.plan_profile_remapping);
  TrajOptSolverProfile::ConstPtr solver_profile = getProfile<TrajOptSolverProfile>(
      name_, profile, *request.profiles, std::make_shared<TrajOptDefaultSolverProfile>());
  solver_profile = applyProfileOverrides(name_, profile, solver_profile, profile_overrides);
  if (!solver_profile)
    throw std::runtime_error("TrajOptMotionPlanner: Invalid profile");

  solver_profile->apply(*pci);

  // Get kinematics information
  tesseract_environment::Environment::ConstPtr env = request.env;
  std::vector<std::string> active_links = pci->kin->getActiveLinkNames();
  std::vector<std::string> joint_names = pci->kin->getJointNames();

  // Flatten the input for planning
  auto move_instructions = request.instructions.flatten(&moveFilter);

  // Create a temp seed storage.
  std::vector<Eigen::VectorXd> seed_states;
  seed_states.reserve(move_instructions.size());

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
    std::string profile = getProfileString(name_, move_instruction.getProfile(), request.plan_profile_remapping);
    TrajOptPlanProfile::ConstPtr cur_plan_profile = getProfile<TrajOptPlanProfile>(
        name_, profile, *request.profiles, std::make_shared<TrajOptDefaultPlanProfile>());
    cur_plan_profile = applyProfileOverrides(name_, profile, cur_plan_profile, move_instruction.getProfileOverrides());
    if (!cur_plan_profile)
      throw std::runtime_error("TrajOptMotionPlanner: Invalid profile");

    if (move_instruction.getWaypoint().isCartesianWaypoint())
    {
      const auto& cwp = move_instruction.getWaypoint().as<CartesianWaypointPoly>();
      cur_plan_profile->apply(*pci, cwp, move_instruction, composite_mi, active_links, i);

      // Seed state
      if (cwp.hasSeed())
      {
        assert(checkJointPositionFormat(joint_names, move_instruction.getWaypoint()));
        seed_states.push_back(cwp.getSeed().position);
      }
      else
      {
        seed_states.push_back(request.env_state.getJointValues(joint_names));
      }

      /** @todo If fixed cartesian and not term_type cost add as fixed */
    }
    else if (move_instruction.getWaypoint().isJointWaypoint())
    {
      assert(checkJointPositionFormat(joint_names, move_instruction.getWaypoint()));

      const auto& jwp = move_instruction.getWaypoint().as<JointWaypointPoly>();
      if (jwp.isConstrained())
      {
        cur_plan_profile->apply(*pci, jwp, move_instruction, composite_mi, active_links, i);

        // Add to fixed indices
        if (!jwp.isToleranced()) /** @todo Should not make fixed if term_type is cost */
          fixed_steps.push_back(i);
      }

      // Add seed state
      seed_states.push_back(jwp.getPosition());
    }
    else if (move_instruction.getWaypoint().isStateWaypoint())
    {
      assert(checkJointPositionFormat(joint_names, move_instruction.getWaypoint()));
      const auto& swp = move_instruction.getWaypoint().as<StateWaypointPoly>();
      JointWaypointPoly jwp = move_instruction.createJointWaypoint();
      jwp.setNames(swp.getNames());
      jwp.setPosition(swp.getPosition());
      cur_plan_profile->apply(*pci, jwp, move_instruction, composite_mi, active_links, i);

      // Add seed state
      seed_states.push_back(swp.getPosition());

      // Add to fixed indices
      fixed_steps.push_back(i); /** @todo Should not make fixed if term_type is cost */
    }
    else
    {
      throw std::runtime_error("TrajOptMotionPlanner: unknown waypoint type");
    }
  }

  // ----------------
  // Create Problem
  // ----------------

  // Setup Basic Info
  pci->basic_info.n_steps = static_cast<int>(move_instructions.size());
  pci->basic_info.manip = composite_mi.manipulator;
  pci->basic_info.use_time = false;

  // Add the fixed timesteps. TrajOpt will constrain the optimization such that any costs applied at these timesteps
  // will be ignored. Costs applied to variables at fixed timesteps generally causes solver failures
  pci->basic_info.fixed_timesteps = fixed_steps;

  // Set TrajOpt seed
  assert(static_cast<long>(seed_states.size()) == pci->basic_info.n_steps);
  pci->init_info.type = trajopt::InitInfo::GIVEN_TRAJ;
  pci->init_info.data.resize(pci->basic_info.n_steps, pci->kin->numJoints());
  for (long i = 0; i < pci->basic_info.n_steps; ++i)
    pci->init_info.data.row(i) = seed_states[static_cast<std::size_t>(i)];

  profile = getProfileString(name_, request.instructions.getProfile(), request.composite_profile_remapping);
  TrajOptCompositeProfile::ConstPtr cur_composite_profile = getProfile<TrajOptCompositeProfile>(
      name_, profile, *request.profiles, std::make_shared<TrajOptDefaultCompositeProfile>());
  cur_composite_profile =
      applyProfileOverrides(name_, profile, cur_composite_profile, request.instructions.getProfileOverrides());
  if (!cur_composite_profile)
    throw std::runtime_error("TrajOptMotionPlanner: Invalid profile");

  cur_composite_profile->apply(*pci, 0, pci->basic_info.n_steps - 1, composite_mi, active_links, fixed_steps);

  return pci;
}
}  // namespace tesseract_planning
