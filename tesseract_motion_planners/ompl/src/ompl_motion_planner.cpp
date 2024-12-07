/**
 * @file ompl_motion_planner.cpp
 * @brief Tesseract OMPL motion planner
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
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/planner_utils.h>

#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/ompl_solver_config.h>
#include <tesseract_motion_planners/ompl/types.h>
#include <tesseract_motion_planners/ompl/utils.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_plan_profile.h>
#include <tesseract_motion_planners/core/types.h>

#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/utils.h>

constexpr auto SOLUTION_FOUND{ "Found valid solution" };
constexpr auto ERROR_INVALID_INPUT{ "Failed invalid input: " };
constexpr auto ERROR_FAILED_TO_FIND_VALID_SOLUTION{ "Failed to find valid solution: " };

using CachedSimpleSetups = std::vector<std::shared_ptr<ompl::geometric::SimpleSetup>>;
using CachedSimpleSetupsPtr = std::shared_ptr<CachedSimpleSetups>;

namespace tesseract_planning
{
bool checkStartState(const ompl::base::ProblemDefinitionPtr& prob_def,
                     const Eigen::Ref<const Eigen::VectorXd>& state,
                     const OMPLStateExtractor& extractor)
{
  if (!(prob_def->getStartStateCount() >= 1))
    return false;

  for (unsigned i = 0; i < prob_def->getStartStateCount(); ++i)
    if (extractor(prob_def->getStartState(i)).isApprox(state, 1e-5))
      return true;

  return false;
}

bool checkGoalState(const ompl::base::ProblemDefinitionPtr& prob_def,
                    const Eigen::Ref<const Eigen::VectorXd>& state,
                    const OMPLStateExtractor& extractor)
{
  ompl::base::GoalPtr goal = prob_def->getGoal();
  if (goal->getType() == ompl::base::GoalType::GOAL_STATE)
    return extractor(prob_def->getGoal()->as<ompl::base::GoalState>()->getState()).isApprox(state, 1e-5);

  if (goal->getType() == ompl::base::GoalType::GOAL_STATES)
  {
    auto* goal_states = prob_def->getGoal()->as<ompl::base::GoalStates>();
    for (unsigned i = 0; i < goal_states->getStateCount(); ++i)
      if (extractor(goal_states->getState(i)).isApprox(state, 1e-5))
        return true;
  }
  else
  {
    CONSOLE_BRIDGE_logWarn("checkGoalStates: Unsupported Goal Type!");
    return true;
  }
  return false;
}

/** @brief Construct a basic planner */
OMPLMotionPlanner::OMPLMotionPlanner(std::string name) : MotionPlanner(std::move(name)) {}

bool OMPLMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

std::pair<bool, std::string> parallelPlan(ompl::geometric::SimpleSetup& simple_setup,
                                          const OMPLSolverConfig& solver_config,
                                          const unsigned num_output_states)
{
  std::string reason;
  simple_setup.setup();
  auto parallel_plan = std::make_shared<ompl::tools::ParallelPlan>(simple_setup.getProblemDefinition());

  for (const auto& planner : solver_config.planners)
    parallel_plan->addPlanner(planner->create(simple_setup.getSpaceInformation()));

  ompl::base::PlannerStatus status;
  if (!solver_config.optimize)
  {
    // Solve problem. Results are stored in the response
    // Disabling hybridization because there is a bug which will return a trajectory that starts at the end state
    // and finishes at the end state.
    status =
        parallel_plan->solve(solver_config.planning_time, 1, static_cast<unsigned>(solver_config.max_solutions), false);
  }
  else
  {
    ompl::time::point end = ompl::time::now() + ompl::time::seconds(solver_config.planning_time);
    const ompl::base::ProblemDefinitionPtr& pdef = simple_setup.getProblemDefinition();
    while (ompl::time::now() < end)
    {
      // Solve problem. Results are stored in the response
      // Disabling hybridization because there is a bug which will return a trajectory that starts at the end state
      // and finishes at the end state.
      ompl::base::PlannerStatus localResult =
          parallel_plan->solve(std::max(ompl::time::seconds(end - ompl::time::now()), 0.0),
                               1,
                               static_cast<unsigned>(solver_config.max_solutions),
                               false);
      if (localResult)
      {
        if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
          status = localResult;

        if (!pdef->hasOptimizationObjective())
        {
          reason = "Terminating early since there is no optimization objective specified";
          CONSOLE_BRIDGE_logDebug(reason.c_str());
          break;
        }

        ompl::base::Cost obj_cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective());
        CONSOLE_BRIDGE_logDebug("Motion Objective Cost: %f", obj_cost.value());

        if (pdef->getOptimizationObjective()->isSatisfied(obj_cost))
        {
          reason = "Terminating early since solution path satisfies the optimization objective";
          CONSOLE_BRIDGE_logDebug(reason.c_str());
          break;
        }

        if (pdef->getSolutionCount() >= static_cast<std::size_t>(solver_config.max_solutions))
        {
          reason =
              "Terminating early since " + std::to_string(solver_config.max_solutions) + " solutions were generated";
          CONSOLE_BRIDGE_logDebug(reason.c_str());
          break;
        }
      }
    }
    if (ompl::time::now() >= end)
      reason = "Exceeded allowed time";
  }

  if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
    return std::make_pair(false, std::string(ERROR_FAILED_TO_FIND_VALID_SOLUTION) + reason);

  if (solver_config.simplify)
  {
    simple_setup.simplifySolution();
  }
  else
  {
    // Interpolate the path if it shouldn't be simplified and there are currently fewer states than requested
    if (simple_setup.getSolutionPath().getStateCount() < num_output_states)
    {
      simple_setup.getSolutionPath().interpolate(num_output_states);
    }
    else
    {
      // Now try to simplify the trajectory to get it under the requested number of output states
      // The interpolate function only executes if the current number of states is less than the requested
      simple_setup.simplifySolution();
      if (simple_setup.getSolutionPath().getStateCount() < num_output_states)
        simple_setup.getSolutionPath().interpolate(num_output_states);
    }
  }

  return std::make_pair(true, "SUCCESS");
}

long OMPLMotionPlanner::assignTrajectory(tesseract_planning::CompositeInstruction& output,
                                         boost::uuids::uuid start_uuid,
                                         boost::uuids::uuid end_uuid,
                                         long start_index,
                                         const std::vector<std::string>& joint_names,
                                         const tesseract_common::TrajArray& traj,
                                         const bool format_result_as_input)
{
  bool found{ false };
  Eigen::Index row{ 0 };
  auto& ci = output.getInstructions();
  for (auto it = ci.begin() + static_cast<long>(start_index); it != ci.end(); ++it)
  {
    if (it->isMoveInstruction())
    {
      auto& mi = it->as<MoveInstructionPoly>();
      if (mi.getUUID() == start_uuid)
        found = true;

      if (mi.getUUID() == end_uuid)
      {
        std::vector<InstructionPoly> extra;
        for (; row < traj.rows() - 1; ++row)
        {
          MoveInstructionPoly child = mi.createChild();
          if (format_result_as_input)
          {
            JointWaypointPoly jwp = mi.createJointWaypoint();
            jwp.setIsConstrained(false);
            jwp.setNames(joint_names);
            jwp.setPosition(traj.row(row));
            child.assignJointWaypoint(jwp);
          }
          else
          {
            StateWaypointPoly swp = mi.createStateWaypoint();
            swp.setNames(joint_names);
            swp.setPosition(traj.row(row));
            child.assignStateWaypoint(swp);
          }

          extra.emplace_back(child);
        }

        assignSolution(mi, joint_names, traj.row(row), format_result_as_input);

        if (!extra.empty())
          ci.insert(it, extra.begin(), extra.end());

        start_index += static_cast<long>(extra.size());
        break;
      }

      if (found)
        assignSolution(mi, joint_names, traj.row(row++), format_result_as_input);
    }

    ++start_index;
  }

  return start_index;
}

PlannerResponse OMPLMotionPlanner::solve(const PlannerRequest& request) const
{
  PlannerResponse response;
  response.results = request.instructions;

  std::string reason;
  if (!checkRequest(request, reason))
  {
    response.successful = false;
    response.message = std::string(ERROR_INVALID_INPUT) + reason;
    return response;
  }

  // Assume all the plan instructions have the same manipulator as the composite
  tesseract_common::ManipulatorInfo composite_mi = request.instructions.getManipulatorInfo();
  assert(!composite_mi.empty());

  // Flatten the input for planning
  auto move_instructions = request.instructions.flatten(&moveFilter);

  // This is for replanning the same problem
  CachedSimpleSetups cached_simple_setups;
  if (request.data != nullptr)
    cached_simple_setups = *std::static_pointer_cast<CachedSimpleSetups>(request.data);
  else
    cached_simple_setups.reserve(move_instructions.size());

  // Transform plan instructions into ompl problem
  unsigned num_output_states = 1;
  long start_index{ 0 };
  std::size_t segment{ 1 };
  std::reference_wrapper<const InstructionPoly> start_instruction = move_instructions.front();
  for (std::size_t i = 1; i < move_instructions.size(); ++i)
  {
    ++num_output_states;

    std::reference_wrapper<const InstructionPoly> end_instruction = move_instructions[i];
    const auto& end_move_instruction = end_instruction.get().as<MoveInstructionPoly>();
    const auto& waypoint = end_move_instruction.getWaypoint();
    if (waypoint.isJointWaypoint() && !waypoint.as<JointWaypointPoly>().isConstrained())
      continue;

    // Get Plan Profile
    auto cur_plan_profile = getProfile<OMPLPlanProfile>(name_,
                                                        end_move_instruction.getProfile(name_),
                                                        *request.profiles,
                                                        std::make_shared<OMPLRealVectorPlanProfile>());

    if (!cur_plan_profile)
      throw std::runtime_error("OMPLMotionPlanner: Invalid profile");

    // Get end state kinematics data
    tesseract_common::ManipulatorInfo end_mi = composite_mi.getCombined(end_move_instruction.getManipulatorInfo());
    tesseract_kinematics::JointGroup::UPtr manip = request.env->getJointGroup(end_mi.manipulator);

    // Create problem data
    const auto& start_move_instruction = start_instruction.get().as<MoveInstructionPoly>();
    std::unique_ptr<OMPLSolverConfig> solver_config = cur_plan_profile->createSolverConfig();
    OMPLStateExtractor extractor = cur_plan_profile->createStateExtractor(*manip);
    std::shared_ptr<ompl::geometric::SimpleSetup> simple_setup;

    if (cached_simple_setups.empty() || segment > cached_simple_setups.size())
    {
      simple_setup =
          cur_plan_profile->createSimpleSetup(start_move_instruction, end_move_instruction, composite_mi, request.env);
      cached_simple_setups.push_back(simple_setup);
    }
    else
    {
      simple_setup = cached_simple_setups.at(segment - 1);
    }

    // Parallel Plan problem
    auto status = parallelPlan(*simple_setup, *solver_config, num_output_states);
    if (!status.first)
    {
      response.successful = false;
      response.message = status.second;
      response.data = std::make_shared<CachedSimpleSetups>(cached_simple_setups);
      return response;
    }

    // Extract Solution
    const std::vector<std::string> joint_names = manip->getJointNames();
    const Eigen::MatrixX2d joint_limits = manip->getLimits().joint_limits;
    tesseract_common::TrajArray traj = toTrajArray(simple_setup->getSolutionPath(), extractor);
    assert(checkStartState(simple_setup->getProblemDefinition(), traj.row(0), extractor));
    assert(checkGoalState(simple_setup->getProblemDefinition(), traj.bottomRows(1).transpose(), extractor));
    assert(traj.rows() >= num_output_states);

    // Enforce limits
    for (Eigen::Index i = 0; i < traj.rows(); i++)
    {
      assert(tesseract_common::satisfiesLimits<double>(traj.row(i), joint_limits, 1e-4));
      tesseract_common::enforceLimits<double>(traj.row(i), joint_limits);
    }

    // Assign trajectory to results
    start_index = assignTrajectory(response.results,
                                   start_move_instruction.getUUID(),
                                   end_move_instruction.getUUID(),
                                   start_index,
                                   joint_names,
                                   traj,
                                   request.format_result_as_input);

    // Reset data for next segment
    start_instruction = end_instruction;
    num_output_states = 1;
    ++segment;
  }

  response.successful = true;
  response.message = SOLUTION_FOUND;
  response.data = std::make_shared<CachedSimpleSetups>(cached_simple_setups);
  return response;
}

void OMPLMotionPlanner::clear() {}

std::unique_ptr<MotionPlanner> OMPLMotionPlanner::clone() const { return std::make_unique<OMPLMotionPlanner>(name_); }

}  // namespace tesseract_planning
