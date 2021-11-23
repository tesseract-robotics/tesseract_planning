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
#include <ompl/tools/multiplan/ParallelPlan.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner_status_category.h>
#include <tesseract_motion_planners/ompl/utils.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_environment/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>

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
OMPLMotionPlanner::OMPLMotionPlanner(std::string name)
  : name_(std::move(name))
{
  if (name_.empty())
    throw std::runtime_error("OMPLMotionPlanner name is empty!");
}

const std::string& OMPLMotionPlanner::getName() const { return name_; }

bool OMPLMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

void OMPLMotionPlanner::clear()
{

}

tesseract_common::StatusCode OMPLMotionPlanner::solve(const PlannerRequest& request,
                                                      PlannerResponse& response,
                                                      bool verbose) const
{
  auto status_category_ = std::make_shared<const OMPLMotionPlannerStatusCategory>(name_);

  // Check the format of the request
  if (!checkUserInput(request)) // NOLINT
  {
    response.status =
        tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
    return response.status;
  }

  // If the verbose set the log level to debug.
  if (verbose)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Get the planner profile
  OMPLPlannerParameters params;
  try
  {
    const std::string profile_name =
        getProfileString(name_, request.instructions.getProfile(), request.plan_profile_remapping);
    auto pp = request.profiles->getProfile<PlannerProfile<OMPLPlannerParameters>>(name_, profile_name);
    params = pp->create();
  }
  catch (const std::exception& ex)
  {
    response.status =
        tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
    return response.status;
  }

  // Get the composite profile
  ompl::geometric::SimpleSetupPtr simple_setup;
  OMPLStateExtractor extractor;
  try
  {
    const std::string profile_name =
        getProfileString(name_, request.instructions.getProfile(), request.composite_profile_remapping);
    auto cp = request.profiles->getProfile<CompositeProfile<OMPLCompositeProfileData>>(name_, profile_name);
    std::tie(simple_setup, extractor) = cp->create(request.instructions, request.env);
  }
  catch (const std::exception& ex)
  {
    CONSOLE_BRIDGE_logError(ex.what());
    response.status =
        tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
    return response.status;
  }

  // Create an OMPL planner that can run multiple OMPL planners in parallele
  auto parallel_plan = std::make_shared<ompl::tools::ParallelPlan>(simple_setup->getProblemDefinition());

  // Add all the specified OMPL planners to the parallel planner
  for (const auto& factory : params.planners)
    parallel_plan->addPlanner(factory->create(simple_setup->getSpaceInformation()));

  ompl::base::PlannerStatus status;
  if (!params.optimize)
  {
    // Solve problem. Results are stored in the response
    // Disabling hybridization because there is a bug which will return a trajectory that starts at the end state
    // and finishes at the end state.
    status = parallel_plan->solve(params.planning_time, 1, static_cast<unsigned>(params.max_solutions), false);
  }
  else
  {
    ompl::time::point end = ompl::time::now() + ompl::time::seconds(params.planning_time);
    const ompl::base::ProblemDefinitionPtr& pdef = simple_setup->getProblemDefinition();
    while (ompl::time::now() < end)
    {
      // Solve problem. Results are stored in the response
      // Disabling hybridization because there is a bug which will return a trajectory that starts at the end state
      // and finishes at the end state.
      ompl::base::PlannerStatus localResult =
          parallel_plan->solve(std::max(ompl::time::seconds(end - ompl::time::now()), 0.0),
                               1,
                               static_cast<unsigned>(params.max_solutions),
                               false);
      if (localResult)
      {
        if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
          status = localResult;

        if (!pdef->hasOptimizationObjective())
        {
          CONSOLE_BRIDGE_logDebug("Terminating early since there is no optimization objective specified");
          break;
        }

        ompl::base::Cost obj_cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective());
        CONSOLE_BRIDGE_logDebug("Motion Objective Cost: %f", obj_cost.value());

        if (pdef->getOptimizationObjective()->isSatisfied(obj_cost))
        {
          CONSOLE_BRIDGE_logDebug("Terminating early since solution path satisfies the optimization objective");
          break;
        }

        if (pdef->getSolutionCount() >= static_cast<std::size_t>(params.max_solutions))
        {
          CONSOLE_BRIDGE_logDebug("Terminating early since %u solutions were generated", params.max_solutions);
          break;
        }
      }
    }
  }

  if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
  {
    response.status = tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorFailedToFindValidSolution,
                                                   status_category_);
    return response.status;
  }

  // The trajectory must have at least as many states as the seed trajectory
  const std::size_t n_states = simple_setup->getSolutionPath().getStateCount();
  if (n_states < request.seed.size())
  {
    // Upsample the number of states to match the length of the seed
    simple_setup->getSolutionPath().interpolate(static_cast<unsigned>(request.seed.size()));
  }
  else if (n_states > request.seed.size() && params.simplify)
  {
    // Simplify the solution
    simple_setup->simplifySolution();

    // Upsample the number of states to match the length of the seed if required
    if (simple_setup->getSolutionPath().getStateCount() < request.seed.size())
      simple_setup->getSolutionPath().interpolate(static_cast<unsigned>(request.seed.size()));
  }
  else
  {
    // Trajectory length matches seed length, so no need to simplify or interpolate
  }

  // Get the results
  tesseract_common::TrajArray trajectory = toTrajArray(simple_setup->getSolutionPath(), extractor);
  assert(checkStartState(simple_setup->getProblemDefinition(), trajectory.row(0), extractor));
  assert(checkGoalState(simple_setup->getProblemDefinition(), trajectory.bottomRows(1).transpose(), extractor));

  // Enforce limits
  {
    const std::string manipulator = request.instructions.getManipulatorInfo().manipulator;
    auto joint_limits = request.env->getJointGroup(manipulator)->getLimits().joint_limits;
    for (Eigen::Index i = 0; i < trajectory.rows(); i++)
      tesseract_common::enforcePositionLimits(trajectory.row(i), joint_limits);
  }

  // Initialize the output by copying the input program and flattening it
  response.results = request.seed;
  std::vector<std::reference_wrapper<Instruction>> results_flattened =
      flattenProgramToPattern(response.results, request.instructions);

  // Element 0 of the flattened results is a start instruction that should contain the first trajectory state
  auto& start_instruction = results_flattened.at(0).get().as<MoveInstruction>();

  // Element 1 of the flattened results is a composite instruction containing the reset of the trajectory
  auto& composite = results_flattened.at(1).get().as<CompositeInstruction>();

  // The results composite will only have as many states as the seed, but the OMPL trajectory might require more states
  // In this case, we need to insert more states into the composite to cover the difference
  if (composite.size() < trajectory.rows())
  {
    const std::size_t diff = static_cast<std::size_t>(trajectory.rows()) - composite.size();
    composite.insert(composite.end(), diff, composite.back());
  }

  // Overwrite the contents of each copied waypoint
  for (Eigen::Index i = 0; i < trajectory.rows(); ++i)
  {
    // The first trajectory state goes into the start instruction
    if (i == 0)
    {
      start_instruction.getWaypoint().as<StateWaypoint>().position = trajectory.row(i);
    }
    else
    {
      // Subsequent trajectory states go into the composite instruction
      auto& move_instruction = composite.at(static_cast<std::size_t>(i)).as<MoveInstruction>();
      move_instruction.getWaypoint().as<StateWaypoint>().position = trajectory.row(i);
    }
  }

  response.status = tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::SolutionFound, status_category_);
  return response.status;
}

MotionPlanner::Ptr OMPLMotionPlanner::clone() const { return std::make_shared<OMPLMotionPlanner>(name_); }

bool OMPLMotionPlanner::checkUserInput(const PlannerRequest& request)
{
  // Check that parameters are valid
  if (request.env == nullptr)
    std::runtime_error("Environment is invalid (nullptr)");

  if (request.instructions.empty())
    std::runtime_error("Request contains no instructions");

  if (request.instructions.size() > 1)
    std::runtime_error("OMPL planner only supports one plan instruction");

  // Attempt to cast the first child instruction to a PlanInstruction
  request.instructions.at(0).as<PlanInstruction>();

  return true;
}

}  // namespace tesseract_planning
