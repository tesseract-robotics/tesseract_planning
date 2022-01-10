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
  if (prob_def->getStartStateCount() < 1)
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

std::vector<ompl::base::ScopedState<>> createOMPLStates(const std::vector<Eigen::VectorXd>& joint_states,
                                                        const ompl::base::SpaceInformationPtr& si)
{
  std::vector<ompl::base::ScopedState<>> states;
  states.reserve(joint_states.size());

  for (const auto& js : joint_states)
  {
    ompl::base::ScopedState<> state(si->getStateSpace());
    for (unsigned i = 0; i < js.size(); ++i)
      state[i] = js[static_cast<Eigen::Index>(i)];

    states.push_back(state);
  }

  return states;
}

std::shared_ptr<ompl::base::PlannerData> plan(const ompl::geometric::SimpleSetupPtr& simple_setup,
                                              const OMPLPlannerParameters& params,
                                              const unsigned n_output_states)
{
  // Create an OMPL planner that can run multiple OMPL planners in parallel
  auto parallel_plan = std::make_shared<ompl::tools::ParallelPlan>(simple_setup->getProblemDefinition());

  // Add all the specified OMPL planners to the parallel planner
  std::vector<ompl::base::PlannerPtr> planners;
  planners.reserve(params.planners.size());
  for (const auto& factory : params.planners)
  {
    ompl::base::PlannerPtr planner = factory->create(simple_setup->getSpaceInformation());

    //    // TODO: construct planner from the saved planner data (only valid for multi-query planners, such as PRM)
    //    if (request.data != nullptr)
    //    {
    //      auto planner_data = std::static_pointer_cast<ompl::base::PlannerData>(request.data);
    //      planner = factory->create(*planner_data);
    //      if (!planner)
    //      {
    //        CONSOLE_BRIDGE_logWarn("Unable to construct planner with saved PlannerData; constructing planner from
    //        scratch instead"); planner = factory->create(simple_setup->getSpaceInformation());
    //      }
    //    }
    //    else
    //    {
    //      planner = factory->create(simple_setup->getSpaceInformation());
    //    }

    // Add the planner to the parallel planner object and save a pointer to it
    parallel_plan->addPlanner(planner);
    planners.push_back(planner);
  }

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

  // Check the planner status
  if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
    throw std::runtime_error("Planning failed to find exact solution");

  // The trajectory must have at least as many states as the seed trajectory
  if (n_output_states > 2)
  {
    const std::size_t n_states = simple_setup->getSolutionPath().getStateCount();

    if (n_states < n_output_states)
    {
      // Upsample the number of states to match the length of the seed
      simple_setup->getSolutionPath().interpolate(n_output_states);
    }
    else if (n_states > n_output_states && params.simplify)
    {
      // Simplify the solution
      simple_setup->simplifySolution();

      // Upsample the number of states to match the length of the seed if required
      if (simple_setup->getSolutionPath().getStateCount() < n_output_states)
        simple_setup->getSolutionPath().interpolate(n_output_states);
    }
    else
    {
      // Trajectory length matches or exceeds seed length and simplification is disabled, so no need to simplify or
      // interpolate
    }
  }

  // Extract the planner data from each planner and concatenate
  // TODO: Investigate whether it is problematic to concatenate planner data from multiple different planners. The
  // getPlannerData function handles new vertex and edge indices correctly, but vertices that are close enough in the
  // space to be considered equivalent are not merged or connected. Also different planners add edges in different ways
  // and set different vertex/edge properties, so it may not make sense to combine them
  auto planner_data = std::make_shared<ompl::base::PlannerData>(simple_setup->getSpaceInformation());
  for (const ompl::base::PlannerPtr& planner : planners)
    planner->getPlannerData(*planner_data);

  // The planning data is actually owned by the planner instances themselves. Deep copy the planning information by
  // decoupling from the planner instances so the data is not lost when the planner instances go out of scope
  planner_data->decoupleFromPlanner();

  return planner_data;
}

CompositeInstruction buildTrajectoryInstruction(const tesseract_common::TrajArray& trajectory,
                                                const CompositeInstruction& seed)
{
  // Initialize the output by copying the input program and flattening it
  CompositeInstruction output(seed);

  // The results composite will only have as many states as the seed, but the OMPL trajectory might require more states
  // In this case, we need to insert more states into the composite to cover the difference
  // Remember the composite does not include the start state, so compare its size with one less than the size of the
  // trajectory
  if (static_cast<Eigen::Index>(seed.size()) < trajectory.rows() - 1)
  {
    const std::size_t diff = static_cast<std::size_t>(trajectory.rows() - 1) - output.size();
    output.insert(output.end(), diff, output.back());
  }

  // Overwrite the contents of each copied waypoint
  for (Eigen::Index i = 0; i < trajectory.rows(); ++i)
  {
    // The first trajectory state goes into the start instruction
    if (i == 0)
    {
      output.getStartInstruction().as<MoveInstruction>().getWaypoint().as<StateWaypoint>().position = trajectory.row(i);
    }
    else
    {
      // Subsequent trajectory states go into the composite instruction
      // The index into the composite of these states is one less than the index of the trajectory state since the first
      // trajectory state was saved outside the composite
      const auto composite_idx = static_cast<std::size_t>(i - 1);
      auto& move_instruction = output.at(composite_idx).as<MoveInstruction>();
      move_instruction.getWaypoint().as<StateWaypoint>().position = trajectory.row(i);
    }
  }

  return output;
}

/** @brief Construct a basic planner */
OMPLMotionPlanner::OMPLMotionPlanner(std::string name) : name_(std::move(name))
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

void OMPLMotionPlanner::clear() {}

tesseract_common::StatusCode OMPLMotionPlanner::solve(const PlannerRequest& request,
                                                      PlannerResponse& response,
                                                      bool verbose) const
{
  // If the verbose set the log level to debug.
  if (verbose)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  auto status_category_ = std::make_shared<const OMPLMotionPlannerStatusCategory>(name_);

  try
  {
    // Check the format of the request
    if (!checkUserInput(request))  // NOLINT
      throw std::runtime_error("Input is invalid");

    // Get the planner profile
    OMPLPlannerParameters params;
    {
      const std::string profile_name =
          getProfileString(name_, request.instructions.getProfile(), request.plan_profile_remapping);
      PlannerProfile::ConstPtr pp = request.profiles->getPlannerProfile(name_, profile_name);
      params = std::any_cast<OMPLPlannerParameters>(pp->create());
    }

    // Get the composite profile
    ompl::geometric::SimpleSetupPtr simple_setup;
    OMPLStateExtractor extractor;
    {
      const std::string profile_name =
          getProfileString(name_, request.instructions.getProfile(), request.composite_profile_remapping);
      CompositeProfile::ConstPtr cp = request.profiles->getCompositeProfile(name_, profile_name);
      std::tie(simple_setup, extractor) =
          std::any_cast<OMPLCompositeProfileData>(cp->create(request.instructions, *request.env));
    }

    // Copy the meta-data from the request instruction into the response and clear any child instructions from the
    // response
    response.results = request.instructions;
    response.results.clear();

    // Set up the output trajectory to be a composite of composites
    response.results.reserve(request.instructions.size());

    // Loop over each pair of waypoints
    for (std::size_t i = 0; i < request.instructions.size(); ++i)
    {
      simple_setup->clearStartStates();
      simple_setup->clear();

      // Add the start state(s)
      {
        std::vector<Eigen::VectorXd> start_states;

        // Get the start waypoint profile and add the states to the SimpleSetup
        if (i == 0)
        {
          const auto& pi = request.instructions.getStartInstruction().as<PlanInstruction>();
          const std::string profile_name =
              getProfileString(name_, pi.getProfile(), request.composite_profile_remapping);
          WaypointProfile::ConstPtr p = request.profiles->getWaypointProfile(name_, profile_name);
          start_states = std::any_cast<std::vector<Eigen::VectorXd>>(p->create(pi, *request.env));
        }
        else
        {
          // Use the last state of the previous trajectory as the single start state for this plan
          const auto& mi = response.results.back().as<CompositeInstruction>().back().as<MoveInstruction>();
          const auto& sw = mi.getWaypoint().as<StateWaypoint>();
          start_states.push_back(sw.position);
        }

        // Add the states to the SimpleSetup
        auto states = createOMPLStates(start_states, simple_setup->getSpaceInformation());
        std::for_each(states.begin(), states.end(), [&simple_setup](const ompl::base::ScopedState<>& state) {
          simple_setup->addStartState(state);
        });
      }

      // Add the goal waypoint(s)
      {
        const auto& pi = request.instructions[i].as<PlanInstruction>();

        const std::string profile_name = getProfileString(name_, pi.getProfile(), request.composite_profile_remapping);
        WaypointProfile::ConstPtr p = request.profiles->getWaypointProfile(name_, profile_name);

        auto states = std::any_cast<std::vector<Eigen::VectorXd>>(p->create(pi, *request.env));
        auto ompl_states = createOMPLStates(states, simple_setup->getSpaceInformation());

        auto goal_states = std::make_shared<ompl::base::GoalStates>(simple_setup->getSpaceInformation());
        std::for_each(ompl_states.begin(), ompl_states.end(), [&goal_states](const ompl::base::ScopedState<>& state) {
          goal_states->addState(state);
        });

        simple_setup->setGoal(goal_states);
      }

      // The number of states in the seed is the size of the composite instruction plus one for the start state
      const unsigned n_seed_states = static_cast<unsigned>(request.seed.at(i).as<CompositeInstruction>().size()) + 1;

      // Plan
      auto planner_data = plan(simple_setup, params, n_seed_states);

      // Save the combined planner data in the response
      //  response.data = std::static_pointer_cast<void>(planner_data);

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

      // Construct the output trajectory instruction and add it to the response
      response.results.push_back(buildTrajectoryInstruction(trajectory, request.seed[i].as<CompositeInstruction>()));
    }

    // Set top-level composite start instruction to first waypoint of first trajectory
    response.results.setStartInstruction(response.results.at(0).as<CompositeInstruction>().getStartInstruction());

    response.status = tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::SolutionFound, status_category_);
  }
  catch (const std::exception& ex)
  {
    CONSOLE_BRIDGE_logError(ex.what());
    response.status =
        tesseract_common::StatusCode(OMPLMotionPlannerStatusCategory::ErrorFailedToFindValidSolution, status_category_);
  }

  return response.status;
}

MotionPlanner::Ptr OMPLMotionPlanner::clone() const { return std::make_shared<OMPLMotionPlanner>(name_); }

bool OMPLMotionPlanner::checkUserInput(const PlannerRequest& request)
{
  // Check that parameters are valid
  if (request.env == nullptr)
    throw std::runtime_error("Environment is invalid (nullptr)");

  if (request.instructions.empty())
    throw std::runtime_error("Request contains no instructions");

  if (request.instructions.size() != request.seed.size())
    throw std::runtime_error("Instruction size (" + std::to_string(request.instructions.size()) +
                             ") does not match seed size (" + std::to_string(request.seed.size()) + ")");

  return true;
}

}  // namespace tesseract_planning
