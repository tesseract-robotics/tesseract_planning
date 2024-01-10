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

#include <tesseract_environment/utils.h>

#include <tesseract_motion_planners/planner_utils.h>

#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/discrete_motion_validator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_command_language/utils.h>

constexpr auto SOLUTION_FOUND{ "Found valid solution" };
constexpr auto ERROR_INVALID_INPUT{ "Failed invalid input" };
constexpr auto ERROR_FAILED_TO_FIND_VALID_SOLUTION{ "Failed to find valid solution" };

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

PlannerResponse OMPLMotionPlanner::solve(const PlannerRequest& request) const
{
  PlannerResponse response;
  if (!checkRequest(request))  // NOLINT
  {
    response.successful = false;
    response.message = ERROR_INVALID_INPUT;
    return response;
  }
  std::vector<OMPLProblemConfig> problems;
  if (request.data)
  {
    problems = *std::static_pointer_cast<std::vector<OMPLProblemConfig>>(request.data);
  }
  else
  {
    try
    {
      problems = createProblems(request);
    }
    catch (std::exception& e)
    {
      CONSOLE_BRIDGE_logError("OMPLPlanner failed to generate problem: %s.", e.what());
      response.successful = false;
      response.message = ERROR_INVALID_INPUT;
      return response;
    }

    response.data = std::make_shared<std::vector<OMPLProblemConfig>>(problems);
  }

  // If the verbose set the log level to debug.
  if (request.verbose)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  /// @todo: Need to expand this to support multiple motion plans leveraging taskflow
  for (auto& pc : problems)
  {
    auto& p = pc.problem;
    p->simple_setup->setup();
    auto parallel_plan = std::make_shared<ompl::tools::ParallelPlan>(p->simple_setup->getProblemDefinition());

    for (const auto& planner : p->planners)
      parallel_plan->addPlanner(planner->create(p->simple_setup->getSpaceInformation()));

    ompl::base::PlannerStatus status;
    if (!p->optimize)
    {
      // Solve problem. Results are stored in the response
      // Disabling hybridization because there is a bug which will return a trajectory that starts at the end state
      // and finishes at the end state.
      status = parallel_plan->solve(p->planning_time, 1, static_cast<unsigned>(p->max_solutions), false);
    }
    else
    {
      ompl::time::point end = ompl::time::now() + ompl::time::seconds(p->planning_time);
      const ompl::base::ProblemDefinitionPtr& pdef = p->simple_setup->getProblemDefinition();
      while (ompl::time::now() < end)
      {
        // Solve problem. Results are stored in the response
        // Disabling hybridization because there is a bug which will return a trajectory that starts at the end state
        // and finishes at the end state.
        ompl::base::PlannerStatus localResult =
            parallel_plan->solve(std::max(ompl::time::seconds(end - ompl::time::now()), 0.0),
                                 1,
                                 static_cast<unsigned>(p->max_solutions),
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

          if (pdef->getSolutionCount() >= static_cast<std::size_t>(p->max_solutions))
          {
            CONSOLE_BRIDGE_logDebug("Terminating early since %u solutions were generated", p->max_solutions);
            break;
          }
        }
      }
    }

    if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
      response.successful = false;
      response.message = ERROR_FAILED_TO_FIND_VALID_SOLUTION;
      return response;
    }

    if (p->simplify)
    {
      p->simple_setup->simplifySolution();
    }
    else
    {
      // Interpolate the path if it shouldn't be simplified and there are currently fewer states than requested
      auto num_output_states = static_cast<unsigned>(p->n_output_states);
      if (p->simple_setup->getSolutionPath().getStateCount() < num_output_states)
      {
        p->simple_setup->getSolutionPath().interpolate(num_output_states);
      }
      else
      {
        // Now try to simplify the trajectory to get it under the requested number of output states
        // The interpolate function only executes if the current number of states is less than the requested
        p->simple_setup->simplifySolution();
        if (p->simple_setup->getSolutionPath().getStateCount() < num_output_states)
          p->simple_setup->getSolutionPath().interpolate(num_output_states);
      }
    }
  }

  // Flatten the results to make them easier to process
  /** @todo Current does not handle if the returned solution is greater than the request */
  /** @todo Switch to processing the composite directly versus a flat list to solve the problem above  */
  response.results = request.instructions;

  std::size_t start_index{ 0 };
  for (auto& pc : problems)
  {
    auto& p = pc.problem;
    tesseract_common::TrajArray traj = p->getTrajectory();
    assert(checkStartState(p->simple_setup->getProblemDefinition(), traj.row(0), p->extractor));
    assert(checkGoalState(p->simple_setup->getProblemDefinition(), traj.bottomRows(1).transpose(), p->extractor));
    assert(traj.rows() >= p->n_output_states);

    const std::vector<std::string> joint_names = p->manip->getJointNames();
    const Eigen::MatrixX2d joint_limits = p->manip->getLimits().joint_limits;

    // Enforce limits
    for (Eigen::Index i = 0; i < traj.rows(); i++)
    {
      assert(tesseract_common::satisfiesPositionLimits<double>(traj.row(i), joint_limits, 1e-4));
      tesseract_common::enforcePositionLimits<double>(traj.row(i), joint_limits);
    }

    bool found{ false };
    Eigen::Index row{ 0 };
    auto& ci = response.results.getInstructions();
    for (auto it = ci.begin() + static_cast<long>(start_index); it != ci.end(); ++it)
    {
      if (it->isMoveInstruction())
      {
        auto& mi = it->as<MoveInstructionPoly>();
        if (mi.getUUID() == pc.start_uuid)
          found = true;

        if (mi.getUUID() == pc.end_uuid)
        {
          std::vector<InstructionPoly> extra;
          for (; row < traj.rows() - 1; ++row)
          {
            MoveInstructionPoly child = mi.createChild();
            if (request.format_result_as_input)
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

          assignSolution(mi, joint_names, traj.row(row), request.format_result_as_input);

          if (!extra.empty())
            ci.insert(it, extra.begin(), extra.end());

          start_index += extra.size();
          break;
        }

        if (found)
          assignSolution(mi, joint_names, traj.row(row++), request.format_result_as_input);
      }

      ++start_index;
    }
  }

  response.successful = true;
  response.message = SOLUTION_FOUND;
  return response;
}

void OMPLMotionPlanner::clear() { parallel_plan_ = nullptr; }

MotionPlanner::Ptr OMPLMotionPlanner::clone() const { return std::make_shared<OMPLMotionPlanner>(name_); }

OMPLProblemConfig OMPLMotionPlanner::createSubProblem(const PlannerRequest& request,
                                                      const tesseract_common::ManipulatorInfo& composite_mi,
                                                      const tesseract_kinematics::JointGroup::ConstPtr& manip,
                                                      const MoveInstructionPoly& start_instruction,
                                                      const MoveInstructionPoly& end_instruction,
                                                      int n_output_states,
                                                      int index) const
{
  std::vector<std::string> joint_names = manip->getJointNames();
  std::vector<std::string> active_link_names = manip->getActiveLinkNames();

  // Get Plan Profile
  std::string profile = end_instruction.getProfile();
  profile = getProfileString(name_, profile, request.plan_profile_remapping);
  auto cur_plan_profile =
      getProfile<OMPLPlanProfile>(name_, profile, *request.profiles, std::make_shared<OMPLDefaultPlanProfile>());
  cur_plan_profile = applyProfileOverrides(name_, profile, cur_plan_profile, end_instruction.getProfileOverrides());
  if (!cur_plan_profile)
    throw std::runtime_error("OMPLMotionPlanner: Invalid profile");

  /** @todo Should check that the joint names match the order of the manipulator */
  OMPLProblemConfig config;
  config.start_uuid = start_instruction.getUUID();
  config.end_uuid = end_instruction.getUUID();
  config.problem = std::make_shared<OMPLProblem>();
  config.problem->env = request.env;
  config.problem->env_state = request.env_state;
  config.problem->manip = manip;
  config.problem->contact_checker = request.env->getDiscreteContactManager();
  config.problem->contact_checker->setCollisionObjectsTransform(request.env_state.link_transforms);
  config.problem->contact_checker->setActiveCollisionObjects(active_link_names);

  cur_plan_profile->setup(*config.problem);
  config.problem->n_output_states = n_output_states;

  if (end_instruction.getWaypoint().isJointWaypoint() || end_instruction.getWaypoint().isStateWaypoint())
  {
    assert(checkJointPositionFormat(joint_names, end_instruction.getWaypoint()));
    const Eigen::VectorXd& cur_position = getJointPosition(end_instruction.getWaypoint());
    cur_plan_profile->applyGoalStates(
        *config.problem, cur_position, end_instruction, composite_mi, active_link_names, index);

    if (start_instruction.getWaypoint().isJointWaypoint() || start_instruction.getWaypoint().isStateWaypoint())
    {
      assert(checkJointPositionFormat(joint_names, start_instruction.getWaypoint()));
      const Eigen::VectorXd& prev_position = getJointPosition(start_instruction.getWaypoint());
      cur_plan_profile->applyStartStates(
          *config.problem, prev_position, start_instruction, composite_mi, active_link_names, index);
    }
    else if (start_instruction.getWaypoint().isCartesianWaypoint())
    {
      const auto& prev_wp = start_instruction.getWaypoint().as<CartesianWaypointPoly>();
      cur_plan_profile->applyStartStates(
          *config.problem, prev_wp.getTransform(), start_instruction, composite_mi, active_link_names, index);
    }
    else
    {
      throw std::runtime_error("OMPLMotionPlanner: unknown waypoint type");
    }

    return config;
  }

  if (end_instruction.getWaypoint().isCartesianWaypoint())
  {
    const auto& cur_wp = end_instruction.getWaypoint().as<CartesianWaypointPoly>();
    cur_plan_profile->applyGoalStates(
        *config.problem, cur_wp.getTransform(), end_instruction, composite_mi, active_link_names, index);

    if (index == 0)
    {
      if (start_instruction.getWaypoint().isJointWaypoint() || start_instruction.getWaypoint().isStateWaypoint())
      {
        assert(checkJointPositionFormat(joint_names, start_instruction.getWaypoint()));
        const Eigen::VectorXd& prev_position = getJointPosition(start_instruction.getWaypoint());
        cur_plan_profile->applyStartStates(
            *config.problem, prev_position, start_instruction, composite_mi, active_link_names, index);
      }
      else if (start_instruction.getWaypoint().isCartesianWaypoint())
      {
        const auto& prev_wp = start_instruction.getWaypoint().as<CartesianWaypointPoly>();
        cur_plan_profile->applyStartStates(
            *config.problem, prev_wp.getTransform(), start_instruction, composite_mi, active_link_names, index);
      }
      else
      {
        throw std::runtime_error("OMPLMotionPlanner: unknown waypoint type");
      }
    }
    else
    {
      /** @todo Update. Extract the solution for the previous plan and set as the start */
      assert(false);
    }

    return config;
  }

  throw std::runtime_error("OMPLMotionPlanner: unknown waypoint type");
}
std::vector<OMPLProblemConfig> OMPLMotionPlanner::createProblems(const PlannerRequest& request) const
{
  std::vector<OMPLProblemConfig> problems;

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!request.instructions.getManipulatorInfo().empty());

  const tesseract_common::ManipulatorInfo& composite_mi = request.instructions.getManipulatorInfo();

  tesseract_kinematics::JointGroup::Ptr manip;
  if (composite_mi.manipulator.empty())
    throw std::runtime_error("OMPL, manipulator is empty!");

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

    manip = kin_group;
  }
  catch (...)
  {
    manip = request.env->getJointGroup(composite_mi.manipulator);
  }

  if (!manip)
    throw std::runtime_error("Failed to get joint/kinematic group: " + composite_mi.manipulator);

  // Flatten the input for planning
  auto move_instructions = request.instructions.flatten(&moveFilter);

  // Transform plan instructions into ompl problem
  int index = 0;
  int num_output_states = 1;
  MoveInstructionPoly start_instruction = move_instructions.front().get().as<MoveInstructionPoly>();

  for (std::size_t i = 1; i < move_instructions.size(); ++i)
  {
    ++num_output_states;
    const auto& instruction = move_instructions[i].get();
    assert(instruction.isMoveInstruction());
    const auto& move_instruction = instruction.as<MoveInstructionPoly>();
    const auto& waypoint = move_instruction.getWaypoint();
    if (waypoint.isCartesianWaypoint() || waypoint.isStateWaypoint() ||
        (waypoint.isJointWaypoint() && waypoint.as<JointWaypointPoly>().isConstrained()))
    {
      problems.push_back(createSubProblem(
          request, composite_mi, manip, start_instruction, move_instruction, num_output_states, index++));
      start_instruction = move_instruction;
      num_output_states = 1;
    }
  }

  return problems;
}

}  // namespace tesseract_planning
