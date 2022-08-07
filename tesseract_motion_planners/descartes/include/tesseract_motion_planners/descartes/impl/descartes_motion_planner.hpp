/**
 * @file descartes_motion_planner.hpp
 * @brief Tesseract ROS Descartes planner
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
#ifndef TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_DECARTES_MOTION_PLANNER_HPP
#define TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_DECARTES_MOTION_PLANNER_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>
#include <descartes_light/samplers/fixed_joint_waypoint_sampler.h>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>

#include <tesseract_command_language/utils.h>

constexpr auto SOLUTION_FOUND{ "Found valid solution" };
constexpr auto ERROR_INVALID_INPUT{ "Failed invalid input" };
constexpr auto ERROR_FAILED_TO_BUILD_GRAPH{ "Failed to build graph" };
constexpr auto ERROR_FAILED_TO_FIND_VALID_SOLUTION{ "Failed to find valid solution" };

namespace tesseract_planning
{
template <typename FloatType>
DescartesMotionPlanner<FloatType>::DescartesMotionPlanner(std::string name) : name_(std::move(name))
{
  if (name_.empty())
    throw std::runtime_error("DescartesMotionPlanner name is empty!");
}

template <typename FloatType>
const std::string& DescartesMotionPlanner<FloatType>::getName() const
{
  return name_;
}

template <typename FloatType>
PlannerResponse DescartesMotionPlanner<FloatType>::solve(const PlannerRequest& request) const
{
  PlannerResponse response;
  std::shared_ptr<DescartesProblem<FloatType>> problem;
  if (request.data)
  {
    problem = std::static_pointer_cast<DescartesProblem<FloatType>>(request.data);
  }
  else
  {
    try
    {
      problem = createProblem(request);
    }
    catch (std::exception& e)
    {
      CONSOLE_BRIDGE_logError("DescartesMotionPlanner failed to generate problem: %s.", e.what());
      response.successful = false;
      response.message = ERROR_INVALID_INPUT;
      return response;
    }

    response.data = problem;
  }

  descartes_light::SearchResult<FloatType> descartes_result;
  try
  {
    descartes_light::LadderGraphSolver<FloatType> solver(problem->num_threads);
    solver.build(problem->samplers, problem->edge_evaluators, problem->state_evaluators);
    descartes_result = solver.search();
    if (descartes_result.trajectory.empty())
    {
      CONSOLE_BRIDGE_logError("Search for graph completion failed");
      response.successful = false;
      response.message = ERROR_FAILED_TO_FIND_VALID_SOLUTION;
      return response;
    }
  }
  catch (...)
  {
    //    CONSOLE_BRIDGE_logError("Failed to build vertices");
    //    for (const auto& i : graph_builder.getFailedVertices())
    //      response.failed_waypoints.push_back(config_->waypoints[i]);

    //    // Copy the waypoint if it is not already in the failed waypoints list
    //    std::copy_if(config_->waypoints.begin(),
    //                 config_->waypoints.end(),
    //                 std::back_inserter(response.succeeded_waypoints),
    //                 [&response](const Waypoint::ConstPtr wp) {
    //                   return std::find(response.failed_waypoints.begin(), response.failed_waypoints.end(), wp) ==
    //                          response.failed_waypoints.end();
    //                 });

    response.successful = false;
    response.message = ERROR_FAILED_TO_BUILD_GRAPH;
    return response;
  }

  // Enforce limits
  std::vector<Eigen::VectorXd> solution{};
  solution.reserve(descartes_result.trajectory.size());
  for (const auto& js : descartes_result.trajectory)
  {
    solution.push_back(js->values.template cast<double>());
    // Using 1e-6 because when using floats with descartes epsilon does not seem to be enough
    assert(
        tesseract_common::satisfiesPositionLimits<double>(solution.back(), problem->manip->getLimits().joint_limits));
    tesseract_common::enforcePositionLimits<double>(solution.back(), problem->manip->getLimits().joint_limits);
  }

  // Flatten the results to make them easier to process
  response.results = request.instructions;
  auto results_instructions = response.results.flatten(&moveFilter);

  // Loop over the flattened results and add them to response if the input was a plan instruction
  std::size_t result_index{ 0 };
  for (std::size_t idx = 0; idx < results_instructions.size(); idx++)
  {
    // If idx is zero then this should be the start instruction
    assert((idx == 0) ? results_instructions.at(idx).get().isMoveInstruction() : true);
    if (results_instructions.at(idx).get().isMoveInstruction())
    {
      auto& plan_instruction = results_instructions.at(idx).get().as<MoveInstructionPoly>();
      if (plan_instruction.getWaypoint().isCartesianWaypoint())
      {
        StateWaypointPoly swp = plan_instruction.createStateWaypoint();
        swp.setNames(problem->manip->getJointNames());
        swp.setPosition(solution[result_index++]);
        plan_instruction.assignStateWaypoint(swp);
      }
      else if (plan_instruction.getWaypoint().isJointWaypoint())
      {
        auto& jwp = plan_instruction.getWaypoint().as<JointWaypointPoly>();
        if (jwp.isConstrained())
        {
          StateWaypointPoly swp = plan_instruction.createStateWaypoint();
          swp.setNames(problem->manip->getJointNames());
          swp.setPosition(solution[result_index++]);
          plan_instruction.assignStateWaypoint(swp);
          continue;
        }

        const Eigen::VectorXd& start_state = solution[result_index - 1];
        Eigen::Index cnt = 0;
        bool is_constrained = jwp.isConstrained();
        while (!is_constrained)
        {
          auto temp = results_instructions.at(idx + static_cast<std::size_t>(++cnt)).get().as<MoveInstructionPoly>();
          if (temp.getWaypoint().isCartesianWaypoint() || temp.getWaypoint().isStateWaypoint())
            is_constrained = true;
          else if (temp.getWaypoint().isJointWaypoint())
            is_constrained = temp.getWaypoint().as<JointWaypointPoly>().isConstrained();
          else
            throw std::runtime_error("Unsupported Waypoint Type!");
        }
        const Eigen::VectorXd& end_state = solution[result_index++];

        Eigen::MatrixXd states = interpolate(start_state, end_state, cnt);
        for (Eigen::Index i = 0; i < cnt; ++i)
        {
          StateWaypointPoly swp = plan_instruction.createStateWaypoint();
          swp.setNames(problem->manip->getJointNames());
          swp.setPosition(states.col(i + 1));
          results_instructions.at(idx++).get().as<MoveInstructionPoly>().assignStateWaypoint(swp);
        }
      }
      else if (plan_instruction.getWaypoint().isStateWaypoint())
      {
        ++result_index;
      }
      else
      {
        throw std::runtime_error("Unsupported Waypoint Type!");
      }
    }
  }

  response.successful = true;
  response.message = SOLUTION_FOUND;
  return response;
}

template <typename FloatType>
bool DescartesMotionPlanner<FloatType>::checkUserInput(const PlannerRequest& request)
{
  // Check that parameters are valid
  if (request.env == nullptr)
  {
    CONSOLE_BRIDGE_logError("In TrajOptPlannerUniversalConfig: env is a required parameter and has not been set");
    return false;
  }

  if (request.instructions.empty())
  {
    CONSOLE_BRIDGE_logError("TrajOptPlannerUniversalConfig requires at least one instruction");
    return false;
  }

  return true;
}

template <typename FloatType>
bool DescartesMotionPlanner<FloatType>::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

template <typename FloatType>
void DescartesMotionPlanner<FloatType>::clear()
{
}

template <typename FloatType>
MotionPlanner::Ptr DescartesMotionPlanner<FloatType>::clone() const
{
  return std::make_shared<DescartesMotionPlanner<FloatType>>(name_);
}

template <typename FloatType>
std::shared_ptr<DescartesProblem<FloatType>>
DescartesMotionPlanner<FloatType>::createProblem(const PlannerRequest& request) const
{
  auto prob = std::make_shared<DescartesProblem<FloatType>>();

  // Clear descartes data
  prob->edge_evaluators.clear();
  prob->samplers.clear();
  prob->state_evaluators.clear();

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!request.instructions.getManipulatorInfo().empty());
  const tesseract_common::ManipulatorInfo& composite_mi = request.instructions.getManipulatorInfo();

  if (composite_mi.manipulator.empty())
    throw std::runtime_error("Descartes, manipulator is empty!");

  if (!request.instructions.hasStartInstruction())
    throw std::runtime_error("Descartes, missing a start instruction!");

  // Get Manipulator Information
  try
  {
    if (composite_mi.manipulator_ik_solver.empty())
      prob->manip = request.env->getKinematicGroup(composite_mi.manipulator);
    else
      prob->manip = request.env->getKinematicGroup(composite_mi.manipulator, composite_mi.manipulator_ik_solver);
  }
  catch (...)
  {
    throw std::runtime_error("Descartes problem generator failed to create kinematic group!");
  }

  if (!prob->manip)
  {
    CONSOLE_BRIDGE_logError("No Kinematics Group found");
    return prob;
  }

  prob->env_state = request.env_state;
  prob->env = request.env;

  std::vector<std::string> joint_names = prob->manip->getJointNames();

  // Flatten the input for planning
  auto move_instructions = request.instructions.flatten(&moveFilter);

  // Transform plan instructions into descartes samplers
  int index = 0;
  for (std::size_t i = 0; i < move_instructions.size(); ++i)
  {
    const auto& instruction = move_instructions[i].get();

    assert(instruction.isMoveInstruction());
    const auto& plan_instruction = instruction.template as<MoveInstructionPoly>();

    // If plan instruction has manipulator information then use it over the one provided by the composite.
    tesseract_common::ManipulatorInfo mi = composite_mi.getCombined(plan_instruction.getManipulatorInfo());

    if (mi.manipulator.empty())
      throw std::runtime_error("Descartes, manipulator is empty!");

    if (mi.tcp_frame.empty())
      throw std::runtime_error("Descartes, tcp_frame is empty!");

    if (mi.working_frame.empty())
      throw std::runtime_error("Descartes, working_frame is empty!");

    // Get Plan Profile
    std::string profile = plan_instruction.getProfile();
    profile = getProfileString(name_, profile, request.plan_profile_remapping);
    auto cur_plan_profile = getProfile<DescartesPlanProfile<FloatType>>(
        name_, profile, *request.profiles, std::make_shared<DescartesDefaultPlanProfile<FloatType>>());
    //      cur_plan_profile = applyProfileOverrides(name_, profile, cur_plan_profile,
    //      plan_instruction.profile_overrides);
    if (!cur_plan_profile)
      throw std::runtime_error("DescartesMotionPlannerConfig: Invalid profile");

    if (plan_instruction.getWaypoint().isCartesianWaypoint())
    {
      const auto& cur_wp = plan_instruction.getWaypoint().template as<CartesianWaypointPoly>();
      cur_plan_profile->apply(*prob, cur_wp.getTransform(), plan_instruction, composite_mi, index);
      ++index;
    }
    else if (plan_instruction.getWaypoint().isJointWaypoint())
    {
      if (plan_instruction.getWaypoint().as<JointWaypointPoly>().isConstrained())
      {
        assert(checkJointPositionFormat(joint_names, plan_instruction.getWaypoint()));
        const Eigen::VectorXd& cur_position = getJointPosition(plan_instruction.getWaypoint());
        cur_plan_profile->apply(*prob, cur_position, plan_instruction, composite_mi, index);
        ++index;
      }
    }
    else if (plan_instruction.getWaypoint().isStateWaypoint())
    {
      assert(checkJointPositionFormat(joint_names, plan_instruction.getWaypoint()));
      const Eigen::VectorXd& cur_position = getJointPosition(plan_instruction.getWaypoint());
      cur_plan_profile->apply(*prob, cur_position, plan_instruction, composite_mi, index);
      ++index;
    }
    else
    {
      throw std::runtime_error("DescartesMotionPlannerConfig: unknown waypoint type");
    }
  }

  // Call the base class generate which checks the problem to make sure everything is in order
  return prob;
}

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_DECARTES_MOTION_PLANNER_HPP
