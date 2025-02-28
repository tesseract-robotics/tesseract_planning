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
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/kinematic_limits.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_environment/environment.h>

#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_ladder_graph_solver_profile.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/planner_utils.h>

constexpr auto SOLUTION_FOUND{ "Found valid solution" };
constexpr auto ERROR_INVALID_INPUT{ "Failed invalid input: " };
constexpr auto ERROR_FAILED_TO_BUILD_GRAPH{ "Failed to build graph" };
constexpr auto ERROR_FAILED_TO_FIND_VALID_SOLUTION{ "Failed to find valid solution" };

namespace tesseract_planning
{
template <typename FloatType>
DescartesMotionPlanner<FloatType>::DescartesMotionPlanner(std::string name) : MotionPlanner(std::move(name))  // NOLINT
{
}

template <typename FloatType>
PlannerResponse DescartesMotionPlanner<FloatType>::solve(const PlannerRequest& request) const
{
  PlannerResponse response;

  // Get solver config
  auto solver_profile =
      getProfile<DescartesSolverProfile<FloatType>>(name_,
                                                    request.instructions.getProfile(name_),
                                                    *request.profiles,
                                                    std::make_shared<DescartesLadderGraphSolverProfile<FloatType>>());

  auto solver = solver_profile->create();

  std::vector<typename descartes_light::EdgeEvaluator<FloatType>::ConstPtr> edge_evaluators;
  std::vector<typename descartes_light::WaypointSampler<FloatType>::ConstPtr> waypoint_samplers;
  std::vector<typename descartes_light::StateEvaluator<FloatType>::ConstPtr> state_evaluators;

  const tesseract_common::ManipulatorInfo& composite_mi = request.instructions.getManipulatorInfo();
  if (composite_mi.empty())
    throw std::runtime_error("Descartes, manipulator info is empty!");

  // Flatten the input for planning
  auto move_instructions = request.instructions.flatten(&moveFilter);

  // Transform plan instructions into descartes samplers
  int index = 0;
  for (const auto& instruction : move_instructions)
  {
    assert(instruction.get().isMoveInstruction());
    const auto& move_instruction = instruction.get().template as<MoveInstructionPoly>();

    // Get Plan Profile
    auto cur_plan_profile =
        getProfile<DescartesPlanProfile<FloatType>>(name_,
                                                    move_instruction.getProfile(name_),
                                                    *request.profiles,
                                                    std::make_shared<DescartesDefaultPlanProfile<FloatType>>());

    if (!cur_plan_profile)
      throw std::runtime_error("DescartesMotionPlanner: Invalid profile");

    if (move_instruction.getWaypoint().isJointWaypoint() &&
        !move_instruction.getWaypoint().as<JointWaypointPoly>().isConstrained())
      continue;

    waypoint_samplers.push_back(cur_plan_profile->createWaypointSampler(move_instruction, composite_mi, request.env));
    state_evaluators.push_back(cur_plan_profile->createStateEvaluator(move_instruction, composite_mi, request.env));
    if (index != 0)
      edge_evaluators.push_back(cur_plan_profile->createEdgeEvaluator(move_instruction, composite_mi, request.env));

    ++index;
  }

  descartes_light::SearchResult<FloatType> descartes_result;
  try
  {
    // Build Graph
    if (!solver->build(waypoint_samplers, edge_evaluators, state_evaluators))
    {
      response.successful = false;
      response.message = ERROR_FAILED_TO_BUILD_GRAPH;
      return response;
    }

    // Search Graph
    descartes_result = solver->search();
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
    response.successful = false;
    response.message = ERROR_FAILED_TO_BUILD_GRAPH;
    return response;
  }

  response.data = std::move(solver);

  // Get Manipulator Information
  tesseract_kinematics::JointGroup::ConstPtr manip = request.env->getJointGroup(composite_mi.manipulator);
  const std::vector<std::string> joint_names = manip->getJointNames();
  const Eigen::MatrixX2d joint_limits = manip->getLimits().joint_limits;

  // Enforce limits
  std::vector<Eigen::VectorXd> solution{};
  solution.reserve(descartes_result.trajectory.size());
  for (const auto& js : descartes_result.trajectory)
  {
    solution.push_back(js->values.template cast<double>());
    assert(tesseract_common::satisfiesLimits<double>(solution.back(), joint_limits));
    tesseract_common::enforceLimits<double>(solution.back(), joint_limits);
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
      auto& move_instruction = results_instructions.at(idx).get().as<MoveInstructionPoly>();
      if (move_instruction.getWaypoint().isCartesianWaypoint())
      {
        assignSolution(move_instruction, joint_names, solution[result_index++], request.format_result_as_input);
      }
      else if (move_instruction.getWaypoint().isJointWaypoint())
      {
        auto& jwp = move_instruction.getWaypoint().as<JointWaypointPoly>();
        if (jwp.isConstrained())
        {
          assignSolution(move_instruction, joint_names, solution[result_index++], request.format_result_as_input);
          continue;
        }

        const Eigen::VectorXd& start_state = solution[result_index - 1];
        Eigen::Index cnt = 1;
        bool is_constrained = jwp.isConstrained();
        while (!is_constrained)
        {
          auto temp = results_instructions.at(idx + static_cast<std::size_t>(cnt++)).get().as<MoveInstructionPoly>();
          if (temp.getWaypoint().isCartesianWaypoint() || temp.getWaypoint().isStateWaypoint())
            is_constrained = true;
          else if (temp.getWaypoint().isJointWaypoint())
            is_constrained = temp.getWaypoint().as<JointWaypointPoly>().isConstrained();
          else
            throw std::runtime_error("Unsupported Waypoint Type!");
        }
        const Eigen::VectorXd& end_state = solution[result_index];

        Eigen::MatrixXd states = interpolate(start_state, end_state, cnt);
        for (Eigen::Index i = 0; i < cnt - 1; ++i)
        {
          if (i != 0)
            ++idx;
          auto& interp_mi = results_instructions.at(idx).get().as<MoveInstructionPoly>();
          assignSolution(interp_mi, joint_names, states.col(i + 1), request.format_result_as_input);
        }
      }
      else if (move_instruction.getWaypoint().isStateWaypoint())
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
std::unique_ptr<MotionPlanner> DescartesMotionPlanner<FloatType>::clone() const
{
  return std::make_unique<DescartesMotionPlanner<FloatType>>(name_);
}

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_DECARTES_MOTION_PLANNER_HPP
