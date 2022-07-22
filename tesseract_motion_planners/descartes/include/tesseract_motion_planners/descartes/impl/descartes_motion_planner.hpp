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

namespace tesseract_planning
{
template <typename FloatType>
DescartesMotionPlanner<FloatType>::DescartesMotionPlanner(std::string name)
  : name_(std::move(name)), status_category_(std::make_shared<const DescartesMotionPlannerStatusCategory>(name_))
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
tesseract_common::StatusCode DescartesMotionPlanner<FloatType>::solve(const PlannerRequest& request,
                                                                      PlannerResponse& response,
                                                                      const bool /*verbose*/) const
{
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
      response.status =
          tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
      return response.status;
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
      response.status = tesseract_common::StatusCode(
          DescartesMotionPlannerStatusCategory::ErrorFailedToFindValidSolution, status_category_);
      return response.status;
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

    response.status =
        tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::ErrorFailedToBuildGraph, status_category_);
    return response.status;
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
  response.results = request.seed;
  auto results_flattened = flattenProgramToPattern(response.results, request.instructions);
  auto instructions_flattened = flattenProgram(request.instructions);

  // Loop over the flattened results and add them to response if the input was a plan instruction
  std::size_t result_index = 0;
  for (std::size_t idx = 0; idx < instructions_flattened.size(); idx++)
  {
    // If idx is zero then this should be the start instruction
    assert((idx == 0) ? instructions_flattened.at(idx).get().isMoveInstruction() : true);
    assert((idx == 0) ? results_flattened[idx].get().isMoveInstruction() : true);
    if (instructions_flattened.at(idx).get().isMoveInstruction())
    {
      const auto& plan_instruction = instructions_flattened.at(idx).get().as<MoveInstructionPoly>();
      if (plan_instruction.isStart())
      {
        assert(idx == 0);
        assert(results_flattened[idx].get().isMoveInstruction());
        auto& move_instruction = results_flattened[idx].get().as<MoveInstructionPoly>();

        auto& swp = move_instruction.getWaypoint().as<StateWaypointPoly>();
        swp.setPosition(solution[result_index++]);
        assert(swp.getNames() == problem->manip->getJointNames());
      }
      else if (plan_instruction.isLinear())
      {
        // This instruction corresponds to a composite. Set all results in that composite to the results
        assert(results_flattened[idx].get().isCompositeInstruction());
        auto& move_instructions = results_flattened[idx].get().as<CompositeInstruction>();
        for (auto& instruction : move_instructions)
        {
          auto& swp = instruction.as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
          swp.setPosition(solution[result_index++]);
          assert(swp.getNames() == problem->manip->getJointNames());
        }
      }
      else if (plan_instruction.isFreespace())
      {
        assert(result_index > 0);
        // Because descartes does not support freespace it just includes the plan instruction waypoint so we will
        // fill out the results with a joint interpolated trajectory.
        Eigen::VectorXd& start = solution[result_index - 1];
        Eigen::VectorXd& stop = solution[result_index++];

        // This instruction corresponds to a composite. Set all results in that composite to the results
        assert(results_flattened[idx].get().isCompositeInstruction());
        auto& move_instructions = results_flattened[idx].get().as<CompositeInstruction>();

        Eigen::MatrixXd temp = interpolate(start, stop, static_cast<int>(move_instructions.size()));

        assert(temp.cols() == static_cast<long>(move_instructions.size()) + 1);
        for (std::size_t i = 0; i < move_instructions.size(); ++i)
        {
          auto& swp = move_instructions[i].as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
          swp.setPosition(temp.col(static_cast<long>(i) + 1));
          assert(swp.getNames() == problem->manip->getJointNames());
        }
      }
      else
      {
        throw std::runtime_error("Unsupported Plan Instruction Type!");
      }
    }
  }

  response.status = tesseract_common::StatusCode(DescartesMotionPlannerStatusCategory::SolutionFound, status_category_);
  return response.status;
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
  auto instructions_flat = flattenProgram(request.instructions);
  auto seed_flat = flattenProgramToPattern(request.seed, request.instructions);

  std::size_t start_index = 0;  // If it has a start instruction then skip first instruction in instructions_flat
  int index = 0;
  std::string profile;
  ProfileDictionary::ConstPtr profile_overrides;
  WaypointPoly start_waypoint;
  MoveInstructionPoly placeholder_instruction;
  const MoveInstructionPoly* start_instruction = nullptr;
  if (request.instructions.hasStartInstruction())
  {
    start_instruction = &(request.instructions.getStartInstruction());
    assert(start_instruction->isStart());
    start_waypoint = start_instruction->getWaypoint();
    profile = start_instruction->getProfile();
    //  profile_overrides = start_instruction.profile_overrides;
    ++start_index;
  }
  else
  {
    MoveInstructionPoly temp_move(*request.instructions.getFirstMoveInstruction());
    StateWaypointPoly swp = temp_move.createStateWaypoint();
    swp.setNames(joint_names);
    swp.setPosition(request.env_state.getJointValues(joint_names));

    temp_move.assignStateWaypoint(swp);
    temp_move.setMoveType(MoveInstructionType::START);

    placeholder_instruction = temp_move;
    start_instruction = &placeholder_instruction;
    start_waypoint = swp;
  }

  profile = getProfileString(name_, profile, request.plan_profile_remapping);
  auto cur_plan_profile = getProfile<DescartesPlanProfile<FloatType>>(
      name_, profile, *request.profiles, std::make_shared<DescartesDefaultPlanProfile<FloatType>>());
  cur_plan_profile = applyProfileOverrides(name_, profile, cur_plan_profile, profile_overrides);
  if (!cur_plan_profile)
    throw std::runtime_error("DescartesMotionPlannerConfig: Invalid profile");

  // Add start waypoint
  if (start_waypoint.isCartesianWaypoint())
  {
    const auto& cwp = start_waypoint.as<CartesianWaypointPoly>();
    cur_plan_profile->apply(*prob, cwp.getTransform(), *start_instruction, composite_mi, index);
  }
  else if (start_waypoint.isJointWaypoint() || start_waypoint.isStateWaypoint())
  {
    assert(checkJointPositionFormat(joint_names, start_waypoint));
    const Eigen::VectorXd& position = getJointPosition(start_waypoint);
    cur_plan_profile->apply(*prob, position, *start_instruction, composite_mi, index);
  }
  else
  {
    throw std::runtime_error("DescartesMotionPlannerConfig: unknown waypoint type.");
  }

  ++index;

  // Transform plan instructions into descartes samplers
  for (std::size_t i = start_index; i < instructions_flat.size(); ++i)
  {
    const auto& instruction = instructions_flat[i].get();
    if (instruction.isMoveInstruction())
    {
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

      Eigen::Isometry3d tcp_offset = request.env->findTCPOffset(mi);

      // The seed should always have a start instruction
      assert(request.seed.hasStartInstruction());
      std::size_t seed_idx = (start_index == 0) ? i + 1 : i;
      assert(seed_flat[seed_idx].get().isCompositeInstruction());
      const auto& seed_composite = seed_flat[seed_idx].get().template as<tesseract_planning::CompositeInstruction>();
      auto interpolate_cnt = static_cast<int>(seed_composite.size());

      // Get Plan Profile
      std::string profile = plan_instruction.getProfile();
      profile = getProfileString(name_, profile, request.plan_profile_remapping);
      auto cur_plan_profile = getProfile<DescartesPlanProfile<FloatType>>(
          name_, profile, *request.profiles, std::make_shared<DescartesDefaultPlanProfile<FloatType>>());
      //      cur_plan_profile = applyProfileOverrides(name_, profile, cur_plan_profile,
      //      plan_instruction.profile_overrides);
      if (!cur_plan_profile)
        throw std::runtime_error("DescartesMotionPlannerConfig: Invalid profile");

      // Get Plan Path Profile: Default is an empty string
      std::string path_profile = plan_instruction.getPathProfile();
      path_profile = getProfileString(name_, path_profile, request.plan_profile_remapping, "");

      if (plan_instruction.isLinear())
      {
        auto cur_path_plan_profile = getProfile<DescartesPlanProfile<FloatType>>(
            name_, path_profile, *request.profiles, std::make_shared<DescartesDefaultPlanProfile<FloatType>>());
        //        cur_path_plan_profile =
        //            applyProfileOverrides(name_, path_profile, cur_path_plan_profile,
        //            plan_instruction.profile_overrides);
        if (!cur_path_plan_profile)
          throw std::runtime_error("DescartesMotionPlannerConfig: Invalid transition profile");

        if (plan_instruction.getWaypoint().isCartesianWaypoint())
        {
          const auto& cur_wp = plan_instruction.getWaypoint().template as<CartesianWaypointPoly>();

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (start_waypoint.isCartesianWaypoint())
          {
            prev_pose = start_waypoint.as<CartesianWaypointPoly>().getTransform();
          }
          else if (start_waypoint.isJointWaypoint() || start_waypoint.isStateWaypoint())
          {
            assert(checkJointPositionFormat(joint_names, start_waypoint));
            const Eigen::VectorXd& position = getJointPosition(start_waypoint);
            prev_pose = prob->manip->calcFwdKin(position)[mi.tcp_frame] * tcp_offset;
          }
          else
          {
            throw std::runtime_error("DescartesMotionPlannerConfig: unknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, cur_wp.getTransform(), interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            cur_path_plan_profile->apply(*prob, poses[p], plan_instruction, composite_mi, index);

            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(*prob, cur_wp.getTransform(), plan_instruction, composite_mi, index);

          ++index;
        }
        else if (plan_instruction.getWaypoint().isJointWaypoint() || plan_instruction.getWaypoint().isStateWaypoint())
        {
          assert(checkJointPositionFormat(joint_names, plan_instruction.getWaypoint()));
          const Eigen::VectorXd& cur_position = getJointPosition(plan_instruction.getWaypoint());
          Eigen::Isometry3d cur_pose = prob->manip->calcFwdKin(cur_position)[mi.tcp_frame] * tcp_offset;

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (start_waypoint.isCartesianWaypoint())
          {
            prev_pose = start_waypoint.as<CartesianWaypointPoly>().getTransform();
          }
          else if (start_waypoint.isJointWaypoint() || start_waypoint.isStateWaypoint())
          {
            assert(checkJointPositionFormat(joint_names, start_waypoint));
            const Eigen::VectorXd& position = getJointPosition(start_waypoint);
            prev_pose = prob->manip->calcFwdKin(position)[mi.tcp_frame] * tcp_offset;
          }
          else
          {
            throw std::runtime_error("DescartesMotionPlannerConfig: unknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, cur_pose, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            cur_path_plan_profile->apply(*prob, poses[p], plan_instruction, composite_mi, index);

            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(*prob, cur_position, plan_instruction, composite_mi, index);

          ++index;
        }
        else
        {
          throw std::runtime_error("DescartesMotionPlannerConfig: unknown waypoint type");
        }
      }
      else if (plan_instruction.isFreespace())
      {
        if (plan_instruction.getWaypoint().isJointWaypoint() || plan_instruction.getWaypoint().isStateWaypoint())
        {
          assert(checkJointPositionFormat(joint_names, plan_instruction.getWaypoint()));
          const Eigen::VectorXd& cur_position = getJointPosition(plan_instruction.getWaypoint());

          // Descartes does not support freespace so it will only include the plan instruction state, then in
          // post processing function will perform interpolation to fill out the seed, but may be in collision.
          // Usually this is only requested when it is being provided as a seed to a planner like trajopt.

          // Add final point with waypoint costs and constraints
          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(*prob, cur_position, plan_instruction, composite_mi, index);

          ++index;
        }
        else if (plan_instruction.getWaypoint().isCartesianWaypoint())
        {
          const auto& cur_wp = plan_instruction.getWaypoint().template as<CartesianWaypointPoly>();

          // Descartes does not support freespace so it will only include the plan instruction state, then in
          // post processing function will perform interpolation to fill out the seed, but may be in collision.
          // Usually this is only requested when it is being provided as a seed to a planner like trajopt.

          // Add final point with waypoint costs and constraints
          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(*prob, cur_wp.getTransform(), plan_instruction, composite_mi, index);

          ++index;
        }
        else
        {
          throw std::runtime_error("DescartesMotionPlannerConfig: unknown waypoint type");
        }
      }
      else
      {
        throw std::runtime_error("DescartesMotionPlannerConfig: Unsupported!");
      }

      start_waypoint = plan_instruction.getWaypoint();
      start_instruction = &plan_instruction;
    }
  }

  // Call the base class generate which checks the problem to make sure everything is in order
  return prob;
}

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_DECARTES_MOTION_PLANNER_HPP
