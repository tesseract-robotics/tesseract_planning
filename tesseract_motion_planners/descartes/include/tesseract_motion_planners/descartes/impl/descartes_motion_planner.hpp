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

#include <tesseract_kinematics/core/validate.h>

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>

#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>

namespace tesseract_planning
{
template <typename FloatType>
DescartesMotionPlanner<FloatType>::DescartesMotionPlanner(std::string name)
  : name_(std::move(name)), status_category_(std::make_shared<const DescartesMotionPlannerStatusCategory>(name_))
{
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
      problem = createProblem(name_, request);
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
  std::vector<Eigen::VectorXd> solution;
  solution.reserve(descartes_result.trajectory.size());
  for (const auto& js : descartes_result.trajectory)
  {
    solution.push_back(js->values.template cast<double>());
    // Using 1e-6 because when using floats with descartes epsilon does not seem to be enough
    assert(tesseract_common::satisfiesPositionLimits(solution.back(), problem->manip->getLimits().joint_limits));
    tesseract_common::enforcePositionLimits(solution.back(), problem->manip->getLimits().joint_limits);
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
    assert((idx == 0) ? isPlanInstruction(instructions_flattened.at(idx).get()) : true);
    assert((idx == 0) ? isMoveInstruction(results_flattened[idx].get()) : true);
    if (isPlanInstruction(instructions_flattened.at(idx).get()))
    {
      const auto& plan_instruction = instructions_flattened.at(idx).get().as<PlanInstruction>();
      if (plan_instruction.isStart())
      {
        assert(idx == 0);
        assert(isMoveInstruction(results_flattened[idx].get()));
        auto& move_instruction = results_flattened[idx].get().as<MoveInstruction>();

        auto& swp = move_instruction.getWaypoint().as<StateWaypoint>();
        swp.position = solution[result_index++];
        assert(swp.joint_names == problem->manip->getJointNames());
      }
      else if (plan_instruction.isLinear())
      {
        // This instruction corresponds to a composite. Set all results in that composite to the results
        assert(isCompositeInstruction(results_flattened[idx].get()));
        auto& move_instructions = results_flattened[idx].get().as<CompositeInstruction>();
        for (auto& instruction : move_instructions)
        {
          auto& swp = instruction.as<MoveInstruction>().getWaypoint().as<StateWaypoint>();
          swp.position = solution[result_index++];
          assert(swp.joint_names == problem->manip->getJointNames());
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
        assert(isCompositeInstruction(results_flattened[idx].get()));
        auto& move_instructions = results_flattened[idx].get().as<CompositeInstruction>();

        Eigen::MatrixXd temp = interpolate(start, stop, static_cast<int>(move_instructions.size()));

        assert(temp.cols() == static_cast<long>(move_instructions.size()) + 1);
        for (std::size_t i = 0; i < move_instructions.size(); ++i)
        {
          auto& swp = move_instructions[i].as<MoveInstruction>().getWaypoint().as<StateWaypoint>();
          swp.position = temp.col(static_cast<long>(i) + 1);
          assert(swp.joint_names == problem->manip->getJointNames());
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
DescartesMotionPlanner<FloatType>::createProblem(const std::string& name, const PlannerRequest& request) const
{
  auto prob = std::make_shared<DescartesProblem<FloatType>>();

  // Clear descartes data
  prob->edge_evaluators.clear();
  prob->samplers.clear();
  prob->state_evaluators.clear();

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!request.instructions.getManipulatorInfo().empty());
  const ManipulatorInfo& composite_mi = request.instructions.getManipulatorInfo();

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

  // Process instructions
  if (!tesseract_kinematics::checkKinematics(*(prob->manip)))
    CONSOLE_BRIDGE_logError("Check Kinematics failed. This means that Inverse Kinematics does not agree with KDL "
                            "(TrajOpt). Did you change the URDF recently?");

  std::vector<std::string> joint_names = prob->manip->getJointNames();

  // Flatten the input for planning
  auto instructions_flat = flattenProgram(request.instructions);
  auto seed_flat = flattenProgramToPattern(request.seed, request.instructions);

  std::size_t start_index = 0;  // If it has a start instruction then skip first instruction in instructions_flat
  int index = 0;
  std::string profile;
  ProfileDictionary::ConstPtr profile_overrides;
  Waypoint start_waypoint{ NullWaypoint() };
  Instruction placeholder_instruction{ NullInstruction() };
  const Instruction* start_instruction = nullptr;
  if (request.instructions.hasStartInstruction())
  {
    assert(isPlanInstruction(request.instructions.getStartInstruction()));
    start_instruction = &(request.instructions.getStartInstruction());
    if (isPlanInstruction(*start_instruction))
    {
      const auto& temp = start_instruction->as<PlanInstruction>();
      assert(temp.isStart());
      start_waypoint = temp.getWaypoint();
      profile = temp.getProfile();
      profile_overrides = temp.profile_overrides;
    }
    else
    {
      throw std::runtime_error("Descartes DefaultProblemGenerator: Unsupported start instruction type!");
    }
    ++start_index;
  }
  else
  {
    Eigen::VectorXd current_jv = request.env_state.getJointValues(joint_names);
    StateWaypoint swp(joint_names, current_jv);

    MoveInstruction temp_move(swp, MoveInstructionType::START);
    placeholder_instruction = temp_move;
    start_instruction = &placeholder_instruction;
    start_waypoint = swp;
  }

  profile = getProfileString(profile, name, request.plan_profile_remapping);
  auto cur_plan_profile = getProfile<DescartesPlanProfile<FloatType>>(
      profile, *request.profiles, std::make_shared<DescartesDefaultPlanProfile<FloatType>>());
  cur_plan_profile = applyProfileOverrides(name, cur_plan_profile, profile_overrides);
  if (!cur_plan_profile)
    throw std::runtime_error("DescartesMotionPlannerConfig: Invalid profile");

  // Add start waypoint
  if (isCartesianWaypoint(start_waypoint))
  {
    const auto& cwp = start_waypoint.as<CartesianWaypoint>();
    cur_plan_profile->apply(*prob, cwp.waypoint, *start_instruction, composite_mi, index);
  }
  else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
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
    if (isPlanInstruction(instruction))
    {
      assert(isPlanInstruction(instruction));
      const auto& plan_instruction = instruction.template as<PlanInstruction>();

      // If plan instruction has manipulator information then use it over the one provided by the composite.
      ManipulatorInfo mi = composite_mi.getCombined(plan_instruction.getManipulatorInfo());

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
      assert(isCompositeInstruction(seed_flat[seed_idx].get()));
      const auto& seed_composite = seed_flat[seed_idx].get().template as<tesseract_planning::CompositeInstruction>();
      auto interpolate_cnt = static_cast<int>(seed_composite.size());

      // Get Plan Profile
      std::string profile = plan_instruction.getProfile();
      profile = getProfileString(profile, name, request.plan_profile_remapping);
      auto cur_plan_profile = getProfile<DescartesPlanProfile<FloatType>>(
          profile, *request.profiles, std::make_shared<DescartesDefaultPlanProfile<FloatType>>());
      cur_plan_profile = applyProfileOverrides(name, cur_plan_profile, plan_instruction.profile_overrides);
      if (!cur_plan_profile)
        throw std::runtime_error("DescartesMotionPlannerConfig: Invalid profile");

      if (plan_instruction.isLinear())
      {
        if (isCartesianWaypoint(plan_instruction.getWaypoint()))
        {
          const auto& cur_wp = plan_instruction.getWaypoint().template as<tesseract_planning::CartesianWaypoint>();

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (isCartesianWaypoint(start_waypoint))
          {
            prev_pose = start_waypoint.as<CartesianWaypoint>().waypoint;
          }
          else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
          {
            assert(checkJointPositionFormat(joint_names, start_waypoint));
            const Eigen::VectorXd& position = getJointPosition(start_waypoint);
            prev_pose = prob->manip->calcFwdKin(position)[mi.tcp_frame] * tcp_offset;
          }
          else
          {
            throw std::runtime_error("DescartesMotionPlannerConfig: unknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, cur_wp, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            cur_plan_profile->apply(*prob, poses[p], plan_instruction, composite_mi, index);

            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(*prob, cur_wp, plan_instruction, composite_mi, index);

          ++index;
        }
        else if (isJointWaypoint(plan_instruction.getWaypoint()) || isStateWaypoint(plan_instruction.getWaypoint()))
        {
          assert(checkJointPositionFormat(joint_names, plan_instruction.getWaypoint()));
          const Eigen::VectorXd& cur_position = getJointPosition(plan_instruction.getWaypoint());
          Eigen::Isometry3d cur_pose = prob->manip->calcFwdKin(cur_position)[mi.tcp_frame] * tcp_offset;

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (isCartesianWaypoint(start_waypoint))
          {
            prev_pose = start_waypoint.as<CartesianWaypoint>().waypoint;
          }
          else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
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
            cur_plan_profile->apply(*prob, poses[p], plan_instruction, composite_mi, index);

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
        if (isJointWaypoint(plan_instruction.getWaypoint()) || isStateWaypoint(plan_instruction.getWaypoint()))
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
        else if (isCartesianWaypoint(plan_instruction.getWaypoint()))
        {
          const auto& cur_wp = plan_instruction.getWaypoint().template as<tesseract_planning::CartesianWaypoint>();

          // Descartes does not support freespace so it will only include the plan instruction state, then in
          // post processing function will perform interpolation to fill out the seed, but may be in collision.
          // Usually this is only requested when it is being provided as a seed to a planner like trajopt.

          // Add final point with waypoint costs and constraints
          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(*prob, cur_wp, plan_instruction, composite_mi, index);

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
      start_instruction = &instruction;
    }
  }

  // Call the base class generate which checks the problem to make sure everything is in order
  return prob;
}

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_DECARTES_MOTION_PLANNER_HPP
