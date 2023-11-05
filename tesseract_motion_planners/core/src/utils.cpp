/**
 * @file utils.h
 * @brief Planner utility functions.
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
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
Eigen::Isometry3d calcPose(const WaypointPoly& wp,
                           const std::string& working_frame,
                           const std::string& tip_link,
                           const Eigen::Isometry3d& tcp,
                           const tesseract_scene_graph::SceneState& current_state,
                           tesseract_scene_graph::StateSolver& state_solver)
{
  if (wp.isStateWaypoint())
  {
    const auto& swp = wp.as<StateWaypointPoly>();
    assert(static_cast<long>(swp.getNames().size()) == swp.getPosition().size());
    tesseract_scene_graph::SceneState state = state_solver.getState(swp.getNames(), swp.getPosition());
    return (state.link_transforms[tip_link] * tcp);
  }

  if (wp.isJointWaypoint())
  {
    const auto& jwp = wp.as<JointWaypointPoly>();
    assert(static_cast<long>(jwp.getNames().size()) == jwp.getPosition().size());
    tesseract_scene_graph::SceneState state = state_solver.getState(jwp.getNames(), jwp.getPosition());
    return (state.link_transforms[tip_link] * tcp);
  }

  if (wp.isCartesianWaypoint())
  {
    const auto& cwp = wp.as<CartesianWaypointPoly>();
    if (working_frame.empty())
      return cwp.getTransform();

    return (current_state.link_transforms.at(working_frame) * cwp.getTransform());
  }

  throw std::runtime_error("toToolpath: Unsupported Waypoint Type!");
}

tesseract_common::Toolpath toToolpath(const InstructionPoly& instruction, const tesseract_environment::Environment& env)
{
  using namespace tesseract_planning;
  if (instruction.isCompositeInstruction())
  {
    const auto& ci = instruction.as<CompositeInstruction>();
    return toToolpath(ci, env);
  }

  if (instruction.isMoveInstruction())
  {
    const auto& mi = instruction.as<MoveInstructionPoly>();
    return toToolpath(mi, env);
  }

  throw std::runtime_error("toToolpath: Unsupported Instruction Type!");
}

tesseract_common::Toolpath toToolpath(const CompositeInstruction& ci, const tesseract_environment::Environment& env)
{
  tesseract_common::Toolpath toolpath;
  tesseract_common::VectorIsometry3d poses;

  tesseract_scene_graph::StateSolver::UPtr state_solver = env.getStateSolver();
  tesseract_scene_graph::SceneState state = env.getState();

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!ci.getManipulatorInfo().empty());
  const tesseract_common::ManipulatorInfo& composite_mi = ci.getManipulatorInfo();

  std::vector<std::reference_wrapper<const InstructionPoly>> fi = ci.flatten(moveFilter);
  for (const auto& i : fi)
  {
    tesseract_common::ManipulatorInfo manip_info;

    // Check for updated manipulator information and get waypoint
    WaypointPoly wp;
    if (i.get().isMoveInstruction())
    {
      const auto& mi = i.get().as<MoveInstructionPoly>();
      manip_info = composite_mi.getCombined(mi.getManipulatorInfo());
      wp = mi.getWaypoint();
    }
    else
    {
      throw std::runtime_error("toToolpath: Unsupported Instruction Type!");
    }

    // Extract TCP
    Eigen::Isometry3d tcp_offset = env.findTCPOffset(manip_info);

    // Calculate pose
    poses.push_back(calcPose(wp, manip_info.working_frame, manip_info.tcp_frame, tcp_offset, state, *state_solver));
  }

  toolpath.push_back(poses);
  return toolpath;
}

tesseract_common::Toolpath toToolpath(const MoveInstructionPoly& mi, const tesseract_environment::Environment& env)
{
  tesseract_common::Toolpath toolpath;
  tesseract_common::VectorIsometry3d poses;

  tesseract_scene_graph::StateSolver::UPtr state_solver = env.getStateSolver();
  tesseract_scene_graph::SceneState state = env.getState();

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!mi.getManipulatorInfo().empty());
  const tesseract_common::ManipulatorInfo& composite_mi = mi.getManipulatorInfo();
  tesseract_common::ManipulatorInfo manip_info = composite_mi.getCombined(mi.getManipulatorInfo());

  // Extract TCP
  Eigen::Isometry3d tcp_offset = env.findTCPOffset(manip_info);

  // Calculate pose
  poses.push_back(
      calcPose(mi.getWaypoint(), manip_info.working_frame, manip_info.tcp_frame, tcp_offset, state, *state_solver));

  toolpath.push_back(poses);
  return toolpath;
}

void assignCurrentStateAsSeed(CompositeInstruction& composite_instructions,
                              const tesseract_environment::Environment& env)
{
  std::unordered_map<std::string, tesseract_common::JointState> manip_joint_state;
  tesseract_scene_graph::SceneState state = env.getState();
  const tesseract_common::ManipulatorInfo& global_mi = composite_instructions.getManipulatorInfo();

  std::vector<std::reference_wrapper<InstructionPoly>> mv_instructions = composite_instructions.flatten(moveFilter);
  for (auto& i : mv_instructions)
  {
    auto& mvi = i.get().as<MoveInstructionPoly>();
    if (mvi.getWaypoint().isCartesianWaypoint())
    {
      auto& cwp = mvi.getWaypoint().as<CartesianWaypointPoly>();
      tesseract_common::ManipulatorInfo mi = global_mi.getCombined(mvi.getManipulatorInfo());
      auto it = manip_joint_state.find(mi.manipulator);
      if (it != manip_joint_state.end())
      {
        cwp.setSeed(it->second);
      }
      else
      {
        std::vector<std::string> joint_names = env.getGroupJointNames(mi.manipulator);
        Eigen::VectorXd jv = state.getJointValues(joint_names);
        tesseract_common::JointState seed(joint_names, jv);
        manip_joint_state[mi.manipulator] = seed;
        cwp.setSeed(seed);
      }
    }
  }
}

bool formatProgramHelper(CompositeInstruction& composite_instructions,
                         const tesseract_environment::Environment& env,
                         const tesseract_common::ManipulatorInfo& manip_info,
                         std::unordered_map<std::string, std::vector<std::string>>& manip_joint_names)
{
  bool format_required = false;
  for (auto& i : composite_instructions)
  {
    if (i.isCompositeInstruction())
    {
      if (formatProgramHelper(i.as<CompositeInstruction>(), env, manip_info, manip_joint_names))
        format_required = true;
    }
    else if (i.isMoveInstruction())
    {
      auto& base_instruction = i.as<MoveInstructionPoly>();
      tesseract_common::ManipulatorInfo mi = manip_info.getCombined(base_instruction.getManipulatorInfo());

      tesseract_common::ManipulatorInfo combined_mi = mi.getCombined(base_instruction.getManipulatorInfo());

      std::vector<std::string> joint_names;
      auto it = manip_joint_names.find(combined_mi.manipulator);
      if (it == manip_joint_names.end())
      {
        joint_names = env.getGroupJointNames(combined_mi.manipulator);
        manip_joint_names[combined_mi.manipulator] = joint_names;
      }
      else
      {
        joint_names = it->second;
      }

      if (base_instruction.getWaypoint().isStateWaypoint() || base_instruction.getWaypoint().isJointWaypoint())
      {
        if (formatJointPosition(joint_names, base_instruction.getWaypoint()))
          format_required = true;
      }
    }
  }
  return format_required;
}

bool formatProgram(CompositeInstruction& composite_instructions, const tesseract_environment::Environment& env)
{
  std::unordered_map<std::string, std::vector<std::string>> manip_joint_names;
  bool format_required = false;
  tesseract_common::ManipulatorInfo mi = composite_instructions.getManipulatorInfo();

  if (formatProgramHelper(composite_instructions, env, mi, manip_joint_names))
    format_required = true;

  return format_required;
}

void printContinuousDebugInfo(const std::vector<std::string>& joint_names,
                              const Eigen::VectorXd& swp0,
                              const Eigen::VectorXd& swp1,
                              std::size_t step_idx,
                              std::size_t step_size,
                              long sub_step_idx = -1)
{
  std::stringstream ss;
  ss << "Continuous collision detected at step: " << step_idx << " of " << step_size;
  if (sub_step_idx >= 0)
    ss << " substep: " << sub_step_idx;
  ss << std::endl;

  ss << "     Names:";
  for (const auto& name : joint_names)
    ss << " " << name;

  ss << std::endl << "    State0: " << swp0 << std::endl << "    State1: " << swp1 << std::endl;

  CONSOLE_BRIDGE_logDebug(ss.str().c_str());
}

void printDiscreteDebugInfo(const std::vector<std::string>& joint_names,
                            const Eigen::VectorXd& swp,
                            std::size_t step_idx,
                            std::size_t step_size,
                            long sub_step_idx = -1)
{
  std::stringstream ss;
  ss << "Discrete collision detected at step: " << step_idx << " of " << step_size;
  if (sub_step_idx >= 0)
    ss << " substep: " << sub_step_idx;
  ss << std::endl;

  ss << "     Names:";
  for (const auto& name : joint_names)
    ss << " " << name;

  ss << std::endl << "    State: " << swp << std::endl;

  CONSOLE_BRIDGE_logDebug(ss.str().c_str());
}

bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                         tesseract_collision::ContinuousContactManager& manager,
                         const tesseract_scene_graph::StateSolver& state_solver,
                         const CompositeInstruction& program,
                         const tesseract_collision::CollisionCheckConfig& config)
{
  if (config.type != tesseract_collision::CollisionEvaluatorType::CONTINUOUS &&
      config.type != tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
    throw std::runtime_error("contactCheckProgram was given an CollisionEvaluatorType that is inconsistent with the "
                             "ContactManager type (Continuous)");

  // Flatten results
  std::vector<std::reference_wrapper<const InstructionPoly>> mi = program.flatten(moveFilter);

  if (mi.size() < 2)
    throw std::runtime_error("contactCheckProgram was given continuous contact manager with a trajectory that only has "
                             "one state.");

  manager.applyContactManagerConfig(config.contact_manager_config);

  bool debug_logging = console_bridge::getLogLevel() < console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO;

  tesseract_collision::ContactTrajectoryResults::UPtr traj_contacts;
  if (debug_logging)
  {
    // Grab the first waypoint to get the joint names
    const auto& joint_names = getJointNames(mi.front().get().as<MoveInstructionPoly>().getWaypoint());
    traj_contacts =
        std::make_unique<tesseract_collision::ContactTrajectoryResults>(joint_names, static_cast<int>(program.size()));
  }

  contacts.clear();
  contacts.reserve(mi.size());

  /** @brief Making this thread_local does not help because it is not called enough during planning */
  tesseract_collision::ContactResultMap state_results;
  tesseract_collision::ContactResultMap sub_state_results;

  bool found = false;
  if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::START_ONLY)
  {
    const auto& joint_names = getJointNames(mi.front().get().as<MoveInstructionPoly>().getWaypoint());
    const auto& joint_positions = getJointPosition(mi.front().get().as<MoveInstructionPoly>().getWaypoint());
    tesseract_scene_graph::SceneState state = state_solver.getState(joint_names, joint_positions);
    sub_state_results.clear();
    tesseract_environment::checkTrajectoryState(
        sub_state_results, manager, state.link_transforms, config.contact_request);

    if (!sub_state_results.empty())
    {
      found = true;
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, false);
      if (debug_logging)
        printContinuousDebugInfo(joint_names, joint_positions, joint_positions, 0, mi.size() - 1);
    }
    contacts.push_back(state_results);
    return found;
  }

  if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::END_ONLY)
  {
    const auto& joint_names = getJointNames(mi.back().get().as<MoveInstructionPoly>().getWaypoint());
    const auto& joint_positions = getJointPosition(mi.back().get().as<MoveInstructionPoly>().getWaypoint());
    tesseract_scene_graph::SceneState state = state_solver.getState(joint_names, joint_positions);
    sub_state_results.clear();
    tesseract_environment::checkTrajectoryState(
        sub_state_results, manager, state.link_transforms, config.contact_request);

    if (!sub_state_results.empty())
    {
      found = true;
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, false);
      if (debug_logging)
        printContinuousDebugInfo(joint_names, joint_positions, joint_positions, 0, mi.size() - 1);
    }
    contacts.push_back(state_results);
    return found;
  }

  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
  {
    assert(config.longest_valid_segment_length > 0);

    for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
    {
      state_results.clear();

      const auto& joint_names = getJointNames(mi.at(iStep).get().as<MoveInstructionPoly>().getWaypoint());
      const auto& joint_positions0 = getJointPosition(mi.at(iStep).get().as<MoveInstructionPoly>().getWaypoint());
      const auto& joint_positions1 = getJointPosition(mi.at(iStep + 1).get().as<MoveInstructionPoly>().getWaypoint());

      // TODO: Should check joint names and make sure they are in the same order
      double dist = (joint_positions1 - joint_positions0).norm();
      if (dist > config.longest_valid_segment_length)
      {
        auto cnt = static_cast<long>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, joint_positions0.size());
        for (long iVar = 0; iVar < joint_positions0.size(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, joint_positions0(iVar), joint_positions1(iVar));

        tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;

        if (debug_logging)
        {
          step_contacts = std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(
              static_cast<int>(iStep + 1), joint_positions0, joint_positions1, static_cast<int>(subtraj.rows()));
        }

        auto sub_segment_last_index = static_cast<int>(subtraj.rows() - 1);

        // Update start and end index based on collision check program mode
        long start_idx{ 0 };
        long end_idx(subtraj.rows() - 1);
        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            ++start_idx;
        }
        if (iStep == (mi.size() - 2))
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            --end_idx;
        }

        for (long iSubStep = start_idx; iSubStep < end_idx; ++iSubStep)
        {
          tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;
          if (debug_logging)
          {
            substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(
                static_cast<int>(iSubStep) + 1, subtraj.row(iSubStep), subtraj.row(iSubStep + 1));
          }

          tesseract_scene_graph::SceneState state0 = state_solver.getState(joint_names, subtraj.row(iSubStep));
          tesseract_scene_graph::SceneState state1 = state_solver.getState(joint_names, subtraj.row(iSubStep + 1));
          sub_state_results.clear();
          tesseract_environment::checkTrajectorySegment(
              sub_state_results, manager, state0.link_transforms, state1.link_transforms, config.contact_request);
          if (!sub_state_results.empty())
          {
            found = true;

            if (debug_logging)
            {
              substep_contacts->contacts = sub_state_results;
              step_contacts->substeps[static_cast<size_t>(iSubStep)] = *substep_contacts;
            }
            double segment_dt = (sub_segment_last_index > 0) ? 1.0 / static_cast<double>(sub_segment_last_index) : 0.0;
            state_results.addInterpolatedCollisionResults(sub_state_results,
                                                          iSubStep,
                                                          sub_segment_last_index,
                                                          manager.getActiveCollisionObjects(),
                                                          segment_dt,
                                                          false);
          }

          if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
            break;
        }
        contacts.push_back(state_results);

        if (debug_logging)
        {
          traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
      else
      {
        // Update start and end index based on collision check program mode
        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            contacts.emplace_back(tesseract_collision::ContactResultMap{});
            continue;
          }
        }
        if (iStep == (mi.size() - 2))
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            continue;
        }

        const auto& joint_names0 = getJointNames(mi.at(iStep).get().as<MoveInstructionPoly>().getWaypoint());
        const auto& joint_positions0 = getJointPosition(mi.at(iStep).get().as<MoveInstructionPoly>().getWaypoint());

        const auto& joint_names1 = getJointNames(mi.at(iStep + 1).get().as<MoveInstructionPoly>().getWaypoint());
        const auto& joint_positions1 = getJointPosition(mi.at(iStep + 1).get().as<MoveInstructionPoly>().getWaypoint());

        tesseract_scene_graph::SceneState state0 = state_solver.getState(joint_names0, joint_positions0);
        tesseract_scene_graph::SceneState state1 = state_solver.getState(joint_names1, joint_positions1);

        tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;
        tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;

        if (debug_logging)
        {
          step_contacts = std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(
              static_cast<int>(iStep + 1), joint_positions0, joint_positions1, 1);
          substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(
              1, joint_positions0, joint_positions1);
        }

        tesseract_environment::checkTrajectorySegment(
            state_results, manager, state0.link_transforms, state1.link_transforms, config);
        if (!state_results.empty())
        {
          found = true;

          if (debug_logging)
          {
            substep_contacts->contacts = state_results;
            step_contacts->substeps[0] = *substep_contacts;
            traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
          }
        }
        contacts.push_back(state_results);

        if (debug_logging)
        {
          traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
    }
  }
  else
  {
    std::size_t start_idx{ 0 };
    std::size_t end_idx(mi.size() - 1);
    if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
        config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
    {
      contacts.emplace_back(tesseract_collision::ContactResultMap{});
      ++start_idx;
    }

    if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
        config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
      --end_idx;

    for (std::size_t iStep = start_idx; iStep < end_idx; ++iStep)
    {
      state_results.clear();

      const auto& joint_names0 = getJointNames(mi.at(iStep).get().as<MoveInstructionPoly>().getWaypoint());
      const auto& joint_positions0 = getJointPosition(mi.at(iStep).get().as<MoveInstructionPoly>().getWaypoint());

      const auto& joint_names1 = getJointNames(mi.at(iStep + 1).get().as<MoveInstructionPoly>().getWaypoint());
      const auto& joint_positions1 = getJointPosition(mi.at(iStep + 1).get().as<MoveInstructionPoly>().getWaypoint());

      tesseract_scene_graph::SceneState state0 = state_solver.getState(joint_names0, joint_positions0);
      tesseract_scene_graph::SceneState state1 = state_solver.getState(joint_names1, joint_positions1);

      tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;
      tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;
      if (debug_logging)
      {
        step_contacts = std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(
            static_cast<int>(iStep + 1), joint_positions0, joint_positions1, 1);
        substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(
            1, joint_positions0, joint_positions1);
      }

      tesseract_environment::checkTrajectorySegment(
          state_results, manager, state0.link_transforms, state1.link_transforms, config);
      if (!state_results.empty())
      {
        found = true;

        if (debug_logging)
        {
          substep_contacts->contacts = state_results;
          step_contacts->substeps[0] = *substep_contacts;
          traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
        }
      }
      contacts.push_back(state_results);

      if (debug_logging)
      {
        traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
      }

      if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }

  if (debug_logging)
    std::cout << traj_contacts->trajectoryCollisionResultsTable().str();

  return found;
}

bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                         tesseract_collision::DiscreteContactManager& manager,
                         const tesseract_scene_graph::StateSolver& state_solver,
                         const CompositeInstruction& program,
                         const tesseract_collision::CollisionCheckConfig& config)
{
  if (config.type != tesseract_collision::CollisionEvaluatorType::DISCRETE &&
      config.type != tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
    throw std::runtime_error("contactCheckProgram was given an CollisionEvaluatorType that is inconsistent with the "
                             "ContactManager type (Discrete)");

  // Flatten results
  std::vector<std::reference_wrapper<const InstructionPoly>> mi = program.flatten(moveFilter);

  if (mi.empty())
    throw std::runtime_error("contactCheckProgram was given continuous contact manager with empty trajectory.");

  manager.applyContactManagerConfig(config.contact_manager_config);

  bool debug_logging = console_bridge::getLogLevel() < console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO;

  tesseract_collision::ContactTrajectoryResults::UPtr traj_contacts;
  if (debug_logging)
  {
    // Grab the first waypoint to get the joint names
    const auto& joint_names = getJointNames(mi.front().get().as<MoveInstructionPoly>().getWaypoint());
    traj_contacts =
        std::make_unique<tesseract_collision::ContactTrajectoryResults>(joint_names, static_cast<int>(program.size()));
  }

  contacts.clear();
  contacts.reserve(mi.size());

  /** @brief Making this thread_local does not help because it is not called enough during planning */
  tesseract_collision::ContactResultMap state_results;
  tesseract_collision::ContactResultMap sub_state_results;

  bool found = false;
  if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::START_ONLY)
  {
    const auto& joint_names = getJointNames(mi.front().get().as<MoveInstructionPoly>().getWaypoint());
    const auto& joint_positions = getJointPosition(mi.front().get().as<MoveInstructionPoly>().getWaypoint());
    tesseract_scene_graph::SceneState state = state_solver.getState(joint_names, joint_positions);
    sub_state_results.clear();
    tesseract_environment::checkTrajectoryState(
        sub_state_results, manager, state.link_transforms, config.contact_request);

    if (!sub_state_results.empty())
    {
      found = true;
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
      if (debug_logging)
        printDiscreteDebugInfo(joint_names, joint_positions, 0, mi.size() - 1);
    }
    contacts.push_back(state_results);
    return found;
  }

  if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::END_ONLY)
  {
    const auto& joint_names = getJointNames(mi.back().get().as<MoveInstructionPoly>().getWaypoint());
    const auto& joint_positions = getJointPosition(mi.back().get().as<MoveInstructionPoly>().getWaypoint());
    tesseract_scene_graph::SceneState state = state_solver.getState(joint_names, joint_positions);
    sub_state_results.clear();
    tesseract_environment::checkTrajectoryState(
        sub_state_results, manager, state.link_transforms, config.contact_request);

    if (!sub_state_results.empty())
    {
      found = true;
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
      if (debug_logging)
        printDiscreteDebugInfo(joint_names, joint_positions, 0, mi.size() - 1);
    }
    contacts.push_back(state_results);
    return found;
  }

  if (mi.size() == 1)
  {
    if (config.check_program_mode != tesseract_collision::CollisionCheckProgramType::ALL)
      return true;

    auto sub_segment_last_index = static_cast<int>(mi.size() - 1);
    state_results.clear();
    const auto& joint_names = getJointNames(mi.front().get().as<MoveInstructionPoly>().getWaypoint());
    const auto& joint_positions = getJointPosition(mi.front().get().as<MoveInstructionPoly>().getWaypoint());
    tesseract_scene_graph::SceneState state = state_solver.getState(joint_names, joint_positions);

    tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;
    tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;
    if (debug_logging)
    {
      step_contacts = std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(1, joint_positions);
      substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(1, joint_positions);
    }

    sub_state_results.clear();
    tesseract_environment::checkTrajectoryState(
        sub_state_results, manager, state.link_transforms, config.contact_request);

    if (debug_logging)
    {
      substep_contacts->contacts = sub_state_results;
      step_contacts->substeps[0] = *substep_contacts;
      traj_contacts->steps[0] = *step_contacts;
    }

    double segment_dt = (sub_segment_last_index > 0) ? 1.0 / static_cast<double>(sub_segment_last_index) : 0.0;
    state_results.addInterpolatedCollisionResults(
        sub_state_results, 0, sub_segment_last_index, manager.getActiveCollisionObjects(), segment_dt, true);
    contacts.push_back(state_results);

    if (debug_logging)
      std::cout << traj_contacts->trajectoryCollisionResultsTable().str();

    return (!state_results.empty());
  }

  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    assert(config.longest_valid_segment_length > 0);

    for (std::size_t iStep = 0; iStep < (mi.size() - 1); ++iStep)
    {
      state_results.clear();

      const auto& wp0 = mi.at(iStep).get().as<MoveInstructionPoly>().getWaypoint();
      const std::vector<std::string>& jn = getJointNames(wp0);
      const Eigen::VectorXd& p0 = getJointPosition(wp0);

      const auto& wp1 = mi.at(iStep + 1).get().as<MoveInstructionPoly>().getWaypoint();
      const Eigen::VectorXd& p1 = getJointPosition(wp1);
      const double dist = (p1 - p0).norm();

      if (dist > config.longest_valid_segment_length)
      {
        auto cnt = static_cast<int>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, p0.size());
        for (long iVar = 0; iVar < p0.size(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, p0(iVar), p1(iVar));

        tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;

        if (debug_logging)
        {
          step_contacts = std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(
              iStep + 1, p0, p1, static_cast<int>(subtraj.rows()));
        }

        auto sub_segment_last_index = static_cast<int>(subtraj.rows() - 1);

        // Update start and end index based on collision check program mode
        long start_idx{ 0 };
        long end_idx(subtraj.rows() - 1);
        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            ++start_idx;
        }
        if (iStep == (mi.size() - 2))
        {
          // This is the last segment so check the last state
          end_idx = subtraj.rows();
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            --end_idx;
        }

        for (long iSubStep = start_idx; iSubStep < end_idx; ++iSubStep)
        {
          tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;
          if (debug_logging)
          {
            substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(
                static_cast<int>(iSubStep) + 1, subtraj.row(iSubStep));
          }

          tesseract_scene_graph::SceneState state = state_solver.getState(jn, subtraj.row(iSubStep));
          sub_state_results.clear();
          tesseract_environment::checkTrajectoryState(sub_state_results, manager, state.link_transforms, config);
          if (!sub_state_results.empty())
          {
            found = true;

            if (debug_logging)
            {
              substep_contacts->contacts = sub_state_results;
              step_contacts->substeps[static_cast<size_t>(iSubStep)] = *substep_contacts;
            }
            double segment_dt = (sub_segment_last_index > 0) ? 1.0 / static_cast<double>(sub_segment_last_index) : 0.0;
            state_results.addInterpolatedCollisionResults(sub_state_results,
                                                          iSubStep,
                                                          sub_segment_last_index,
                                                          manager.getActiveCollisionObjects(),
                                                          segment_dt,
                                                          true);
          }

          if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
            break;
        }
        contacts.push_back(state_results);

        if (debug_logging)
        {
          traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
      else
      {
        tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;
        tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;
        tesseract_collision::ContactTrajectorySubstepResults::UPtr end_substep_contacts;
        if (debug_logging)
        {
          step_contacts = std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(iStep + 1, p0);
          substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(1, p0);
          end_substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(2, p1);
        }

        if (iStep == 0 && mi.size() == 2)
        {
          if (config.check_program_mode != tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START &&
              config.check_program_mode != tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            tesseract_scene_graph::SceneState state = state_solver.getState(jn, p0);
            sub_state_results.clear();
            tesseract_environment::checkTrajectoryState(
                sub_state_results, manager, state.link_transforms, config.contact_request);
            if (!sub_state_results.empty())
            {
              found = true;
              state_results.addInterpolatedCollisionResults(
                  sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);

              if (debug_logging)
              {
                substep_contacts->contacts = state_results;
                step_contacts->substeps[0] = *substep_contacts;
                traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
              }
            }

            if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
            {
              contacts.push_back(state_results);
              break;
            }
          }

          if (config.check_program_mode != tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END &&
              config.check_program_mode != tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            tesseract_scene_graph::SceneState state = state_solver.getState(jn, p1);
            sub_state_results.clear();
            tesseract_environment::checkTrajectoryState(
                sub_state_results, manager, state.link_transforms, config.contact_request);
            if (!sub_state_results.empty())
            {
              found = true;
              if (debug_logging)
              {
                end_substep_contacts->contacts = sub_state_results;
                step_contacts->substeps[1] = *end_substep_contacts;
                traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
              }
              state_results.addInterpolatedCollisionResults(
                  sub_state_results, 1, 1, manager.getActiveCollisionObjects(), 1, true);
            }

            if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
            {
              contacts.push_back(state_results);
              break;
            }
          }

          contacts.push_back(state_results);
          break;
        }

        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            contacts.emplace_back(tesseract_collision::ContactResultMap{});
            continue;
          }
        }

        tesseract_scene_graph::SceneState state = state_solver.getState(jn, p0);
        sub_state_results.clear();
        tesseract_environment::checkTrajectoryState(
            sub_state_results, manager, state.link_transforms, config.contact_request);
        if (!sub_state_results.empty())
        {
          found = true;
          if (debug_logging)
          {
            substep_contacts->contacts = sub_state_results;
            step_contacts->substeps[0] = *substep_contacts;
            traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
          }
          state_results.addInterpolatedCollisionResults(
              sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
        {
          contacts.push_back(state_results);
          break;
        }

        // If last segment check the end state
        if (iStep == (mi.size() - 2))
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            contacts.push_back(state_results);
            continue;
          }

          tesseract_scene_graph::SceneState state = state_solver.getState(jn, p1);
          sub_state_results.clear();
          tesseract_environment::checkTrajectoryState(
              sub_state_results, manager, state.link_transforms, config.contact_request);
          if (!sub_state_results.empty())
          {
            found = true;
            if (debug_logging)
            {
              end_substep_contacts->contacts = sub_state_results;
              step_contacts->substeps[1] = *end_substep_contacts;
              traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
            }
            state_results.addInterpolatedCollisionResults(
                sub_state_results, 1, 1, manager.getActiveCollisionObjects(), 1, true);
          }

          if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          {
            contacts.push_back(state_results);
            break;
          }
        }

        contacts.push_back(state_results);

        if (debug_logging)
        {
          traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
        }
      }
    }
  }
  else
  {
    std::size_t start_idx{ 0 };
    std::size_t end_idx(mi.size());

    if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
        config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
    {
      contacts.emplace_back(tesseract_collision::ContactResultMap{});
      ++start_idx;
    }

    if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
        config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
      --end_idx;

    for (std::size_t iStep = start_idx; iStep < end_idx; ++iStep)
    {
      state_results.clear();

      const auto& wp0 = mi.at(iStep).get().as<MoveInstructionPoly>().getWaypoint();
      const std::vector<std::string>& jn = getJointNames(wp0);
      const Eigen::VectorXd& p0 = getJointPosition(wp0);

      tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;
      tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;
      if (debug_logging)
      {
        step_contacts =
            std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(static_cast<int>(iStep + 1), p0);
        substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(1, p0);
      }

      tesseract_scene_graph::SceneState state = state_solver.getState(jn, p0);
      sub_state_results.clear();
      tesseract_environment::checkTrajectoryState(
          sub_state_results, manager, state.link_transforms, config.contact_request);
      if (!sub_state_results.empty())
      {
        found = true;
        if (debug_logging)
        {
          substep_contacts->contacts = sub_state_results;
          step_contacts->substeps[0] = *substep_contacts;
          traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
        }
        state_results.addInterpolatedCollisionResults(
            sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
      }
      contacts.push_back(state_results);

      if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }

  if (debug_logging)
    std::cout << traj_contacts->trajectoryCollisionResultsTable().str();

  return found;
}

}  // namespace tesseract_planning
