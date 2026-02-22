/**
 * @file utils.h
 * @brief Planner utility functions.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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

#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/joint_state.h>

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/discrete_contact_manager.h>

#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_state_solver/state_solver.h>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>

#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>

#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_motion_planners/core/utils.h>

namespace tesseract::motion_planners
{
Eigen::Isometry3d calcPose(const tesseract::command_language::WaypointPoly& wp,
                           const std::string& working_frame,
                           const std::string& tip_link,
                           const Eigen::Isometry3d& tcp,
                           const tesseract::scene_graph::SceneState& current_state,
                           tesseract::scene_graph::StateSolver& state_solver)
{
  if (wp.isStateWaypoint())
  {
    const auto& swp = wp.as<tesseract::command_language::StateWaypointPoly>();
    assert(static_cast<long>(swp.getNames().size()) == swp.getPosition().size());
    tesseract::scene_graph::SceneState state = state_solver.getState(swp.getNames(), swp.getPosition());
    return (state.link_transforms[tip_link] * tcp);
  }

  if (wp.isJointWaypoint())
  {
    const auto& jwp = wp.as<tesseract::command_language::JointWaypointPoly>();
    assert(static_cast<long>(jwp.getNames().size()) == jwp.getPosition().size());
    tesseract::scene_graph::SceneState state = state_solver.getState(jwp.getNames(), jwp.getPosition());
    return (state.link_transforms[tip_link] * tcp);
  }

  if (wp.isCartesianWaypoint())
  {
    const auto& cwp = wp.as<tesseract::command_language::CartesianWaypointPoly>();
    if (working_frame.empty())
      return cwp.getTransform();

    return (current_state.link_transforms.at(working_frame) * cwp.getTransform());
  }

  throw std::runtime_error("toToolpath: Unsupported Waypoint Type!");
}

tesseract::common::Toolpath toToolpath(const tesseract::command_language::InstructionPoly& instruction,
                                       const tesseract::environment::Environment& env)
{
  using namespace tesseract::motion_planners;
  if (instruction.isCompositeInstruction())
  {
    const auto& ci = instruction.as<tesseract::command_language::CompositeInstruction>();
    return toToolpath(ci, env);
  }

  if (instruction.isMoveInstruction())
  {
    const auto& mi = instruction.as<tesseract::command_language::MoveInstructionPoly>();
    return toToolpath(mi, env);
  }

  throw std::runtime_error("toToolpath: Unsupported Instruction Type!");
}

tesseract::common::VectorIsometry3d toPoses(const tesseract::command_language::CompositeInstruction& ci,
                                            const tesseract::common::ManipulatorInfo& parent_mi,
                                            const tesseract::environment::Environment& env,
                                            const tesseract::scene_graph::SceneState& state,
                                            tesseract::scene_graph::StateSolver& state_solver)
{
  tesseract::common::VectorIsometry3d poses;
  std::vector<std::reference_wrapper<const tesseract::command_language::InstructionPoly>> fi =
      ci.flatten(tesseract::command_language::moveFilter);
  for (const auto& i : fi)
  {
    tesseract::common::ManipulatorInfo manip_info;

    // Check for updated manipulator information and get waypoint
    tesseract::command_language::WaypointPoly wp;
    if (i.get().isMoveInstruction())
    {
      const auto& mi = i.get().as<tesseract::command_language::MoveInstructionPoly>();
      manip_info = parent_mi.getCombined(mi.getManipulatorInfo());
      wp = mi.getWaypoint();
    }
    else
    {
      throw std::runtime_error("toToolpath: Unsupported Instruction Type!");
    }

    // Extract TCP
    Eigen::Isometry3d tcp_offset = env.findTCPOffset(manip_info);

    // Calculate pose
    poses.push_back(calcPose(wp, manip_info.working_frame, manip_info.tcp_frame, tcp_offset, state, state_solver));
  }

  return poses;
}

tesseract::common::Toolpath toToolpath(const tesseract::command_language::CompositeInstruction& ci,
                                       const tesseract::environment::Environment& env)
{
  if (ci.empty())
    return {};

  tesseract::scene_graph::StateSolver::UPtr state_solver = env.getStateSolver();
  tesseract::scene_graph::SceneState state = env.getState();

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!ci.getManipulatorInfo().empty());
  const tesseract::common::ManipulatorInfo& composite_mi = ci.getManipulatorInfo();

  if (ci.front().isCompositeInstruction())
  {
    tesseract::common::Toolpath toolpath;
    for (const auto& sub_ci : ci)
    {
      if (!sub_ci.isCompositeInstruction())
        break;

      toolpath.push_back(toPoses(
          sub_ci.as<tesseract::command_language::CompositeInstruction>(), composite_mi, env, state, *state_solver));
    }
    return toolpath;
  }

  tesseract::common::Toolpath toolpath;
  toolpath.push_back(toPoses(ci, composite_mi, env, state, *state_solver));

  return toolpath;
}

tesseract::common::Toolpath toToolpath(const tesseract::command_language::MoveInstructionPoly& mi,
                                       const tesseract::environment::Environment& env)
{
  tesseract::common::Toolpath toolpath;
  tesseract::common::VectorIsometry3d poses;

  tesseract::scene_graph::StateSolver::UPtr state_solver = env.getStateSolver();
  tesseract::scene_graph::SceneState state = env.getState();

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!mi.getManipulatorInfo().empty());
  const tesseract::common::ManipulatorInfo& composite_mi = mi.getManipulatorInfo();
  tesseract::common::ManipulatorInfo manip_info = composite_mi.getCombined(mi.getManipulatorInfo());

  // Extract TCP
  Eigen::Isometry3d tcp_offset = env.findTCPOffset(manip_info);

  // Calculate pose
  poses.push_back(
      calcPose(mi.getWaypoint(), manip_info.working_frame, manip_info.tcp_frame, tcp_offset, state, *state_solver));

  toolpath.push_back(poses);
  return toolpath;
}

void assignCurrentStateAsSeed(tesseract::command_language::CompositeInstruction& composite_instructions,
                              const tesseract::environment::Environment& env)
{
  std::unordered_map<std::string, tesseract::common::JointState> manip_joint_state;
  tesseract::scene_graph::SceneState state = env.getState();
  const tesseract::common::ManipulatorInfo& global_mi = composite_instructions.getManipulatorInfo();

  std::vector<std::reference_wrapper<tesseract::command_language::InstructionPoly>> mv_instructions =
      composite_instructions.flatten(tesseract::command_language::moveFilter);
  for (auto& i : mv_instructions)
  {
    auto& mvi = i.get().as<tesseract::command_language::MoveInstructionPoly>();
    if (mvi.getWaypoint().isCartesianWaypoint())
    {
      auto& cwp = mvi.getWaypoint().as<tesseract::command_language::CartesianWaypointPoly>();
      tesseract::common::ManipulatorInfo mi = global_mi.getCombined(mvi.getManipulatorInfo());
      auto it = manip_joint_state.find(mi.manipulator);
      if (it != manip_joint_state.end())
      {
        cwp.setSeed(it->second);
      }
      else
      {
        std::vector<std::string> joint_names = env.getGroupJointNames(mi.manipulator);
        Eigen::VectorXd jv = state.getJointValues(joint_names);
        tesseract::common::JointState seed(joint_names, jv);
        manip_joint_state[mi.manipulator] = seed;
        cwp.setSeed(seed);
      }
    }
  }
}

bool formatProgramHelper(tesseract::command_language::CompositeInstruction& composite_instructions,
                         const tesseract::environment::Environment& env,
                         const tesseract::common::ManipulatorInfo& manip_info,
                         std::unordered_map<std::string, std::vector<std::string>>& manip_joint_names)
{
  bool format_required = false;
  for (auto& i : composite_instructions)
  {
    if (i.isCompositeInstruction())
    {
      if (formatProgramHelper(
              i.as<tesseract::command_language::CompositeInstruction>(), env, manip_info, manip_joint_names))
        format_required = true;
    }
    else if (i.isMoveInstruction())
    {
      auto& base_instruction = i.as<tesseract::command_language::MoveInstructionPoly>();
      tesseract::common::ManipulatorInfo mi = manip_info.getCombined(base_instruction.getManipulatorInfo());

      std::vector<std::string> joint_names;
      auto it = manip_joint_names.find(mi.manipulator);
      if (it == manip_joint_names.end())
      {
        joint_names = env.getGroupJointNames(mi.manipulator);
        manip_joint_names[mi.manipulator] = joint_names;
      }
      else
      {
        joint_names = it->second;
      }

      auto& wp = base_instruction.getWaypoint();
      if (wp.isStateWaypoint() || wp.isJointWaypoint() || wp.isCartesianWaypoint())
      {
        if (formatJointPosition(joint_names, base_instruction.getWaypoint()))
          format_required = true;
      }
    }
  }
  return format_required;
}

bool formatProgram(tesseract::command_language::CompositeInstruction& composite_instructions,
                   const tesseract::environment::Environment& env)
{
  std::unordered_map<std::string, std::vector<std::string>> manip_joint_names;
  bool format_required = false;
  tesseract::common::ManipulatorInfo mi = composite_instructions.getManipulatorInfo();

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
  ss << "\n";

  ss << "     Names:";
  for (const auto& name : joint_names)
    ss << " " << name;

  ss << "\n    State0: " << swp0 << "\n    State1: " << swp1 << "\n";

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
  ss << "\n";

  ss << "     Names:";
  for (const auto& name : joint_names)
    ss << " " << name;

  ss << "\n    State: " << swp << "\n";

  CONSOLE_BRIDGE_logDebug(ss.str().c_str());
}

tesseract::collision::ContactTrajectoryResults
contactCheckProgram(std::vector<tesseract::collision::ContactResultMap>& contacts,
                    tesseract::collision::ContinuousContactManager& manager,
                    const tesseract::scene_graph::StateSolver& state_solver,
                    const tesseract::command_language::CompositeInstruction& program,
                    const tesseract::collision::CollisionCheckConfig& config)
{
  if (config.type != tesseract::collision::CollisionEvaluatorType::CONTINUOUS &&
      config.type != tesseract::collision::CollisionEvaluatorType::LVS_CONTINUOUS)
    throw std::runtime_error("contactCheckProgram was given an CollisionEvaluatorType that is inconsistent with the "
                             "ContactManager type (Continuous)");

  // Flatten results
  std::vector<std::reference_wrapper<const tesseract::command_language::InstructionPoly>> mi =
      program.flatten(tesseract::command_language::moveFilter);

  if (mi.size() < 2)
    throw std::runtime_error("contactCheckProgram was given continuous contact manager with a trajectory that only has "
                             "one state.");

  bool debug_logging = console_bridge::getLogLevel() < console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO;

  // Grab the first waypoint to get the joint names
  const auto& joint_names =
      getJointNames(mi.front().get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());
  tesseract::collision::ContactTrajectoryResults traj_contacts(joint_names, static_cast<int>(mi.size()));

  contacts.clear();
  contacts.reserve(mi.size());

  tesseract::common::TransformMap link_transforms;
  tesseract::common::TransformMap link_transforms1;

  /** @brief Making this thread_local does not help because it is not called enough during planning */
  tesseract::collision::ContactResultMap state_results;
  tesseract::collision::ContactResultMap sub_state_results;

  if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::START_ONLY)
  {
    const auto& joint_positions =
        getJointPosition(mi.front().get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());
    state_solver.getLinkTransforms(link_transforms, joint_names, joint_positions);
    sub_state_results.clear();
    tesseract::environment::checkTrajectoryState(sub_state_results, manager, link_transforms, config.contact_request);

    if (!sub_state_results.empty())
    {
      traj_contacts.addContact(
          0, 0, 1, joint_positions, joint_positions, joint_positions, joint_positions, sub_state_results);
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, false);
      if (debug_logging)
        printContinuousDebugInfo(joint_names, joint_positions, joint_positions, 0, mi.size() - 1);
    }
    contacts.push_back(state_results);
    return traj_contacts;
  }

  if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::END_ONLY)
  {
    const auto& joint_positions =
        getJointPosition(mi.back().get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());
    state_solver.getLinkTransforms(link_transforms, joint_names, joint_positions);
    sub_state_results.clear();
    tesseract::environment::checkTrajectoryState(sub_state_results, manager, link_transforms, config.contact_request);

    if (!sub_state_results.empty())
    {
      traj_contacts.addContact(static_cast<int>(mi.size() - 1),
                               0,
                               1,
                               joint_positions,
                               joint_positions,
                               joint_positions,
                               joint_positions,
                               sub_state_results);
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, false);
      if (debug_logging)
        printContinuousDebugInfo(joint_names, joint_positions, joint_positions, 0, mi.size() - 1);
    }
    contacts.push_back(state_results);
    return traj_contacts;
  }

  if (config.type == tesseract::collision::CollisionEvaluatorType::LVS_CONTINUOUS)
  {
    assert(config.longest_valid_segment_length > 0);

    bool found = false;
    for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
    {
      state_results.clear();

      const auto& joint_positions0 =
          getJointPosition(mi.at(iStep).get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());
      const auto& joint_positions1 =
          getJointPosition(mi.at(iStep + 1).get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());

      // TODO: Should check joint names and make sure they are in the same order
      double dist = (joint_positions1 - joint_positions0).norm();
      if (dist > config.longest_valid_segment_length)
      {
        auto cnt = static_cast<long>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract::common::TrajArray subtraj(cnt, joint_positions0.size());
        for (long iVar = 0; iVar < joint_positions0.size(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, joint_positions0(iVar), joint_positions1(iVar));

        auto sub_segment_last_index = static_cast<int>(subtraj.rows() - 1);

        // Update start and end index based on collision check program mode
        long start_idx{ 0 };
        long end_idx(subtraj.rows() - 1);
        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            ++start_idx;
        }
        if (iStep == (mi.size() - 2))
        {
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            --end_idx;
        }

        for (long iSubStep = start_idx; iSubStep < end_idx; ++iSubStep)
        {
          state_solver.getLinkTransforms(link_transforms, joint_names, subtraj.row(iSubStep));
          state_solver.getLinkTransforms(link_transforms1, joint_names, subtraj.row(iSubStep + 1));
          sub_state_results.clear();
          tesseract::environment::checkTrajectorySegment(
              sub_state_results, manager, link_transforms, link_transforms1, config.contact_request);
          if (!sub_state_results.empty())
          {
            found = true;
            traj_contacts.addContact(static_cast<int>(iStep),
                                     static_cast<int>(iSubStep),
                                     static_cast<int>(subtraj.rows()),
                                     joint_positions0,
                                     joint_positions1,
                                     subtraj.row(iSubStep),
                                     subtraj.row(iSubStep + 1),
                                     sub_state_results);

            if (debug_logging)
              printContinuousDebugInfo(
                  joint_names, subtraj.row(iSubStep), subtraj.row(iSubStep + 1), iStep, mi.size() - 1, iSubStep);

            double segment_dt = (sub_segment_last_index > 0) ? 1.0 / static_cast<double>(sub_segment_last_index) : 0.0;
            state_results.addInterpolatedCollisionResults(sub_state_results,
                                                          iSubStep,
                                                          sub_segment_last_index,
                                                          manager.getActiveCollisionObjects(),
                                                          segment_dt,
                                                          false);
            // If only one contact per step is requested, stop checking additional substates for this step
            if (config.exit_condition == tesseract::collision::CollisionCheckExitType::ONE_PER_STEP)
              break;
          }

          if (found && (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST))
            break;
        }
        contacts.push_back(state_results);

        if (found && (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST))
          break;
      }
      else
      {
        // Update start and end index based on collision check program mode
        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            contacts.emplace_back();
            continue;
          }
        }
        if (iStep == (mi.size() - 2))
        {
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            continue;
        }

        const auto& joint_names0 =
            getJointNames(mi.at(iStep).get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());
        const auto& joint_names1 =
            getJointNames(mi.at(iStep + 1).get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());

        state_solver.getLinkTransforms(link_transforms, joint_names0, joint_positions0);
        state_solver.getLinkTransforms(link_transforms1, joint_names1, joint_positions1);

        tesseract::environment::checkTrajectorySegment(
            state_results, manager, link_transforms, link_transforms1, config.contact_request);
        if (!state_results.empty())
        {
          found = true;
          traj_contacts.addContact(static_cast<int>(iStep),
                                   0,
                                   1,
                                   joint_positions0,
                                   joint_positions1,
                                   joint_positions0,
                                   joint_positions1,
                                   state_results);

          if (debug_logging)
            printContinuousDebugInfo(joint_names, joint_positions0, joint_positions1, iStep, mi.size() - 1);
        }
        contacts.push_back(state_results);

        if (found && (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST))
          break;
      }
    }
  }
  else
  {
    std::size_t start_idx{ 0 };
    std::size_t end_idx(mi.size() - 1);
    if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
        config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
    {
      contacts.emplace_back();
      ++start_idx;
    }

    if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
        config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
      --end_idx;

    bool found = false;
    for (std::size_t iStep = start_idx; iStep < end_idx; ++iStep)
    {
      state_results.clear();

      const auto& joint_names0 =
          getJointNames(mi.at(iStep).get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());
      const auto& joint_positions0 =
          getJointPosition(mi.at(iStep).get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());

      const auto& joint_names1 =
          getJointNames(mi.at(iStep + 1).get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());
      const auto& joint_positions1 =
          getJointPosition(mi.at(iStep + 1).get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());

      state_solver.getLinkTransforms(link_transforms, joint_names0, joint_positions0);
      state_solver.getLinkTransforms(link_transforms1, joint_names1, joint_positions1);

      tesseract::environment::checkTrajectorySegment(
          state_results, manager, link_transforms, link_transforms1, config.contact_request);
      if (!state_results.empty())
      {
        found = true;
        traj_contacts.addContact(static_cast<int>(iStep),
                                 0,
                                 1,
                                 joint_positions0,
                                 joint_positions1,
                                 joint_positions0,
                                 joint_positions1,
                                 state_results);

        if (debug_logging)
          printContinuousDebugInfo(joint_names, joint_positions0, joint_positions1, iStep, mi.size() - 1);
      }
      contacts.push_back(state_results);

      if (found && (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST))
        break;
    }
  }

  if (debug_logging && traj_contacts)
    std::cout << traj_contacts.trajectoryCollisionResultsTable().str();

  return traj_contacts;
}

tesseract::collision::ContactTrajectoryResults
contactCheckProgram(std::vector<tesseract::collision::ContactResultMap>& contacts,
                    tesseract::collision::DiscreteContactManager& manager,
                    const tesseract::scene_graph::StateSolver& state_solver,
                    const tesseract::command_language::CompositeInstruction& program,
                    const tesseract::collision::CollisionCheckConfig& config)
{
  if (config.type != tesseract::collision::CollisionEvaluatorType::DISCRETE &&
      config.type != tesseract::collision::CollisionEvaluatorType::LVS_DISCRETE)
    throw std::runtime_error("contactCheckProgram was given an CollisionEvaluatorType that is inconsistent with the "
                             "ContactManager type (Discrete)");

  // Flatten results
  std::vector<std::reference_wrapper<const tesseract::command_language::InstructionPoly>> mi =
      program.flatten(tesseract::command_language::moveFilter);

  if (mi.empty())
    throw std::runtime_error("contactCheckProgram was given continuous contact manager with empty trajectory.");

  bool debug_logging = console_bridge::getLogLevel() < console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO;

  // Grab the first waypoint to get the joint names
  const auto& joint_names =
      getJointNames(mi.front().get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());
  tesseract::collision::ContactTrajectoryResults traj_contacts(joint_names, static_cast<int>(mi.size()));

  contacts.clear();
  contacts.reserve(mi.size());

  tesseract::common::TransformMap link_transforms;

  /** @brief Making this thread_local does not help because it is not called enough during planning */
  tesseract::collision::ContactResultMap state_results;
  tesseract::collision::ContactResultMap sub_state_results;

  if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::START_ONLY)
  {
    const auto& joint_positions =
        getJointPosition(mi.front().get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());
    state_solver.getLinkTransforms(link_transforms, joint_names, joint_positions);
    sub_state_results.clear();
    tesseract::environment::checkTrajectoryState(sub_state_results, manager, link_transforms, config.contact_request);

    if (!sub_state_results.empty())
    {
      traj_contacts.addContact(
          0, 0, 1, joint_positions, joint_positions, joint_positions, joint_positions, sub_state_results);
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
      if (debug_logging)
        printDiscreteDebugInfo(joint_names, joint_positions, 0, mi.size() - 1);
    }
    contacts.push_back(state_results);
    return traj_contacts;
  }

  if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::END_ONLY)
  {
    const auto& joint_positions =
        getJointPosition(mi.back().get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());
    state_solver.getLinkTransforms(link_transforms, joint_names, joint_positions);
    sub_state_results.clear();
    tesseract::environment::checkTrajectoryState(sub_state_results, manager, link_transforms, config.contact_request);

    if (!sub_state_results.empty())
    {
      traj_contacts.addContact(static_cast<int>(mi.size() - 1),
                               0,
                               1,
                               joint_positions,
                               joint_positions,
                               joint_positions,
                               joint_positions,
                               sub_state_results);
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
      if (debug_logging)
        printDiscreteDebugInfo(joint_names, joint_positions, 0, mi.size() - 1);
    }
    contacts.push_back(state_results);
    return traj_contacts;
  }

  if (mi.size() == 1)
  {
    if (config.check_program_mode != tesseract::collision::CollisionCheckProgramType::ALL)
      return traj_contacts;

    auto sub_segment_last_index = static_cast<int>(mi.size() - 1);
    state_results.clear();
    const auto& joint_positions =
        getJointPosition(mi.front().get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint());
    state_solver.getLinkTransforms(link_transforms, joint_names, joint_positions);

    sub_state_results.clear();
    tesseract::environment::checkTrajectoryState(sub_state_results, manager, link_transforms, config.contact_request);

    if (!sub_state_results.empty())
    {
      traj_contacts.addContact(
          0, 0, 1, joint_positions, joint_positions, joint_positions, joint_positions, sub_state_results);

      if (debug_logging)
        printDiscreteDebugInfo(joint_names, joint_positions, 0, mi.size() - 1);
    }

    double segment_dt = (sub_segment_last_index > 0) ? 1.0 / static_cast<double>(sub_segment_last_index) : 0.0;
    state_results.addInterpolatedCollisionResults(
        sub_state_results, 0, sub_segment_last_index, manager.getActiveCollisionObjects(), segment_dt, true);
    contacts.push_back(state_results);

    if (debug_logging && traj_contacts)
      std::cout << traj_contacts.trajectoryCollisionResultsTable().str();

    return traj_contacts;
  }

  if (config.type == tesseract::collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    assert(config.longest_valid_segment_length > 0);

    bool found = false;
    for (std::size_t iStep = 0; iStep < (mi.size() - 1); ++iStep)
    {
      state_results.clear();

      const auto& wp0 = mi.at(iStep).get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint();
      const std::vector<std::string>& jn = getJointNames(wp0);
      const Eigen::VectorXd& p0 = getJointPosition(wp0);

      const auto& wp1 = mi.at(iStep + 1).get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint();
      const Eigen::VectorXd& p1 = getJointPosition(wp1);
      const double dist = (p1 - p0).norm();

      if (dist > config.longest_valid_segment_length)
      {
        auto cnt = static_cast<int>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract::common::TrajArray subtraj(cnt, p0.size());
        for (long iVar = 0; iVar < p0.size(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, p0(iVar), p1(iVar));

        tesseract::collision::ContactTrajectoryStepResults::UPtr step_contacts;

        if (debug_logging)
        {
          step_contacts = std::make_unique<tesseract::collision::ContactTrajectoryStepResults>(
              iStep + 1, p0, p1, static_cast<int>(subtraj.rows()));
        }

        auto sub_segment_last_index = static_cast<int>(subtraj.rows() - 1);

        // Update start and end index based on collision check program mode
        long start_idx{ 0 };
        long end_idx(subtraj.rows() - 1);
        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            ++start_idx;
        }
        if (iStep == (mi.size() - 2))
        {
          // This is the last segment so check the last state
          end_idx = subtraj.rows();
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            --end_idx;
        }

        for (long iSubStep = start_idx; iSubStep < end_idx; ++iSubStep)
        {
          state_solver.getLinkTransforms(link_transforms, jn, subtraj.row(iSubStep));
          sub_state_results.clear();
          tesseract::environment::checkTrajectoryState(
              sub_state_results, manager, link_transforms, config.contact_request);
          if (!sub_state_results.empty())
          {
            found = true;
            traj_contacts.addContact(static_cast<int>(iStep),
                                     static_cast<int>(iSubStep),
                                     static_cast<int>(subtraj.rows()),
                                     p0,
                                     p1,
                                     subtraj.row(iSubStep),
                                     subtraj.row(iSubStep),
                                     sub_state_results);

            if (debug_logging)
              printDiscreteDebugInfo(jn, subtraj.row(iSubStep), iStep, mi.size() - 1, iSubStep);

            double segment_dt = (sub_segment_last_index > 0) ? 1.0 / static_cast<double>(sub_segment_last_index) : 0.0;
            state_results.addInterpolatedCollisionResults(sub_state_results,
                                                          iSubStep,
                                                          sub_segment_last_index,
                                                          manager.getActiveCollisionObjects(),
                                                          segment_dt,
                                                          true);
            // If only one contact per step is requested, stop checking additional substates for this step
            if (config.exit_condition == tesseract::collision::CollisionCheckExitType::ONE_PER_STEP)
              break;
          }

          if (found && (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST))
            break;
        }
        contacts.push_back(state_results);

        if (found && (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST))
          break;
      }
      else
      {
        if (iStep == 0 && mi.size() == 2)
        {
          if (config.check_program_mode != tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START &&
              config.check_program_mode != tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            state_solver.getLinkTransforms(link_transforms, jn, p0);
            sub_state_results.clear();
            tesseract::environment::checkTrajectoryState(
                sub_state_results, manager, link_transforms, config.contact_request);
            if (!sub_state_results.empty())
            {
              found = true;
              traj_contacts.addContact(static_cast<int>(iStep), 0, 1, p0, p1, p0, p0, sub_state_results);
              state_results.addInterpolatedCollisionResults(
                  sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);

              if (debug_logging)
                printDiscreteDebugInfo(jn, p0, iStep, mi.size() - 1);
            }

            if (found && (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST))
            {
              contacts.push_back(state_results);
              break;
            }
          }

          if (config.check_program_mode != tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END &&
              config.check_program_mode != tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            state_solver.getLinkTransforms(link_transforms, jn, p1);
            sub_state_results.clear();
            tesseract::environment::checkTrajectoryState(
                sub_state_results, manager, link_transforms, config.contact_request);
            if (!sub_state_results.empty())
            {
              found = true;
              traj_contacts.addContact(static_cast<int>(iStep), 1, 2, p0, p1, p1, p1, sub_state_results);
              state_results.addInterpolatedCollisionResults(
                  sub_state_results, 1, 1, manager.getActiveCollisionObjects(), 1, true);

              if (debug_logging)
                printDiscreteDebugInfo(jn, p1, iStep, mi.size() - 1, 1);
            }

            if (found && (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST))
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
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            contacts.emplace_back();
            continue;
          }
        }

        state_solver.getLinkTransforms(link_transforms, jn, p0);
        sub_state_results.clear();
        tesseract::environment::checkTrajectoryState(
            sub_state_results, manager, link_transforms, config.contact_request);
        if (!sub_state_results.empty())
        {
          found = true;
          traj_contacts.addContact(static_cast<int>(iStep), 0, 1, p0, p1, p0, p0, sub_state_results);
          state_results.addInterpolatedCollisionResults(
              sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);

          if (debug_logging)
            printDiscreteDebugInfo(jn, p0, iStep, mi.size() - 1);
        }

        if (found && (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST))
        {
          contacts.push_back(state_results);
          break;
        }

        // If last segment check the end state
        if (iStep == (mi.size() - 2))
        {
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            contacts.push_back(state_results);
            continue;
          }

          state_solver.getLinkTransforms(link_transforms, jn, p1);
          sub_state_results.clear();
          tesseract::environment::checkTrajectoryState(
              sub_state_results, manager, link_transforms, config.contact_request);
          if (!sub_state_results.empty())
          {
            found = true;
            traj_contacts.addContact(static_cast<int>(iStep), 1, 2, p0, p1, p1, p1, sub_state_results);
            state_results.addInterpolatedCollisionResults(
                sub_state_results, 1, 1, manager.getActiveCollisionObjects(), 1, true);

            if (debug_logging)
              printDiscreteDebugInfo(jn, p1, iStep, mi.size() - 1, 1);
          }

          if (found && (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST))
          {
            contacts.push_back(state_results);
            break;
          }
        }

        contacts.push_back(state_results);
      }
    }
  }
  else
  {
    std::size_t start_idx{ 0 };
    std::size_t end_idx(mi.size());

    if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
        config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
    {
      contacts.emplace_back();
      ++start_idx;
    }

    if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
        config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
      --end_idx;

    bool found = false;
    for (std::size_t iStep = start_idx; iStep < end_idx; ++iStep)
    {
      state_results.clear();

      const auto& wp0 = mi.at(iStep).get().as<tesseract::command_language::MoveInstructionPoly>().getWaypoint();
      const std::vector<std::string>& jn = getJointNames(wp0);
      const Eigen::VectorXd& p0 = getJointPosition(wp0);

      state_solver.getLinkTransforms(link_transforms, jn, p0);
      sub_state_results.clear();
      tesseract::environment::checkTrajectoryState(sub_state_results, manager, link_transforms, config.contact_request);
      if (!sub_state_results.empty())
      {
        found = true;
        traj_contacts.addContact(static_cast<int>(iStep), 0, 1, p0, p0, p0, p0, sub_state_results);
        state_results.addInterpolatedCollisionResults(
            sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);

        if (debug_logging)
          printDiscreteDebugInfo(jn, p0, iStep, mi.size() - 1);
      }
      contacts.push_back(state_results);

      if (found && (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST))
        break;
    }
  }

  if (debug_logging && traj_contacts)
    std::cout << traj_contacts.trajectoryCollisionResultsTable().str();

  return traj_contacts;
}

}  // namespace tesseract::motion_planners
