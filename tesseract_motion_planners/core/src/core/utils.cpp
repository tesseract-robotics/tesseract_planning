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
#include <memory>
#include <typeindex>
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

  tesseract_common::Toolpath toolpath;
  tesseract_common::VectorIsometry3d poses;

  tesseract_scene_graph::StateSolver::UPtr state_solver = env.getStateSolver();
  tesseract_scene_graph::SceneState state = env.getState();
  if (instruction.isCompositeInstruction())
  {
    const auto& ci = instruction.as<CompositeInstruction>();

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
  }
  else if (instruction.isMoveInstruction())
  {
    assert(instruction.isMoveInstruction());
    const auto& pi = instruction.as<MoveInstructionPoly>();

    // Assume all the plan instructions have the same manipulator as the composite
    assert(!pi.getManipulatorInfo().empty());
    const tesseract_common::ManipulatorInfo& composite_mi = pi.getManipulatorInfo();
    tesseract_common::ManipulatorInfo manip_info = composite_mi.getCombined(pi.getManipulatorInfo());

    // Extract TCP
    Eigen::Isometry3d tcp_offset = env.findTCPOffset(manip_info);

    // Calculate pose
    poses.push_back(
        calcPose(pi.getWaypoint(), manip_info.working_frame, manip_info.tcp_frame, tcp_offset, state, *state_solver));
  }
  else
  {
    throw std::runtime_error("toToolpath: Unsupported Instruction Type!");
  }

  toolpath.push_back(poses);
  return toolpath;
}

tesseract_common::VectorIsometry3d interpolate(const Eigen::Isometry3d& start,
                                               const Eigen::Isometry3d& stop,
                                               long steps)
{
  // Required position change
  Eigen::Vector3d delta_translation = (stop.translation() - start.translation());
  Eigen::Vector3d start_pos = start.translation();
  Eigen::Affine3d stop_prime = start.inverse() * stop;
  Eigen::AngleAxisd delta_rotation(stop_prime.rotation());

  // Step size
  Eigen::Vector3d step = delta_translation / steps;

  // Orientation interpolation
  Eigen::Quaterniond start_q(start.rotation());
  Eigen::Quaterniond stop_q(stop.rotation());
  double slerp_ratio = 1.0 / static_cast<double>(steps);

  tesseract_common::VectorIsometry3d result;
  Eigen::Vector3d trans;
  Eigen::Quaterniond q;
  Eigen::Isometry3d pose;
  result.reserve(static_cast<size_t>(steps) + 1);
  for (unsigned i = 0; i <= static_cast<unsigned>(steps); ++i)
  {
    trans = start_pos + step * i;
    q = start_q.slerp(slerp_ratio * i, stop_q);
    pose = (Eigen::Translation3d(trans) * q);
    result.push_back(pose);
  }
  return result;
}

Eigen::MatrixXd interpolate(const Eigen::Ref<const Eigen::VectorXd>& start,
                            const Eigen::Ref<const Eigen::VectorXd>& stop,
                            long steps)
{
  assert(start.size() == stop.size());

  Eigen::MatrixXd result(start.size(), steps + 1);

  for (int i = 0; i < start.size(); ++i)
    result.row(i) = Eigen::VectorXd::LinSpaced(steps + 1, start(i), stop(i));

  return result;
}

std::vector<WaypointPoly> interpolate_waypoint(const WaypointPoly& start, const WaypointPoly& stop, long steps)
{
  if (start.isCartesianWaypoint())
  {
    const auto& cwp1 = start.as<CartesianWaypointPoly>();
    const auto& cwp2 = stop.as<CartesianWaypointPoly>();
    tesseract_common::VectorIsometry3d eigen_poses = interpolate(cwp1.getTransform(), cwp2.getTransform(), steps);

    std::vector<WaypointPoly> result;
    result.reserve(eigen_poses.size());
    for (auto& eigen_pose : eigen_poses)
    {
      CartesianWaypointPoly copy(cwp2);
      copy.setTransform(eigen_pose);
      result.emplace_back(copy);
    }

    return result;
  }

  if (start.isJointWaypoint())
  {
    const auto& jwp1 = start.as<JointWaypointPoly>();
    const auto& jwp2 = stop.as<JointWaypointPoly>();

    // TODO: Should check joint names are in the same order
    Eigen::MatrixXd joint_poses = interpolate(jwp1.getPosition(), jwp2.getPosition(), steps);

    std::vector<WaypointPoly> result;
    result.reserve(static_cast<std::size_t>(joint_poses.cols()));
    for (int i = 0; i < joint_poses.cols(); ++i)
    {
      JointWaypointPoly copy(jwp2);
      copy.setPosition(joint_poses.col(i));
      result.emplace_back(copy);
    }

    return result;
  }

  CONSOLE_BRIDGE_logError("Interpolator for Waypoint type %d is currently not support!", start.getType().hash_code());
  return {};
}

bool programFlattenFilter(const InstructionPoly& instruction,
                          const CompositeInstruction& /*composite*/,
                          bool parent_is_first_composite)
{
  if (instruction.isMoveInstruction())
  {
    if (instruction.as<MoveInstructionPoly>().isStart())
      return (parent_is_first_composite);
  }
  else if (instruction.isCompositeInstruction())
  {
    return false;
  }

  return true;
};

std::vector<std::reference_wrapper<InstructionPoly>> flattenProgram(CompositeInstruction& composite_instruction)
{
  return composite_instruction.flatten(programFlattenFilter);
}

std::vector<std::reference_wrapper<const InstructionPoly>>
flattenProgram(const CompositeInstruction& composite_instruction)
{
  return composite_instruction.flatten(programFlattenFilter);
}

std::vector<std::reference_wrapper<InstructionPoly>>
flattenProgramToPattern(CompositeInstruction& composite_instruction, const CompositeInstruction& pattern)
{
  return composite_instruction.flattenToPattern(pattern, programFlattenFilter);
}

std::vector<std::reference_wrapper<const InstructionPoly>>
flattenProgramToPattern(const CompositeInstruction& composite_instruction, const CompositeInstruction& pattern)
{
  return composite_instruction.flattenToPattern(pattern, programFlattenFilter);
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
  manager.applyContactManagerConfig(config.contact_manager_config);

  // Flatten results
  std::vector<std::reference_wrapper<const InstructionPoly>> mi = program.flatten(moveFilter);

  bool found = false;

  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
  {
    assert(config.longest_valid_segment_length > 0);

    contacts.resize(mi.size() - 1);
    for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
    {
      tesseract_collision::ContactResultMap& segment_results = contacts[static_cast<size_t>(iStep)];
      segment_results.clear();

      const auto& swp0 = mi.at(iStep).get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
      const auto& swp1 = mi.at(iStep + 1).get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();

      // TODO: Should check joint names and make sure they are in the same order
      double dist = (swp1.getPosition() - swp0.getPosition()).norm();
      if (dist > config.longest_valid_segment_length)
      {
        auto cnt = static_cast<long>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, swp0.getPosition().size());
        for (long iVar = 0; iVar < swp0.getPosition().size(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, swp0.getPosition()(iVar), swp1.getPosition()(iVar));

        for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
        {
          tesseract_scene_graph::SceneState state0 = state_solver.getState(swp0.getNames(), subtraj.row(iSubStep));
          tesseract_scene_graph::SceneState state1 = state_solver.getState(swp0.getNames(), subtraj.row(iSubStep + 1));
          tesseract_collision::ContactResultMap sub_segment_results = tesseract_environment::checkTrajectorySegment(
              manager, state0.link_transforms, state1.link_transforms, config.contact_request);
          if (!sub_segment_results.empty())
          {
            found = true;
            tesseract_environment::processInterpolatedSubSegmentCollisionResults(segment_results,
                                                                                 sub_segment_results,
                                                                                 iSubStep,
                                                                                 static_cast<int>(subtraj.rows() - 1),
                                                                                 manager.getActiveCollisionObjects(),
                                                                                 false);

            if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
            {
              std::stringstream ss;
              ss << "Continuous collision detected at step: " << iStep << " of " << (mi.size() - 1)
                 << " substep: " << iSubStep << std::endl;

              ss << "     Names:";
              for (const auto& name : swp0.getNames())
                ss << " " << name;

              ss << std::endl
                 << "    State0: " << subtraj.row(iSubStep) << std::endl
                 << "    State1: " << subtraj.row(iSubStep + 1) << std::endl;

              CONSOLE_BRIDGE_logError(ss.str().c_str());
            }
          }

          if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
            break;
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
      else
      {
        tesseract_collision::ContactResultMap& segment_results = contacts[static_cast<size_t>(iStep)];
        segment_results.clear();

        const auto& swp0 = mi.at(iStep).get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
        const auto& swp1 = mi.at(iStep + 1).get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
        tesseract_scene_graph::SceneState state0 = state_solver.getState(swp0.getNames(), swp0.getPosition());
        tesseract_scene_graph::SceneState state1 = state_solver.getState(swp1.getNames(), swp1.getPosition());
        segment_results = tesseract_environment::checkTrajectorySegment(
            manager, state0.link_transforms, state1.link_transforms, config);
        if (!segment_results.empty())
        {
          found = true;
          if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
          {
            std::stringstream ss;
            ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

            ss << "     Names:";
            for (const auto& name : swp0.getNames())
              ss << " " << name;

            ss << std::endl
               << "    State0: " << swp0.getPosition() << std::endl
               << "    State1: " << swp1.getPosition() << std::endl;

            CONSOLE_BRIDGE_logError(ss.str().c_str());
          }
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
    }
  }
  else
  {
    contacts.resize(mi.size() - 1);
    for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
    {
      tesseract_collision::ContactResultMap& segment_results = contacts[static_cast<size_t>(iStep)];
      segment_results.clear();

      const auto& swp0 = mi.at(iStep).get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
      const auto& swp1 = mi.at(iStep + 1).get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
      tesseract_scene_graph::SceneState state0 = state_solver.getState(swp0.getNames(), swp0.getPosition());
      tesseract_scene_graph::SceneState state1 = state_solver.getState(swp1.getNames(), swp1.getPosition());
      segment_results = tesseract_environment::checkTrajectorySegment(
          manager, state0.link_transforms, state1.link_transforms, config);
      if (!segment_results.empty())
      {
        found = true;
        if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
        {
          std::stringstream ss;
          ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

          ss << "     Names:";
          for (const auto& name : swp0.getNames())
            ss << " " << name;

          ss << std::endl
             << "    State0: " << swp0.getPosition() << std::endl
             << "    State1: " << swp1.getPosition() << std::endl;

          CONSOLE_BRIDGE_logError(ss.str().c_str());
        }
      }

      if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }
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

  manager.applyContactManagerConfig(config.contact_manager_config);
  bool found = false;

  // Flatten results
  std::vector<std::reference_wrapper<const InstructionPoly>> mi = program.flatten(moveFilter);

  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    assert(config.longest_valid_segment_length > 0);

    contacts.resize(mi.size());
    for (std::size_t iStep = 0; iStep < mi.size(); ++iStep)
    {
      tesseract_collision::ContactResultMap& segment_results = contacts[static_cast<size_t>(iStep)];
      segment_results.clear();

      const auto& swp0 = mi.at(iStep).get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
      const StateWaypointPoly* swp1 = nullptr;

      double dist = -1;
      if (iStep < mi.size() - 1)
      {
        swp1 = &mi.at(iStep + 1).get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
        dist = (swp1->getPosition() - swp0.getPosition()).norm();
      }

      if (dist > 0 && dist > config.longest_valid_segment_length)
      {
        auto cnt = static_cast<int>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, swp0.getPosition().size());
        for (long iVar = 0; iVar < swp0.getPosition().size(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, swp0.getPosition()(iVar), swp1->getPosition()(iVar));

        for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
        {
          tesseract_scene_graph::SceneState state = state_solver.getState(swp0.getNames(), subtraj.row(iSubStep));
          tesseract_collision::ContactResultMap sub_segment_results =
              tesseract_environment::checkTrajectoryState(manager, state.link_transforms, config);
          if (!sub_segment_results.empty())
          {
            found = true;
            tesseract_environment::processInterpolatedSubSegmentCollisionResults(segment_results,
                                                                                 sub_segment_results,
                                                                                 iSubStep,
                                                                                 static_cast<int>(subtraj.rows() - 1),
                                                                                 manager.getActiveCollisionObjects(),
                                                                                 true);

            if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
            {
              std::stringstream ss;
              ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1)
                 << " substate: " << iSubStep << std::endl;

              ss << "     Names:";
              for (const auto& name : swp0.getNames())
                ss << " " << name;

              ss << std::endl << "    State: " << subtraj.row(iSubStep) << std::endl;

              CONSOLE_BRIDGE_logError(ss.str().c_str());
            }
          }

          if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
            break;
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
      else
      {
        tesseract_scene_graph::SceneState state = state_solver.getState(swp0.getNames(), swp0.getPosition());
        tesseract_collision::ContactResultMap sub_segment_results =
            tesseract_environment::checkTrajectoryState(manager, state.link_transforms, config.contact_request);
        if (!sub_segment_results.empty())
        {
          found = true;
          tesseract_environment::processInterpolatedSubSegmentCollisionResults(
              segment_results, sub_segment_results, 0, 0, manager.getActiveCollisionObjects(), true);
          if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
          {
            std::stringstream ss;
            ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

            ss << "     Names:";
            for (const auto& name : swp0.getNames())
              ss << " " << name;

            ss << std::endl << "    State: " << swp0.getPosition() << std::endl;

            CONSOLE_BRIDGE_logError(ss.str().c_str());
          }
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
    }
  }
  else
  {
    contacts.resize(mi.size());
    for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
    {
      tesseract_collision::ContactResultMap& segment_results = contacts[static_cast<size_t>(iStep)];
      segment_results.clear();

      const auto& swp0 = mi.at(iStep).get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();

      tesseract_scene_graph::SceneState state = state_solver.getState(swp0.getNames(), swp0.getPosition());
      tesseract_collision::ContactResultMap sub_segment_results =
          tesseract_environment::checkTrajectoryState(manager, state.link_transforms, config.contact_request);
      if (!sub_segment_results.empty())
      {
        found = true;
        tesseract_environment::processInterpolatedSubSegmentCollisionResults(
            segment_results, sub_segment_results, 0, 0, manager.getActiveCollisionObjects(), true);
        if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
        {
          std::stringstream ss;
          ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

          ss << "     Names:";
          for (const auto& name : swp0.getNames())
            ss << " " << name;

          ss << std::endl << "    State0: " << swp0.getPosition() << std::endl;

          CONSOLE_BRIDGE_logError(ss.str().c_str());
        }
      }

      if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }
  return found;
}

void generateNaiveSeedHelper(CompositeInstruction& composite_instructions,
                             const tesseract_environment::Environment& env,
                             const tesseract_scene_graph::SceneState& state,
                             const tesseract_common::ManipulatorInfo& manip_info,
                             std::unordered_map<std::string, std::vector<std::string>>& manip_joint_names)
{
  std::vector<std::string> group_joint_names;
  for (auto& i : composite_instructions)
  {
    if (i.isCompositeInstruction())
    {
      generateNaiveSeedHelper(i.as<CompositeInstruction>(), env, state, manip_info, manip_joint_names);
    }
    else if (i.isMoveInstruction())
    {
      auto& base_instruction = i.as<MoveInstructionPoly>();
      tesseract_common::ManipulatorInfo mi = manip_info.getCombined(base_instruction.getManipulatorInfo());

      CompositeInstruction ci;
      ci.setProfile(base_instruction.getProfile());
      ci.setDescription(base_instruction.getDescription());
      ci.setManipulatorInfo(base_instruction.getManipulatorInfo());
      ci.setProfileOverrides(base_instruction.getProfileOverrides());

      auto it = manip_joint_names.find(mi.manipulator);
      if (it == manip_joint_names.end())
      {
        group_joint_names = env.getGroupJointNames(mi.manipulator);
        manip_joint_names[mi.manipulator] = group_joint_names;
      }
      else
      {
        group_joint_names = it->second;
      }

      Eigen::VectorXd jv = state.getJointValues(group_joint_names);

      // Get move type base on base instruction type
      MoveInstructionType move_type{};
      if (base_instruction.isLinear())
        move_type = MoveInstructionType::LINEAR;
      else if (base_instruction.isFreespace())
        move_type = MoveInstructionType::FREESPACE;
      else
        throw std::runtime_error("generateNaiveSeed: Unsupported Move Instruction Type!");

      if (base_instruction.getWaypoint().isStateWaypoint())
      {
        assert(checkJointPositionFormat(group_joint_names, base_instruction.getWaypoint()));
        MoveInstructionPoly move_instruction(base_instruction);
        ci.appendMoveInstruction(move_instruction);
      }
      else if (base_instruction.getWaypoint().isJointWaypoint())
      {
        assert(checkJointPositionFormat(group_joint_names, base_instruction.getWaypoint()));
        const auto& jwp = base_instruction.getWaypoint().as<JointWaypointPoly>();
        MoveInstructionPoly move_instruction(base_instruction);
        StateWaypointPoly swp = move_instruction.createStateWaypoint();
        swp.setNames(jwp.getNames());
        swp.setPosition(jwp.getPosition());
        move_instruction.assignStateWaypoint(swp);
        move_instruction.setMoveType(move_type);
        ci.appendMoveInstruction(move_instruction);
      }
      else
      {
        MoveInstructionPoly move_instruction(base_instruction);
        StateWaypointPoly swp = move_instruction.createStateWaypoint();
        swp.setNames(group_joint_names);
        swp.setPosition(jv);
        move_instruction.assignStateWaypoint(swp);
        move_instruction.setMoveType(move_type);
        ci.appendMoveInstruction(move_instruction);
      }

      i = ci;
    }
  }
}

CompositeInstruction generateNaiveSeed(const CompositeInstruction& composite_instructions,
                                       const tesseract_environment::Environment& env)
{
  if (!composite_instructions.hasStartInstruction())
    throw std::runtime_error("Top most composite instruction is missing start instruction!");

  std::unordered_map<std::string, std::vector<std::string>> manip_joint_names;
  tesseract_scene_graph::SceneState state = env.getState();
  CompositeInstruction seed = composite_instructions;
  const tesseract_common::ManipulatorInfo& mi = composite_instructions.getManipulatorInfo();

  if (!seed.hasStartInstruction())
    throw std::runtime_error("Top most composite instruction start instruction has invalid waypoint type!");

  MoveInstructionPoly base_instruction = seed.getStartInstruction();
  tesseract_common::ManipulatorInfo base_mi = base_instruction.getManipulatorInfo();

  tesseract_common::ManipulatorInfo start_mi = mi.getCombined(base_mi);
  std::vector<std::string> joint_names = env.getGroupJointNames(start_mi.manipulator);
  manip_joint_names[start_mi.manipulator] = joint_names;
  Eigen::VectorXd jv = state.getJointValues(joint_names);

  if (base_instruction.getWaypoint().isStateWaypoint())
  {
    assert(checkJointPositionFormat(joint_names, base_instruction.getWaypoint()));

    MoveInstructionPoly move_instruction(base_instruction);
    move_instruction.setMoveType(MoveInstructionType::START);
    move_instruction.setManipulatorInfo(base_mi);
    seed.setStartInstruction(move_instruction);
  }
  else if (base_instruction.getWaypoint().isJointWaypoint())
  {
    assert(checkJointPositionFormat(joint_names, base_instruction.getWaypoint()));
    const auto& jwp = base_instruction.getWaypoint().as<JointWaypointPoly>();
    MoveInstructionPoly move_instruction(base_instruction);
    StateWaypointPoly swp = move_instruction.createStateWaypoint();
    swp.setNames(jwp.getNames());
    swp.setPosition(jwp.getPosition());
    move_instruction.assignStateWaypoint(swp);
    move_instruction.setMoveType(MoveInstructionType::START);
    move_instruction.setManipulatorInfo(base_mi);
    seed.setStartInstruction(move_instruction);
  }
  else
  {
    MoveInstructionPoly move_instruction(base_instruction);
    StateWaypointPoly swp = move_instruction.createStateWaypoint();
    swp.setNames(joint_names);
    swp.setPosition(jv);
    move_instruction.assignStateWaypoint(swp);
    move_instruction.setMoveType(MoveInstructionType::START);
    move_instruction.setManipulatorInfo(base_mi);
    seed.setStartInstruction(move_instruction);
  }

  generateNaiveSeedHelper(seed, env, state, mi, manip_joint_names);
  return seed;
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
  if (!composite_instructions.hasStartInstruction())
    throw std::runtime_error("Top most composite instruction is missing start instruction!");

  std::unordered_map<std::string, std::vector<std::string>> manip_joint_names;
  bool format_required = false;
  tesseract_common::ManipulatorInfo mi = composite_instructions.getManipulatorInfo();

  std::unordered_map<std::string, tesseract_kinematics::JointGroup::UPtr> manipulators;

  if (composite_instructions.hasStartInstruction())
  {
    auto& pi = composite_instructions.getStartInstruction();

    tesseract_common::ManipulatorInfo start_mi = mi.getCombined(pi.getManipulatorInfo());

    std::vector<std::string> joint_names;
    auto it = manip_joint_names.find(start_mi.manipulator);
    if (it == manip_joint_names.end())
    {
      joint_names = env.getGroupJointNames(start_mi.manipulator);
      manip_joint_names[start_mi.manipulator] = joint_names;
    }
    else
    {
      joint_names = it->second;
    }

    if (pi.getWaypoint().isStateWaypoint() || pi.getWaypoint().isJointWaypoint())
    {
      if (formatJointPosition(joint_names, pi.getWaypoint()))
        format_required = true;
    }
  }
  else
    throw std::runtime_error("Top most composite instruction start instruction has invalid waypoint type!");

  if (formatProgramHelper(composite_instructions, env, mi, manip_joint_names))
    format_required = true;

  return format_required;
}
};  // namespace tesseract_planning
