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
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
Eigen::Isometry3d calcPose(const Waypoint& wp,
                           const std::string& working_frame,
                           const std::string& tip_link,
                           const Eigen::Isometry3d& tcp,
                           const tesseract_scene_graph::SceneState& current_state,
                           tesseract_scene_graph::StateSolver& state_solver)
{
  if (isStateWaypoint(wp))
  {
    const auto& swp = wp.as<StateWaypoint>();
    assert(static_cast<long>(swp.joint_names.size()) == swp.position.size());
    tesseract_scene_graph::SceneState state = state_solver.getState(swp.joint_names, swp.position);
    return (state.link_transforms[tip_link] * tcp);
  }

  if (isJointWaypoint(wp))
  {
    const auto& jwp = wp.as<JointWaypoint>();
    assert(static_cast<long>(jwp.joint_names.size()) == jwp.size());
    tesseract_scene_graph::SceneState state = state_solver.getState(jwp.joint_names, jwp);
    return (state.link_transforms[tip_link] * tcp);
  }

  if (isCartesianWaypoint(wp))
  {
    const auto& cwp = wp.as<CartesianWaypoint>();
    if (working_frame.empty())
      return cwp.waypoint;

    return (current_state.link_transforms.at(working_frame) * cwp);
  }

  throw std::runtime_error("toToolpath: Unsupported Waypoint Type!");
}

tesseract_common::Toolpath toToolpath(const Instruction& instruction, const tesseract_environment::Environment& env)
{
  using namespace tesseract_planning;

  tesseract_common::Toolpath toolpath;
  tesseract_common::VectorIsometry3d poses;

  tesseract_scene_graph::StateSolver::UPtr state_solver = env.getStateSolver();
  tesseract_scene_graph::SceneState state = env.getState();
  if (isCompositeInstruction(instruction))
  {
    const auto& ci = instruction.as<CompositeInstruction>();

    // Assume all the plan instructions have the same manipulator as the composite
    assert(!ci.getManipulatorInfo().empty());
    const ManipulatorInfo& composite_mi = ci.getManipulatorInfo();

    std::vector<std::reference_wrapper<const Instruction>> fi = tesseract_planning::flatten(ci, planFilter);
    if (fi.empty())
      fi = tesseract_planning::flatten(ci, moveFilter);

    for (const auto& i : fi)
    {
      ManipulatorInfo manip_info;

      // Check for updated manipulator information and get waypoint
      Waypoint wp{ NullWaypoint() };
      if (isPlanInstruction(i.get()))
      {
        const auto& pi = i.get().as<PlanInstruction>();
        manip_info = composite_mi.getCombined(pi.getManipulatorInfo());
        wp = pi.getWaypoint();
      }
      else if (isMoveInstruction(i.get()))
      {
        const auto& mi = i.get().as<MoveInstruction>();
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
  else if (isPlanInstruction(instruction))
  {
    assert(isPlanInstruction(instruction));
    const auto& pi = instruction.as<PlanInstruction>();

    // Assume all the plan instructions have the same manipulator as the composite
    assert(!pi.getManipulatorInfo().empty());
    const ManipulatorInfo& composite_mi = pi.getManipulatorInfo();
    ManipulatorInfo manip_info = composite_mi.getCombined(pi.getManipulatorInfo());

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

std::vector<Waypoint> interpolate_waypoint(const Waypoint& start, const Waypoint& stop, long steps)
{
  if (isCartesianWaypoint(start))
  {
    const Eigen::Isometry3d& w1 = start.as<CartesianWaypoint>().waypoint;
    const Eigen::Isometry3d& w2 = stop.as<CartesianWaypoint>().waypoint;
    tesseract_common::VectorIsometry3d eigen_poses = interpolate(w1, w2, steps);

    std::vector<Waypoint> result;
    result.reserve(eigen_poses.size());
    for (auto& eigen_pose : eigen_poses)
      result.emplace_back(CartesianWaypoint(eigen_pose));

    return result;
  }

  if (isJointWaypoint(start))
  {
    const auto& jwp1 = start.as<JointWaypoint>();
    //      const auto* jwp2 = stop.cast_const<JointWaypoint>();

    // TODO: Should check joint names are in the same order

    const Eigen::VectorXd& w1 = start.as<JointWaypoint>().waypoint;
    const Eigen::VectorXd& w2 = stop.as<JointWaypoint>().waypoint;
    Eigen::MatrixXd joint_poses = interpolate(w1, w2, steps);

    std::vector<Waypoint> result;
    result.reserve(static_cast<std::size_t>(joint_poses.cols()));
    for (int i = 0; i < joint_poses.cols(); ++i)
      result.emplace_back(JointWaypoint(jwp1.joint_names, joint_poses.col(i)));

    return result;
  }

  CONSOLE_BRIDGE_logError("Interpolator for Waypoint type %d is currently not support!", start.getType().hash_code());
  return std::vector<Waypoint>();
}

bool programFlattenFilter(const Instruction& instruction,
                          const CompositeInstruction& /*composite*/,
                          bool parent_is_first_composite)
{
  if (isMoveInstruction(instruction))
  {
    if (instruction.as<MoveInstruction>().isStart())
      return (parent_is_first_composite);
  }
  else if (isPlanInstruction(instruction))
  {
    if (instruction.as<PlanInstruction>().isStart())
      return (parent_is_first_composite);
  }
  else if (isCompositeInstruction(instruction))
  {
    return false;
  }

  return true;
};

std::vector<std::reference_wrapper<Instruction>> flattenProgram(CompositeInstruction& composite_instruction)
{
  return flatten(composite_instruction, programFlattenFilter);
}

std::vector<std::reference_wrapper<const Instruction>> flattenProgram(const CompositeInstruction& composite_instruction)
{
  return flatten(composite_instruction, programFlattenFilter);
}

std::vector<std::reference_wrapper<Instruction>> flattenProgramToPattern(CompositeInstruction& composite_instruction,
                                                                         const CompositeInstruction& pattern)
{
  return flattenToPattern(composite_instruction, pattern, programFlattenFilter);
}

std::vector<std::reference_wrapper<const Instruction>>
flattenProgramToPattern(const CompositeInstruction& composite_instruction, const CompositeInstruction& pattern)
{
  return flattenToPattern(composite_instruction, pattern, programFlattenFilter);
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
  bool found = false;

  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
  {
    assert(config.longest_valid_segment_length > 0);

    // Flatten results
    std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

    contacts.reserve(mi.size() - 1);
    for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
    {
      const auto& swp0 = mi.at(iStep).get().as<MoveInstruction>().getWaypoint().as<StateWaypoint>();
      const auto& swp1 = mi.at(iStep + 1).get().as<MoveInstruction>().getWaypoint().as<StateWaypoint>();

      // TODO: Should check joint names and make sure they are in the same order
      double dist = (swp1.position - swp0.position).norm();
      if (dist > config.longest_valid_segment_length)
      {
        auto cnt = static_cast<long>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, swp0.position.size());
        for (long iVar = 0; iVar < swp0.position.size(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, swp0.position(iVar), swp1.position(iVar));

        for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
        {
          tesseract_scene_graph::SceneState state0 = state_solver.getState(swp0.joint_names, subtraj.row(iSubStep));
          tesseract_scene_graph::SceneState state1 = state_solver.getState(swp0.joint_names, subtraj.row(iSubStep + 1));
          if (tesseract_environment::checkTrajectorySegment(
                  contacts, manager, state0.link_transforms, state1.link_transforms, config.contact_request))
          {
            found = true;
            if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
            {
              std::stringstream ss;
              ss << "Continuous collision detected at step: " << iStep << " of " << (mi.size() - 1)
                 << " substep: " << iSubStep << std::endl;

              ss << "     Names:";
              for (const auto& name : swp0.joint_names)
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
        // Flatten results
        std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

        contacts.reserve(mi.size() - 1);
        for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
        {
          const auto& swp0 = mi.at(iStep).get().as<MoveInstruction>().getWaypoint().as<StateWaypoint>();
          const auto& swp1 = mi.at(iStep + 1).get().as<MoveInstruction>().getWaypoint().as<StateWaypoint>();

          // TODO: Should check joint names and make sure they are in the same order
          double dist = (swp1.position - swp0.position).norm();
          if (dist > config.longest_valid_segment_length)
          {
            auto cnt = static_cast<long>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
            tesseract_common::TrajArray subtraj(cnt, swp0.position.size());
            for (long iVar = 0; iVar < swp0.position.size(); ++iVar)
              subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, swp0.position(iVar), swp1.position(iVar));

            for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
            {
              tesseract_scene_graph::SceneState state0 = state_solver.getState(swp0.joint_names, subtraj.row(iSubStep));
              tesseract_scene_graph::SceneState state1 =
                  state_solver.getState(swp0.joint_names, subtraj.row(iSubStep + 1));
              if (tesseract_environment::checkTrajectorySegment(
                      contacts, manager, state0.link_transforms, state1.link_transforms, config.contact_request))
              {
                found = true;
                if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
                {
                  std::stringstream ss;
                  ss << "Continuous collision detected at step: " << iStep << " of " << (mi.size() - 1)
                     << " substep: " << iSubStep << std::endl;

                  ss << "     Names:";
                  for (const auto& name : swp0.joint_names)
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
        }
      }
    }
  }
  else
  {
    bool found = false;

    // Flatten results
    std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

    contacts.reserve(mi.size());
    for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
    {
      const auto& swp0 = mi.at(iStep).get().as<MoveInstruction>().getWaypoint().as<StateWaypoint>();
      const auto& swp1 = mi.at(iStep + 1).get().as<MoveInstruction>().getWaypoint().as<StateWaypoint>();
      tesseract_scene_graph::SceneState state0 = state_solver.getState(swp0.joint_names, swp0.position);
      tesseract_scene_graph::SceneState state1 = state_solver.getState(swp1.joint_names, swp1.position);

      if (tesseract_environment::checkTrajectorySegment(
              contacts, manager, state0.link_transforms, state1.link_transforms, config))
      {
        found = true;
        if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
        {
          std::stringstream ss;
          ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

          ss << "     Names:";
          for (const auto& name : swp0.joint_names)
            ss << " " << name;

          ss << std::endl
             << "    State0: " << swp0.position << std::endl
             << "    State1: " << swp1.position << std::endl;

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

  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    assert(config.longest_valid_segment_length > 0);

    // Flatten results
    std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

    contacts.reserve(mi.size());
    for (std::size_t iStep = 0; iStep < mi.size(); ++iStep)
    {
      const auto& swp0 = mi.at(iStep).get().as<MoveInstruction>().getWaypoint().as<StateWaypoint>();
      const StateWaypoint* swp1 = nullptr;

      double dist = -1;
      if (iStep < mi.size() - 1)
      {
        swp1 = &mi.at(iStep + 1).get().as<MoveInstruction>().getWaypoint().as<StateWaypoint>();
        dist = (swp1->position - swp0.position).norm();
      }

      if (dist > 0 && dist > config.longest_valid_segment_length)
      {
        auto cnt = static_cast<int>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, swp0.position.size());
        for (long iVar = 0; iVar < swp0.position.size(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, swp0.position(iVar), swp1->position(iVar));

        for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
        {
          tesseract_scene_graph::SceneState state = state_solver.getState(swp0.joint_names, subtraj.row(iSubStep));
          if (tesseract_environment::checkTrajectoryState(contacts, manager, state.link_transforms, config))
          {
            found = true;
            if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
            {
              std::stringstream ss;
              ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1)
                 << " substate: " << iSubStep << std::endl;

              ss << "     Names:";
              for (const auto& name : swp0.joint_names)
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
        tesseract_scene_graph::SceneState state = state_solver.getState(swp0.joint_names, swp0.position);
        if (tesseract_environment::checkTrajectoryState(
                contacts, manager, state.link_transforms, config.contact_request))
        {
          found = true;
          if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
          {
            std::stringstream ss;
            ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

            ss << "     Names:";
            for (const auto& name : swp0.joint_names)
              ss << " " << name;

            ss << std::endl << "    State: " << swp0.position << std::endl;

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
    // Flatten results
    std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

    contacts.reserve(mi.size());
    for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
    {
      const auto& swp0 = mi.at(iStep).get().as<MoveInstruction>().getWaypoint().as<StateWaypoint>();

      tesseract_scene_graph::SceneState state = state_solver.getState(swp0.joint_names, swp0.position);
      if (tesseract_environment::checkTrajectoryState(contacts, manager, state.link_transforms, config.contact_request))
      {
        found = true;
        if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
        {
          std::stringstream ss;
          ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

          ss << "     Names:";
          for (const auto& name : swp0.joint_names)
            ss << " " << name;

          ss << std::endl << "    State0: " << swp0.position << std::endl;

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
                             const ManipulatorInfo& manip_info,
                             std::unordered_map<std::string, std::vector<std::string>>& manip_joint_names)
{
  std::vector<std::string> group_joint_names;
  for (auto& i : composite_instructions)
  {
    if (isCompositeInstruction(i))
    {
      generateNaiveSeedHelper(i.as<CompositeInstruction>(), env, state, manip_info, manip_joint_names);
    }
    else if (isPlanInstruction(i))
    {
      auto& base_instruction = i.as<PlanInstruction>();
      ManipulatorInfo mi = manip_info.getCombined(base_instruction.getManipulatorInfo());

      CompositeInstruction ci;
      ci.setProfile(base_instruction.getProfile());
      ci.setDescription(base_instruction.getDescription());
      ci.setManipulatorInfo(base_instruction.getManipulatorInfo());
      ci.profile_overrides = base_instruction.profile_overrides;

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
      MoveInstructionType move_type;
      if (base_instruction.isLinear())
        move_type = MoveInstructionType::LINEAR;
      else if (base_instruction.isFreespace())
        move_type = MoveInstructionType::FREESPACE;
      else
        throw std::runtime_error("generateNaiveSeed: Unsupported Move Instruction Type!");

      if (isStateWaypoint(base_instruction.getWaypoint()))
      {
        assert(checkJointPositionFormat(group_joint_names, base_instruction.getWaypoint()));
        MoveInstruction move_instruction(base_instruction.getWaypoint(), move_type);
        move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
        move_instruction.setDescription(base_instruction.getDescription());
        move_instruction.setProfile(base_instruction.getProfile());
        move_instruction.profile_overrides = base_instruction.profile_overrides;
        ci.push_back(move_instruction);
      }
      else if (isJointWaypoint(base_instruction.getWaypoint()))
      {
        assert(checkJointPositionFormat(group_joint_names, base_instruction.getWaypoint()));
        const auto& jwp = base_instruction.getWaypoint().as<JointWaypoint>();
        MoveInstruction move_instruction(StateWaypoint(jwp.joint_names, jwp.waypoint), move_type);
        move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
        move_instruction.setDescription(base_instruction.getDescription());
        move_instruction.setProfile(base_instruction.getProfile());
        move_instruction.profile_overrides = base_instruction.profile_overrides;
        ci.push_back(move_instruction);
      }
      else
      {
        MoveInstruction move_instruction(StateWaypoint(group_joint_names, jv), move_type);
        move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
        move_instruction.setDescription(base_instruction.getDescription());
        move_instruction.setProfile(base_instruction.getProfile());
        move_instruction.profile_overrides = base_instruction.profile_overrides;
        ci.push_back(move_instruction);
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
  const ManipulatorInfo& mi = composite_instructions.getManipulatorInfo();

  Waypoint wp{ NullWaypoint() };
  ManipulatorInfo base_mi;
  std::string description;
  std::string profile;
  ProfileDictionary::Ptr profile_overrides;
  if (isPlanInstruction(seed.getStartInstruction()))
  {
    const auto& pi = seed.getStartInstruction().as<PlanInstruction>();
    wp = pi.getWaypoint();
    base_mi = pi.getManipulatorInfo();
    description = pi.getDescription();
    profile = pi.getProfile();
    profile_overrides = pi.profile_overrides;
  }
  else if (isMoveInstruction(seed.getStartInstruction()))
  {
    const auto& pi = seed.getStartInstruction().as<MoveInstruction>();
    wp = pi.getWaypoint();
    base_mi = pi.getManipulatorInfo();
    description = pi.getDescription();
    profile = pi.getProfile();
    profile_overrides = pi.profile_overrides;
  }
  else
    throw std::runtime_error("Top most composite instruction start instruction has invalid waypoint type!");

  ManipulatorInfo start_mi = mi.getCombined(base_mi);
  std::vector<std::string> joint_names = env.getGroupJointNames(start_mi.manipulator);
  manip_joint_names[start_mi.manipulator] = joint_names;
  Eigen::VectorXd jv = state.getJointValues(joint_names);

  if (isStateWaypoint(wp))
  {
    assert(checkJointPositionFormat(joint_names, wp));
    MoveInstruction move_instruction(wp, MoveInstructionType::START);
    move_instruction.setManipulatorInfo(base_mi);
    move_instruction.setDescription(description);
    move_instruction.setProfile(profile);
    move_instruction.profile_overrides = profile_overrides;
    seed.setStartInstruction(move_instruction);
  }
  else if (isJointWaypoint(wp))
  {
    assert(checkJointPositionFormat(joint_names, wp));
    const auto& jwp = wp.as<JointWaypoint>();
    MoveInstruction move_instruction(StateWaypoint(jwp.joint_names, jwp.waypoint), MoveInstructionType::START);
    move_instruction.setManipulatorInfo(base_mi);
    move_instruction.setDescription(description);
    move_instruction.setProfile(profile);
    move_instruction.profile_overrides = profile_overrides;
    seed.setStartInstruction(move_instruction);
  }
  else
  {
    MoveInstruction move_instruction(StateWaypoint(joint_names, jv), MoveInstructionType::START);
    move_instruction.setManipulatorInfo(base_mi);
    move_instruction.setDescription(description);
    move_instruction.setProfile(profile);
    move_instruction.profile_overrides = profile_overrides;
    seed.setStartInstruction(move_instruction);
  }

  generateNaiveSeedHelper(seed, env, state, mi, manip_joint_names);
  return seed;
}

bool formatProgramHelper(CompositeInstruction& composite_instructions,
                         const tesseract_environment::Environment& env,
                         const ManipulatorInfo& manip_info,
                         std::unordered_map<std::string, std::vector<std::string>>& manip_joint_names)
{
  bool format_required = false;
  for (auto& i : composite_instructions)
  {
    if (isCompositeInstruction(i))
    {
      if (formatProgramHelper(i.as<CompositeInstruction>(), env, manip_info, manip_joint_names))
        format_required = true;
    }
    else if (isPlanInstruction(i))
    {
      auto& base_instruction = i.as<PlanInstruction>();
      ManipulatorInfo mi = manip_info.getCombined(base_instruction.getManipulatorInfo());

      ManipulatorInfo combined_mi = mi.getCombined(base_instruction.getManipulatorInfo());

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

      if (isStateWaypoint(base_instruction.getWaypoint()) || isJointWaypoint(base_instruction.getWaypoint()))
      {
        if (formatJointPosition(joint_names, base_instruction.getWaypoint()))
          format_required = true;
      }
    }
    else if (isMoveInstruction(i))
    {
      auto& base_instruction = i.as<MoveInstruction>();
      ManipulatorInfo mi = manip_info.getCombined(base_instruction.getManipulatorInfo());

      ManipulatorInfo combined_mi = mi.getCombined(base_instruction.getManipulatorInfo());

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

      if (isStateWaypoint(base_instruction.getWaypoint()) || isJointWaypoint(base_instruction.getWaypoint()))
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
  ManipulatorInfo mi = composite_instructions.getManipulatorInfo();

  std::unordered_map<std::string, tesseract_kinematics::JointGroup::UPtr> manipulators;

  if (isPlanInstruction(composite_instructions.getStartInstruction()))
  {
    auto& pi = composite_instructions.getStartInstruction().as<PlanInstruction>();

    ManipulatorInfo start_mi = mi.getCombined(pi.getManipulatorInfo());

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

    if (isStateWaypoint(pi.getWaypoint()) || isJointWaypoint(pi.getWaypoint()))
    {
      if (formatJointPosition(joint_names, pi.getWaypoint()))
        format_required = true;
    }
  }
  else if (isMoveInstruction(composite_instructions.getStartInstruction()))
  {
    auto& pi = composite_instructions.getStartInstruction().as<MoveInstruction>();

    ManipulatorInfo start_mi = mi.getCombined(pi.getManipulatorInfo());

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

    if (isStateWaypoint(pi.getWaypoint()) || isJointWaypoint(pi.getWaypoint()))
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
