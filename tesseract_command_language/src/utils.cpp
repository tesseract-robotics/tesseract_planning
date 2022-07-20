/**
 * @file utils.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <algorithm>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/utils.h>
#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/waypoint_type.h>
#include <tesseract_command_language/core/cartesian_waypoint_poly.h>
#include <tesseract_command_language/core/joint_waypoint_poly.h>
#include <tesseract_command_language/core/state_waypoint_poly.h>

namespace tesseract_planning
{
static const tesseract_planning::locateFilterFn toJointTrajectoryInstructionFilter =
    [](const tesseract_planning::Instruction& i,
       const tesseract_planning::CompositeInstruction& /*composite*/,
       bool parent_is_first_composite) {
      if (isMoveInstruction(i))
      {
        if (i.as<MoveInstructionPoly>().isStart())
          return (parent_is_first_composite);

        return true;
      }

      return false;
    };

tesseract_common::JointTrajectory toJointTrajectory(const CompositeInstruction& composite_instructions)
{
  tesseract_common::JointTrajectory trajectory;
  std::vector<std::reference_wrapper<const Instruction>> flattened_program =
      composite_instructions.flatten(toJointTrajectoryInstructionFilter);
  trajectory.reserve(flattened_program.size());
  trajectory.description = composite_instructions.getDescription();

  double last_time = 0;
  double current_time = 0;
  double total_time = 0;
  for (auto& i : flattened_program)
  {
    if (isMoveInstruction(i))
    {
      const auto& pi = i.get().as<MoveInstructionPoly>();
      if (tesseract_planning::isJointWaypoint(pi.getWaypoint()))
      {
        const auto& jwp = pi.getWaypoint().as<JointWaypointPoly>();
        tesseract_common::JointState joint_state;
        joint_state.joint_names = jwp.getNames();
        joint_state.position = jwp.getPosition();

        double dt = 1;
        current_time = current_time + dt;
        total_time += dt;
        joint_state.time = total_time;
        last_time = current_time;
        trajectory.push_back(joint_state);
      }
      else if (isStateWaypoint(pi.getWaypoint()))
      {
        const auto& swp = pi.getWaypoint().as<StateWaypointPoly>();

        tesseract_common::JointState joint_state;
        joint_state.joint_names = swp.getNames();
        joint_state.position = swp.getPosition();
        joint_state.velocity = swp.getVelocity();
        joint_state.acceleration = swp.getAcceleration();
        joint_state.time = swp.getTime();

        // It is possible for sub composites to start back from zero, this accounts for it
        current_time = joint_state.time;
        if (current_time < last_time)
          last_time = 0;

        double dt = current_time - last_time;
        total_time += dt;
        joint_state.time = total_time;
        last_time = current_time;
        trajectory.push_back(joint_state);
      }
    }
  }
  return trajectory;
}

const Eigen::VectorXd& getJointPosition(const Waypoint& waypoint)
{
  if (isJointWaypoint(waypoint))
    return waypoint.as<JointWaypointPoly>().getPosition();

  if (isStateWaypoint(waypoint))
    return waypoint.as<StateWaypointPoly>().getPosition();

  throw std::runtime_error("Unsupported waypoint type.");
}

const std::vector<std::string>& getJointNames(const Waypoint& waypoint)
{
  if (isJointWaypoint(waypoint))
    return waypoint.as<JointWaypointPoly>().getNames();

  if (isStateWaypoint(waypoint))
    return waypoint.as<StateWaypointPoly>().getNames();

  throw std::runtime_error("Unsupported waypoint type.");
}

Eigen::VectorXd getJointPosition(const std::vector<std::string>& joint_names, const Waypoint& waypoint)
{
  Eigen::VectorXd jv;
  std::vector<std::string> jn;
  if (isJointWaypoint(waypoint))
  {
    const auto& jwp = waypoint.as<JointWaypointPoly>();
    jv = jwp.getPosition();
    jn = jwp.getNames();
  }
  else if (isStateWaypoint(waypoint))
  {
    const auto& swp = waypoint.as<StateWaypointPoly>();
    jv = swp.getPosition();
    jn = swp.getNames();
  }
  else
  {
    throw std::runtime_error("Unsupported waypoint type.");
  }

  if (jn.size() != joint_names.size())
    throw std::runtime_error("Joint name sizes do not match!");

  if (joint_names == jn)
    return jv;

  Eigen::VectorXd output = jv;
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    if (joint_names[i] == jn[i])
      continue;

    auto it = std::find(jn.begin(), jn.end(), joint_names[i]);
    if (it == jn.end())
      throw std::runtime_error("Joint names do not match!");

    long idx = std::distance(jn.begin(), it);
    output(static_cast<long>(i)) = jv(static_cast<long>(idx));
  }

  return output;
}

bool formatJointPosition(const std::vector<std::string>& joint_names, Waypoint& waypoint)
{
  Eigen::VectorXd* jv{ nullptr };
  std::vector<std::string>* jn{ nullptr };
  if (isJointWaypoint(waypoint))
  {
    auto& jwp = waypoint.as<JointWaypointPoly>();
    jv = &(jwp.getPosition());
    jn = &(jwp.getNames());
  }
  else if (isStateWaypoint(waypoint))
  {
    auto& swp = waypoint.as<StateWaypointPoly>();
    jv = &(swp.getPosition());
    jn = &(swp.getNames());
  }
  else
  {
    throw std::runtime_error("Unsupported waypoint type.");
  }

  if (jn->size() != joint_names.size())
    throw std::runtime_error("Joint name sizes do not match!");

  if (joint_names == *jn)
    return false;

  Eigen::VectorXd output = *jv;
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    if (joint_names[i] == (*jn)[i])
      continue;

    auto it = std::find(jn->begin(), jn->end(), joint_names[i]);
    if (it == jn->end())
      throw std::runtime_error("Joint names do not match!");

    long idx = std::distance(jn->begin(), it);
    output(static_cast<long>(i)) = (*jv)(static_cast<long>(idx));
  }

  *jn = joint_names;
  *jv = output;

  return true;
}

bool checkJointPositionFormat(const std::vector<std::string>& joint_names, const Waypoint& waypoint)
{
  if (isJointWaypoint(waypoint))
    return (joint_names == waypoint.as<JointWaypointPoly>().getNames());

  if (isStateWaypoint(waypoint))
    return (joint_names == waypoint.as<StateWaypointPoly>().getNames());

  throw std::runtime_error("Unsupported waypoint type.");
}

bool setJointPosition(Waypoint& waypoint, const Eigen::Ref<const Eigen::VectorXd>& position)
{
  if (isJointWaypoint(waypoint))
    waypoint.as<JointWaypointPoly>().setPosition(position);
  else if (isStateWaypoint(waypoint))
    waypoint.as<StateWaypointPoly>().setPosition(position);
  else
    return false;

  return true;
}

bool isWithinJointLimits(const Waypoint& wp, const Eigen::Ref<const Eigen::MatrixX2d>& limits)
{
  if (isJointWaypoint(wp) || isStateWaypoint(wp))
  {
    const Eigen::VectorXd& cmd_pos = getJointPosition(wp);
    return tesseract_common::isWithinPositionLimits<double>(cmd_pos, limits);
  }

  return true;
}

bool clampToJointLimits(Waypoint& wp, const Eigen::Ref<const Eigen::MatrixX2d>& limits, double max_deviation)
{
  const Eigen::VectorXd deviation_vec = Eigen::VectorXd::Constant(limits.rows(), max_deviation);
  return clampToJointLimits(wp, limits, deviation_vec);
}

bool clampToJointLimits(Waypoint& wp,
                        const Eigen::Ref<const Eigen::MatrixX2d>& limits,
                        const Eigen::Ref<const Eigen::VectorXd>& max_deviation)
{
  if (isJointWaypoint(wp) || isStateWaypoint(wp))
  {
    Eigen::VectorXd cmd_pos = getJointPosition(wp);

    const Eigen::VectorXd max_rel_diff =
        Eigen::VectorXd::Constant(cmd_pos.size(), std::numeric_limits<double>::epsilon());
    if (!tesseract_common::satisfiesPositionLimits<double>(cmd_pos, limits, max_deviation, max_rel_diff))
      return false;

    CONSOLE_BRIDGE_logDebug("Clamping Waypoint to joint limits");
    tesseract_common::enforcePositionLimits<double>(cmd_pos, limits);
    return setJointPosition(wp, cmd_pos);
  }

  return true;
}

void generateSkeletonSeedHelper(CompositeInstruction& composite_instructions)
{
  for (auto& i : composite_instructions)
  {
    if (isCompositeInstruction(i))
    {
      generateSkeletonSeedHelper(i.as<CompositeInstruction>());
    }
    else if (isMoveInstruction(i))
    {
      CompositeInstruction ci;
      const auto& pi = i.as<MoveInstructionPoly>();
      ci.setProfile(pi.getProfile());
      ci.setDescription(pi.getDescription());
      ci.setManipulatorInfo(pi.getManipulatorInfo());
      //      ci.profile_overrides = pi.profile_overrides;

      i = ci;
    }
  }
}

bool toDelimitedFile(const CompositeInstruction& composite_instructions, const std::string& file_path, char separator)
{
  static const Eigen::IOFormat eigen_format(
      Eigen::StreamPrecision, Eigen::DontAlignCols, "", std::string(&separator, 1));
  std::ofstream myfile;
  myfile.open(file_path);

  std::vector<std::reference_wrapper<const Instruction>> mi = composite_instructions.flatten(&moveFilter);

  // Write Joint names as header
  std::vector<std::string> joint_names = getJointNames(mi.front().get().as<MoveInstructionPoly>().getWaypoint());

  for (std::size_t i = 0; i < joint_names.size() - 1; ++i)
    myfile << joint_names[i] << separator;

  myfile << joint_names.back() << std::endl;

  // Write Positions
  for (const auto& i : mi)
  {
    const Eigen::VectorXd& p = getJointPosition(i.get().as<MoveInstructionPoly>().getWaypoint());
    myfile << p.format(eigen_format) << std::endl;
  }

  myfile.close();
  return true;
}

CompositeInstruction generateSkeletonSeed(const CompositeInstruction& composite_instructions)
{
  CompositeInstruction seed = composite_instructions;
  generateSkeletonSeedHelper(seed);
  return seed;
}

}  // namespace tesseract_planning
