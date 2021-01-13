/**
 * @file fixed_size_interpolation.h
 * @brief
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date July 23, 2020
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
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/simple/step_generators/fixed_size_interpolation.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/state_waypoint.h>

namespace tesseract_planning
{
FixedSizeTransitionInfo::FixedSizeTransitionInfo(const InstructionInfo& prev,
                                                 const InstructionInfo& base,
                                                 const PlannerRequest& request)
  : prev(prev), base(base), request(request)
{
}

CompositeInstruction simplePlannerGeneratorFixedSize(const PlanInstruction& prev_instruction,
                                                     const PlanInstruction& base_instruction,
                                                     const PlannerRequest& request,
                                                     const ManipulatorInfo& manip_info,
                                                     int freespace_steps,
                                                     int linear_steps)
{
  InstructionInfo info1(prev_instruction, request, manip_info);
  InstructionInfo info2(base_instruction, request, manip_info);

  FixedSizeTransitionInfo trans_info(info1, info2, request);
  trans_info.freespace_steps = freespace_steps;
  trans_info.linear_steps = linear_steps;

  if (!info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return stateJointJointWaypointFixedSize(trans_info);

  if (!info1.has_cartesian_waypoint && info2.has_cartesian_waypoint)
    return stateJointCartWaypointFixedSize(trans_info);

  if (info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return stateCartJointWaypointFixedSize(trans_info);

  return stateCartCartWaypointFixedSize(trans_info);
}

CompositeInstruction stateJointJointWaypointFixedSize(const FixedSizeTransitionInfo& trans_info)
{
  // Calculate FK for start and end
  const Eigen::VectorXd& j1 = getJointPosition(trans_info.prev.instruction.getWaypoint());
  const Eigen::VectorXd& j2 = getJointPosition(trans_info.base.instruction.getWaypoint());

  Eigen::MatrixXd states;
  if (trans_info.base.instruction.getPlanType() == PlanInstructionType::LINEAR)
  {
    if (trans_info.linear_steps > 1)
      states = interpolate(j1, j2, trans_info.linear_steps);
    else
      states = j2.replicate(1, 2);
  }
  else if (trans_info.base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
  {
    if (trans_info.freespace_steps > 1)
      states = interpolate(j1, j2, trans_info.freespace_steps);
    else
      states = j2.replicate(1, 2);
  }
  else
  {
    throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported PlanInstructionType!");
  }

  return getInterpolatedComposite(states, trans_info.base.fwd_kin, trans_info.base.instruction);
}

CompositeInstruction stateJointCartWaypointFixedSize(const FixedSizeTransitionInfo& trans_info)
{
  const Eigen::VectorXd& j1 = getJointPosition(trans_info.prev.instruction.getWaypoint());

  Eigen::Isometry3d p2_world = trans_info.base.extractCartesianWorldPose();
  Eigen::Isometry3d p2 = trans_info.base.calcCartesianLocalPose(p2_world);

  Eigen::VectorXd j2 = getClosestJointSolution(p2, trans_info.base.inv_kin, j1);

  Eigen::MatrixXd states;
  if (j2.size() == 0)
  {
    if (trans_info.base.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = j1.replicate(1, trans_info.linear_steps + 1);
    else if (trans_info.base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = j1.replicate(1, trans_info.freespace_steps + 1);
    else
      throw std::runtime_error("stateJointCartWaypointFixedSize: Unsupported PlanInstructionType!");
  }
  else
  {
    if (trans_info.base.instruction.getPlanType() == PlanInstructionType::LINEAR)
    {
      if (trans_info.linear_steps > 1)
        states = interpolate(j1, j2, trans_info.linear_steps);
      else
        states = j2.replicate(1, 2);
    }
    else if (trans_info.base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
    {
      if (trans_info.freespace_steps > 1)
        states = interpolate(j1, j2, trans_info.freespace_steps);
      else
        states = j2.replicate(1, 2);
    }
    else
    {
      throw std::runtime_error("stateJointCartWaypointFixedSize: Unsupported PlanInstructionType!");
    }
  }

  return getInterpolatedComposite(states, trans_info.base.fwd_kin, trans_info.base.instruction);
}

CompositeInstruction stateCartJointWaypointFixedSize(const FixedSizeTransitionInfo& trans_info)
{
  const Eigen::VectorXd& j2 = getJointPosition(trans_info.base.instruction.getWaypoint());

  Eigen::Isometry3d p1_world = trans_info.prev.extractCartesianWorldPose();
  Eigen::Isometry3d p1 = trans_info.prev.calcCartesianLocalPose(p1_world);
  Eigen::VectorXd j1 = getClosestJointSolution(p1, trans_info.prev.inv_kin, j2);

  Eigen::MatrixXd states;
  if (j1.size() == 0)
  {
    if (trans_info.base.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = j2.replicate(1, trans_info.linear_steps + 1);
    else if (trans_info.base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = j2.replicate(1, trans_info.freespace_steps + 1);
    else
      throw std::runtime_error("stateJointCartWaypointFixedSize: Unsupported PlanInstructionType!");
  }
  else
  {
    if (trans_info.base.instruction.getPlanType() == PlanInstructionType::LINEAR)
    {
      if (trans_info.linear_steps > 1)
        states = interpolate(j1, j2, trans_info.linear_steps);
      else
        states = j2.replicate(1, 2);
    }
    else if (trans_info.base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
    {
      if (trans_info.freespace_steps > 1)
        states = interpolate(j1, j2, trans_info.freespace_steps);
      else
        states = j2.replicate(1, 2);
    }
    else
    {
      throw std::runtime_error("stateJointCartWaypointFixedSize: Unsupported PlanInstructionType!");
    }
  }

  return getInterpolatedComposite(states, trans_info.base.fwd_kin, trans_info.base.instruction);
}

CompositeInstruction stateCartCartWaypointFixedSize(const FixedSizeTransitionInfo& trans_info)
{
  // Get IK seed
  Eigen::VectorXd seed = trans_info.request.env_state->getJointValues(trans_info.base.inv_kin->getJointNames());

  // Calculate IK for start and end
  Eigen::Isometry3d p1_world = trans_info.prev.extractCartesianWorldPose();
  Eigen::Isometry3d p1 = trans_info.prev.calcCartesianLocalPose(p1_world);

  Eigen::Isometry3d p2_world = trans_info.base.extractCartesianWorldPose();
  Eigen::Isometry3d p2 = trans_info.base.calcCartesianLocalPose(p2_world);

  std::array<Eigen::VectorXd, 2> sol =
      getClosestJointSolution(p1, p2, trans_info.prev.inv_kin, trans_info.base.inv_kin, seed);

  Eigen::MatrixXd states;
  if (sol[0].size() != 0 && sol[1].size() != 0)
  {
    if (trans_info.base.instruction.getPlanType() == PlanInstructionType::LINEAR)
    {
      if (trans_info.linear_steps > 1)
        states = interpolate(sol[0], sol[1], trans_info.linear_steps);
      else
        states = sol[1].replicate(1, 2);
    }
    else if (trans_info.base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
    {
      if (trans_info.freespace_steps > 1)
        states = interpolate(sol[0], sol[1], trans_info.freespace_steps);
      else
        states = sol[1].replicate(1, 2);
    }
    else
    {
      throw std::runtime_error("stateCartCartWaypointFixedSize: Unsupported PlanInstructionType!");
    }
  }
  else if (sol[0].size() != 0)
  {
    if (trans_info.base.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = sol[0].replicate(1, trans_info.linear_steps + 1);
    else if (trans_info.base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = sol[0].replicate(1, trans_info.freespace_steps + 1);
    else
      throw std::runtime_error("stateCartCartWaypointFixedSize: Unsupported PlanInstructionType!");
  }
  else if (sol[1].size() != 0)
  {
    if (trans_info.base.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = sol[1].replicate(1, trans_info.linear_steps + 1);
    else if (trans_info.base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = sol[1].replicate(1, trans_info.freespace_steps + 1);
    else
      throw std::runtime_error("stateCartCartWaypointFixedSize: Unsupported PlanInstructionType!");
  }
  else
  {
    if (trans_info.base.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = seed.replicate(1, trans_info.linear_steps + 1);
    else if (trans_info.base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = seed.replicate(1, trans_info.freespace_steps + 1);
    else
      throw std::runtime_error("stateCartCartWaypointFixedSize: Unsupported PlanInstructionType!");
  }

  // Convert to MoveInstructions
  return getInterpolatedComposite(states, trans_info.base.fwd_kin, trans_info.base.instruction);
}

}  // namespace tesseract_planning
