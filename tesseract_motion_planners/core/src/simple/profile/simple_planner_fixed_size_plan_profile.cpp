/**
 * @file simple_planner_interpolation_plan_profile.cpp
 * @brief
 *
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

#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
SimplePlannerFixedSizePlanProfile::SimplePlannerFixedSizePlanProfile(int freespace_steps, int linear_steps)
  : freespace_steps(freespace_steps), linear_steps(linear_steps)
{
}

CompositeInstruction SimplePlannerFixedSizePlanProfile::generate(const PlanInstruction& prev_instruction,
                                                                 const PlanInstruction& base_instruction,
                                                                 const PlannerRequest& request,
                                                                 const ManipulatorInfo& global_manip_info) const
{
  InstructionInfo info1(prev_instruction, request, global_manip_info);
  InstructionInfo info2(base_instruction, request, global_manip_info);

  if (!info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return stateJointJointWaypoint(info1, info2);

  if (!info1.has_cartesian_waypoint && info2.has_cartesian_waypoint)
    return stateJointCartWaypoint(info1, info2);

  if (info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return stateCartJointWaypoint(info1, info2);

  return stateCartCartWaypoint(info1, info2, request);
}

CompositeInstruction SimplePlannerFixedSizePlanProfile::stateJointJointWaypoint(const InstructionInfo& prev,
                                                                                const InstructionInfo& base) const
{
  // Calculate FK for start and end
  const Eigen::VectorXd& j1 = prev.extractJointPosition();
  const Eigen::VectorXd& j2 = base.extractJointPosition();

  Eigen::MatrixXd states;
  if (base.instruction.getPlanType() == PlanInstructionType::LINEAR)
  {
    if (linear_steps > 1)
      states = interpolate(j1, j2, linear_steps);
    else
      states = j2.replicate(1, 2);
  }
  else if (base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
  {
    if (freespace_steps > 1)
      states = interpolate(j1, j2, freespace_steps);
    else
      states = j2.replicate(1, 2);
  }
  else
  {
    throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported PlanInstructionType!");
  }

  return getInterpolatedComposite(base.fwd_kin->getJointNames(), states, base.instruction);
}

CompositeInstruction SimplePlannerFixedSizePlanProfile::stateJointCartWaypoint(const InstructionInfo& prev,
                                                                               const InstructionInfo& base) const
{
  const Eigen::VectorXd& j1 = prev.extractJointPosition();

  Eigen::Isometry3d p2_world = base.extractCartesianWorldPose();
  Eigen::Isometry3d p2 = base.calcCartesianLocalPose(p2_world);

  Eigen::VectorXd j2 = getClosestJointSolution(p2, base.inv_kin, j1);

  Eigen::MatrixXd states;
  if (j2.size() == 0)
  {
    if (base.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = j1.replicate(1, linear_steps + 1);
    else if (base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = j1.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointCartWaypointFixedSize: Unsupported PlanInstructionType!");
  }
  else
  {
    if (base.instruction.getPlanType() == PlanInstructionType::LINEAR)
    {
      if (linear_steps > 1)
        states = interpolate(j1, j2, linear_steps);
      else
        states = j2.replicate(1, 2);
    }
    else if (base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
    {
      if (freespace_steps > 1)
        states = interpolate(j1, j2, freespace_steps);
      else
        states = j2.replicate(1, 2);
    }
    else
    {
      throw std::runtime_error("stateJointCartWaypointFixedSize: Unsupported PlanInstructionType!");
    }
  }

  return getInterpolatedComposite(base.fwd_kin->getJointNames(), states, base.instruction);
}

CompositeInstruction SimplePlannerFixedSizePlanProfile::stateCartJointWaypoint(const InstructionInfo& prev,
                                                                               const InstructionInfo& base) const
{
  const Eigen::VectorXd& j2 = base.extractJointPosition();

  Eigen::Isometry3d p1_world = prev.extractCartesianWorldPose();
  Eigen::Isometry3d p1 = prev.calcCartesianLocalPose(p1_world);
  Eigen::VectorXd j1 = getClosestJointSolution(p1, prev.inv_kin, j2);

  Eigen::MatrixXd states;
  if (j1.size() == 0)
  {
    if (base.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = j2.replicate(1, linear_steps + 1);
    else if (base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = j2.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointCartWaypointFixedSize: Unsupported PlanInstructionType!");
  }
  else
  {
    if (base.instruction.getPlanType() == PlanInstructionType::LINEAR)
    {
      if (linear_steps > 1)
        states = interpolate(j1, j2, linear_steps);
      else
        states = j2.replicate(1, 2);
    }
    else if (base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
    {
      if (freespace_steps > 1)
        states = interpolate(j1, j2, freespace_steps);
      else
        states = j2.replicate(1, 2);
    }
    else
    {
      throw std::runtime_error("stateJointCartWaypointFixedSize: Unsupported PlanInstructionType!");
    }
  }

  return getInterpolatedComposite(base.fwd_kin->getJointNames(), states, base.instruction);
}

CompositeInstruction SimplePlannerFixedSizePlanProfile::stateCartCartWaypoint(const InstructionInfo& prev,
                                                                              const InstructionInfo& base,
                                                                              const PlannerRequest& request) const
{
  // Get IK seed
  Eigen::VectorXd seed = request.env_state->getJointValues(base.inv_kin->getJointNames());
  tesseract_common::enforcePositionLimits(seed, base.fwd_kin->getLimits().joint_limits);

  // Calculate IK for start and end
  Eigen::Isometry3d p1_world = prev.extractCartesianWorldPose();
  Eigen::Isometry3d p1 = prev.calcCartesianLocalPose(p1_world);

  Eigen::Isometry3d p2_world = base.extractCartesianWorldPose();
  Eigen::Isometry3d p2 = base.calcCartesianLocalPose(p2_world);

  std::array<Eigen::VectorXd, 2> sol = getClosestJointSolution(p1, p2, prev.inv_kin, base.inv_kin, seed);

  Eigen::MatrixXd states;
  if (sol[0].size() != 0 && sol[1].size() != 0)
  {
    if (base.instruction.getPlanType() == PlanInstructionType::LINEAR)
    {
      if (linear_steps > 1)
        states = interpolate(sol[0], sol[1], linear_steps);
      else
        states = sol[1].replicate(1, 2);
    }
    else if (base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
    {
      if (freespace_steps > 1)
        states = interpolate(sol[0], sol[1], freespace_steps);
      else
        states = sol[1].replicate(1, 2);
    }
    else
    {
      throw std::runtime_error("SimplePlannerFixedSizePlanProfile: Unsupported PlanInstructionType!");
    }
  }
  else if (sol[0].size() != 0)
  {
    if (base.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = sol[0].replicate(1, linear_steps + 1);
    else if (base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = sol[0].replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("SimplePlannerFixedSizePlanProfile: Unsupported PlanInstructionType!");
  }
  else if (sol[1].size() != 0)
  {
    if (base.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = sol[1].replicate(1, linear_steps + 1);
    else if (base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = sol[1].replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("SimplePlannerFixedSizePlanProfile: Unsupported PlanInstructionType!");
  }
  else
  {
    if (base.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = seed.replicate(1, linear_steps + 1);
    else if (base.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = seed.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("SimplePlannerFixedSizePlanProfile: Unsupported PlanInstructionType!");
  }

  // Convert to MoveInstructions
  return getInterpolatedComposite(base.fwd_kin->getJointNames(), states, base.instruction);
}

}  // namespace tesseract_planning
