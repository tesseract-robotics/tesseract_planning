/**
 * @file simple_planner_default_plan_profile.cpp
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

#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
SimplePlannerFixedSizeAssignPlanProfile::SimplePlannerFixedSizeAssignPlanProfile(int freespace_steps, int linear_steps)
  : freespace_steps(freespace_steps), linear_steps(linear_steps)
{
}

CompositeInstruction
SimplePlannerFixedSizeAssignPlanProfile::generate(const MoveInstructionPoly& prev_instruction,
                                                  const MoveInstructionPoly& /*prev_seed*/,
                                                  const MoveInstructionPoly& base_instruction,
                                                  const InstructionPoly& /*next_instruction*/,
                                                  const PlannerRequest& request,
                                                  const tesseract_common::ManipulatorInfo& global_manip_info) const
{
  KinematicGroupInstructionInfo info1(prev_instruction, request, global_manip_info);
  KinematicGroupInstructionInfo info2(base_instruction, request, global_manip_info);

  Eigen::MatrixXd states;
  if (!info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
  {
    const Eigen::VectorXd& jp = info2.extractJointPosition();
    if (info2.instruction.isLinear())
      states = jp.replicate(1, linear_steps + 1);
    else if (info2.instruction.isFreespace())
      states = jp.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");
  }
  else if (!info1.has_cartesian_waypoint && info2.has_cartesian_waypoint)
  {
    const Eigen::VectorXd& jp = info1.extractJointPosition();
    if (info2.instruction.isLinear())
      states = jp.replicate(1, linear_steps + 1);
    else if (info2.instruction.isFreespace())
      states = jp.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");
  }
  else if (info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
  {
    const Eigen::VectorXd& jp = info2.extractJointPosition();
    if (info2.instruction.isLinear())
      states = jp.replicate(1, linear_steps + 1);
    else if (info2.instruction.isFreespace())
      states = jp.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");
  }
  else
  {
    Eigen::VectorXd seed = request.env_state.getJointValues(info2.manip->getJointNames());
    tesseract_common::enforcePositionLimits<double>(seed, info2.manip->getLimits().joint_limits);

    if (info2.instruction.isLinear())
      states = seed.replicate(1, linear_steps + 1);
    else if (info2.instruction.isFreespace())
      states = seed.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");
  }

  return getInterpolatedComposite(info2.manip->getJointNames(), states, info2.instruction);
}

}  // namespace tesseract_planning
