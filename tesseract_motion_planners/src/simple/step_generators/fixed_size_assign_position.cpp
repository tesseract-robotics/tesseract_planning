/**
 * @file fixed_size_assign_position.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date July 24, 2020
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

#include <tesseract_motion_planners/simple/step_generators/fixed_size_assign_position.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/state_waypoint.h>

namespace tesseract_planning
{
CompositeInstruction simplePlannerGeneratorFixedSizeAssign(const PlanInstruction& prev_instruction,
                                                           const PlanInstruction& base_instruction,
                                                           const PlannerRequest& request,
                                                           const ManipulatorInfo& manip_info,
                                                           int freespace_steps,
                                                           int linear_steps)
{
  InstructionInfo info1(prev_instruction, request, manip_info);
  InstructionInfo info2(base_instruction, request, manip_info);

  Eigen::MatrixXd states;
  if (!info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
  {
    const Eigen::VectorXd& jp = getJointPosition(info2.instruction.getWaypoint());
    if (info2.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = jp.replicate(1, linear_steps + 1);
    else if (info2.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = jp.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported PlanInstructionType!");
  }
  else if (!info1.has_cartesian_waypoint && info2.has_cartesian_waypoint)
  {
    const Eigen::VectorXd& jp = getJointPosition(info1.instruction.getWaypoint());
    if (info2.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = jp.replicate(1, linear_steps + 1);
    else if (info2.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = jp.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported PlanInstructionType!");
  }
  else if (info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
  {
    const Eigen::VectorXd& jp = getJointPosition(info2.instruction.getWaypoint());
    if (info2.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = jp.replicate(1, linear_steps + 1);
    else if (info2.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = jp.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported PlanInstructionType!");
  }
  else
  {
    Eigen::VectorXd seed = request.env_state->getJointValues(info2.inv_kin->getJointNames());
    if (info2.instruction.getPlanType() == PlanInstructionType::LINEAR)
      states = seed.replicate(1, linear_steps + 1);
    else if (info2.instruction.getPlanType() == PlanInstructionType::FREESPACE)
      states = seed.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported PlanInstructionType!");
  }

  return getInterpolatedComposite(states, info2.fwd_kin, info2.instruction);
}

}  // namespace tesseract_planning
