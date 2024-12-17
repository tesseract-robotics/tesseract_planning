/**
 * @file simple_planner_fixed_size_assign_no_ik_plan_profile.cpp
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

#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_no_ik_plan_profile.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/kinematic_limits.h>
#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/kinematic_group.h>

#include <tesseract_command_language/poly/move_instruction_poly.h>

namespace tesseract_planning
{
SimplePlannerFixedSizeAssignNoIKPlanProfile::SimplePlannerFixedSizeAssignNoIKPlanProfile(int freespace_steps,
                                                                                         int linear_steps)
  : freespace_steps(freespace_steps), linear_steps(linear_steps)
{
}

std::vector<MoveInstructionPoly> SimplePlannerFixedSizeAssignNoIKPlanProfile::generate(
    const MoveInstructionPoly& prev_instruction,
    const MoveInstructionPoly& /*prev_seed*/,
    const MoveInstructionPoly& base_instruction,
    const InstructionPoly& /*next_instruction*/,
    const std::shared_ptr<const tesseract_environment::Environment>& env,
    const tesseract_common::ManipulatorInfo& global_manip_info) const
{
  JointGroupInstructionInfo prev(prev_instruction, *env, global_manip_info);
  JointGroupInstructionInfo base(base_instruction, *env, global_manip_info);

  Eigen::MatrixXd states;
  if (!prev.has_cartesian_waypoint && !base.has_cartesian_waypoint)
  {
    const Eigen::VectorXd& jp = base.extractJointPosition();
    if (base.instruction.isLinear())
      states = jp.replicate(1, linear_steps + 1);
    else if (base.instruction.isFreespace())
      states = jp.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");
  }
  else if (!prev.has_cartesian_waypoint && base.has_cartesian_waypoint)
  {
    const Eigen::VectorXd& jp = prev.extractJointPosition();
    if (base.instruction.isLinear())
      states = jp.replicate(1, linear_steps + 1);
    else if (base.instruction.isFreespace())
      states = jp.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");
  }
  else if (prev.has_cartesian_waypoint && !base.has_cartesian_waypoint)
  {
    const Eigen::VectorXd& jp = base.extractJointPosition();
    if (base.instruction.isLinear())
      states = jp.replicate(1, linear_steps + 1);
    else if (base.instruction.isFreespace())
      states = jp.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");
  }
  else
  {
    Eigen::VectorXd seed = env->getCurrentJointValues(base.manip->getJointNames());
    tesseract_common::enforceLimits<double>(seed, base.manip->getLimits().joint_limits);

    if (base.instruction.isLinear())
      states = seed.replicate(1, linear_steps + 1);
    else if (base.instruction.isFreespace())
      states = seed.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");
  }

  // Linearly interpolate in cartesian space if linear move
  if (base_instruction.isLinear())
  {
    Eigen::Isometry3d p1_world;
    if (prev.has_cartesian_waypoint)
      p1_world = prev.extractCartesianPose();
    else
      p1_world = prev.calcCartesianPose(prev.extractJointPosition());

    Eigen::Isometry3d p2_world;
    if (base.has_cartesian_waypoint)
      p2_world = base.extractCartesianPose();
    else
      p2_world = base.calcCartesianPose(base.extractJointPosition());

    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, linear_steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

}  // namespace tesseract_planning
