/**
 * @file simple_planner_lvs_assign_plan_profile.cpp
 * @brief
 *
 * @author Roelof Oomen
 * @date March 19, 2024
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2024, ROS Industrial Consortium
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

#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_assign_plan_profile.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
SimplePlannerLVSAssignPlanProfile::SimplePlannerLVSAssignPlanProfile(double state_longest_valid_segment_length,
                                                                     double translation_longest_valid_segment_length,
                                                                     double rotation_longest_valid_segment_length,
                                                                     int min_steps,
                                                                     int max_steps)
  : state_longest_valid_segment_length(state_longest_valid_segment_length)
  , translation_longest_valid_segment_length(translation_longest_valid_segment_length)
  , rotation_longest_valid_segment_length(rotation_longest_valid_segment_length)
  , min_steps(min_steps)
  , max_steps(max_steps)
{
}

std::vector<MoveInstructionPoly>
SimplePlannerLVSAssignPlanProfile::generate(const MoveInstructionPoly& prev_instruction,
                                            const MoveInstructionPoly& /*prev_seed*/,
                                            const MoveInstructionPoly& base_instruction,
                                            const InstructionPoly& /*next_instruction*/,
                                            const PlannerRequest& request,
                                            const tesseract_common::ManipulatorInfo& global_manip_info) const
{
  KinematicGroupInstructionInfo prev(prev_instruction, request, global_manip_info);
  KinematicGroupInstructionInfo base(base_instruction, request, global_manip_info);

  Eigen::VectorXd j1;
  Eigen::Isometry3d p1_world;
  if (!prev.has_cartesian_waypoint)
  {
    j1 = prev.extractJointPosition();
    p1_world = prev.calcCartesianPose(j1);
  }
  else
  {
    p1_world = prev.extractCartesianPose();
    const auto& prev_cwp = prev.instruction.getWaypoint().as<CartesianWaypointPoly>();
    if (prev_cwp.hasSeed())
    {
      // Use joint position of cartesian base_instruction
      j1 = prev_cwp.getSeed().position;
    }
    else
    {
      if (base.has_cartesian_waypoint)
      {
        const auto& base_cwp = base.instruction.getWaypoint().as<CartesianWaypointPoly>();
        if (base_cwp.hasSeed())
        {
          // Use joint position of cartesian base_instruction as seed
          j1 = getClosestJointSolution(prev, base_cwp.getSeed().position);
        }
        else
        {
          // Use current env_state as seed
          j1 = getClosestJointSolution(prev, request.env_state.getJointValues(prev.manip->getJointNames()));
        }
      }
      else
      {
        // Use base_instruction as seed
        j1 = getClosestJointSolution(prev, base.extractJointPosition());
      }
    }
    tesseract_common::enforcePositionLimits<double>(j1, prev.manip->getLimits().joint_limits);
  }

  Eigen::VectorXd j2;
  Eigen::Isometry3d p2_world;
  if (!base.has_cartesian_waypoint)
  {
    j2 = base.extractJointPosition();
    p2_world = base.calcCartesianPose(j2);
  }
  else
  {
    p2_world = base.extractCartesianPose();
    const auto& base_cwp = base.instruction.getWaypoint().as<CartesianWaypointPoly>();
    if (base_cwp.hasSeed())
    {
      // Use joint position of cartesian base_instruction
      j2 = base_cwp.getSeed().position;
    }
    else
    {
      if (prev.has_cartesian_waypoint)
      {
        const auto& prev_cwp = prev.instruction.getWaypoint().as<CartesianWaypointPoly>();
        if (prev_cwp.hasSeed())
        {
          // Use joint position of cartesian prev_instruction as seed
          j2 = getClosestJointSolution(base, prev_cwp.getSeed().position);
        }
        else
        {
          // Use current env_state as seed
          j2 = getClosestJointSolution(base, request.env_state.getJointValues(base.manip->getJointNames()));
        }
      }
      else
      {
        // Use prev_instruction as seed
        j2 = getClosestJointSolution(base, prev.extractJointPosition());
      }
    }
    tesseract_common::enforcePositionLimits<double>(j2, base.manip->getLimits().joint_limits);
  }

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  double joint_dist = (j2 - j1).norm();

  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int joint_steps = int(joint_dist / state_longest_valid_segment_length) + 1;

  int steps = std::max(trans_steps, rot_steps);
  steps = std::max(steps, joint_steps);
  steps = std::max(steps, min_steps);
  steps = std::min(steps, max_steps);

  Eigen::MatrixXd states;
  // Replicate base_instruction joint position
  states = j2.replicate(1, steps + 1);

  // Linearly interpolate in cartesian space if linear move
  if (base_instruction.isLinear())
  {
    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

}  // namespace tesseract_planning
