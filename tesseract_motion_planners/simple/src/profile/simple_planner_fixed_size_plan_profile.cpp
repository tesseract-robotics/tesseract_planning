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
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
SimplePlannerFixedSizePlanProfile::SimplePlannerFixedSizePlanProfile(int freespace_steps, int linear_steps)
  : freespace_steps(freespace_steps), linear_steps(linear_steps)
{
}

std::vector<MoveInstructionPoly>
SimplePlannerFixedSizePlanProfile::generate(const MoveInstructionPoly& prev_instruction,
                                            const MoveInstructionPoly& /*prev_seed*/,
                                            const MoveInstructionPoly& base_instruction,
                                            const InstructionPoly& /*next_instruction*/,
                                            const PlannerRequest& request,
                                            const tesseract_common::ManipulatorInfo& global_manip_info) const
{
  KinematicGroupInstructionInfo info1(prev_instruction, request, global_manip_info);
  KinematicGroupInstructionInfo info2(base_instruction, request, global_manip_info);

  if (!info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return interpolateJointJointWaypoint(info1, info2, linear_steps, freespace_steps);

  if (!info1.has_cartesian_waypoint && info2.has_cartesian_waypoint)
    return interpolateJointCartWaypoint(info1, info2, linear_steps, freespace_steps);

  if (info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return interpolateCartJointWaypoint(info1, info2, linear_steps, freespace_steps);

  return interpolateCartCartWaypoint(info1, info2, linear_steps, freespace_steps, request.env_state);
}

}  // namespace tesseract_planning
