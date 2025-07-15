/**
 * @file simple_planner_fixed_size_move_profile.cpp
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

#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_move_profile.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_common/manipulator_info.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>

#include <boost/serialization/nvp.hpp>

namespace tesseract_planning
{
SimplePlannerFixedSizeMoveProfile::SimplePlannerFixedSizeMoveProfile(int freespace_steps, int linear_steps)
  : freespace_steps(freespace_steps), linear_steps(linear_steps)
{
}

std::vector<MoveInstructionPoly>
SimplePlannerFixedSizeMoveProfile::generate(const MoveInstructionPoly& prev_instruction,
                                            const MoveInstructionPoly& /*prev_seed*/,
                                            const MoveInstructionPoly& base_instruction,
                                            const InstructionPoly& /*next_instruction*/,
                                            const std::shared_ptr<const tesseract_environment::Environment>& env,
                                            const tesseract_common::ManipulatorInfo& global_manip_info) const
{
  KinematicGroupInstructionInfo prev(prev_instruction, *env, global_manip_info);
  KinematicGroupInstructionInfo base(base_instruction, *env, global_manip_info);

  if (!prev.has_cartesian_waypoint && !base.has_cartesian_waypoint)
    return interpolateJointJointWaypoint(prev, base, linear_steps, freespace_steps);

  if (!prev.has_cartesian_waypoint && base.has_cartesian_waypoint)
    return interpolateJointCartWaypoint(prev, base, linear_steps, freespace_steps);

  if (prev.has_cartesian_waypoint && !base.has_cartesian_waypoint)
    return interpolateCartJointWaypoint(prev, base, linear_steps, freespace_steps);

  return interpolateCartCartWaypoint(prev, base, linear_steps, freespace_steps, env->getState());
}

template <class Archive>
void SimplePlannerFixedSizeMoveProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(SimplePlannerMoveProfile);
  ar& BOOST_SERIALIZATION_NVP(freespace_steps);
  ar& BOOST_SERIALIZATION_NVP(linear_steps);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::SimplePlannerFixedSizeMoveProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::SimplePlannerFixedSizeMoveProfile)
