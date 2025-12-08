/**
 * @file simple_planner_fixed_size_move_profile.cpp
 * @brief
 *
 * @author Matthew Powelson
 * @date July 23, 2020
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

#include <yaml-cpp/yaml.h>
#include <tesseract_common/profile_plugin_factory.h>

namespace tesseract_planning
{
SimplePlannerFixedSizeMoveProfile::SimplePlannerFixedSizeMoveProfile(int freespace_steps, int linear_steps)
  : freespace_steps(freespace_steps), linear_steps(linear_steps)
{
}

SimplePlannerFixedSizeMoveProfile::SimplePlannerFixedSizeMoveProfile(
    const YAML::Node& config,
    const tesseract_common::ProfilePluginFactory& /*plugin_factory*/)
  : SimplePlannerFixedSizeMoveProfile()
{
  try
  {
    if (YAML::Node n = config["freespace_steps"])
      freespace_steps = n.as<int>();
    if (YAML::Node n = config["linear_steps"])
      linear_steps = n.as<int>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("SimplePlannerFixedSizeMoveProfile: Failed to parse yaml config! Details: " +
                             std::string(e.what()));
  }
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

bool SimplePlannerFixedSizeMoveProfile::operator==(const SimplePlannerFixedSizeMoveProfile& rhs) const
{
  bool equal = true;
  equal &= (freespace_steps == rhs.freespace_steps);
  equal &= (linear_steps == rhs.linear_steps);
  return equal;
}

bool SimplePlannerFixedSizeMoveProfile::operator!=(const SimplePlannerFixedSizeMoveProfile& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract_planning
