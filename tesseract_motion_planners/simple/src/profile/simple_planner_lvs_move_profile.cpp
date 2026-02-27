/**
 * @file simple_planner_lvs_plan_move.cpp
 * @brief
 *
 * @author Tyler Marr
 * @date Septemeber 16, 2020
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

#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_move_profile.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract/common/manipulator_info.h>
#include <tesseract/environment/environment.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>

#include <yaml-cpp/yaml.h>
#include <tesseract/common/profile_plugin_factory.h>
#include <tesseract/common/utils.h>

namespace tesseract::motion_planners
{
SimplePlannerLVSMoveProfile::SimplePlannerLVSMoveProfile(double state_longest_valid_segment_length,
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

SimplePlannerLVSMoveProfile::SimplePlannerLVSMoveProfile(
    const YAML::Node& config,
    const tesseract::common::ProfilePluginFactory& /*plugin_factory*/)
  : SimplePlannerLVSMoveProfile()
{
  try
  {
    if (YAML::Node n = config["state_longest_valid_segment_length"])
      state_longest_valid_segment_length = n.as<double>();
    if (YAML::Node n = config["translation_longest_valid_segment_length"])
      translation_longest_valid_segment_length = n.as<double>();
    if (YAML::Node n = config["rotation_longest_valid_segment_length"])
      rotation_longest_valid_segment_length = n.as<double>();
    if (YAML::Node n = config["min_steps"])
      min_steps = n.as<int>();
    if (YAML::Node n = config["max_steps"])
      max_steps = n.as<int>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("SimplePlannerLVSMoveProfile: Failed to parse yaml config! Details: " +
                             std::string(e.what()));
  }
}

std::vector<tesseract::command_language::MoveInstructionPoly>
SimplePlannerLVSMoveProfile::generate(const tesseract::command_language::MoveInstructionPoly& prev_instruction,
                                      const tesseract::command_language::MoveInstructionPoly& /*prev_seed*/,
                                      const tesseract::command_language::MoveInstructionPoly& base_instruction,
                                      const tesseract::command_language::InstructionPoly& /*next_instruction*/,
                                      const std::shared_ptr<const tesseract::environment::Environment>& env,
                                      const tesseract::common::ManipulatorInfo& global_manip_info) const
{
  KinematicGroupInstructionInfo prev(prev_instruction, *env, global_manip_info);
  KinematicGroupInstructionInfo base(base_instruction, *env, global_manip_info);

  if (!prev.has_cartesian_waypoint && !base.has_cartesian_waypoint)
    return interpolateJointJointWaypoint(prev,
                                         base,
                                         state_longest_valid_segment_length,
                                         translation_longest_valid_segment_length,
                                         rotation_longest_valid_segment_length,
                                         min_steps,
                                         max_steps);

  if (!prev.has_cartesian_waypoint && base.has_cartesian_waypoint)
    return interpolateJointCartWaypoint(prev,
                                        base,
                                        state_longest_valid_segment_length,
                                        translation_longest_valid_segment_length,
                                        rotation_longest_valid_segment_length,
                                        min_steps,
                                        max_steps);

  if (prev.has_cartesian_waypoint && !base.has_cartesian_waypoint)
    return interpolateCartJointWaypoint(prev,
                                        base,
                                        state_longest_valid_segment_length,
                                        translation_longest_valid_segment_length,
                                        rotation_longest_valid_segment_length,
                                        min_steps,
                                        max_steps);

  return interpolateCartCartWaypoint(prev,
                                     base,
                                     state_longest_valid_segment_length,
                                     translation_longest_valid_segment_length,
                                     rotation_longest_valid_segment_length,
                                     min_steps,
                                     max_steps,
                                     env->getState());
}

bool SimplePlannerLVSMoveProfile::operator==(const SimplePlannerLVSMoveProfile& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      state_longest_valid_segment_length, rhs.state_longest_valid_segment_length, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      translation_longest_valid_segment_length, rhs.translation_longest_valid_segment_length, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      rotation_longest_valid_segment_length, rhs.rotation_longest_valid_segment_length, max_diff);
  equal &= (min_steps == rhs.min_steps);
  equal &= (max_steps == rhs.max_steps);
  return equal;
}

bool SimplePlannerLVSMoveProfile::operator!=(const SimplePlannerLVSMoveProfile& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::motion_planners
