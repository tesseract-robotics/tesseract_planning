/**
 * @file simple_planner_default_lvs_plan_profile.cpp
 * @brief
 *
 * @author Tyler Marr
 * @date Septemeber 16, 2020
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

#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_common/manipulator_info.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>

namespace tesseract_planning
{
SimplePlannerLVSPlanProfile::SimplePlannerLVSPlanProfile(double state_longest_valid_segment_length,
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
SimplePlannerLVSPlanProfile::generate(const MoveInstructionPoly& prev_instruction,
                                      const MoveInstructionPoly& /*prev_seed*/,
                                      const MoveInstructionPoly& base_instruction,
                                      const InstructionPoly& /*next_instruction*/,
                                      const std::shared_ptr<const tesseract_environment::Environment>& env,
                                      const tesseract_common::ManipulatorInfo& global_manip_info) const
{
  KinematicGroupInstructionInfo info1(prev_instruction, *env, global_manip_info);
  KinematicGroupInstructionInfo info2(base_instruction, *env, global_manip_info);

  if (!info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return interpolateJointJointWaypoint(info1,
                                         info2,
                                         state_longest_valid_segment_length,
                                         translation_longest_valid_segment_length,
                                         rotation_longest_valid_segment_length,
                                         min_steps,
                                         max_steps);

  if (!info1.has_cartesian_waypoint && info2.has_cartesian_waypoint)
    return interpolateJointCartWaypoint(info1,
                                        info2,
                                        state_longest_valid_segment_length,
                                        translation_longest_valid_segment_length,
                                        rotation_longest_valid_segment_length,
                                        min_steps,
                                        max_steps);

  if (info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return interpolateCartJointWaypoint(info1,
                                        info2,
                                        state_longest_valid_segment_length,
                                        translation_longest_valid_segment_length,
                                        rotation_longest_valid_segment_length,
                                        min_steps,
                                        max_steps);

  return interpolateCartCartWaypoint(info1,
                                     info2,
                                     state_longest_valid_segment_length,
                                     translation_longest_valid_segment_length,
                                     rotation_longest_valid_segment_length,
                                     min_steps,
                                     max_steps,
                                     env->getState());
}

template <class Archive>
void SimplePlannerLVSPlanProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(SimplePlannerPlanProfile);
  ar& BOOST_SERIALIZATION_NVP(state_longest_valid_segment_length);
  ar& BOOST_SERIALIZATION_NVP(translation_longest_valid_segment_length);
  ar& BOOST_SERIALIZATION_NVP(rotation_longest_valid_segment_length);
  ar& BOOST_SERIALIZATION_NVP(min_steps);
  ar& BOOST_SERIALIZATION_NVP(max_steps);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::SimplePlannerLVSPlanProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::SimplePlannerLVSPlanProfile)
