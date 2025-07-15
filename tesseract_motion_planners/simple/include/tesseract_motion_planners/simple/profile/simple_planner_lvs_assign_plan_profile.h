/**
 * @file simple_planner_lvs_assign_plan_profile.h
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

#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_PLANNER_LVS_ASSIGN_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_PLANNER_LVS_ASSIGN_PLAN_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

namespace tesseract_planning
{
class SimplePlannerLVSAssignPlanProfile : public SimplePlannerPlanProfile
{
public:
  using Ptr = std::shared_ptr<SimplePlannerLVSAssignPlanProfile>;
  using ConstPtr = std::shared_ptr<const SimplePlannerLVSAssignPlanProfile>;

  /**
   * @brief SimplePlannerFixedSizeAssignPlanProfile
   * @param freespace_steps The number of steps to use for freespace instruction
   * @param linear_steps The number of steps to use for linear instruction
   */
  SimplePlannerLVSAssignPlanProfile(double state_longest_valid_segment_length = 5 * M_PI / 180,
                                    double translation_longest_valid_segment_length = 0.1,
                                    double rotation_longest_valid_segment_length = 5 * M_PI / 180,
                                    int min_steps = 1,
                                    int max_steps = std::numeric_limits<int>::max());

  std::vector<MoveInstructionPoly> generate(const MoveInstructionPoly& prev_instruction,
                                            const MoveInstructionPoly& prev_seed,
                                            const MoveInstructionPoly& base_instruction,
                                            const InstructionPoly& next_instruction,
                                            const PlannerRequest& request,
                                            const tesseract_common::ManipulatorInfo& global_manip_info) const override;

  /** @brief The maximum joint distance, the norm of changes to all joint positions between successive steps. */
  double state_longest_valid_segment_length;

  /** @brief The maximum translation distance between successive steps */
  double translation_longest_valid_segment_length;

  /** @brief The maximum rotational distance between successive steps */
  double rotation_longest_valid_segment_length;

  /** @brief The minimum number of steps for the plan */
  int min_steps;

  /** @brief The maximum number of steps for the plan */
  int max_steps;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_PLANNER_LVS_ASSIGN_PLAN_PROFILE_H
