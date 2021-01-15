/**
 * @file simple_planner_default_plan_profile.h
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

#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_FIXED_SIZE_ASSIGN_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_FIXED_SIZE_ASSIGN_PLAN_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt_sco/modeling_utils.hpp>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_utils.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::SimplePlannerFixedSizeAssignPlanProfile)
#endif  // SWIG

namespace tesseract_planning
{
class SimplePlannerFixedSizeAssignPlanProfile : public SimplePlannerPlanProfile
{
public:
  using Ptr = std::shared_ptr<SimplePlannerFixedSizeAssignPlanProfile>;
  using ConstPtr = std::shared_ptr<const SimplePlannerFixedSizeAssignPlanProfile>;

  /**
   * @brief SimplePlannerFixedSizeAssignPlanProfile
   * @param freespace_steps The number of steps to use for freespace instruction
   * @param linear_steps The number of steps to use for linear instruction
   */
  SimplePlannerFixedSizeAssignPlanProfile(int freespace_steps = 10, int linear_steps = 10);

  CompositeInstruction generate(const PlanInstruction& prev_instruction,
                                const PlanInstruction& base_instruction,
                                const PlannerRequest& request,
                                const ManipulatorInfo& global_manip_info) const override;

  /** @brief The number of steps to use for freespace instruction */
  int freespace_steps;

  /** @brief The number of steps to use for linear instruction */
  int linear_steps;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_FIXED_SIZE_ASSIGN_PLAN_PROFILE_H
