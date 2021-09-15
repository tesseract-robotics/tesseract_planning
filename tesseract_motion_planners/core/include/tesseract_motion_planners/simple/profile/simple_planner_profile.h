/**
 * @file simple_planner_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/command_language.h>
#include <tesseract_motion_planners/core/types.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::SimplePlannerPlanProfile)
%shared_ptr(tesseract_planning::SimplePlannerPlanCompositeProfile)
#endif  // SWIG

namespace tesseract_planning
{
/**
 * @brief Plan Profile for the simple planner. It defines some functions that handle each of the waypoint cases. The
 * planner then simply loops over all of the plan instructions and calls the correct function
 */
class SimplePlannerPlanProfile
{
public:
  using Ptr = std::shared_ptr<SimplePlannerPlanProfile>;
  using ConstPtr = std::shared_ptr<const SimplePlannerPlanProfile>;

  SimplePlannerPlanProfile() = default;
  virtual ~SimplePlannerPlanProfile() = default;
  SimplePlannerPlanProfile(const SimplePlannerPlanProfile&) = default;
  SimplePlannerPlanProfile& operator=(const SimplePlannerPlanProfile&) = default;
  SimplePlannerPlanProfile(SimplePlannerPlanProfile&&) noexcept = default;
  SimplePlannerPlanProfile& operator=(SimplePlannerPlanProfile&&) noexcept = default;

  /**
   * @brief Generate a seed for the provided base_instruction
   * @param prev_instruction The previous instruction
   * @param base_instruction The base/current instruction to generate the seed for
   * @param request The planning request
   * @param global_manip_info The global manipulator information
   * @return A composite instruction representing the seed for the base_instruction
   */
  virtual CompositeInstruction generate(const PlanInstruction& prev_instruction,
                                        const PlanInstruction& base_instruction,
                                        const PlannerRequest& request,
                                        const ManipulatorInfo& global_manip_info) const = 0;
};

class SimplePlannerCompositeProfile
{
public:
  using Ptr = std::shared_ptr<SimplePlannerCompositeProfile>;
  using ConstPtr = std::shared_ptr<const SimplePlannerCompositeProfile>;

  SimplePlannerCompositeProfile() = default;
  virtual ~SimplePlannerCompositeProfile() = default;
  SimplePlannerCompositeProfile(const SimplePlannerCompositeProfile&) = default;
  SimplePlannerCompositeProfile& operator=(const SimplePlannerCompositeProfile&) = default;
  SimplePlannerCompositeProfile(SimplePlannerCompositeProfile&&) noexcept = default;
  SimplePlannerCompositeProfile& operator=(SimplePlannerCompositeProfile&&) noexcept = default;

  // This contains functions for composite processing. Get start for example
};

using SimplePlannerPlanProfileMap = std::unordered_map<std::string, SimplePlannerPlanProfile::ConstPtr>;
using SimplePlannerCompositeProfileMap = std::unordered_map<std::string, SimplePlannerCompositeProfile::ConstPtr>;
}  // namespace tesseract_planning

#ifdef SWIG
%template(SimplePlannerPlanProfileMap) std::unordered_map<std::string, tesseract_planning::SimplePlannerPlanProfile::ConstPtr>;
%template(SimplePlannerCompositeProfileMap) std::unordered_map<std::string, tesseract_planning::SimplePlannerCompositeProfile::ConstPtr>;
%tesseract_command_language_add_profile_type(SimplePlannerPlanProfile);
%tesseract_command_language_add_profile_type(SimplePlannerCompositeProfile);
#endif  // SWIG

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_PROFILE_H
