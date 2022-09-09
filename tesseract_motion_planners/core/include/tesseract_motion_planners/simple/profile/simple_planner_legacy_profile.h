/**
 * @file simple_planner_legacy_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_PLANNER_LEGACY_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_PLANNER_LEGACY_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_motion_planners/core/types.h>

namespace tesseract_planning
{
/**
 * @brief Plan Profile for the simple planner. It defines some functions that handle each of the waypoint cases. The
 * planner then simply loops over all of the plan instructions and calls the correct function
 */
class SimplePlannerLegacyPlanProfile
{
public:
  using Ptr = std::shared_ptr<SimplePlannerLegacyPlanProfile>;
  using ConstPtr = std::shared_ptr<const SimplePlannerLegacyPlanProfile>;

  SimplePlannerLegacyPlanProfile() = default;
  virtual ~SimplePlannerLegacyPlanProfile() = default;
  SimplePlannerLegacyPlanProfile(const SimplePlannerLegacyPlanProfile&) = default;
  SimplePlannerLegacyPlanProfile& operator=(const SimplePlannerLegacyPlanProfile&) = default;
  SimplePlannerLegacyPlanProfile(SimplePlannerLegacyPlanProfile&&) noexcept = default;
  SimplePlannerLegacyPlanProfile& operator=(SimplePlannerLegacyPlanProfile&&) noexcept = default;

  /**
   * @brief Generate a seed for the provided base_instruction
   * @param prev_instruction The previous instruction
   * @param prev_seed The previous seed
   * @param base_instruction The base/current instruction to generate the seed for
   * @param next_instruction The next instruction. This will be a null instruction for the final instruction
   * @param request The planning request
   * @param global_manip_info The global manipulator information
   * @return A composite instruction representing the seed for the base_instruction
   */
  virtual CompositeInstruction generate(const MoveInstructionPoly& prev_instruction,
                                        const MoveInstructionPoly& prev_seed,
                                        const MoveInstructionPoly& base_instruction,
                                        const InstructionPoly& next_instruction,
                                        const PlannerRequest& request,
                                        const tesseract_common::ManipulatorInfo& global_manip_info) const = 0;
};

class SimplePlannerLegacyCompositeProfile
{
public:
  using Ptr = std::shared_ptr<SimplePlannerLegacyCompositeProfile>;
  using ConstPtr = std::shared_ptr<const SimplePlannerLegacyCompositeProfile>;

  SimplePlannerLegacyCompositeProfile() = default;
  virtual ~SimplePlannerLegacyCompositeProfile() = default;
  SimplePlannerLegacyCompositeProfile(const SimplePlannerLegacyCompositeProfile&) = default;
  SimplePlannerLegacyCompositeProfile& operator=(const SimplePlannerLegacyCompositeProfile&) = default;
  SimplePlannerLegacyCompositeProfile(SimplePlannerLegacyCompositeProfile&&) noexcept = default;
  SimplePlannerLegacyCompositeProfile& operator=(SimplePlannerLegacyCompositeProfile&&) noexcept = default;

  // This contains functions for composite processing. Get start for example
};

using SimplePlannerLegacyPlanProfileMap = std::unordered_map<std::string, SimplePlannerLegacyPlanProfile::ConstPtr>;
using SimplePlannerLegacyCompositeProfileMap =
    std::unordered_map<std::string, SimplePlannerLegacyCompositeProfile::ConstPtr>;
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_PLANNER_LEGACY_PROFILE_H
