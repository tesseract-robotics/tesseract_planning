/**
 * @file simple_motion_planner.h
 * @brief The simple planner is meant to be a tool for assigning values to the seed. The planner simply loops over all
 * of the MoveInstructions and then calls the appropriate function from the profile. These functions do not depend on
 * the seed, so this may be used to initialize the seed appropriately using e.g. linear interpolation.
 *
 * @author Matthew Powelson
 * @date July 23, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>

namespace tesseract_planning
{
class SimpleMotionPlannerStatusCategory;

/**
 * @brief The simple planner is meant to be a tool for assigning values to the seed. The planner simply loops over all
 * of the MoveInstructions and then calls the appropriate function from the profile. These functions do not depend on
 * the seed, so this may be used to initialize the seed appropriately using e.g. linear interpolation.
 */
class SimpleMotionPlanner : public MotionPlanner
{
public:
  using Ptr = std::shared_ptr<SimpleMotionPlanner>;
  using ConstPtr = std::shared_ptr<const SimpleMotionPlanner>;

  /** @brief Construct a basic planner */
  SimpleMotionPlanner(std::string name);
  ~SimpleMotionPlanner() override = default;
  SimpleMotionPlanner(const SimpleMotionPlanner&) = delete;
  SimpleMotionPlanner& operator=(const SimpleMotionPlanner&) = delete;
  SimpleMotionPlanner(SimpleMotionPlanner&&) = delete;
  SimpleMotionPlanner& operator=(SimpleMotionPlanner&&) = delete;

  PlannerResponse solve(const PlannerRequest& request) const override;

  bool terminate() override;

  void clear() override;

  MotionPlanner::Ptr clone() const override;

protected:
  CompositeInstruction processCompositeInstruction(const CompositeInstruction& instructions,
                                                   MoveInstructionPoly& prev_instruction,
                                                   MoveInstructionPoly& prev_seed,
                                                   const PlannerRequest& request) const;
};

}  // namespace tesseract_planning
#endif  // TESSERACT_PLANNING_SIMPLE_PLANNER_H
