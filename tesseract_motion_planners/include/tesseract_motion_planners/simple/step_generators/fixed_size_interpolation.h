/**
 * @file fixed_size_interpolation.h
 * @brief Provides interpolators where the number of steps are specified
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
#ifndef TESSERACT_MOTION_PLANNERS_FIXED_SIZE_INTERPOLATION_H
#define TESSERACT_MOTION_PLANNERS_FIXED_SIZE_INTERPOLATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/command_language.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/simple/step_generators/utils.h>

namespace tesseract_planning
{
/** @brief A container of the transition information used */
struct FixedSizeTransitionInfo
{
  FixedSizeTransitionInfo(const InstructionInfo& prev, const InstructionInfo& base, const PlannerRequest& request);

  const InstructionInfo& prev;
  const InstructionInfo& base;
  const PlannerRequest& request;
  int freespace_steps;
  int linear_steps;
};

/**
 * @brief This function used to generate a seed
 * @param prev_instruction The previous instruction
 * @param base_instruction The current instruction
 * @param request The planning request information
 * @param manip_info The manipulator information provided by the parent composite instruction
 * @param freespace_steps The number of steps to use for freespace instruction
 * @param linear_steps The number of steps to use for linear instruction
 * @return A composite instruction of move instruction with state waypoints
 */
CompositeInstruction simplePlannerGeneratorFixedSize(const PlanInstruction& prev_instruction,
                                                     const PlanInstruction& base_instruction,
                                                     const PlannerRequest& request,
                                                     const ManipulatorInfo& manip_info,
                                                     int freespace_steps,
                                                     int linear_steps);

CompositeInstruction stateJointJointWaypointFixedSize(const FixedSizeTransitionInfo& trans_info);

CompositeInstruction stateJointCartWaypointFixedSize(const FixedSizeTransitionInfo& trans_info);

CompositeInstruction stateCartJointWaypointFixedSize(const FixedSizeTransitionInfo& trans_info);

CompositeInstruction stateCartCartWaypointFixedSize(const FixedSizeTransitionInfo& trans_info);

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_FIXED_SIZE_INTERPOLATION_H
