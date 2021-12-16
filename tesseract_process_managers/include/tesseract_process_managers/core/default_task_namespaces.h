/**
 * @file default_task_namespaces.h
 * @brief The default namespaces for each of the task generators
 *
 * @author Levi Armstrong
 * @date November 18, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_PROCESS_MANAGERS_DEFAULT_TASK_NAMESPACES_H
#define TESSERACT_PROCESS_MANAGERS_DEFAULT_TASK_NAMESPACES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/default_planner_namespaces.h>

namespace tesseract_planning::profile_ns
{
// clang-format off
static const std::string CHECK_INPUT_DEFAULT_NAMESPACE                       = "CHECK_TASK_INPUT";
static const std::string CONTINUOUS_CONTACT_CHECK_DEFAULT_NAMESPACE          = "CONTINUOUS_CONTACT_CHECK";
static const std::string DISCRETE_CONTACT_CHECK_DEFAULT_NAMESPACE            = "DISCRETE_CONTACT_CHECK";
static const std::string FIX_STATE_BOUNDS_DEFAULT_NAMESPACE                  = "FIX_STATE_BOUNDS";
static const std::string FIX_STATE_COLLISION_DEFAULT_NAMESPACE               = "FIX_STATE_COLLISION";
static const std::string HAS_SEED_DEFAULT_NAMESPACE                          = "HAS_SEED_CHECK";
static const std::string ITERATIVE_SPLINE_PARAMETERIZATION_DEFAULT_NAMESPACE = "ITERATIVE_SPLINE_PARAMETERIZATION";
static const std::string PROFILE_SWITCH_DEFAULT_NAMESPACE                    = "PROFILE_SWITCH";
static const std::string SEED_MIN_LENGTH_DEFAULT_NAMESPACE                   = "SEED_MIN_LENGTH_CHECK";
static const std::string TIME_OPTIMAL_PARAMETERIZATION_DEFAULT_NAMESPACE     = "TIME_OPTIMAL_PARAMETERIZATION";
static const std::string UPSAMPLE_TRAJECTORY_DEFAULT_NAMESPACE               = "UPSAMPLE_TRAJECTORY";

// clang-format on
}  // namespace tesseract_planning::profile_ns
#endif  // TESSERACT_PROCESS_MANAGERS_DEFAULT_TASK_NAMESPACES_H
