/**
 * @file check_input_profile.h
 * @brief Profile used for checking input data structure
 *
 * @author Levi Armstrong
 * @date November 2. 2021
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
#ifndef TESSERACT_TASK_COMPOSER_CHECK_INPUT_PROFILE_H
#define TESSERACT_TASK_COMPOSER_CHECK_INPUT_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
struct TaskComposerContext;

struct CheckInputProfile
{
  using Ptr = std::shared_ptr<CheckInputProfile>;
  using ConstPtr = std::shared_ptr<const CheckInputProfile>;

  virtual ~CheckInputProfile() = default;

  /**
   * @brief Check if the task input is valid
   * @param context The task context to check
   * @return True if valid otherwise false
   */
  virtual bool isValid(const TaskComposerContext& context) const;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_CHECK_INPUT_PROFILE_H
