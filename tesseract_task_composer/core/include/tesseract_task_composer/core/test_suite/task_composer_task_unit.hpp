/**
 * @file task_composer_task_unit.hpp
 * @brief Collection of unit tests for TaskComposerTask
 *
 * @author Levi Armstrong
 * @date June 12, 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Levi Armstrong
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_TASK_UNIT_HPP
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_TASK_UNIT_HPP
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/test_suite/task_composer_node_unit.hpp>
#include <tesseract_task_composer/core/test_suite/task_composer_serialization_utils.hpp>

namespace tesseract_planning::test_suite
{
template <typename T>
void runTaskComposerTaskTest()
{
  T task;
  runTaskComposerNodeTest<T>(type);
}
}  // namespace tesseract_planning::test_suite

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_TASK_UNIT_HPP
