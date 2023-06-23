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

namespace tesseract_planning::test_suite
{
template <typename T>
void runTaskComposerTaskAbortTest(T& task)
{
  auto input = std::make_unique<TaskComposerInput>(std::make_unique<TaskComposerProblem>());
  input->abort();
  EXPECT_EQ(task.run(*input), 0);
  auto node_info = input->task_infos.getInfo(task.getUUID());
  EXPECT_EQ(node_info->color, "white");
  EXPECT_EQ(node_info->return_value, 0);
  EXPECT_EQ(node_info->message, "Aborted");
  EXPECT_EQ(node_info->isAborted(), true);
  EXPECT_EQ(input->isAborted(), true);
  EXPECT_EQ(input->isSuccessful(), false);
  EXPECT_TRUE(input->task_infos.getAbortingNode().is_nil());
}
}  // namespace tesseract_planning::test_suite

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_TASK_UNIT_HPP
