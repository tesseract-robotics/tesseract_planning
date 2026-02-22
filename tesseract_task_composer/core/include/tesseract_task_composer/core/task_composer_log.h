/**
 * @copyright Copyright (c) 2024, Levi Armstrong
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_LOG_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_LOG_H

#include <memory>
#include <tesseract_common/fwd.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

namespace tesseract::task_composer
{
class TaskComposerContext;

class TaskComposerLog
{
public:
  TaskComposerLog(std::string desc = "");
  virtual ~TaskComposerLog() = default;

  std::string description;
  TaskComposerDataStorage initial_data;
  std::shared_ptr<TaskComposerContext> context;
  std::string dotgraph;

  bool operator==(const TaskComposerLog& rhs) const;
  bool operator!=(const TaskComposerLog& rhs) const;
};
}  // namespace tesseract::task_composer

TESSERACT_CLASS_EXTENSION(tesseract::task_composer::TaskComposerLog, ".tclx", ".tclj", ".tclb")

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_LOG_H
