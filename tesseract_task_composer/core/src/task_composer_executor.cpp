/**
 * @file task_composer_executor.cpp
 * @brief The executor for executing task graphs
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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

#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>

namespace tesseract::task_composer
{
TaskComposerExecutor::TaskComposerExecutor(std::string name) : name_(std::move(name)) {}

const std::string& TaskComposerExecutor::getName() const { return name_; }

std::unique_ptr<TaskComposerFuture> TaskComposerExecutor::run(const TaskComposerNode& node,
                                                              std::shared_ptr<TaskComposerContext> context)
{
  if (context->task_infos->getRootNode().is_nil())
    context->task_infos->setRootNode(node.getUUID());

  return runImpl(node, std::move(context));
}

}  // namespace tesseract::task_composer
