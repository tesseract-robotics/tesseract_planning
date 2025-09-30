/**
 * @file task_composer_executor.cpp
 * @brief The executor for executing task graphs
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
#include <tesseract_common/serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>

namespace tesseract_planning
{
TaskComposerExecutor::TaskComposerExecutor(std::string name) : name_(std::move(name)) {}

const std::string& TaskComposerExecutor::getName() const { return name_; }

std::unique_ptr<TaskComposerFuture> TaskComposerExecutor::run(const TaskComposerNode& node,
                                                              std::shared_ptr<TaskComposerDataStorage> data_storage,
                                                              bool dotgraph)
{
  return run(node, std::move(data_storage), std::make_shared<TaskComposerNodeInfoContainer>(), dotgraph);
}

std::unique_ptr<TaskComposerFuture> TaskComposerExecutor::run(const TaskComposerNode& node,
                                                              std::shared_ptr<TaskComposerDataStorage> data_storage,
                                                              std::shared_ptr<TaskComposerNodeInfoContainer> task_infos,
                                                              bool dotgraph)
{
  auto context =
      std::make_shared<TaskComposerContext>(node.getName(), std::move(data_storage), std::move(task_infos), dotgraph);

  if (context->task_infos->getRootNode().is_nil())
    context->task_infos->setRootNode(node.getUUID());

  return run(node, context);
}

bool TaskComposerExecutor::operator==(const TaskComposerExecutor& rhs) const { return (name_ == rhs.name_); }

// LCOV_EXCL_START
bool TaskComposerExecutor::operator!=(const TaskComposerExecutor& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void TaskComposerExecutor::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("name", name_);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerExecutor)
