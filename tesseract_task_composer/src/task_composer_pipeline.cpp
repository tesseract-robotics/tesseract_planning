/**
 * @file task_composer_pipeline.cpp
 * @brief A task pipeline
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
#include <tesseract_task_composer/task_composer_pipeline.h>

namespace tesseract_planning
{
TaskComposerPipeline::TaskComposerPipeline(std::string name) : name_(std::move(name)) {}

const std::string& TaskComposerPipeline::getName() const { return name_; }

int TaskComposerPipeline::addNode(TaskComposerNode::UPtr task_node)
{
  nodes_.emplace_back(std::move(task_node));
  return static_cast<int>(nodes_.size() - 1);
}

void TaskComposerPipeline::addEdges(int source, std::vector<int> destinations)
{
  TaskComposerNode::Ptr& node = nodes_.at(static_cast<std::size_t>(source));
  node->edges_.insert(node->edges_.end(), destinations.begin(), destinations.end());
}

std::vector<TaskComposerNode::ConstPtr> TaskComposerPipeline::getNodes()
{
  return std::vector<TaskComposerNode::ConstPtr>{ nodes_.begin(), nodes_.end() };
}

}  // namespace tesseract_planning
