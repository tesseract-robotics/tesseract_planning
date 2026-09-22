/**
 * @file task_composer_task.cpp
 * @brief A task
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/task_composer/task_composer_task.h>
#include <tesseract/task_composer/task_composer_context.h>
#include <tesseract/common/property_tree.h>

namespace tesseract::task_composer
{
TaskComposerTask::TaskComposerTask(std::string name, TaskComposerNodePorts ports, bool conditional)
  : TaskComposerNode(std::move(name), TaskComposerNodeType::TASK, std::move(ports), conditional)
{
}

TaskComposerTask::TaskComposerTask(std::string name, TaskComposerNodePorts ports, const YAML::Node& config)
  : TaskComposerNode(std::move(name), TaskComposerNodeType::TASK, std::move(ports), config)
{
  if (YAML::Node n = config["trigger_abort"])
    trigger_abort_ = n.as<bool>();
}

void TaskComposerTask::setTriggerAbort(bool enable) { trigger_abort_ = enable; }

tesseract::common::PropertyTree TaskComposerTask::schema(const TaskComposerNodePorts& ports)
{
  using namespace tesseract::common;
  // clang-format off
  auto schema = PropertyTreeBuilder()
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .compose(TaskComposerNode::commonSchema())
      .boolean("trigger_abort").defaultVal(false).done()
      .build();
  // clang-format on
  if (!ports.inputs().empty())
    schema["inputs"] = ports.inputSchema();
  if (!ports.outputs().empty())
    schema["outputs"] = ports.outputSchema();
  return schema;
}

}  // namespace tesseract::task_composer
