/**
 * @file task_composer_task.cpp
 * @brief A task
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
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>
#include <boost/serialization/base_object.hpp>
#include <tesseract_common/serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_context.h>

namespace tesseract_planning
{
TaskComposerTask::TaskComposerTask() : TaskComposerTask("TaskComposerTask", TaskComposerNodePorts{}, true) {}
TaskComposerTask::TaskComposerTask(std::string name, TaskComposerNodePorts ports, bool conditional)
  : TaskComposerNode(std::move(name), TaskComposerNodeType::TASK, std::move(ports), conditional)
{
}

TaskComposerTask::TaskComposerTask(std::string name, TaskComposerNodePorts ports, const YAML::Node& config)
  : TaskComposerNode(std::move(name), TaskComposerNodeType::TASK, std::move(ports), config)
{
  try
  {
    if (YAML::Node n = config["trigger_abort"])
      trigger_abort_ = n.as<bool>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("TaskComposerTask: Failed to parse yaml config entry 'trigger_abort'! Details: " +
                             std::string(e.what()));
  }
}

void TaskComposerTask::setTriggerAbort(bool enable) { trigger_abort_ = enable; }

bool TaskComposerTask::operator==(const TaskComposerTask& rhs) const { return (TaskComposerNode::operator==(rhs)); }

// LCOV_EXCL_START
bool TaskComposerTask::operator!=(const TaskComposerTask& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void TaskComposerTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNode);
}

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerTask)
