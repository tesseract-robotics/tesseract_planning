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
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract_planning
{
TaskComposerTask::TaskComposerTask(std::string name) : TaskComposerTask(name, true) {}
TaskComposerTask::TaskComposerTask(std::string name, bool conditional)
  : TaskComposerNode(std::move(name), TaskComposerNodeType::TASK, conditional)
{
}

TaskComposerTask::TaskComposerTask(std::string name, const YAML::Node& config)
  : TaskComposerNode(std::move(name), TaskComposerNodeType::TASK, config)
{
}

int TaskComposerTask::run(TaskComposerInput& input, OptionalTaskComposerExecutor executor) const
{
  TaskComposerNodeInfo::UPtr results;
  try
  {
    results = runImpl(input, executor);
  }
  catch (const std::exception& e)
  {
    results = std::make_unique<TaskComposerNodeInfo>(*this, input);
    results->color = "red";
    results->message = "Exception thrown: " + std::string(e.what());
    results->return_value = 0;
  }

  int value = results->return_value;
  assert(value >= 0);
  input.task_infos.addInfo(std::move(results));
  return value;
}

bool TaskComposerTask::operator==(const TaskComposerTask& rhs) const { return (TaskComposerNode::operator==(rhs)); }
bool TaskComposerTask::operator!=(const TaskComposerTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNode);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerTask)
