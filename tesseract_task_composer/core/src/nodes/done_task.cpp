/**
 * @file done_task.h
 *
 * @author Levi Armstrong
 * @date August 5, 2022
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
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/nodes/done_task.h>

namespace tesseract_planning
{
DoneTask::DoneTask() : TaskComposerTask("DoneTask", false) {}
DoneTask::DoneTask(std::string name, bool is_conditional) : TaskComposerTask(std::move(name), is_conditional) {}
DoneTask::DoneTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
}

TaskComposerNodeInfo::UPtr DoneTask::runImpl(TaskComposerContext& /*context*/,
                                             OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->color = "green";
  info->return_value = 1;
  info->message = "Successful";
  CONSOLE_BRIDGE_logDebug("%s", info->message.c_str());
  return info;
}

bool DoneTask::operator==(const DoneTask& rhs) const { return TaskComposerTask::operator==(rhs); }
bool DoneTask::operator!=(const DoneTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void DoneTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::DoneTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::DoneTask)
