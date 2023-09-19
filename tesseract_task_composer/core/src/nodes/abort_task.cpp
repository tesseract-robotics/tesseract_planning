/**
 * @file abort_task.cpp
 *
 * @author Levi Armstrong
 * @date June 22, 2023
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/nodes/abort_task.h>

namespace tesseract_planning
{
AbortTask::AbortTask() : TaskComposerTask("AbortTask", false) {}
AbortTask::AbortTask(std::string name, bool is_conditional) : TaskComposerTask(std::move(name), is_conditional) {}
AbortTask::AbortTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
}

TaskComposerNodeInfo::UPtr AbortTask::runImpl(TaskComposerContext& context,
                                              OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->color = "red";
  info->return_value = 0;
  info->message = "Aborted";
  context.abort(uuid_);
  CONSOLE_BRIDGE_logDebug("%s", info->message.c_str());
  return info;
}

bool AbortTask::operator==(const AbortTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool AbortTask::operator!=(const AbortTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void AbortTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::AbortTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::AbortTask)
