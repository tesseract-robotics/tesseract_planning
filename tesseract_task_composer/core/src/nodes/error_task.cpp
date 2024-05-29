/**
 * @file error_task.h
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
#include <yaml-cpp/yaml.h>
#include <tesseract_common/serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/nodes/error_task.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>

namespace tesseract_planning
{
ErrorTask::ErrorTask() : TaskComposerTask("ErrorTask", false) {}
ErrorTask::ErrorTask(std::string name, bool is_conditional) : TaskComposerTask(std::move(name), is_conditional) {}
ErrorTask::ErrorTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
}

std::unique_ptr<TaskComposerNodeInfo> ErrorTask::runImpl(TaskComposerContext& /*context*/,
                                                         OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->color = "red";
  info->return_value = 0;
  info->status_code = 0;
  info->status_message = "Error";
  CONSOLE_BRIDGE_logDebug("%s", info->status_message.c_str());
  return info;
}

bool ErrorTask::operator==(const ErrorTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool ErrorTask::operator!=(const ErrorTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void ErrorTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ErrorTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ErrorTask)
