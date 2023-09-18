/**
 * @file start_task.h
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

#include <tesseract_task_composer/core/nodes/start_task.h>

namespace tesseract_planning
{
StartTask::StartTask(std::string name) : TaskComposerTask(std::move(name), false) {}
StartTask::StartTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
  if (conditional_)
    throw std::runtime_error("StartTask, config is_conditional should not be true");

  if (!input_keys_.empty())
    throw std::runtime_error("StartTask, config does not support 'inputs' entry");

  if (!output_keys_.empty())
    throw std::runtime_error("StartTask, config does not support 'outputs' entry");
}
TaskComposerNodeInfo::UPtr StartTask::runImpl(TaskComposerContext& /*context*/,
                                              OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->color = "green";
  info->message = "Successful";
  info->return_value = 1;
  return info;
}

bool StartTask::operator==(const StartTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool StartTask::operator!=(const StartTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void StartTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::StartTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::StartTask)
