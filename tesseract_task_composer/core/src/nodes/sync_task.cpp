/**
 * @file sync_task.cpp
 *
 * @author Levi Armstrong
 * @date August 13, 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Plectix Robotics
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

#include <tesseract_task_composer/core/nodes/sync_task.h>

namespace tesseract_planning
{
SyncTask::SyncTask(std::string name) : TaskComposerTask(std::move(name), false) {}
SyncTask::SyncTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
  if (conditional_)
    throw std::runtime_error("SyncTask, config is_conditional should not be true");

  if (!input_keys_.empty())
    throw std::runtime_error("SyncTask, config does not support 'inputs' entry");

  if (!output_keys_.empty())
    throw std::runtime_error("SyncTask, config does not support 'outputs' entry");
}
TaskComposerNodeInfo::UPtr SyncTask::runImpl(TaskComposerContext& /*context*/,
                                             OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->color = "green";
  info->message = "Successful";
  info->return_value = 1;
  return info;
}

bool SyncTask::operator==(const SyncTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool SyncTask::operator!=(const SyncTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void SyncTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::SyncTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::SyncTask)
