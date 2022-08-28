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
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/nodes/start_task.h>

namespace tesseract_planning
{
StartTask::StartTask(std::string name) : TaskComposerTask(false, std::move(name)) {}

int StartTask::run(TaskComposerInput& input, OptionalTaskComposerExecutor /*executor*/) const
{
  if (input.isAborted())
    return 0;

  CONSOLE_BRIDGE_logDebug("Successful");
  auto info = std::make_unique<StartTaskInfo>(uuid_, name_);
  info->return_value = 1;
  info->message = "Successful";
  //    saveOutputs(*info, input);
  info->elapsed_time = 0;
  input.addTaskInfo(std::move(info));
  return 1;
}

TaskComposerNode::UPtr StartTask::clone() const { return std::make_unique<StartTask>(name_); }

bool StartTask::operator==(const StartTask& rhs) const
{
  bool equal = true;
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool StartTask::operator!=(const StartTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void StartTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

StartTaskInfo::StartTaskInfo(boost::uuids::uuid uuid, std::string name) : TaskComposerNodeInfo(uuid, std::move(name)) {}

TaskComposerNodeInfo::UPtr StartTaskInfo::clone() const { return std::make_unique<StartTaskInfo>(*this); }

bool StartTaskInfo::operator==(const StartTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskComposerNodeInfo::operator==(rhs);
  return equal;
}
bool StartTaskInfo::operator!=(const StartTaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void StartTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNodeInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::StartTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::StartTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::StartTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::StartTaskInfo)
