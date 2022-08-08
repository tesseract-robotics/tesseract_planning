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
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/nodes/error_task.h>

namespace tesseract_planning
{
ErrorTask::ErrorTask() : TaskComposerNode("ErrorTask") {}

int ErrorTask::run(TaskComposerInput& input) const
{
  CONSOLE_BRIDGE_logDebug("Error");
  input.abort();
  auto info = std::make_unique<ErrorTaskInfo>(uuid_, name_);
  info->return_value = 1;
  info->message = "Error";
  //    saveOutputs(*info, input);
  info->elapsed_time = 0;
  input.addTaskInfo(std::move(info));
  return 0;
}

bool ErrorTask::operator==(const ErrorTask& rhs) const
{
  bool equal = true;
  equal &= TaskComposerNode::operator==(rhs);
  return equal;
}
bool ErrorTask::operator!=(const ErrorTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void ErrorTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNode);
}

ErrorTaskInfo::ErrorTaskInfo(boost::uuids::uuid uuid, std::string name) : TaskComposerNodeInfo(uuid, std::move(name)) {}

TaskComposerNodeInfo::UPtr ErrorTaskInfo::clone() const { return std::make_unique<ErrorTaskInfo>(*this); }

bool ErrorTaskInfo::operator==(const ErrorTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskComposerNodeInfo::operator==(rhs);
  return equal;
}
bool ErrorTaskInfo::operator!=(const ErrorTaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void ErrorTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNodeInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ErrorTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ErrorTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ErrorTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ErrorTaskInfo)
