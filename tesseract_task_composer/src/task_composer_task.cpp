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

#include <tesseract_task_composer/task_composer_task.h>

namespace tesseract_planning
{
TaskComposerTask::TaskComposerTask(bool is_conditional, std::string name)
  : TaskComposerNode(std::move(name), TaskComposerNodeType::TASK), is_conditional_(is_conditional)
{
}

bool TaskComposerTask::isConditional() const { return is_conditional_; }

bool TaskComposerTask::operator==(const TaskComposerTask& rhs) const
{
  bool equal = true;
  equal &= (is_conditional_ == rhs.is_conditional_);
  equal &= TaskComposerNode::operator==(rhs);
  return equal;
}
bool TaskComposerTask::operator!=(const TaskComposerTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(is_conditional_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNode);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerTask)
