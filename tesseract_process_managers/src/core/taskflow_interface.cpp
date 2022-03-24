/**
 * @file taskflow_interface.h
 * @brief Process Inteface
 *
 * @author Levi Armstrong
 * @date December 8. 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <boost/serialization/shared_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/atomic_serialization.h>
#include <tesseract_process_managers/core/task_info.h>
#include <tesseract_process_managers/core/taskflow_interface.h>

namespace tesseract_planning
{
bool TaskflowInterface::isAborted() const { return abort_; }

bool TaskflowInterface::isSuccessful() const { return !abort_; }

void TaskflowInterface::abort() { abort_ = true; }

TaskInfo::UPtr TaskflowInterface::getTaskInfo(const std::size_t& index) const
{
  if (task_infos_)
    return (*task_infos_)[index];
  return nullptr;
}

std::map<std::size_t, TaskInfo::UPtr> TaskflowInterface::getTaskInfoMap() const
{
  return task_infos_->getTaskInfoMap();
}

TaskInfoContainer::Ptr TaskflowInterface::getTaskInfoContainer() const { return task_infos_; }

bool TaskflowInterface::operator==(const tesseract_planning::TaskflowInterface& rhs) const
{
  bool equal = true;
  equal &= abort_ == rhs.abort_;
  equal &= tesseract_common::pointersEqual(task_infos_, rhs.task_infos_);

  return equal;
}

bool TaskflowInterface::operator!=(const tesseract_planning::TaskflowInterface& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskflowInterface::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(abort_);
  ar& BOOST_SERIALIZATION_NVP(task_infos_);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskflowInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskflowInterface)
