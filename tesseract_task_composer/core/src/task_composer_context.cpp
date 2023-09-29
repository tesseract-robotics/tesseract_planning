/**
 * @file task_composer_context.cpp
 * @brief The context data structure to the pipeline
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
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/unique_ptr.hpp>
#if (BOOST_VERSION >= 107400) && (BOOST_VERSION < 107500)
#include <boost/serialization/library_version_type.hpp>
#endif
#include <boost/serialization/unordered_map.hpp>
#include <tesseract_common/atomic_serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_context.h>

namespace tesseract_planning
{
TaskComposerContext::TaskComposerContext(TaskComposerProblem::Ptr problem)
  : TaskComposerContext(std::move(problem), std::make_shared<TaskComposerDataStorage>())
{
}

TaskComposerContext::TaskComposerContext(tesseract_planning::TaskComposerProblem::Ptr problem,
                                         TaskComposerDataStorage::Ptr data_storage)
  : problem(std::move(problem)), data_storage(std::move(data_storage))
{
}

bool TaskComposerContext::isAborted() const { return aborted_; }

bool TaskComposerContext::isSuccessful() const { return !aborted_; }

void TaskComposerContext::abort(const boost::uuids::uuid& calling_node)
{
  if (!calling_node.is_nil())
    task_infos.setAborted(calling_node);

  aborted_ = true;
}

bool TaskComposerContext::operator==(const TaskComposerContext& rhs) const
{
  bool equal = true;
  if (problem != nullptr && rhs.problem != nullptr)
    equal &= (*problem == *rhs.problem);
  else
    equal &= (problem == nullptr && rhs.problem == nullptr);

  if (data_storage != nullptr && rhs.data_storage != nullptr)
    equal &= (*data_storage == *rhs.data_storage);
  else
    equal &= (data_storage == nullptr && rhs.data_storage == nullptr);

  equal &= task_infos == rhs.task_infos;
  equal &= aborted_ == rhs.aborted_;
  return equal;
}

bool TaskComposerContext::operator!=(const TaskComposerContext& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerContext::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("problem", problem);
  ar& boost::serialization::make_nvp("data_storage", data_storage);
  ar& boost::serialization::make_nvp("task_infos", task_infos);
  ar& boost::serialization::make_nvp("aborted", aborted_);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerContext)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerContext)
