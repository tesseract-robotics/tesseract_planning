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
TaskComposerContext::TaskComposerContext(TaskComposerProblem::UPtr problem)
  : problem_(std::move(problem)), data_storage_(std::make_shared<TaskComposerDataStorage>(this->problem_->input_data))
{
}

TaskComposerProblem& TaskComposerContext::getProblem() { return *problem_; }
const TaskComposerProblem& TaskComposerContext::getProblem() const { return *problem_; }

TaskComposerDataStorage& TaskComposerContext::getDataStorage() { return *data_storage_; }
const TaskComposerDataStorage& TaskComposerContext::getDataStorage() const { return *data_storage_; }

TaskComposerNodeInfoContainer& TaskComposerContext::getTaskInfos() { return task_infos_; }
const TaskComposerNodeInfoContainer& TaskComposerContext::getTaskInfos() const { return task_infos_; }

bool TaskComposerContext::isAborted() const { return aborted_; }

bool TaskComposerContext::isSuccessful() const { return !aborted_; }

void TaskComposerContext::abort(const boost::uuids::uuid& calling_node)
{
  if (!calling_node.is_nil())
    task_infos_.setAborted(calling_node);

  aborted_ = true;
}

void TaskComposerContext::reset()
{
  aborted_ = false;
  data_storage_ = std::make_shared<TaskComposerDataStorage>(problem_->input_data);
  task_infos_.clear();
}

/** @brief Create a copy */
TaskComposerContext::UPtr TaskComposerContext::createChild()
{
  auto child = std::make_unique<TaskComposerContext>();
  child->problem_ = problem_;
  child->data_storage_ = data_storage_;
  return child;
}

bool TaskComposerContext::operator==(const TaskComposerContext& rhs) const
{
  bool equal = true;
  if (problem_ != nullptr && rhs.problem_ != nullptr)
    equal &= (*problem_ == *rhs.problem_);
  else
    equal &= (problem_ == nullptr && rhs.problem_ == nullptr);

  if (data_storage_ != nullptr && rhs.data_storage_ != nullptr)
    equal &= (*data_storage_ == *rhs.data_storage_);
  else
    equal &= (data_storage_ == nullptr && rhs.data_storage_ == nullptr);

  equal &= task_infos_ == rhs.task_infos_;
  equal &= aborted_ == rhs.aborted_;
  return equal;
}

bool TaskComposerContext::operator!=(const TaskComposerContext& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerContext::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("problem", problem_);
  ar& boost::serialization::make_nvp("data_storage", data_storage_);
  ar& boost::serialization::make_nvp("task_infos", task_infos_);
  ar& boost::serialization::make_nvp("aborted", aborted_);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerContext)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerContext)
