/**
 * @file task_composer_input.cpp
 * @brief The input data structure to the pipeline
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

#include <tesseract_task_composer/task_composer_input.h>
#include <tesseract_task_composer/task_composer_node.h>

namespace tesseract_planning
{
TaskComposerInput::TaskComposerInput(TaskComposerProblem::UPtr problem, ProfileDictionary::ConstPtr profiles)
  : problem(std::move(problem)), profiles(std::move(profiles)), data_storage(this->problem->input_data)
{
}

bool TaskComposerInput::isAborted() const { return aborted_; }

bool TaskComposerInput::isSuccessful() const { return !aborted_; }

void TaskComposerInput::abort(const boost::uuids::uuid& calling_node)
{
  if (!calling_node.is_nil())
    task_infos.setAborted(calling_node);

  aborted_ = true;
}

void TaskComposerInput::reset()
{
  aborted_ = false;
  data_storage = problem->input_data;
  task_infos.clear();
}

bool TaskComposerInput::operator==(const TaskComposerInput& rhs) const
{
  bool equal = true;
  equal &= problem == rhs.problem;
  //  equal &= tesseract_common::pointersEqual(profiles, rhs.profiles);
  equal &= data_storage == rhs.data_storage;
  equal &= task_infos == rhs.task_infos;
  equal &= aborted_ == rhs.aborted_;
  return equal;
}

bool TaskComposerInput::operator!=(const TaskComposerInput& rhs) const { return !operator==(rhs); }

// TaskComposerInput::TaskComposerInput(const TaskComposerInput& rhs)
//  : problem(rhs.problem)
//  , profiles(rhs.profiles)
//  , data_storage(rhs.data_storage)
//  , task_infos(rhs.task_infos)
//  , aborted_(rhs.aborted_.load())
//{
//}

TaskComposerInput::TaskComposerInput(TaskComposerInput&& rhs) noexcept
  : problem(std::move(rhs.problem))
  , profiles(std::move(rhs.profiles))
  , data_storage(std::move(rhs.data_storage))
  , task_infos(std::move(rhs.task_infos))
  , aborted_(rhs.aborted_.load())
{
}

template <class Archive>
void TaskComposerInput::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("problem", problem);
  /** @todo Fix after profiles are serializable */
  //  ar& boost::serialization::make_nvp("profiles", profiles);
  ar& boost::serialization::make_nvp("data_storage", data_storage);
  ar& boost::serialization::make_nvp("task_infos", task_infos);
  ar& boost::serialization::make_nvp("aborted", aborted_);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerInput)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerInput)
