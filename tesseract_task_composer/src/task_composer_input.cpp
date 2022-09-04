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
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <tesseract_common/atomic_serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_input.h>

namespace tesseract_planning
{
TaskComposerInput::TaskComposerInput(TaskComposerDataStorage::Ptr data_storage) : data_storage(std::move(data_storage))
{
}

TaskComposerInput::TaskComposerInput(tesseract_environment::Environment::ConstPtr env,
                                     tesseract_common::ManipulatorInfo manip_info,
                                     ProfileDictionary::ConstPtr profiles,
                                     TaskComposerDataStorage::Ptr data_storage)
  : env(std::move(env))
  , manip_info(std::move(manip_info))
  , profiles(std::move(profiles))
  , data_storage(std::move(data_storage))
  , original_data_storage_(std::make_shared<TaskComposerDataStorage>(*(this->data_storage)))
{
}

TaskComposerInput::TaskComposerInput(tesseract_environment::Environment::ConstPtr env,
                                     tesseract_common::ManipulatorInfo manip_info,
                                     ProfileRemapping move_profile_remapping,
                                     ProfileRemapping composite_profile_remapping,
                                     ProfileDictionary::ConstPtr profiles,
                                     TaskComposerDataStorage::Ptr data_storage)
  : env(std::move(env))
  , manip_info(std::move(manip_info))
  , move_profile_remapping(std::move(move_profile_remapping))
  , composite_profile_remapping(std::move(composite_profile_remapping))
  , profiles(std::move(profiles))
  , data_storage(std::move(data_storage))
  , original_data_storage_(std::make_shared<TaskComposerDataStorage>(*(this->data_storage)))
{
}

TaskComposerInput::TaskComposerInput(tesseract_environment::Environment::ConstPtr env,
                                     ProfileRemapping move_profile_remapping,
                                     ProfileRemapping composite_profile_remapping,
                                     ProfileDictionary::ConstPtr profiles,
                                     TaskComposerDataStorage::Ptr data_storage)
  : env(std::move(env))
  , move_profile_remapping(std::move(move_profile_remapping))
  , composite_profile_remapping(std::move(composite_profile_remapping))
  , profiles(std::move(profiles))
  , data_storage(std::move(data_storage))
  , original_data_storage_(std::make_shared<TaskComposerDataStorage>(*(this->data_storage)))
{
}

TaskComposerInput::TaskComposerInput(tesseract_environment::Environment::ConstPtr env,
                                     ProfileDictionary::ConstPtr profiles,
                                     TaskComposerDataStorage::Ptr data_storage)
  : env(std::move(env)), profiles(std::move(profiles)), data_storage(std::move(data_storage))
{
}

bool TaskComposerInput::isAborted() const { return aborted_; }

bool TaskComposerInput::isSuccessful() const { return !aborted_; }

void TaskComposerInput::abort() { aborted_ = true; }

void TaskComposerInput::reset()
{
  aborted_ = false;
  data_storage = std::make_shared<TaskComposerDataStorage>(*original_data_storage_);
}

bool TaskComposerInput::operator==(const TaskComposerInput& rhs) const
{
  bool equal = true;
  equal &= tesseract_common::pointersEqual(env, rhs.env);
  equal &= manip_info == rhs.manip_info;
  equal &= move_profile_remapping == rhs.move_profile_remapping;
  equal &= composite_profile_remapping == rhs.composite_profile_remapping;
  //  equal &= tesseract_common::pointersEqual(profiles, rhs.profiles);
  equal &= tesseract_common::pointersEqual(data_storage, rhs.data_storage);
  equal &= task_infos == rhs.task_infos;
  equal &= aborted_ == rhs.aborted_;
  equal &= tesseract_common::pointersEqual(original_data_storage_, rhs.original_data_storage_);
  return equal;
}

bool TaskComposerInput::operator!=(const TaskComposerInput& rhs) const { return !operator==(rhs); }

TaskComposerInput::TaskComposerInput(const TaskComposerInput& rhs)
  : env(rhs.env)
  , manip_info(rhs.manip_info)
  , move_profile_remapping(rhs.move_profile_remapping)
  , composite_profile_remapping(rhs.composite_profile_remapping)
  , profiles(rhs.profiles)
  , data_storage(rhs.data_storage)
  , task_infos(rhs.task_infos)
  , aborted_(rhs.aborted_.load())
  , original_data_storage_(rhs.original_data_storage_)
{
}

template <class Archive>
void TaskComposerInput::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("environment", env);
  ar& boost::serialization::make_nvp("manip_info", manip_info);
  ar& boost::serialization::make_nvp("move_profile_remapping", move_profile_remapping);
  ar& boost::serialization::make_nvp("composite_profile_remapping", composite_profile_remapping);
  /** @todo Fix after profiles are serializable */
  //  ar& boost::serialization::make_nvp("profiles", profiles);
  ar& boost::serialization::make_nvp("data_storage", data_storage);
  ar& boost::serialization::make_nvp("task_infos", task_infos);
  ar& boost::serialization::make_nvp("aborted", aborted_);
  ar& boost::serialization::make_nvp("original_data_storage", original_data_storage_);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerInput)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerInput)
