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

}  // namespace tesseract_planning
