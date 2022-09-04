/**
 * @file task_composer_input.h
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_INPUT_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_INPUT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <atomic>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_task_composer/task_composer_data_storage.h>
#include <tesseract_task_composer/task_composer_node_info.h>

namespace tesseract_planning
{
/**
 * @brief This struct is passed as an input to each process in the decision tree
 *
 * Note that it does not have ownership of any of its members (except the pointer). This means that if a TaskInput
 * spawns a child that is a subset, it does not have to remain in scope as the references will still be valid
 */
struct TaskComposerInput
{
  using Ptr = std::shared_ptr<TaskComposerInput>;
  using ConstPtr = std::shared_ptr<const TaskComposerInput>;

  TaskComposerInput(TaskComposerDataStorage::Ptr data_storage);

  TaskComposerInput(tesseract_environment::Environment::ConstPtr env,
                    tesseract_common::ManipulatorInfo manip_info,
                    ProfileDictionary::ConstPtr profiles,
                    TaskComposerDataStorage::Ptr data_storage);

  TaskComposerInput(tesseract_environment::Environment::ConstPtr env,
                    tesseract_common::ManipulatorInfo manip_info,
                    ProfileRemapping move_profile_remapping,
                    ProfileRemapping composite_profile_remapping,
                    ProfileDictionary::ConstPtr profiles,
                    TaskComposerDataStorage::Ptr data_storage);

  TaskComposerInput(tesseract_environment::Environment::ConstPtr env,
                    ProfileRemapping move_profile_remapping,
                    ProfileRemapping composite_profile_remapping,
                    ProfileDictionary::ConstPtr profiles,
                    TaskComposerDataStorage::Ptr data_storage);

  TaskComposerInput(tesseract_environment::Environment::ConstPtr env,
                    ProfileDictionary::ConstPtr profiles,
                    TaskComposerDataStorage::Ptr data_storage);

  /** @brief Tesseract associated with current state of the system */
  tesseract_environment::Environment::ConstPtr env;

  /** @brief Global Manipulator Information */
  tesseract_common::ManipulatorInfo manip_info;

  /**
   * @brief This allows the remapping of the Move Profile identified in the command language to a specific profile for a
   * given motion planner.
   */
  ProfileRemapping move_profile_remapping;

  /**
   * @brief This allows the remapping of the Composite Profile identified in the command language to a specific profile
   * for a given motion planner.
   */
  ProfileRemapping composite_profile_remapping;

  /** @brief The Profiles to use */
  ProfileDictionary::ConstPtr profiles;

  /** @brief The location data is stored and retrieved during execution */
  TaskComposerDataStorage::Ptr data_storage;

  /** @brief The location where task info is stored during execution */
  TaskComposerNodeInfoContainer task_infos;

  /**
   * @brief Check if process has been aborted
   * @details This accesses the internal process interface class
   * @return True if aborted otherwise false;
   */
  bool isAborted() const;

  /**
   * @brief If it was not aborted then it was successful
   * @return True if successful, otherwise false
   */
  bool isSuccessful() const;

  /**
   * @brief Abort the process input
   * @details This accesses the internal process interface class to abort the process
   */
  void abort();

  /** @brief Reset abort and data storage to constructed state */
  void reset();

  bool operator==(const TaskComposerInput& rhs) const;
  bool operator!=(const TaskComposerInput& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  TaskComposerInput() = default;  // Required for serialization
  TaskComposerInput(const TaskComposerInput&);

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  mutable std::atomic<bool> aborted_{ false };

  /** @brief Store a copy of the original data storage for resolving using reset() */
  TaskComposerDataStorage::ConstPtr original_data_storage_;
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TaskComposerInput, "TaskComposerInput")

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_INPUT_H
