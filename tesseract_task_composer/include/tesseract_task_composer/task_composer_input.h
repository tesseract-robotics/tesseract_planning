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

#include <tesseract_environment/environment.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_task_composer/task_composer_data_storage.h>
#include <tesseract_task_composer/task_composer_node_info.h>

namespace tf
{
class Executor;
}

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
  const tesseract_environment::Environment::ConstPtr env;

  /** @brief Global Manipulator Information */
  const tesseract_common::ManipulatorInfo manip_info;

  /**
   * @brief This allows the remapping of the Move Profile identified in the command language to a specific profile for a
   * given motion planner.
   */
  const ProfileRemapping move_profile_remapping;

  /**
   * @brief This allows the remapping of the Composite Profile identified in the command language to a specific profile
   * for a given motion planner.
   */
  const ProfileRemapping composite_profile_remapping;

  /** @brief The Profiles to use */
  const ProfileDictionary::ConstPtr profiles;

  /** @brief The location data is stored and retrieved during execution */
  const TaskComposerDataStorage::Ptr data_storage;

  /** @brief The task executor */
  const std::shared_ptr<tf::Executor> executor;

  /** @brief The location where task info is stored during execution */
  TaskComposerNodeInfoContainer task_infos;

  /** @brief This indicates if a seed was provided */
  bool has_seed{ false };

  /** @brief If true the task will save the inputs and outputs to the TaskInfo*/
  bool save_io{ false };

  //  /**
  //   * @brief Gets the task interface for checking success and aborting active process
  //   * @return The task interface for checking success and aborting active process
  //   */
  //  TaskflowInterface::Ptr getTaskInterface();

  /**
   * @brief Check if process has been aborted
   * @details This accesses the internal process interface class
   * @return True if aborted otherwise false;
   */
  bool isAborted() const;

  /**
   * @brief Abort the process input
   * @details This accesses the internal process interface class to abort the process
   */
  void abort();

  void addTaskInfo(TaskComposerNodeInfo::UPtr task_info);
  TaskComposerNodeInfo::UPtr getTaskInfo(const boost::uuids::uuid& key) const;
  std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr> getTaskInfoMap() const;

protected:
  mutable bool aborted_{ false };
  //  /** @brief Used to store if process input is aborted which is thread safe */
  //  TaskflowInterface::Ptr interface_{ std::make_shared<TaskflowInterface>() };
};
}  // namespace tesseract_planning
#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_INPUT_H
