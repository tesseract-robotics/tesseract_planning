/**
 * @file task_composer_data_storage.h
 * @brief Data storage used when executing the pipeline
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_DATA_STORAGE_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_DATA_STORAGE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <unordered_map>
#include <shared_mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/any.h>

namespace tesseract_planning
{
/** @brief A thread save data storage */
class TaskComposerDataStorage
{
public:
  using Ptr = std::shared_ptr<TaskComposerDataStorage>;
  using ConstPtr = std::shared_ptr<const TaskComposerDataStorage>;
  using UPtr = std::unique_ptr<TaskComposerDataStorage>;
  using ConstUPtr = std::unique_ptr<const TaskComposerDataStorage>;

  TaskComposerDataStorage() = default;
  ~TaskComposerDataStorage() = default;
  TaskComposerDataStorage(const TaskComposerDataStorage&);
  TaskComposerDataStorage& operator=(const TaskComposerDataStorage&);
  TaskComposerDataStorage(TaskComposerDataStorage&&) = delete;
  TaskComposerDataStorage& operator=(TaskComposerDataStorage&&) = delete;

  /**
   * @brief Check if key exists
   * @param key The key to check for
   * @return True if the key exist, otherwise false
   */
  bool hasKey(const std::string& key);

  /**
   * @brief Set data for the provided key
   * @param key The key to set data for
   * @param data The data to assign to the provided key
   */
  void setData(const std::string& key, tesseract_common::Any data);

  /**
   * @brief Get the data for the provided key
   * @details If the key does not exist it will be null
   * @param key The key to retreive the data
   * @return The data associated with the key
   */
  tesseract_common::Any getData(const std::string& key) const;

  /**
   * @brief Remove data for the provide key
   * @param key The key to remove data for
   */
  void removeData(const std::string& key);

protected:
  mutable std::shared_mutex mutex_;
  std::unordered_map<std::string, tesseract_common::Any> data_;
};

}  // namespace tesseract_planning
#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_DATA_STORAGE_H
