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
#include <map>
#include <memory>
#include <unordered_map>
#include <shared_mutex>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/any_poly.h>

namespace tesseract_planning
{
class TaskComposerKeys;

/** @brief A thread save data storage */
class TaskComposerDataStorage
{
public:
  using Ptr = std::shared_ptr<TaskComposerDataStorage>;
  using ConstPtr = std::shared_ptr<const TaskComposerDataStorage>;
  using UPtr = std::unique_ptr<TaskComposerDataStorage>;
  using ConstUPtr = std::unique_ptr<const TaskComposerDataStorage>;

  TaskComposerDataStorage(std::string name = "");
  ~TaskComposerDataStorage() = default;
  TaskComposerDataStorage(const TaskComposerDataStorage&);
  TaskComposerDataStorage& operator=(const TaskComposerDataStorage&);
  TaskComposerDataStorage(TaskComposerDataStorage&&) noexcept;
  TaskComposerDataStorage& operator=(TaskComposerDataStorage&&) noexcept;

  /**
   * @brief Get the name data storage.
   * @details This is primarily used to hold the pipeline name
   * @return The name
   */
  std::string getName() const;

  /**
   * @brief Set the name
   * @param name The name
   */
  void setName(const std::string& name);

  /**
   * @brief Check if key exists
   * @param key The key to check for
   * @return True if the key exist, otherwise false
   */
  bool hasKey(const std::string& key) const;

  /**
   * @brief Set data for the provided key
   * @param key The key to set data for
   * @param data The data to assign to the provided key
   */
  void setData(const std::string& key, tesseract_common::AnyPoly data);

  /**
   * @brief Get the data for the provided key
   * @details If the key does not exist it will be null
   * @param key The key to retreive the data
   * @return The data associated with the key
   */
  tesseract_common::AnyPoly getData(const std::string& key) const;

  /**
   * @brief Remove data for the provide key
   * @param key The key to remove data for
   */
  void removeData(const std::string& key);

  /**
   * @brief Get all data stored
   * @return A copy of the data
   */
  std::unordered_map<std::string, tesseract_common::AnyPoly> getData() const;

  /**
   * @brief Remap data from one key to another
   * @param remapping The key value pairs to remap data from the first to the second
   * @param copy Default behavior is not move the data, but if copy is desired set this to true
   * @return True if successful, otherwise false
   */
  bool remapData(const std::map<std::string, std::string>& remapping, bool copy = false);

  /**
   * @brief Copy data as input data from the specified keys from another data storage
   * @param data_storage The input data storage from which to copy data
   * @param keys The keys from which to copy data from the input data storage
   * @param override_keys The override keys if any when looking up data
   */
  void copyAsInputData(const TaskComposerDataStorage& data_storage,
                       const TaskComposerKeys& keys,
                       const TaskComposerKeys& override_keys);

  /**
   * @brief Copy data as output data from the specified keys from another data storage
   * @param data_storage The input data storage from which to copy data
   * @param keys The keys from which to copy data from the input data storage
   * @param override_keys The override keys to use if any when storing data
   */
  void copyAsOutputData(const TaskComposerDataStorage& data_storage,
                        const TaskComposerKeys& keys,
                        const TaskComposerKeys& override_keys);

  bool operator==(const TaskComposerDataStorage& rhs) const;
  bool operator!=(const TaskComposerDataStorage& rhs) const;

private:
  mutable std::shared_mutex mutex_;
  std::string name_;
  std::unordered_map<std::string, tesseract_common::AnyPoly> data_;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

using TaskComposerDataStoragePtrAnyPoly = tesseract_common::AnyWrapper<TaskComposerDataStorage::Ptr>;

}  // namespace tesseract_planning
BOOST_CLASS_EXPORT_KEY(tesseract_planning::TaskComposerDataStorage)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TaskComposerDataStoragePtrAnyPoly)
BOOST_CLASS_TRACKING(tesseract_planning::TaskComposerDataStoragePtrAnyPoly, boost::serialization::track_never)

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_DATA_STORAGE_H
