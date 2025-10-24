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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/version.hpp>
#if (BOOST_VERSION >= 107400) && (BOOST_VERSION < 107500)
#include <boost/serialization/library_version_type.hpp>
#endif
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <mutex>
#include <console_bridge/console.h>
#include <tesseract_common/serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_keys.h>

namespace tesseract_planning
{
TaskComposerDataStorage::TaskComposerDataStorage(std::string name) : name_(std::move(name)) {}

// NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
TaskComposerDataStorage::TaskComposerDataStorage(const TaskComposerDataStorage& other)
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  name_ = other.name_;  // NOLINT(cppcoreguidelines-prefer-member-initializer)
  data_ = other.data_;  // NOLINT(cppcoreguidelines-prefer-member-initializer)
}

// NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
TaskComposerDataStorage& TaskComposerDataStorage::operator=(const TaskComposerDataStorage& other)
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  name_ = other.name_;  // NOLINT(cppcoreguidelines-prefer-member-initializer)
  data_ = other.data_;  // NOLINT(cppcoreguidelines-prefer-member-initializer)
  return *this;
}

// NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
TaskComposerDataStorage::TaskComposerDataStorage(TaskComposerDataStorage&& other) noexcept
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::unique_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  name_ = std::move(other.name_);  // NOLINT(cppcoreguidelines-prefer-member-initializer)
  data_ = std::move(other.data_);  // NOLINT(cppcoreguidelines-prefer-member-initializer)
}

// NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
TaskComposerDataStorage& TaskComposerDataStorage::operator=(TaskComposerDataStorage&& other) noexcept
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::unique_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  name_ = std::move(other.name_);  // NOLINT(cppcoreguidelines-prefer-member-initializer)
  data_ = std::move(other.data_);  // NOLINT(cppcoreguidelines-prefer-member-initializer)
  return *this;
}

std::string TaskComposerDataStorage::getName() const
{
  std::shared_lock lock(mutex_);
  return name_;
}

void TaskComposerDataStorage::setName(const std::string& name)
{
  std::unique_lock lock(mutex_);
  name_ = name;
}

bool TaskComposerDataStorage::hasKey(const std::string& key) const
{
  std::shared_lock lock(mutex_);
  return (data_.find(key) != data_.end());
}

void TaskComposerDataStorage::setData(const std::string& key, tesseract_common::AnyPoly data)
{
  std::unique_lock lock(mutex_);
  data_[key] = std::move(data);
}

tesseract_common::AnyPoly TaskComposerDataStorage::getData(const std::string& key) const
{
  std::shared_lock lock(mutex_);
  auto it = data_.find(key);
  if (it == data_.end())
    return {};

  return it->second;
}

void TaskComposerDataStorage::removeData(const std::string& key)
{
  std::unique_lock lock(mutex_);
  data_.erase(key);
}

std::unordered_map<std::string, tesseract_common::AnyPoly> TaskComposerDataStorage::getData() const
{
  std::shared_lock lock(mutex_);
  return data_;
}

bool TaskComposerDataStorage::remapData(const std::map<std::string, std::string>& remapping, bool copy)
{
  std::unique_lock lock(mutex_);

  if (copy)
  {
    for (const auto& pair : remapping)
    {
      auto it = data_.find(pair.first);
      if (it != data_.end())
      {
        data_[pair.second] = it->second;
      }
      else
      {
        CONSOLE_BRIDGE_logError(
            "TaskComposerDataStorage, unable to remap data '%s' to '%s'", pair.first.c_str(), pair.second.c_str());
        return false;
      }
    }
  }
  else
  {
    for (const auto& pair : remapping)
    {
      if (auto nh = data_.extract(pair.first); !nh.empty())
      {
        nh.key() = pair.second;
        data_.insert(std::move(nh));
      }
      else
      {
        CONSOLE_BRIDGE_logError(
            "TaskComposerDataStorage, unable to remap data '%s' to '%s'", pair.first.c_str(), pair.second.c_str());
        return false;
      }
    }
  }

  return true;
}

// void TaskComposerDataStorage::copyData(const TaskComposerDataStorage& data_storage,
//                                        const TaskComposerKeys& keys,
//                                        const std::map<std::string, std::string> &override_keys)
// {
//   for (const auto& pair : keys.data())
//   {
//     if (pair.second.index() == 0)
//     {
//       const auto& key = std::get<std::string>(pair.second);

//       // Check if the port has an override and if so use its key for retrieving the data from the parent data
//       storage.
//       // Otherwise use the original key for retrieving the data
//       auto it = override_keys.find(key);
//       const std::string& lookup_key = (it == override_keys.end()) ? key : it->second;
//       tesseract_common::AnyPoly entry = data_storage.getData(lookup_key);

//       if (entry.isNull())
//         throw std::runtime_error("TaskComposerDataStorage, unable to copy data for " + pair.first + ":" +
//         lookup_key);

//       setData(key, entry);
//     }
//     else
//     {
//       const auto& keys = std::get<std::vector<std::string>>(pair.second);
//       for (const auto& key : keys)
//       {
//         // Check if the port has an override and if so use its key for retrieving the data from the parent data
//         storage.
//         // Otherwise use the original key for retrieving the data
//         auto it = override_keys.find(key);
//         const std::string& lookup_key = (it == override_keys.end()) ? key : it->second;
//         tesseract_common::AnyPoly entry = data_storage.getData(lookup_key);

//         if (entry.isNull())
//           throw std::runtime_error("TaskComposerDataStorage, unable to copy data for " + pair.first + ":" +
//           lookup_key);

//         setData(key, entry);
//       }
//     }
//   }
// }

void copyDataHelper(TaskComposerDataStorage& ods,
                    const TaskComposerDataStorage& ids,
                    const std::string& lookup_key,
                    const std::string& storage_key)
{
  tesseract_common::AnyPoly entry = ids.getData(lookup_key);

  if (entry.isNull())
    throw std::runtime_error("TaskComposerDataStorage, unable to copy data for '" + lookup_key + "'");

  ods.setData(storage_key, entry);
}

void TaskComposerDataStorage::copyInputData(const TaskComposerDataStorage& data_storage,
                                            const TaskComposerKeys& keys,
                                            const TaskComposerKeys& override_keys)
{
  for (const auto& pair : keys.data())
  {
    if (pair.second.index() == 0)
    {
      const auto& key = std::get<std::string>(pair.second);

      // Check if the port has an override and if so use its key for retrieving the data from the parent data storage.
      // Otherwise use the original key for retrieving the data
      const std::string& lookup_key = override_keys.has(pair.first) ? override_keys.get<std::string>(pair.first) : key;
      copyDataHelper(*this, data_storage, lookup_key, key);
    }
    else
    {
      const auto& keys = std::get<std::vector<std::string>>(pair.second);

      // Check if the port has an override and if so use its key for retrieving the data from the parent data storage.
      // Otherwise use the original key for retrieving the data
      const std::vector<std::string>& lookup_keys =
          override_keys.has(pair.first) ? override_keys.get<std::vector<std::string>>(pair.first) : keys;

      if (keys.size() != lookup_keys.size())
        throw std::runtime_error("TaskComposerDataStorage, unable to copy data for port '" + pair.first +
                                 "' with override because size is not the same.");

      for (std::size_t i = 0; i < lookup_keys.size(); ++i)
        copyDataHelper(*this, data_storage, lookup_keys[i], keys[i]);
    }
  }
}

void TaskComposerDataStorage::copyOutputData(const TaskComposerDataStorage& data_storage,
                                             const TaskComposerKeys& keys,
                                             const TaskComposerKeys& override_keys)
{
  for (const auto& pair : keys.data())
  {
    if (pair.second.index() == 0)
    {
      const auto& key = std::get<std::string>(pair.second);

      // Check if the port has an override and if so use its key for retrieving the data from the parent data storage.
      // Otherwise use the original key for retrieving the data
      const std::string& storage_key = override_keys.has(pair.first) ? override_keys.get<std::string>(pair.first) : key;
      copyDataHelper(*this, data_storage, key, storage_key);
    }
    else
    {
      const auto& keys = std::get<std::vector<std::string>>(pair.second);

      // Check if the port has an override and if so use its key for retrieving the data from the parent data storage.
      // Otherwise use the original key for retrieving the data
      const std::vector<std::string>& storage_keys =
          override_keys.has(pair.first) ? override_keys.get<std::vector<std::string>>(pair.first) : keys;

      if (keys.size() != storage_keys.size())
        throw std::runtime_error("TaskComposerDataStorage, unable to copy data for port '" + pair.first +
                                 "' with override because size is not the same.");

      for (std::size_t i = 0; i < keys.size(); ++i)
        copyDataHelper(*this, data_storage, keys[i], storage_keys[i]);
    }
  }
}

bool TaskComposerDataStorage::operator==(const TaskComposerDataStorage& rhs) const
{
  std::shared_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(rhs.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };
  return ((data_ == rhs.data_) && (name_ == rhs.name_));
}

bool TaskComposerDataStorage::operator!=(const TaskComposerDataStorage& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerDataStorage::serialize(Archive& ar, const unsigned int /*version*/)
{
  std::unique_lock lock(mutex_);
  ar& boost::serialization::make_nvp("name", name_);
  ar& boost::serialization::make_nvp("data", data_);
}

}  // namespace tesseract_planning
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerDataStorage)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerDataStorage)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerDataStoragePtrAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerDataStoragePtrAnyPoly)
