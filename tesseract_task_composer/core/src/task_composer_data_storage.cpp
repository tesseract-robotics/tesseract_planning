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
#include <mutex>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_data_storage.h>
namespace tesseract_planning
{
TaskComposerDataStorage::TaskComposerDataStorage(const TaskComposerDataStorage& other)
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  data_ = other.data_;
}
TaskComposerDataStorage& TaskComposerDataStorage::operator=(const TaskComposerDataStorage& other)
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  data_ = other.data_;
  return *this;
}
TaskComposerDataStorage::TaskComposerDataStorage(TaskComposerDataStorage&& other) noexcept
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::unique_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  data_ = std::move(other.data_);
}
TaskComposerDataStorage& TaskComposerDataStorage::operator=(TaskComposerDataStorage&& other) noexcept
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::unique_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  data_ = std::move(other.data_);
  return *this;
}

bool TaskComposerDataStorage::hasKey(const std::string& key)
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

bool TaskComposerDataStorage::operator==(const TaskComposerDataStorage& rhs) const
{
  std::shared_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(rhs.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  bool equal = true;
  equal &= data_ == rhs.data_;
  return equal;
}

bool TaskComposerDataStorage::operator!=(const TaskComposerDataStorage& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerDataStorage::serialize(Archive& ar, const unsigned int /*version*/)
{
  std::unique_lock lock(mutex_);
  ar& boost::serialization::make_nvp("data", data_);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerDataStorage)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerDataStorage)
