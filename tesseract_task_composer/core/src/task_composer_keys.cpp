/**
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

#include <tesseract_task_composer/core/task_composer_keys.h>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <tesseract_common/std_variant_serialization.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/any_poly.h>

namespace tesseract_planning
{
void TaskComposerKeys::add(const std::string& port, std::string key) { keys_[port] = std::move(key); }

void TaskComposerKeys::add(const std::string& port, std::vector<std::string> keys) { keys_[port] = std::move(keys); }

void TaskComposerKeys::rename(const std::map<std::string, std::string>& keys)
{
  for (auto& key : keys_)
  {
    if (key.second.index() == 0)
    {
      auto it = keys.find(std::get<std::string>(key.second));
      if (it != keys.end())
        key.second = it->second;
    }
    else
    {
      auto& vs = std::get<std::vector<std::string>>(key.second);
      for (auto& s : vs)
      {
        auto it = keys.find(s);
        if (it != keys.end())
          s = it->second;
      }
    }
  }
}

bool TaskComposerKeys::has(const std::string& port) const
{
  auto it = keys_.find(port);
  return (it != keys_.end());
}

template <>
const std::string& TaskComposerKeys::get(const std::string& port) const
{
  const auto& entry = keys_.at(port);
  return std::get<std::string>(entry);
}

template <>
const std::vector<std::string>& TaskComposerKeys::get(const std::string& port) const
{
  const auto& entry = keys_.at(port);
  return std::get<std::vector<std::string>>(entry);
}

const TaskComposerKeys::ContainerType& TaskComposerKeys::data() const { return keys_; }

std::size_t TaskComposerKeys::size() const { return keys_.size(); }
bool TaskComposerKeys::empty() const { return keys_.empty(); }
bool TaskComposerKeys::operator==(const TaskComposerKeys& rhs) const { return (keys_ == rhs.keys_); }
bool TaskComposerKeys::operator!=(const TaskComposerKeys& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerKeys::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("keys", keys_);
}

std::ostream& operator<<(std::ostream& os, const TaskComposerKeys& keys)
{
  os << "[";
  std::size_t cnt{ 0 };
  for (const auto& pair : keys.data())
  {
    if (pair.second.index() == 0)
    {
      os << pair.first << ":" << std::get<std::string>(pair.second);
    }
    else
    {
      os << pair.first << ":[";
      const auto& vs = std::get<std::vector<std::string>>(pair.second);
      for (std::size_t i = 0; i < vs.size(); ++i)
      {
        os << vs[i];
        if (i < vs.size() - 1)
          os << ", ";
      }
      os << "]";
    }
    if (cnt < keys.size() - 1)
      os << ", ";

    ++cnt;
  }
  os << "]";

  return os;
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerKeys)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerKeys)
