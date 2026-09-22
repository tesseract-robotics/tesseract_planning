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

#include <tesseract/task_composer/task_composer_port_map.h>
#include <tesseract/common/any_poly.h>

#include <ostream>
#include <stdexcept>

namespace tesseract::task_composer
{
namespace
{
void validatePortName(const std::string& port)
{
  if (port.empty())
    throw std::invalid_argument("Port name must not be empty");
}
}  // namespace

void TaskComposerPortMap::set(std::string port, std::string storage_key)
{
  validatePortName(port);
  if (storage_key.empty())
    throw std::invalid_argument("Storage key for port '" + port + "' must not be empty");
  mappings_[std::move(port)] = std::move(storage_key);
}

void TaskComposerPortMap::set(std::string port, std::vector<std::string> storage_keys)
{
  validatePortName(port);
  if (storage_keys.empty())
    throw std::invalid_argument("Storage key list for port '" + port + "' must not be empty");
  for (std::size_t index = 0; index < storage_keys.size(); ++index)
  {
    if (storage_keys[index].empty())
      throw std::invalid_argument("Storage key for port '" + port + "' at index " + std::to_string(index) +
                                  " must not be empty");
  }
  mappings_[std::move(port)] = std::move(storage_keys);
}

void TaskComposerPortMap::erase(const std::string& port) { mappings_.erase(port); }

void TaskComposerPortMap::renameStorageKeys(const std::map<std::string, std::string>& storage_key_remapping)
{
  for (auto& mapping : mappings_)
  {
    if (std::holds_alternative<std::string>(mapping.second))
    {
      auto it = storage_key_remapping.find(std::get<std::string>(mapping.second));
      if (it != storage_key_remapping.end())
      {
        if (it->second.empty())
          throw std::invalid_argument("Renamed storage key for port '" + mapping.first + "' must not be empty");
        mapping.second = it->second;
      }
    }
    else
    {
      auto& storage_keys = std::get<std::vector<std::string>>(mapping.second);
      for (auto& storage_key : storage_keys)
      {
        auto it = storage_key_remapping.find(storage_key);
        if (it != storage_key_remapping.end())
        {
          if (it->second.empty())
            throw std::invalid_argument("Renamed storage key for port '" + mapping.first + "' must not be empty");
          storage_key = it->second;
        }
      }
    }
  }
}

bool TaskComposerPortMap::contains(const std::string& port) const { return mappings_.find(port) != mappings_.end(); }

const TaskComposerPortMap::Mapping& TaskComposerPortMap::at(const std::string& port) const
{
  return mappings_.at(port);
}

const std::string& TaskComposerPortMap::single(const std::string& port) const
{
  const auto& mapping = at(port);
  if (!std::holds_alternative<std::string>(mapping))
    throw std::invalid_argument("Port '" + port + "' does not contain a single storage key");
  return std::get<std::string>(mapping);
}

const std::vector<std::string>& TaskComposerPortMap::multiple(const std::string& port) const
{
  const auto& mapping = at(port);
  if (!std::holds_alternative<std::vector<std::string>>(mapping))
    throw std::invalid_argument("Port '" + port + "' does not contain multiple storage keys");
  return std::get<std::vector<std::string>>(mapping);
}

const TaskComposerPortMap::ContainerType& TaskComposerPortMap::data() const { return mappings_; }

std::size_t TaskComposerPortMap::size() const { return mappings_.size(); }
bool TaskComposerPortMap::empty() const { return mappings_.empty(); }
bool TaskComposerPortMap::operator==(const TaskComposerPortMap& rhs) const { return (mappings_ == rhs.mappings_); }
bool TaskComposerPortMap::operator!=(const TaskComposerPortMap& rhs) const { return !operator==(rhs); }

std::ostream& operator<<(std::ostream& os, const TaskComposerPortMap& port_map)
{
  using namespace std;
  for (const auto& pair : port_map.data())
  {
    if (std::holds_alternative<std::string>(pair.second))
    {
      os << "\t" << pair.first << ": " << std::get<std::string>(pair.second);
    }
    else
    {
      os << "\t" << pair.first << ":[";
      const auto& vs = std::get<std::vector<std::string>>(pair.second);
      for (std::size_t i = 0; i < vs.size(); ++i)
      {
        os << vs[i];
        if (i < vs.size() - 1)
          os << ", ";
      }
      os << "]";
    }
    os << "\\l";
  }

  return os;
}

}  // namespace tesseract::task_composer
