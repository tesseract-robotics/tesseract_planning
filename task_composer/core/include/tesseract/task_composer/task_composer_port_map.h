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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PORT_MAP_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PORT_MAP_H

#include <unordered_map>
#include <map>
#include <string>
#include <vector>
#include <variant>

#include <tesseract/common/fwd.h>

namespace tesseract::task_composer
{
class TaskComposerPortMap;

template <class Archive>
void serialize(Archive& ar, TaskComposerPortMap& obj);

class TaskComposerPortMap
{
public:
  using Mapping = std::variant<std::string, std::vector<std::string>>;
  using ContainerType = std::unordered_map<std::string, Mapping>;

  void set(std::string port, std::string storage_key);
  void set(std::string port, std::vector<std::string> storage_keys);
  void erase(const std::string& port);
  bool contains(const std::string& port) const;
  const Mapping& at(const std::string& port) const;
  const std::string& single(const std::string& port) const;
  const std::vector<std::string>& multiple(const std::string& port) const;

  /**
   * @brief Rename storage keys
   * @param storage_key_remapping The old-to-new storage key mapping
   */
  void renameStorageKeys(const std::map<std::string, std::string>& storage_key_remapping);

  /**
   * @brief Get the data container object
   * @return The data container object
   */
  const ContainerType& data() const;

  /** @brief The size */
  std::size_t size() const;

  /** @brief Check if empty */
  bool empty() const;

  bool operator==(const TaskComposerPortMap& rhs) const;
  bool operator!=(const TaskComposerPortMap& rhs) const;

private:
  ContainerType mappings_;

  template <class Archive>
  friend void ::tesseract::task_composer::serialize(Archive& ar, TaskComposerPortMap& obj);
};

std::ostream& operator<<(std::ostream& os, const TaskComposerPortMap& port_map);

}  // namespace tesseract::task_composer

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PORT_MAP_H
