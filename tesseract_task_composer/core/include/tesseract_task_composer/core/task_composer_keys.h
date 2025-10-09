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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_KEYS_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_KEYS_H

#include <unordered_map>
#include <map>
#include <string>
#include <vector>
#include <variant>

#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>

#include <tesseract_common/fwd.h>

namespace tesseract_planning
{
class TaskComposerKeys
{
public:
  using EntryType = std::variant<std::string, std::vector<std::string>>;
  using ContainerType = std::unordered_map<std::string, EntryType>;

  /**
   * @brief Add key
   * @param port The port name associated with the key
   * @param key The key assigned to the port
   */
  void add(const std::string& port, std::string key);

  /**
   * @brief Add keys
   * @param port The port name associated with the keys
   * @param key The keys assigned to the port
   */
  void add(const std::string& port, std::vector<std::string> keys);

  /**
   * @brief Remove port entry
   * @param port The port to remove
   */
  void remove(const std::string& port);

  /**
   * @brief Rename keys
   * @param keys The key renamming map
   */
  void rename(const std::map<std::string, std::string>& keys);

  /**
   * @brief Check if port exist
   * @param port The port name
   * @return True if entry exist, otherwise false
   */
  bool has(const std::string& port) const;

  /**
   * @brief Get key/keys assigned to port
   * @param port The port to retrieve key/keys
   * @return The key/keys assigned to port
   */
  template <typename T = std::string>
  const T& get(const std::string& port) const;

  /**
   * @brief Get the data container object
   * @return The data container object
   */
  const ContainerType& data() const;

  /** @brief The size */
  std::size_t size() const;

  /** @brief Check if empty */
  bool empty() const;

  bool operator==(const TaskComposerKeys& rhs) const;
  bool operator!=(const TaskComposerKeys& rhs) const;

private:
  ContainerType keys_;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

std::ostream& operator<<(std::ostream& os, const TaskComposerKeys& keys);
}  // namespace tesseract_planning
BOOST_CLASS_EXPORT_KEY(tesseract_planning::TaskComposerKeys)
#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_KEYS_H
