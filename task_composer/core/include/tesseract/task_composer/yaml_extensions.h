/**
 * @file yaml_extensions.h
 * @brief YAML Type conversions
 *
 * @author Samantha Smith
 * @date July 14, 2025
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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
#ifndef TESSERACT_TASK_COMPOSER_CORE_YAML_EXTENSIONS_H
#define TESSERACT_TASK_COMPOSER_CORE_YAML_EXTENSIONS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/yaml_extensions.h>
#include <tesseract/common/fwd.h>
#include <tesseract/task_composer/task_composer_port_map.h>

namespace tesseract::task_composer
{
/** @brief Registry key for a YAML value that may be a string or a list of strings. */
inline constexpr const char* STRING_OR_STRING_LIST_SCHEMA_KEY = "tesseract::task_composer::StringOrStringList";

/** @brief Registry key for a required task-composer port mapped to a string or list of strings. */
inline constexpr const char* REQUIRED_STRING_OR_STRING_LIST_SCHEMA_KEY = "tesseract::task_composer::"
                                                                         "RequiredStringOrStringList";
}  // namespace tesseract::task_composer

namespace YAML
{
//=========================== Task Composer Port Map ===========================
template <>
struct convert<tesseract::task_composer::TaskComposerPortMap>
{
  static Node encode(const tesseract::task_composer::TaskComposerPortMap& rhs)
  {
    Node node;
    for (const auto& entry : rhs.data())
    {
      if (std::holds_alternative<std::string>(entry.second))
        node[entry.first] = std::get<std::string>(entry.second);
      else
        node[entry.first] = std::get<std::vector<std::string>>(entry.second);
    }

    return node;
  }

  static bool decode(const Node& node, tesseract::task_composer::TaskComposerPortMap& rhs)
  {
    if (!node.IsMap())
      throw std::runtime_error("TaskComposerPortMap must be a YAML map");

    for (const auto& dict : node)
    {
      if (dict.second.IsSequence())
        rhs.set(dict.first.as<std::string>(), dict.second.as<std::vector<std::string>>());
      else if (dict.second.IsScalar())
        rhs.set(dict.first.as<std::string>(), dict.second.as<std::string>());
      else
        throw std::runtime_error("TaskComposerPortMap values must be a string or list of strings");
    }

    return true;
  }

  static tesseract::common::PropertyTree schema();
};

}  // namespace YAML

#endif  // TESSERACT_TASK_COMPOSER_CORE_YAML_EXTENSIONS_H
