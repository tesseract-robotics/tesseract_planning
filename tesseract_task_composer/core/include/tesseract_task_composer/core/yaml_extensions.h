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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/yaml_extensions.h>
#include <tesseract_task_composer/core/task_composer_keys.h>

namespace YAML
{
//=========================== Task Composer Keys ===========================
template <>
struct convert<tesseract_planning::TaskComposerKeys>
{
  static Node encode(const tesseract_planning::TaskComposerKeys& rhs)
  {
    Node node;
    for (const auto& entry : rhs.data())
    {
      if (entry.second.index() == 0)
        node[entry.first] = std::get<std::string>(entry.second);
      else
        node[entry.first] = std::get<std::vector<std::string>>(entry.second);
    }

    return node;
  }

  static bool decode(const Node& node, tesseract_planning::TaskComposerKeys& rhs)
  {
    if (!node.IsMap())
      throw std::runtime_error("TaskComposerKeys, must be a yaml map");

    for (const auto& dict : node)
    {
      if (dict.second.IsSequence())
        rhs.add(dict.first.as<std::string>(), dict.second.as<std::vector<std::string>>());
      else if (dict.second.IsScalar())
        rhs.add(dict.first.as<std::string>(), dict.second.as<std::string>());
      else
        throw std::runtime_error("TaskComposerKeys, allowed port type is std::string or std::vector<std::string>");
    }

    return true;
  }
};

}  // namespace YAML

#endif  // TESSERACT_TASK_COMPOSER_CORE_YAML_EXTENSIONS_H
