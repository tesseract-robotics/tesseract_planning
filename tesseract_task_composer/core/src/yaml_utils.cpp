/**
 * @file yaml_utils.cpp
 * @brief YAML utility functions
 *
 * @author Levi Armstrong
 * @date October 12, 2025
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_task_composer/core/yaml_extensions.h>
#include <tesseract_task_composer/core/yaml_utils.h>
#include <tesseract_task_composer/core/task_composer_keys.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_graph.h>

#include <yaml-cpp/yaml.h>

namespace tesseract_planning
{
void loadSubTaskConfig(TaskComposerNode& node, const YAML::Node& config)
{
  if (YAML::Node n = config["conditional"])
    node.setConditional(n.as<bool>());

  if (YAML::Node n = config["abort_terminal"])
  {
    if (node.getType() != TaskComposerNodeType::GRAPH && node.getType() != TaskComposerNodeType::PIPELINE)
      throw std::runtime_error("YAML entry 'abort_terminal' is only supported for GRAPH and PIPELINE types");

    static_cast<TaskComposerGraph&>(node).setTerminalTriggerAbortByIndex(n.as<int>());
  }

  if (YAML::Node override_keys = config["overrides"])
  {
    if (node.getType() != TaskComposerNodeType::GRAPH && node.getType() != TaskComposerNodeType::PIPELINE)
      throw std::runtime_error("YAML entry 'overrides' is only supported for GRAPH and PIPELINE types");

    auto& task_graph = static_cast<TaskComposerGraph&>(node);
    if (YAML::Node n = override_keys["inputs"])
    {
      if (!n.IsMap())
        throw std::runtime_error("YAML entry 'overrides' inputs must be a map type");

      task_graph.setOverrideInputKeys(n.as<TaskComposerKeys>());
    }

    if (YAML::Node n = override_keys["outputs"])
    {
      if (!n.IsMap())
        throw std::runtime_error("YAML entry 'overrides' outputs must be a map type");

      task_graph.setOverrideOutputKeys(n.as<TaskComposerKeys>());
    }
  }
}
}  // namespace tesseract_planning
