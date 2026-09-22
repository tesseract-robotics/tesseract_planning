/**
 * @file yaml_utils.cpp
 * @brief YAML utility functions
 *
 * @author Levi Armstrong
 * @date October 12, 2025
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

#include <tesseract/task_composer/yaml_utils.h>
#include <tesseract/task_composer/yaml_extensions.h>
#include <tesseract/task_composer/task_composer_port_map.h>
#include <tesseract/task_composer/task_composer_node.h>
#include <tesseract/task_composer/task_composer_graph.h>
#include <tesseract/task_composer/task_composer_plugin_factory.h>

#include <tesseract/common/yaml_utils.h>
#include <tesseract/common/property_tree.h>
#include <tesseract/common/schema_registration.h>

#include <yaml-cpp/yaml.h>

namespace tesseract::task_composer
{
void loadSubTaskConfig(TaskComposerNode& node, const YAML::Node& config)
{
  if (node.getType() != TaskComposerNodeType::GRAPH && node.getType() != TaskComposerNodeType::PIPELINE)
    throw std::runtime_error("Sub task is only supported for GRAPH and PIPELINE types");

  if (YAML::Node n = config["conditional"])
    node.setConditional(n.as<bool>());

  auto& graph_node = static_cast<TaskComposerGraph&>(node);

  if (YAML::Node n = config["abort_terminal"])
    graph_node.setTerminalTriggerAbortByIndex(n.as<int>());

  if (YAML::Node override_config = config["override"])
  {
    if (YAML::Node n = override_config["inputs"])
      graph_node.setOverrideInputPortMappings(n.as<TaskComposerPortMap>());

    if (YAML::Node n = override_config["outputs"])
      graph_node.setOverrideOutputPortMappings(n.as<TaskComposerPortMap>());
  }
}

std::unique_ptr<TaskComposerNode> loadSubTask(const std::string& parent_name,
                                              const std::string& name,
                                              const YAML::Node& entry,
                                              const TaskComposerPluginFactory& plugin_factory)
{
  if (YAML::Node fn = entry["class"])
  {
    tesseract::common::PluginInfo plugin_info;
    plugin_info.class_name = fn.as<std::string>();
    if (YAML::Node cn = entry["config"])
      plugin_info.config = cn;

    std::unique_ptr<TaskComposerNode> task_node = plugin_factory.createTaskComposerNode(name, plugin_info);
    if (task_node == nullptr)
      throw std::runtime_error("Sub task for '" + parent_name + "' failed to create node '" + name + "'");

    return task_node;
  }

  if (YAML::Node tn = entry["task"])
  {
    auto task_name = tn.as<std::string>();
    std::unique_ptr<TaskComposerNode> task_node = plugin_factory.createTaskComposerNode(task_name);
    if (task_node == nullptr)
      throw std::runtime_error("Sub task for '" + parent_name + "' failed to create task '" + task_name +
                               "' for node '" + name + "'");

    task_node->setName(name);

    if (YAML::Node tc = entry["config"])
      loadSubTaskConfig(*task_node, tc);

    return task_node;
  }

  return plugin_factory.createTaskComposerNode(entry["task"].as<std::string>());
}

tesseract::common::PropertyTree subTaskConfigSchema()
{
  using namespace tesseract::common;
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .boolean("conditional").done()
      .int32("abort_terminal").done()
      .container("override")
        .customType("inputs", "tesseract::task_composer::TaskComposerPortMap")
            .validator(validateCustomType).done()
        .customType("outputs", "tesseract::task_composer::TaskComposerPortMap")
            .validator(validateCustomType).done()
      .done()
      .build();
  // clang-format on
}

tesseract::common::PropertyTree subTaskSchema()
{
  using namespace tesseract::common;
  // clang-format off
  return PropertyTreeBuilder()
      .oneOf()
        .customType("by_class", "tesseract::task_composer::TaskComposerNodeFactory")
          .acceptsDerivedTypes().done()
        .container("by_task")
          .string("task").required().done()
          .customType("config", SUB_TASK_CONFIG_SCHEMA_KEY)
              .validator(validateCustomType).done()
        .done()
      .build();
  // clang-format on
}

tesseract::common::PropertyTree graphEdgeSchema()
{
  using namespace tesseract::common;
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .string("source").required().done()
      .customType("destinations", STRING_OR_STRING_LIST_SCHEMA_KEY).required()
        .validator(validateCustomType).done()
      .build();
  // clang-format on
}

}  // namespace tesseract::task_composer

TESSERACT_SCHEMA_REGISTER(tesseract::task_composer::GraphEdge, tesseract::task_composer::graphEdgeSchema);
TESSERACT_SCHEMA_REGISTER(tesseract::task_composer::SubTaskConfig, tesseract::task_composer::subTaskConfigSchema);
TESSERACT_SCHEMA_REGISTER(tesseract::task_composer::SubTask, tesseract::task_composer::subTaskSchema);
