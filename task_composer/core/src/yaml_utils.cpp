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
#include <tesseract/task_composer/task_composer_keys.h>
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

  if (YAML::Node override_keys = config["override"])
  {
    if (YAML::Node n = override_keys["inputs"])
    {
      if (!n.IsMap())
        throw std::runtime_error("YAML entry 'override' inputs must be a map type");

      graph_node.setOverrideInputKeys(n.as<TaskComposerKeys>());
    }

    if (YAML::Node n = override_keys["outputs"])
    {
      if (!n.IsMap())
        throw std::runtime_error("YAML entry 'override' outputs must be a map type");

      graph_node.setOverrideOutputKeys(n.as<TaskComposerKeys>());
    }
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
    {
      static const std::set<std::string> tasks_expected_keys{ "conditional", "abort_terminal", "override" };
      tesseract::common::checkForUnknownKeys(tc, tasks_expected_keys);

      loadSubTaskConfig(*task_node, tc);
    }

    return task_node;
  }

  throw std::runtime_error("Sub task for '" + parent_name + "' node '" + name + "' missing 'class' or 'task' entry");
}

void validateSubTask(const std::string& parent_name, const std::string& key, const YAML::Node& node)
{
  if (!node.IsMap())
    throw std::runtime_error("Sub task for '" + parent_name + "' node '" + key + "' should be a map");

  bool is_class{ false };
  bool is_task{ false };
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
  {
    auto key = it->first.as<std::string>();
    if (key == "class")
      is_class = true;
    else if (key == "task")
      is_task = true;
  }

  if (is_class && is_task)
    throw std::runtime_error("Sub task for '" + parent_name + "' node '" + key + "' has both 'class' and 'task' entry");

  if (!is_class && !is_task)
    throw std::runtime_error("Sub task for '" + parent_name + "' node '" + key + "' missing 'class' or 'task' entry");
}

tesseract::common::PropertyTree subTaskConfigSchema()
{
  using namespace tesseract::common;
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .boolean("conditional").done()
      .integer("abort_terminal").done()
      .container("override")
        .customType("inputs", "tesseract::task_composer::TaskComposerKeys")
            .validator(validateCustomType).done()
        .customType("outputs", "tesseract::task_composer::TaskComposerKeys")
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
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .beginOneOf()
        .customType("by_class", "tesseract::task_composer::TaskComposerNodeFactory")
            .acceptsDerivedTypes().validator(validateCustomType).done()
        .container("by_task")
          .string("task").required().done()
          .customType("config", SUB_TASK_CONFIG_SCHEMA_KEY)
              .validator(validateCustomType).done()
        .done()
      .endOneOf()
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
      .customType("destinations", property_type::createList(property_type::STRING)).required().done()
      .build();
  // clang-format on
}

}  // namespace tesseract::task_composer

TESSERACT_SCHEMA_REGISTER(tesseract::task_composer::GraphEdge, tesseract::task_composer::graphEdgeSchema);
TESSERACT_SCHEMA_REGISTER(tesseract::task_composer::SubTaskConfig, tesseract::task_composer::subTaskConfigSchema);
TESSERACT_SCHEMA_REGISTER(tesseract::task_composer::SubTask, tesseract::task_composer::subTaskSchema);
