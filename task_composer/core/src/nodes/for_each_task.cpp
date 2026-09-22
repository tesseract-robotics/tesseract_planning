/**
 * @file for_each_task.cpp
 * @brief This runs the same task for each element in a vector
 *
 * @author Matthew Powelson
 * @date July 15, 2020
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>

#include <tesseract/common/yaml_utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/property_tree.h>

#include <tesseract/task_composer/nodes/for_each_task.h>
#include <tesseract/task_composer/nodes/start_task.h>
#include <tesseract/task_composer/task_composer_context.h>
#include <tesseract/task_composer/task_composer_data_storage.h>
#include <tesseract/task_composer/task_composer_future.h>
#include <tesseract/task_composer/task_composer_executor.h>
#include <tesseract/task_composer/task_composer_plugin_factory.h>
#include <tesseract/task_composer/task_composer_graph.h>
#include <tesseract/task_composer/task_composer_node_info.h>
#include <tesseract/task_composer/yaml_utils.h>

namespace tesseract::task_composer
{
namespace
{
void validateOperationSubTask(const tesseract::common::PropertyTree& node,
                              const std::string& path,
                              std::vector<std::string>& errors)
{
  YAML::Node config = node.toYAML();
  if (!config || !config.IsMap())
    return;

  config.remove("input_port");
  config.remove("output_port");

  auto schema = subTaskSchema();
  try
  {
    for (std::string error : schema.applyConfig(config))
    {
      const std::size_t separator = error.find(':');
      if (separator != std::string::npos)
        error.erase(0, separator + 1);
      std::string full_error = path;
      full_error += ":";
      full_error += error;
      errors.push_back(std::move(full_error));
    }
  }
  catch (const std::exception& e)
  {
    std::string error = path;
    error += ": ";
    error += e.what();
    errors.push_back(std::move(error));
  }
}
}  // namespace

// Requried

ForEachTask::ForEachTask() : TaskComposerTask("ForEachTask", ForEachTask::ports(), true) {}

ForEachTask::ForEachTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& plugin_factory)
  : TaskComposerTask(std::move(name), ForEachTask::ports(), config)
{
  static const std::string operation_key{ "operation" };
  const YAML::Node operation_config = config[operation_key];
  task_input_port_ = operation_config["input_port"].as<std::string>();
  task_output_port_ = operation_config["output_port"].as<std::string>();

  task_factory_ = [operation_config, input_port = task_input_port_, output_port = task_output_port_, &plugin_factory](
                      const std::string& parent_name, const std::string& name, std::size_t index) {
    ForEachTask::TaskFactoryResults tr;
    tr.node = loadSubTask(parent_name, name, operation_config, plugin_factory);
    tr.node->setConditional(false);
    tr.input_key = tr.node->getInputPortMappings().single(input_port) + std::to_string(index);
    tr.output_key = tr.node->getOutputPortMappings().single(output_port) + std::to_string(index);

    if (tr.node->getType() == TaskComposerNodeType::TASK)
    {
      TaskComposerPortMap input_port_mappings = tr.node->getInputPortMappings();
      TaskComposerPortMap output_port_mappings = tr.node->getOutputPortMappings();
      input_port_mappings.set(input_port, tr.input_key);
      output_port_mappings.set(output_port, tr.output_key);
      tr.node->setPortMappings(std::move(input_port_mappings), std::move(output_port_mappings));
    }
    else
    {
      auto& graph_node = static_cast<TaskComposerGraph&>(*tr.node);
      TaskComposerPortMap override_input_port_mappings;
      TaskComposerPortMap override_output_port_mappings;
      override_input_port_mappings.set(input_port, tr.input_key);
      override_output_port_mappings.set(output_port, tr.output_key);
      graph_node.setOverrideInputPortMappings(override_input_port_mappings);
      graph_node.setOverrideOutputPortMappings(override_output_port_mappings);
    }

    return tr;
  };
}

const TaskComposerNodePorts& ForEachTask::ports()
{
  static const TaskComposerNodePorts ports = []() {
    TaskComposerNodePorts ports;
    ports.addRequiredInput(INOUT_PORT);
    ports.addRequiredOutput(INOUT_PORT);
    return ports;
  }();
  return ports;
}

TaskComposerNodeInfo ForEachTask::runImpl(TaskComposerContext& context, OptionalTaskComposerExecutor executor) const
{
  TaskComposerNodeInfo info(*this);
  info.return_value = 0;
  info.status_code = 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = getData(context, INOUT_PORT);
  try
  {
    checkTaskInput(input_data_poly);
  }
  catch (const std::exception& e)
  {
    info.status_message = e.what();
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    return info;
  }

  auto& inputs = input_data_poly.template as<std::vector<tesseract::common::AnyPoly>>();

  // Task and Task Data Storage
  TaskComposerGraph task_graph(name_ + " (Subgraph)", uuid_);

  // Create subgraph task input and output port mappings.
  // Copy the parent mappings, then remove the program port mapping that will be assigned later.
  TaskComposerPortMap task_input_port_mappings{ input_port_mappings_ };
  TaskComposerPortMap task_output_port_mappings{ output_port_mappings_ };
  task_input_port_mappings.erase(INOUT_PORT);
  task_output_port_mappings.erase(INOUT_PORT);

  // Create a sub graph data storage and copy the input data relevant to this graph.
  const TaskComposerDataStorage::Ptr parent_data_storage = getDataStorage(context);
  auto task_graph_data_storage = std::make_shared<TaskComposerDataStorage>(uuid_str_);
  task_graph_data_storage->copyAsInputData(*parent_data_storage, task_input_port_mappings, {});

  // Create container to store the sub graph program port keys
  std::vector<std::string> input_storage_keys;
  std::vector<std::string> output_storage_keys;
  input_storage_keys.reserve(inputs.size());
  output_storage_keys.reserve(inputs.size());

  // Start Task
  auto start_task = std::make_unique<StartTask>();
  auto start_uuid = task_graph.addNode(std::move(start_task));

  std::vector<std::pair<boost::uuids::uuid, std::pair<std::string, std::string>>> tasks;
  tasks.reserve(inputs.size());

  // Generate all of the tasks. They don't depend on anything
  for (std::size_t idx = 0; idx < inputs.size(); ++idx)
  {
    const std::string task_name = "Task #" + std::to_string(idx + 1);
    auto task_results = task_factory_(name_, task_name, idx + 1);

    auto task_uuid = task_graph.addNode(std::move(task_results.node));
    tasks.emplace_back(task_uuid, std::make_pair(task_results.input_key, task_results.output_key));
    input_storage_keys.push_back(task_results.input_key);
    output_storage_keys.push_back(task_results.output_key);
    task_graph_data_storage->setData(task_results.input_key, inputs[idx]);
    task_graph.addEdges(start_uuid, { task_uuid });
  }

  if (!executor.has_value())
    throw std::runtime_error("ForEachTask, executor is null!");

  // Set subgraph input and output port mappings.
  task_input_port_mappings.set(task_input_port_, input_storage_keys);
  task_output_port_mappings.set(task_output_port_, output_storage_keys);
  task_graph.setPortMappings(std::move(task_input_port_mappings), std::move(task_output_port_mappings));

  // Store sub data storage in parent data storage
  context.data_storage->setData(uuid_str_, task_graph_data_storage);

  TaskComposerFuture::UPtr future = executor.value().get().run(task_graph, context.shared_from_this());
  future->wait();

  auto info_map = context.task_infos->getInfoMap();
  if (context.dotgraph)
  {
    std::stringstream dot_graph;
    dot_graph << "subgraph cluster_" << toString(uuid_) << " {\n color=black;\n label = \"" << name_ << "\\n("
              << uuid_str_ << ")\";";
    task_graph.dump(dot_graph, this, info_map);  // dump the graph including dynamic tasks
    dot_graph << "}\n";
    info.dotgraph = dot_graph.str();
  }

  if (context.isAborted())
  {
    info.status_message = "ForEachTask subgraph failed";
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    return info;
  }

  std::vector<tesseract::common::AnyPoly> output;
  output.reserve(inputs.size());
  for (const auto& task : tasks)
  {
    std::optional<TaskComposerNodeInfo> task_info = context.task_infos->getInfo(task.first);
    if (!task_info.has_value())
      continue;

    // Zero is always reserved for error
    if (task_info.value().return_value > 0)
      output.emplace_back(task_graph_data_storage->getData(task.second.second));
  }

  setData(context, INOUT_PORT, tesseract::common::AnyPoly(output));

  info.color = "green";
  info.status_code = 1;
  info.status_message = "Successful";
  info.return_value = 1;
  return info;
}

tesseract::common::PropertyTree ForEachTask::schema()
{
  using namespace tesseract::common;
  // clang-format off
  auto schema = PropertyTreeBuilder()
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .compose(TaskComposerTask::schema(ForEachTask::ports()))
      .container("operation").required()
        // Validates class/config using the registered TaskComposerNodeFactory
        // derived schema, or validates a named task's SubTaskConfig.
        .validator(validateOperationSubTask)
        .string("input_port").required().minimumLength(1).done()
        .string("output_port").required().minimumLength(1).done()
        .string("class").minimumLength(1).done()
        .string("task").minimumLength(1).done()
      .done()
      .build();
  // clang-format on

  // The concrete plugin or named-task schema owns the shape of this field.
  schema.at("operation")["config"];
  return schema;
}

void ForEachTask::checkTaskInput(const tesseract::common::AnyPoly& input)
{
  // -------------
  // Check Input
  // -------------
  if (input.isNull())
    throw std::runtime_error("ForEachTask, input is null");

  if (input.getType() != std::type_index(typeid(std::vector<tesseract::common::AnyPoly>)))
    throw std::runtime_error("ForEachTask, input is not a std::vector<tesseract::common::AnyPoly>");
}

}  // namespace tesseract::task_composer
