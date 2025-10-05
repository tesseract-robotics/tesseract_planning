/**
 * @file for_each_task.cpp
 * @brief This runs the same task for each element in a vector
 *
 * @author Matthew Powelson
 * @date July 15, 2020
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>

#include <tesseract_common/serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/nodes/for_each_task.h>
#include <tesseract_task_composer/core/nodes/start_task.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>

namespace
{
tesseract_planning::ForEachTask::TaskFactoryResults
createTask(const std::string& name,
           const std::string& task_name,
           const std::string& input_port,
           const std::string& output_port,
           const std::map<std::string, std::string>& remapping,
           const std::vector<std::string>& indexing,
           const tesseract_planning::TaskComposerPluginFactory& plugin_factory,
           std::size_t index)
{
  tesseract_planning::ForEachTask::TaskFactoryResults tf_results;
  tf_results.node = plugin_factory.createTaskComposerNode(task_name);
  tf_results.node->setName(name);

  if (!remapping.empty())
  {
    tf_results.node->renameInputKeys(remapping);
    tf_results.node->renameOutputKeys(remapping);
  }

  if (!indexing.empty())
  {
    std::map<std::string, std::string> renaming;
    for (const auto& x : indexing)
      renaming[x] = x + std::to_string(index);

    tf_results.node->renameInputKeys(renaming);
    tf_results.node->renameOutputKeys(renaming);
  }

  tf_results.input_key = tf_results.node->getInputKeys().get(input_port);
  tf_results.output_key = tf_results.node->getOutputKeys().get(output_port);

  return tf_results;
}
}  // namespace

namespace tesseract_planning
{
// Requried
const std::string ForEachTask::INOUT_PORT = "container";

ForEachTask::ForEachTask() : TaskComposerTask("ForEachTask", ForEachTask::ports(), true) {}

ForEachTask::ForEachTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& plugin_factory)
  : TaskComposerTask(std::move(name), ForEachTask::ports(), config)
{
  if (YAML::Node operation_config = config["operation"])
  {
    std::string task_name;
    bool has_abort_terminal_entry{ false };
    int abort_terminal_index{ -1 };
    std::vector<std::string> indexing;
    std::map<std::string, std::string> remapping;

    if (YAML::Node n = operation_config["task"])
      task_name = n.as<std::string>();
    else
      throw std::runtime_error("ForEachTask, entry 'operation' missing 'task' entry");

    if (YAML::Node task_config = operation_config["config"])
    {
      if (YAML::Node n = task_config["abort_terminal"])
      {
        has_abort_terminal_entry = true;
        abort_terminal_index = n.as<int>();
      }

      if (YAML::Node n = task_config["remapping"])
        remapping = n.as<std::map<std::string, std::string>>();

      if (YAML::Node n = task_config["indexing"])
        indexing = n.as<std::vector<std::string>>();
      else
        throw std::runtime_error("ForEachTask, entry 'operation' missing 'indexing' entry");

      if (YAML::Node n = task_config["input_port"])
        task_input_port_ = n.as<std::string>();
      else
        throw std::runtime_error("ForEachTask, entry 'operation' missing 'input_port' entry");

      if (YAML::Node n = task_config["output_port"])
        task_output_port_ = n.as<std::string>();
      else
        throw std::runtime_error("ForEachTask, entry 'operation' missing 'output_port' entry");
    }
    else
    {
      throw std::runtime_error("ForEachTask, entry 'operation' missing 'config' entry");
    }

    if (has_abort_terminal_entry)
    {
      task_factory_ = [task_name,
                       input_port = task_input_port_,
                       output_port = task_output_port_,
                       abort_terminal_index,
                       remapping,
                       indexing,
                       &plugin_factory](const std::string& name, std::size_t index) {
        auto tr = createTask(name, task_name, input_port, output_port, remapping, indexing, plugin_factory, index);

        static_cast<TaskComposerGraph&>(*tr.node).setTerminalTriggerAbortByIndex(abort_terminal_index);

        return tr;
      };
    }
    else
    {
      task_factory_ = [task_name,
                       input_port = task_input_port_,
                       output_port = task_output_port_,
                       remapping,
                       indexing,
                       &plugin_factory](const std::string& name, std::size_t index) {
        return createTask(name, task_name, input_port, output_port, remapping, indexing, plugin_factory, index);
      };
    }
  }
  else
  {
    throw std::runtime_error("ForEachTask: missing 'sub_task' entry");
  }
}

TaskComposerNodePorts ForEachTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INOUT_PORT] = TaskComposerNodePorts::SINGLE;
  ports.output_required[INOUT_PORT] = TaskComposerNodePorts::SINGLE;
  return ports;
}

bool ForEachTask::operator==(const ForEachTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool ForEachTask::operator!=(const ForEachTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void ForEachTask::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
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

  auto& inputs = input_data_poly.template as<std::vector<tesseract_common::AnyPoly>>();

  // Task and Task Data Storage
  TaskComposerGraph task_graph(name_ + " (Subgraph)");
  task_graph.setParentUUID(uuid_);
  TaskComposerKeys task_input_keys{ input_keys_ };
  TaskComposerKeys task_output_keys{ output_keys_ };
  task_input_keys.remove(INOUT_PORT);
  task_output_keys.remove(INOUT_PORT);

  std::vector<std::string> input_keys;
  std::vector<std::string> output_keys;
  input_keys.reserve(inputs.size());
  output_keys.reserve(inputs.size());

  auto task_graph_data_storage = std::make_shared<TaskComposerDataStorage>(uuid_str_);
  task_graph_data_storage->copyData(*context.data_storage, task_input_keys);

  // Start Task
  auto start_task = std::make_unique<StartTask>();
  auto start_uuid = task_graph.addNode(std::move(start_task));

  std::vector<std::pair<boost::uuids::uuid, std::pair<std::string, std::string>>> tasks;
  tasks.reserve(inputs.size());

  // Generate all of the tasks. They don't depend on anything
  for (std::size_t idx = 0; idx < inputs.size(); ++idx)
  {
    const std::string task_name = "Task #" + std::to_string(idx + 1);
    auto task_results = task_factory_(task_name, idx + 1);
    task_results.node->setConditional(false);
    auto task_uuid = task_graph.addNode(std::move(task_results.node));
    tasks.emplace_back(task_uuid, std::make_pair(task_results.input_key, task_results.output_key));
    input_keys.push_back(task_results.input_key);
    output_keys.push_back(task_results.output_key);
    task_graph_data_storage->setData(task_results.input_key, inputs[idx]);
    task_graph.addEdges(start_uuid, { task_uuid });
  }

  if (!executor.has_value())
    throw std::runtime_error("ForEachTask, executor is null!");

  // Set graph input and output keys
  task_input_keys.add(task_input_port_, input_keys);
  task_output_keys.add(task_output_port_, output_keys);
  task_graph.setInputKeys(task_input_keys);
  task_graph.setOutputKeys(task_output_keys);

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

  std::vector<tesseract_common::AnyPoly> output;
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

  setData(context, INOUT_PORT, tesseract_common::AnyPoly(output));

  info.color = "green";
  info.status_code = 1;
  info.status_message = "Successful";
  info.return_value = 1;
  return info;
}

void ForEachTask::checkTaskInput(const tesseract_common::AnyPoly& input)
{
  // -------------
  // Check Input
  // -------------
  if (input.isNull())
    throw std::runtime_error("ForEachTask, input is null");

  if (input.getType() != std::type_index(typeid(std::vector<tesseract_common::AnyPoly>)))
    throw std::runtime_error("ForEachTask, input is not a std::vector<tesseract_common::AnyPoly>");
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ForEachTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ForEachTask)
