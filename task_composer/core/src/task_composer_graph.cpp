/**
 * @file task_composer_graph.cpp
 * @brief A task graph
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 *
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>
#include <boost/uuid/uuid_io.hpp>

#include <tesseract/common/plugin_info.h>
#include <tesseract/common/yaml_utils.h>
#include <tesseract/common/yaml_extensions.h>
#include <tesseract/common/stopwatch.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/task_composer/task_composer_port_map.h>
#include <tesseract/task_composer/task_composer_context.h>
#include <tesseract/task_composer/task_composer_future.h>
#include <tesseract/task_composer/task_composer_executor.h>
#include <tesseract/task_composer/task_composer_graph.h>
#include <tesseract/task_composer/task_composer_task.h>
#include <tesseract/task_composer/task_composer_pipeline.h>
#include <tesseract/task_composer/task_composer_node_info.h>
#include <tesseract/task_composer/task_composer_plugin_factory.h>
#include <tesseract/task_composer/yaml_extensions.h>
#include <tesseract/task_composer/yaml_utils.h>
#include <tesseract/common/property_tree.h>

#include <set>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace tesseract::task_composer
{
namespace
{
void validateOverrides(const TaskComposerPortMap& mappings,
                       const TaskComposerPortMap& overrides,
                       std::string_view direction)
{
  for (const auto& [port, override_mapping] : overrides.data())
  {
    if (!mappings.contains(port))
      throw std::runtime_error("Graph " + std::string(direction) + " override references undeclared port '" + port +
                               "'");

    const auto& mapping = mappings.at(port);
    if (std::holds_alternative<std::string>(mapping) != std::holds_alternative<std::string>(override_mapping))
      throw std::runtime_error("Graph " + std::string(direction) + " override for port '" + port +
                               "' has different cardinality than its mapping");

    if (std::holds_alternative<std::vector<std::string>>(mapping) &&
        std::get<std::vector<std::string>>(mapping).size() !=
            std::get<std::vector<std::string>>(override_mapping).size())
    {
      throw std::runtime_error("Graph " + std::string(direction) + " override for port '" + port +
                               "' must contain the same number of keys as its mapping");
    }
  }
}

void collectMappingKeys(const TaskComposerPortMap& mappings,
                        const TaskComposerPortMap* overrides,
                        std::set<std::string>& keys)
{
  for (const auto& [port, mapping] : mappings.data())
  {
    const auto& effective_mapping = overrides != nullptr && overrides->contains(port) ? overrides->at(port) : mapping;
    if (std::holds_alternative<std::string>(effective_mapping))
    {
      keys.insert(std::get<std::string>(effective_mapping));
    }
    else
    {
      const auto& multiple = std::get<std::vector<std::string>>(effective_mapping);
      keys.insert(multiple.begin(), multiple.end());
    }
  }
}
}  // namespace

TaskComposerGraph::TaskComposerGraph(std::string name, boost::uuids::uuid parent_uuid)
  : TaskComposerGraph(std::move(name), TaskComposerNodeType::GRAPH, false)
{
  parent_uuid_ = parent_uuid;
  parent_uuid_str_ = boost::uuids::to_string(parent_uuid);
}

TaskComposerGraph::TaskComposerGraph(std::string name, TaskComposerNodeType type, bool conditional)
  : TaskComposerNode(std::move(name), type, DynamicPortsTag{}, conditional)
{
}

TaskComposerGraph::TaskComposerGraph(std::string name,
                                     const YAML::Node& config,
                                     const TaskComposerPluginFactory& plugin_factory)
  : TaskComposerGraph(std::move(name), TaskComposerNodeType::GRAPH, config, plugin_factory)
{
  if (conditional_)
    throw std::runtime_error("TaskComposerGraph, conditional should not be true");
}

TaskComposerGraph::TaskComposerGraph(std::string name,
                                     TaskComposerNodeType type,
                                     const YAML::Node& config,
                                     const TaskComposerPluginFactory& plugin_factory)
  : TaskComposerNode(std::move(name), type, DynamicPortsTag{}, config)
{
  std::unordered_map<std::string, boost::uuids::uuid> node_uuids;
  YAML::Node nodes = config["nodes"];

  for (auto node_it = nodes.begin(); node_it != nodes.end(); ++node_it)
  {
    const auto node_name = node_it->first.as<std::string>();
    node_uuids[node_name] = addNode(loadSubTask(name_, node_name, node_it->second, plugin_factory));
  }

  YAML::Node edges = config["edges"];
  for (auto edge_it = edges.begin(); edge_it != edges.end(); ++edge_it)
  {
    const YAML::Node& edge = *edge_it;

    const auto source = edge["source"].as<std::string>();
    std::vector<std::string> destinations;
    const YAML::Node destination_config = edge["destinations"];
    if (destination_config.IsSequence())
      destinations = destination_config.as<std::vector<std::string>>();
    else
      destinations.push_back(destination_config.as<std::string>());

    auto source_it = node_uuids.find(source);
    if (source_it == node_uuids.end())
      throw std::runtime_error("Task Composer Graph '" + name_ + "' failed to find source '" + source + "'");

    std::vector<boost::uuids::uuid> destination_uuids;
    destination_uuids.reserve(destinations.size());
    for (const auto& destination : destinations)
    {
      auto destination_it = node_uuids.find(destination);
      if (destination_it == node_uuids.end())
        throw std::runtime_error("Task Composer Graph '" + name_ + "' failed to find destination '" + destination +
                                 "'");

      destination_uuids.push_back(destination_it->second);
    }

    addEdges(source_it->second, destination_uuids);
  }

  auto terminals = config["terminals"].as<std::vector<std::string>>();
  terminals_.clear();
  terminals_.reserve(terminals.size());
  for (const auto& terminal : terminals)
  {
    auto terminal_it = node_uuids.find(terminal);
    if (terminal_it == node_uuids.end())
      throw std::runtime_error("Task Composer Graph '" + name_ + "' failed to find terminal '" + terminal + "'");

    terminals_.push_back(terminal_it->second);
  }

  auto is_valid = TaskComposerGraph::isValid();
  if (!is_valid.first)
    throw std::runtime_error(is_valid.second);
}

TaskComposerNodeInfo TaskComposerGraph::runImpl(TaskComposerContext& context,
                                                OptionalTaskComposerExecutor executor) const
{
  const auto validity = isValid();
  if (!validity.first)
    throw std::runtime_error(validity.second);

  if (terminals_.empty())
    throw std::runtime_error("TaskComposerGraph, with name '" + name_ + "' does not have terminals!");

  tesseract::common::Stopwatch stopwatch;
  stopwatch.start();

  if (!executor.has_value())
    throw std::runtime_error("TaskComposerGraph, the optional executor is null!");

  // Run
  TaskComposerFuture::UPtr future = executor.value().get().run(*this, context.shared_from_this());
  future->wait();

  TaskComposerNodeInfo info(*this);
  auto info_map = context.task_infos->getInfoMap();
  if (context.dotgraph)
  {
    std::stringstream dot_graph;
    dot_graph << "subgraph cluster_" << toString(uuid_) << " {\n color=black;\n label = \"" << name_ << "\\n("
              << uuid_str_ << ")\";\n";
    dump(dot_graph, this, info_map);  // dump the graph including dynamic tasks
    dot_graph << "}\n";
    info.dotgraph = dot_graph.str();
  }

  for (std::size_t i = 0; i < terminals_.size(); ++i)
  {
    auto node_info = context.task_infos->getInfo(terminals_[i]);
    if (node_info.has_value())
    {
      stopwatch.stop();
      info.input_port_mappings = input_port_mappings_;
      info.output_port_mappings = output_port_mappings_;
      info.return_value = static_cast<int>(i);
      info.color = node_info->color;
      info.status_code = node_info->status_code;
      info.status_message = node_info->status_message;
      info.elapsed_time = stopwatch.elapsedSeconds();
      return info;
    }
  }

  throw std::runtime_error("TaskComposerGraph, with name '" + name_ + "' has no node info for any of the leaf nodes!");
}

boost::uuids::uuid TaskComposerGraph::getRootNode() const
{
  boost::uuids::uuid root_node{};
  for (const auto& pair : nodes_)
  {
    if (pair.second->getInboundEdges().empty())
    {
      root_node = pair.first;
      break;
    }
  }
  return root_node;
}

boost::uuids::uuid TaskComposerGraph::addNode(std::unique_ptr<TaskComposerNode> task_node)
{
  boost::uuids::uuid uuid = task_node->getUUID();
  task_node->parent_uuid_ = uuid_;
  task_node->parent_uuid_str_ = uuid_str_;
  nodes_[uuid] = std::move(task_node);
  return uuid;
}

boost::uuids::uuid TaskComposerGraph::addNodePython(std::shared_ptr<TaskComposerNode> task_node)
{
  boost::uuids::uuid uuid = task_node->getUUID();
  task_node->parent_uuid_ = uuid_;
  task_node->parent_uuid_str_ = uuid_str_;
  nodes_[uuid] = std::move(task_node);
  return uuid;
}

void TaskComposerGraph::addEdges(boost::uuids::uuid source, std::vector<boost::uuids::uuid> destinations)
{
  TaskComposerNode::Ptr& node = nodes_.at(source);

  node->outbound_edges_.insert(node->outbound_edges_.end(), destinations.begin(), destinations.end());
  for (const auto& d : destinations)
    nodes_.at(d)->inbound_edges_.push_back(source);
}

std::map<boost::uuids::uuid, std::shared_ptr<const TaskComposerNode>> TaskComposerGraph::getNodes() const
{
  return std::map<boost::uuids::uuid, std::shared_ptr<const TaskComposerNode>>{ nodes_.begin(), nodes_.end() };
}

std::shared_ptr<const TaskComposerNode> TaskComposerGraph::getNodeByName(const std::string& name) const
{
  for (const auto& pair : nodes_)
  {
    if (pair.second->getName() == name)
      return pair.second;
  }

  return nullptr;
}

void TaskComposerGraph::setTerminals(std::vector<boost::uuids::uuid> terminals)
{
  for (const auto& terminal : terminals)
  {
    auto it = nodes_.find(terminal);
    if (it == nodes_.end())
      throw std::runtime_error("TaskComposerGraph, terminal node does not exist!");

    if (!it->second->getOutboundEdges().empty())
      throw std::runtime_error("TaskComposerGraph, terminal node has outbound edges!");
  }

  terminals_ = std::move(terminals);
}

std::vector<boost::uuids::uuid> TaskComposerGraph::getTerminals() const { return terminals_; }

void TaskComposerGraph::setTerminalTriggerAbort(boost::uuids::uuid terminal)
{
  if (!terminal.is_nil())
  {
    abort_terminal_ = -1;
    for (std::size_t i = 0; i < terminals_.size(); ++i)
    {
      const boost::uuids::uuid& uuid = terminals_[i];
      if (uuid == terminal)
      {
        abort_terminal_ = static_cast<int>(i);
        auto& n = nodes_.at(terminal);
        if (n->getType() == TaskComposerNodeType::TASK)
          static_cast<TaskComposerTask&>(*n).setTriggerAbort(true);
        else
          throw std::runtime_error("Tasks can only trigger abort!");

        break;
      }
    }
    if (abort_terminal_ < 0)
      throw std::runtime_error("Task with uuid: " + boost::uuids::to_string(terminal) + " is not a terminal node");
  }
  else
  {
    abort_terminal_ = -1;
    for (const auto& t : terminals_)
    {
      auto& n = nodes_.at(t);
      if (n->getType() == TaskComposerNodeType::TASK)
        static_cast<TaskComposerTask&>(*n).setTriggerAbort(false);
    }
  }
}

void TaskComposerGraph::setTerminalTriggerAbortByIndex(int terminal_index)
{
  if (terminal_index >= 0)
  {
    abort_terminal_ = terminal_index;
    auto& n = nodes_.at(terminals_.at(static_cast<std::size_t>(terminal_index)));
    if (n->getType() == TaskComposerNodeType::TASK)
      static_cast<TaskComposerTask&>(*n).setTriggerAbort(true);
    else
      throw std::runtime_error("Tasks can only trigger abort!");
  }
  else
  {
    abort_terminal_ = -1;
    for (const auto& terminal : terminals_)
    {
      auto& n = nodes_.at(terminal);
      if (n->getType() == TaskComposerNodeType::TASK)
        static_cast<TaskComposerTask&>(*n).setTriggerAbort(false);
    }
  }
}

boost::uuids::uuid TaskComposerGraph::getAbortTerminal() const
{
  if (abort_terminal_ >= 0)
    return terminals_.at(static_cast<std::size_t>(abort_terminal_));

  return {};
}

int TaskComposerGraph::getAbortTerminalIndex() const { return abort_terminal_; }

void TaskComposerGraph::setPortMappings(TaskComposerPortMap input_port_mappings,
                                        TaskComposerPortMap output_port_mappings)
{
  validateOverrides(input_port_mappings, override_input_port_mappings_, "input");
  validateOverrides(output_port_mappings, override_output_port_mappings_, "output");
  TaskComposerNode::setPortMappings(std::move(input_port_mappings), std::move(output_port_mappings));
}

void TaskComposerGraph::setOverrideInputPortMappings(TaskComposerPortMap override_input_port_mappings)
{
  validateOverrides(input_port_mappings_, override_input_port_mappings, "input");
  override_input_port_mappings_ = std::move(override_input_port_mappings);
}

void TaskComposerGraph::setOverrideOutputPortMappings(TaskComposerPortMap override_output_port_mappings)
{
  validateOverrides(output_port_mappings_, override_output_port_mappings, "output");
  override_output_port_mappings_ = std::move(override_output_port_mappings);
}

const TaskComposerPortMap& TaskComposerGraph::getOverrideInputPortMappings() const
{
  return override_input_port_mappings_;
}

const TaskComposerPortMap& TaskComposerGraph::getOverrideOutputPortMappings() const
{
  return override_output_port_mappings_;
}

std::pair<bool, std::string> TaskComposerGraph::isValid() const
{
  try
  {
    validateOverrides(input_port_mappings_, override_input_port_mappings_, "input");
    validateOverrides(output_port_mappings_, override_output_port_mappings_, "output");
  }
  catch (const std::exception& exception)
  {
    return { false, "Task Composer Graph '" + name_ + "': " + exception.what() };
  }

  int root_node_cnt{ 0 };
  for (const auto& pair : nodes_)
  {
    auto node_inbound_edges = pair.second->getInboundEdges();
    if (node_inbound_edges.empty())
      root_node_cnt++;

    if (root_node_cnt > 1)
      return { false, "Task Composer Graph '" + name_ + "' has multiple root nodes" };
  }

  for (const auto& terminal : terminals_)
  {
    if (!nodes_.at(terminal)->getOutboundEdges().empty())
      return { false, "Task Composer Graph '" + name_ + "' has terminal node with outbound edges" };
  }

  return validateDataFlow();
}

std::pair<bool, std::string> TaskComposerGraph::validateDataFlow() const
{
  struct NodeDataFlow
  {
    std::set<std::string> inputs;
    std::set<std::string> outputs;
    std::vector<boost::uuids::uuid> predecessors;
  };

  std::set<std::string> graph_inputs;
  collectMappingKeys(input_port_mappings_, nullptr, graph_inputs);

  std::map<boost::uuids::uuid, NodeDataFlow> data_flow;
  for (const auto& [uuid, node] : nodes_)
  {
    const TaskComposerPortMap* input_overrides{ nullptr };
    const TaskComposerPortMap* output_overrides{ nullptr };
    if (node->getType() == TaskComposerNodeType::GRAPH || node->getType() == TaskComposerNodeType::PIPELINE)
    {
      const auto& graph_node = static_cast<const TaskComposerGraph&>(*node);
      try
      {
        validateOverrides(graph_node.getInputPortMappings(), graph_node.getOverrideInputPortMappings(), "input");
        validateOverrides(graph_node.getOutputPortMappings(), graph_node.getOverrideOutputPortMappings(), "output");
      }
      catch (const std::exception& exception)
      {
        return { false, "Task Composer Graph '" + name_ + "' child '" + node->getName() + "': " + exception.what() };
      }
      input_overrides = &graph_node.getOverrideInputPortMappings();
      output_overrides = &graph_node.getOverrideOutputPortMappings();
    }

    auto& node_data = data_flow[uuid];
    collectMappingKeys(node->getInputPortMappings(), input_overrides, node_data.inputs);
    collectMappingKeys(node->getOutputPortMappings(), output_overrides, node_data.outputs);
    node_data.predecessors = node->getInboundEdges();
  }

  std::set<std::string> produced_keys = graph_inputs;
  for (const auto& [uuid, node_data] : data_flow)
  {
    static_cast<void>(uuid);
    produced_keys.insert(node_data.outputs.begin(), node_data.outputs.end());
  }

  for (const auto& [uuid, node] : nodes_)
  {
    std::set<std::string> available_keys = graph_inputs;
    std::set<boost::uuids::uuid> visited;
    std::vector<boost::uuids::uuid> pending = data_flow.at(uuid).predecessors;
    while (!pending.empty())
    {
      const boost::uuids::uuid predecessor_uuid = pending.back();
      pending.pop_back();
      if (!visited.insert(predecessor_uuid).second)
        continue;

      const auto predecessor = data_flow.find(predecessor_uuid);
      if (predecessor == data_flow.end())
        continue;

      available_keys.insert(predecessor->second.outputs.begin(), predecessor->second.outputs.end());
      pending.insert(pending.end(), predecessor->second.predecessors.begin(), predecessor->second.predecessors.end());
    }

    for (const auto& input_key : data_flow.at(uuid).inputs)
    {
      if (available_keys.find(input_key) == available_keys.end())
      {
        return { false,
                 "Task Composer Graph '" + name_ + "' child '" + node->getName() + "' input key '" + input_key +
                     "' is not supplied by a graph input or predecessor output" };
      }
    }
  }

  std::set<std::string> graph_outputs;
  collectMappingKeys(output_port_mappings_, nullptr, graph_outputs);
  for (const auto& output_key : graph_outputs)
  {
    if (produced_keys.find(output_key) == produced_keys.end())
    {
      return { false,
               "Task Composer Graph '" + name_ + "' output key '" + output_key +
                   "' is not supplied by a graph input or child output" };
    }
  }

  return { true, "Task Composer Graph Valid" };
}

std::string TaskComposerGraph::dump(std::ostream& os,
                                    const TaskComposerNode* parent,
                                    const std::map<boost::uuids::uuid, TaskComposerNodeInfo>& results_map) const
{
  if (parent == nullptr)
    os << "digraph TaskComposer {\n";

  std::ostringstream sub_graphs;
  const std::string tmp = toString(uuid_);
  os << "subgraph cluster_" << tmp << " {\n color=black;\n nojustify=true label = \"" << name_
     << "\\nUUID: " << uuid_str_ << "\\l";
  os << "Inputs:\\l" << input_port_mappings_;
  os << "Outputs:\\l" << output_port_mappings_;

  if (!override_input_port_mappings_.empty())
    os << "Override Inputs:\\l" << override_input_port_mappings_;

  if (!override_output_port_mappings_.empty())
    os << "Override Outputs:\\l" << override_output_port_mappings_;

  os << "Abort Terminal: " << abort_terminal_ << "\\l";
  os << "Conditional: " << ((conditional_) ? "True" : "False") << "\\l";
  if (getType() == TaskComposerNodeType::PIPELINE || getType() == TaskComposerNodeType::GRAPH)
  {
    auto it = results_map.find(getUUID());
    if (it != results_map.end())
      os << "Time: " << std::fixed << std::setprecision(3) << it->second.elapsed_time << "s\\l";
  }
  os << "\";";
  for (const auto& pair : nodes_)
  {
    const auto& node = pair.second;
    if (node->getType() == TaskComposerNodeType::TASK)
    {
      sub_graphs << node->dump(os, this, results_map);
    }
    else if (node->getType() == TaskComposerNodeType::GRAPH || node->getType() == TaskComposerNodeType::PIPELINE)
    {
      const auto& graph_node = static_cast<const TaskComposerGraph&>(*node);

      auto it = results_map.find(graph_node.getUUID());
      std::string color = (it != results_map.end() && it->second.color != "white") ? it->second.color : "blue";
      const std::string tmp = toString(graph_node.uuid_, "node_");
      const TaskComposerPortMap& input_port_mappings = graph_node.getInputPortMappings();
      const TaskComposerPortMap& output_port_mappings = graph_node.getOutputPortMappings();
      const TaskComposerPortMap& override_input_port_mappings = graph_node.getOverrideInputPortMappings();
      const TaskComposerPortMap& override_output_port_mappings = graph_node.getOverrideOutputPortMappings();
      os << "\n"
         << tmp << " [shape=box3d, nojustify=true label=\"Subgraph: " << graph_node.name_
         << "\\nUUID: " << graph_node.uuid_str_ << "\\l";
      os << "Inputs:\\l" << input_port_mappings;
      os << "Outputs:\\l" << output_port_mappings;

      if (!override_input_port_mappings.empty())
        os << "Override Inputs:\\l" << override_input_port_mappings;

      if (!override_output_port_mappings.empty())
        os << "Override Outputs:\\l" << override_output_port_mappings;

      os << "Abort Terminal: " << graph_node.abort_terminal_ << "\\l";
      os << "Conditional: " << ((node->isConditional()) ? "True" : "False") << "\\l";
      if (it != results_map.end())
      {
        os << "Time: " << std::fixed << std::setprecision(3) << it->second.elapsed_time << "s\\l"
           << "Status Code: " << std::to_string(it->second.status_code) << "\\l"
           << "Status Msg: " << it->second.status_message << "\\l";
      }

      os << "\", margin=\"0.1\", color=" << color << "];\n";  // NOLINT
      node->dump(sub_graphs, this, results_map);
    }
  }

  if (type_ == TaskComposerNodeType::GRAPH || type_ == TaskComposerNodeType::PIPELINE)
  {
    if (conditional_)
    {
      int return_value = -1;

      auto it = results_map.find(uuid_);
      if (it != results_map.end())
        return_value = it->second.return_value;

      for (std::size_t i = 0; i < outbound_edges_.size(); ++i)
      {
        std::string line_type = (return_value == static_cast<int>(i)) ? "bold" : "dashed";
        os << "node_" << tmp << " -> " << toString(outbound_edges_[i], "node_") << " [style=" << line_type
           << ", label=\"[" << std::to_string(i) << "]\"" << "];\n";
      }
    }
    else
    {
      for (const auto& edge : outbound_edges_)
      {
        os << "node_" << tmp << " -> " << toString(edge, "node_") << ";\n";
      }
    }
  }
  else
  {
    for (const auto& edge : outbound_edges_)
    {
      os << "node_" << tmp << " -> " << toString(edge, "node_") << ";\n";
    }
  }

  os << "}\n";

  // Dump subgraphs outside this subgraph
  os << sub_graphs.str();

  // Close out digraph or subgraph
  if (parent == nullptr)
    os << "}\n";

  return {};
}

namespace
{
/** @brief Validator that rejects conditional execution for plain graphs. */
void validateGraphConditional(const tesseract::common::PropertyTree& conditional,
                              const std::string& path,
                              std::vector<std::string>& errors)
{
  if (!conditional.isNull())
  {
    try
    {
      if (conditional.as<bool>())
        errors.push_back(path + ": TaskComposerGraph does not support conditional execution");
    }
    catch (const YAML::Exception&)
    {
      // The base boolean validator reports the type conversion error.
      return;
    }
  }
}

struct MappedKey
{
  std::string key;
  std::string path;
};

void collectMappedKeys(const YAML::Node& mappings, const std::string& path, std::vector<MappedKey>& keys)
{
  if (!mappings || !mappings.IsMap())
    return;

  for (const auto& mapping : mappings)
  {
    if (!mapping.first.IsScalar())
      continue;

    const std::string mapping_path = path + "." + mapping.first.Scalar();
    if (mapping.second.IsScalar())
    {
      keys.push_back({ mapping.second.Scalar(), mapping_path });
    }
    else if (mapping.second.IsSequence())
    {
      for (std::size_t index = 0; index < mapping.second.size(); ++index)
      {
        if (mapping.second[index].IsScalar())
          keys.push_back({ mapping.second[index].Scalar(), mapping_path + "[" + std::to_string(index) + "]" });
      }
    }
  }
}

std::set<std::string> mappedKeySet(const YAML::Node& mappings)
{
  std::vector<MappedKey> mapped_keys;
  collectMappedKeys(mappings, {}, mapped_keys);

  std::set<std::string> keys;
  for (const auto& mapped_key : mapped_keys)
    keys.insert(mapped_key.key);
  return keys;
}

struct GraphNodeData
{
  std::vector<MappedKey> inputs;
  std::set<std::string> outputs;
  std::set<std::string> predecessors;
};

/** @brief Validate that every configured child input can be populated by graph inputs or predecessor outputs. */
void validateGraphDataFlow(const tesseract::common::PropertyTree& node,
                           const std::string& path,
                           std::vector<std::string>& errors)
{
  const auto* nodes_property = node.find("nodes");
  if (nodes_property == nullptr)
    return;

  const YAML::Node& nodes = nodes_property->getValue();
  if (!nodes || !nodes.IsMap())
    return;

  std::unordered_map<std::string, GraphNodeData> graph_nodes;
  bool has_named_subtask{ false };
  for (const auto& node_entry : nodes)
  {
    if (!node_entry.first.IsScalar() || !node_entry.second.IsMap())
      continue;

    const std::string node_name = node_entry.first.Scalar();
    auto& node_data = graph_nodes[node_name];
    if (node_entry.second["task"])
      has_named_subtask = true;

    const YAML::Node node_config = node_entry.second["config"];
    if (!node_config || !node_config.IsMap())
      continue;

    std::string inputs_path = path;
    inputs_path += ".nodes.";
    inputs_path += node_name;
    inputs_path += ".config.inputs";
    collectMappedKeys(node_config["inputs"], inputs_path, node_data.inputs);
    node_data.outputs = mappedKeySet(node_config["outputs"]);
  }

  if (has_named_subtask)
    return;

  const auto* edges_property = node.find("edges");
  const YAML::Node edges = (edges_property == nullptr) ? YAML::Node() : edges_property->getValue();
  if (edges && edges.IsSequence())
  {
    for (const auto& edge : edges)
    {
      if (!edge.IsMap() || !edge["source"] || !edge["source"].IsScalar() || !edge["destinations"])
        continue;

      const std::string source = edge["source"].Scalar();
      const YAML::Node destinations = edge["destinations"];
      if (destinations.IsScalar())
      {
        auto destination = graph_nodes.find(destinations.Scalar());
        if (destination != graph_nodes.end())
          destination->second.predecessors.insert(source);
      }
      else if (destinations.IsSequence())
      {
        for (const auto& destination_node : destinations)
        {
          if (!destination_node.IsScalar())
            continue;

          auto destination = graph_nodes.find(destination_node.Scalar());
          if (destination != graph_nodes.end())
            destination->second.predecessors.insert(source);
        }
      }
    }
  }

  const auto* inputs_property = node.find("inputs");
  const YAML::Node graph_input_mappings = (inputs_property == nullptr) ? YAML::Node() : inputs_property->getValue();
  const std::set<std::string> graph_inputs = mappedKeySet(graph_input_mappings);
  if (graph_inputs.empty())
    return;

  for (const auto& [node_name, node_data] : graph_nodes)
  {
    std::set<std::string> available_keys = graph_inputs;
    std::set<std::string> visited;
    std::vector<std::string> pending(node_data.predecessors.begin(), node_data.predecessors.end());
    while (!pending.empty())
    {
      const std::string predecessor_name = std::move(pending.back());
      pending.pop_back();
      if (!visited.insert(predecessor_name).second)
        continue;

      const auto predecessor = graph_nodes.find(predecessor_name);
      if (predecessor == graph_nodes.end())
        continue;

      available_keys.insert(predecessor->second.outputs.begin(), predecessor->second.outputs.end());
      pending.insert(pending.end(), predecessor->second.predecessors.begin(), predecessor->second.predecessors.end());
    }

    for (const auto& input : node_data.inputs)
    {
      if (available_keys.find(input.key) == available_keys.end())
      {
        errors.push_back(input.path + ": key '" + input.key +
                         "' is not supplied by graph inputs or predecessor outputs");
      }
    }
  }
}

/**
 * @brief Validator that checks terminals, edge sources, and edge destinations all reference keys in the nodes map.
 */
void validateGraphNodeReferences(const tesseract::common::PropertyTree& node,
                                 const std::string& path,
                                 std::vector<std::string>& errors)
{
  // Collect node keys from the nodes map
  const auto* nodes_node = node.find("nodes");
  if (nodes_node == nullptr || nodes_node->getValue().IsNull())
    return;  // nodes missing — other validators handle that

  std::set<std::string> node_keys;
  if (nodes_node->getValue().IsMap())
  {
    for (auto it = nodes_node->getValue().begin(); it != nodes_node->getValue().end(); ++it)
      node_keys.insert(it->first.as<std::string>());
  }

  if (node_keys.empty())
    return;

  // Validate terminals
  const auto* terminals_node = node.find("terminals");
  if (terminals_node != nullptr && terminals_node->getValue().IsSequence())
  {
    for (std::size_t i = 0; i < terminals_node->getValue().size(); ++i)
    {
      auto name = terminals_node->getValue()[i].as<std::string>();
      if (node_keys.find(name) == node_keys.end())
      {
        std::string error = path;
        error += ".terminals[";
        error += std::to_string(i);
        error += "]: '";
        error += name;
        error += "' not found in nodes";
        errors.push_back(std::move(error));
      }
    }
  }

  // Validate edge sources and destinations
  const auto* edges_node = node.find("edges");
  if (edges_node != nullptr && edges_node->getValue().IsSequence())
  {
    for (std::size_t i = 0; i < edges_node->getValue().size(); ++i)
    {
      const auto& edge = edges_node->getValue()[i];
      if (edge["source"])
      {
        auto source = edge["source"].as<std::string>();
        if (node_keys.find(source) == node_keys.end())
        {
          std::string error = path;
          error += ".edges[";
          error += std::to_string(i);
          error += "].source: '";
          error += source;
          error += "' not found in nodes";
          errors.push_back(std::move(error));
        }
      }
      if (edge["destinations"])
      {
        const auto& dests = edge["destinations"];
        if (dests.IsSequence())
        {
          for (std::size_t j = 0; j < dests.size(); ++j)
          {
            auto dest = dests[j].as<std::string>();
            if (node_keys.find(dest) == node_keys.end())
            {
              std::string error = path;
              error += ".edges[";
              error += std::to_string(i);
              error += "].destinations[";
              error += std::to_string(j);
              error += "]: '";
              error += dest;
              error += "' not found in nodes";
              errors.push_back(std::move(error));
            }
          }
        }
        else if (dests.IsScalar())
        {
          auto dest = dests.as<std::string>();
          if (node_keys.find(dest) == node_keys.end())
          {
            std::string error = path;
            error += ".edges[";
            error += std::to_string(i);
            error += "].destinations: '";
            error += dest;
            error += "' not found in nodes";
            errors.push_back(std::move(error));
          }
        }
      }
    }
  }
}

tesseract::common::PropertyTree createGraphSchema(const tesseract::common::PropertyTree& node_schema,
                                                  bool allow_conditional)
{
  using namespace tesseract::common;
  // clang-format off
  PropertyTreeBuilder builder;
  builder
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .compose(node_schema)
      .validator(validateGraphNodeReferences)
      .validator(validateGraphDataFlow)
    .customType("nodes", property_type::createMap(SUB_TASK_SCHEMA_KEY))
      .required().done()
    .customType("edges", property_type::createList(GRAPH_EDGE_SCHEMA_KEY))
      .required().validator(validateCustomType).done()
    .customType("terminals", property_type::createList(property_type::STRING))
      .required().done();
  // clang-format on

  auto schema = builder.build();
  if (!allow_conditional)
    schema.at("conditional").addValidator(validateGraphConditional);

  return schema;
}
}  // namespace

tesseract::common::PropertyTree TaskComposerGraph::schema() { return graphSchema(false); }

tesseract::common::PropertyTree TaskComposerGraph::graphSchema(bool allow_conditional)
{
  using namespace tesseract::common;
  auto node_schema = TaskComposerNode::commonSchema();
  node_schema["inputs"].setAttribute(property_attribute::TYPE,
                                     property_type::createMap(REQUIRED_STRING_OR_STRING_LIST_SCHEMA_KEY));
  node_schema["inputs"].addValidator(validateCustomType);
  node_schema["outputs"].setAttribute(property_attribute::TYPE,
                                      property_type::createMap(REQUIRED_STRING_OR_STRING_LIST_SCHEMA_KEY));
  node_schema["outputs"].addValidator(validateCustomType);
  return createGraphSchema(node_schema, allow_conditional);
}

}  // namespace tesseract::task_composer
