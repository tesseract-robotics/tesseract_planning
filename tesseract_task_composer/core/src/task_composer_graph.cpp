/**
 * @file task_composer_graph.cpp
 * @brief A task graph
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_serialize.hpp>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/plugin_info.h>
#include <tesseract_common/yaml_utils.h>
#include <tesseract_common/yaml_extenstions.h>
#include <tesseract_common/stopwatch.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_pipeline.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

namespace tesseract_planning
{
TaskComposerGraph::TaskComposerGraph(std::string name)
  : TaskComposerGraph(std::move(name), TaskComposerNodeType::GRAPH, false)
{
}

TaskComposerGraph::TaskComposerGraph(std::string name, TaskComposerNodeType type, bool conditional)
  : TaskComposerNode(std::move(name), type, TaskComposerNodePorts{}, conditional)
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
  : TaskComposerNode(std::move(name), type, TaskComposerNodePorts{}, config)
{
  static const std::set<std::string> graph_expected_keys{ "conditional", "inputs", "outputs",
                                                          "nodes",       "edges",  "terminals" };
  tesseract_common::checkForUnknownKeys(config, graph_expected_keys);

  std::unordered_map<std::string, boost::uuids::uuid> node_uuids;
  YAML::Node nodes = config["nodes"];
  if (!nodes.IsMap())
    throw std::runtime_error("Task Composer Graph '" + name_ + "' 'nodes' entry is not a map");

  for (auto node_it = nodes.begin(); node_it != nodes.end(); ++node_it)
  {
    static const std::set<std::string> nodes_expected_keys{ "class", "task", "config" };
    tesseract_common::checkForUnknownKeys(node_it->second, nodes_expected_keys);

    auto node_name = node_it->first.as<std::string>();
    if (YAML::Node fn = node_it->second["class"])
    {
      tesseract_common::PluginInfo plugin_info;
      plugin_info.class_name = fn.as<std::string>();
      if (YAML::Node cn = node_it->second["config"])
        plugin_info.config = cn;

      std::unique_ptr<TaskComposerNode> task_node = plugin_factory.createTaskComposerNode(node_name, plugin_info);
      if (task_node == nullptr)
        throw std::runtime_error("Task Composer Graph '" + name_ + "' failed to create node '" + node_name + "'");

      node_uuids[node_name] = addNode(std::move(task_node));
    }
    else if (YAML::Node tn = node_it->second["task"])
    {
      auto task_name = tn.as<std::string>();
      std::unique_ptr<TaskComposerNode> task_node = plugin_factory.createTaskComposerNode(task_name);
      if (task_node == nullptr)
        throw std::runtime_error("Task Composer Graph '" + name_ + "' failed to create task '" + task_name +
                                     "' for node '" += node_name + "'");

      task_node->setName(node_name);

      if (YAML::Node tc = node_it->second["config"])
      {
        static const std::set<std::string> tasks_expected_keys{ "conditional", "abort_terminal", "remapping" };
        tesseract_common::checkForUnknownKeys(tc, tasks_expected_keys);

        if (YAML::Node n = tc["conditional"])
          task_node->setConditional(n.as<bool>());

        if (YAML::Node n = tc["abort_terminal"])
        {
          if (task_node->getType() == TaskComposerNodeType::GRAPH ||
              task_node->getType() == TaskComposerNodeType::PIPELINE)
            static_cast<TaskComposerGraph&>(*task_node).setTerminalTriggerAbortByIndex(n.as<int>());
          else
            throw std::runtime_error("YAML entry 'abort_terminal' is only supported for GRAPH and PIPELINE types");
        }

        if (YAML::Node n = tc["remapping"])
        {
          auto remapping = n.as<std::map<std::string, std::string>>();
          task_node->renameInputKeys(remapping);
          task_node->renameOutputKeys(remapping);
        }
      }

      node_uuids[node_name] = addNode(std::move(task_node));
    }
    else
    {
      throw std::runtime_error("Task Composer Graph '" + name_ + "' node '" + node_name +
                               "' missing 'class' or 'task' entry");
    }
  }

  YAML::Node edges = config["edges"];
  if (!edges.IsSequence())
    throw std::runtime_error("Task Composer Graph '" + name_ + "' 'edges' entry is not a sequence");

  for (auto edge_it = edges.begin(); edge_it != edges.end(); ++edge_it)
  {
    const YAML::Node& edge = *edge_it;

    std::string source;
    std::vector<std::string> destinations;
    if (YAML::Node n = edge["source"])
      source = n.as<std::string>();
    else
      throw std::runtime_error("Task Composer Graph '" + name_ + "' edge is missing 'source' entry");

    if (YAML::Node n = edge["destinations"])
    {
      if (n.IsSequence())
        destinations = n.as<std::vector<std::string>>();
      else if (n.IsScalar())
        destinations.push_back(n.as<std::string>());
      else
        throw std::runtime_error("Task Composer Graph '" + name_ +
                                 "' entry 'destinations' must be a scalar or sequence");
    }
    else
      throw std::runtime_error("Task Composer Graph '" + name_ + "' edge is missing 'destinations' entry");

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

  if (YAML::Node n = config["terminals"])
  {
    auto terminals = n.as<std::vector<std::string>>();
    terminals_.clear();
    terminals_.reserve(terminals.size());
    for (const auto& terminal : terminals)
    {
      auto terminal_it = node_uuids.find(terminal);
      if (terminal_it == node_uuids.end())
        throw std::runtime_error("Task Composer Graph '" + name_ + "' failed to find terminal '" + terminal + "'");

      terminals_.push_back(terminal_it->second);
    }
  }
  else
  {
    throw std::runtime_error("Task Composer Graph '" + name_ + "' is missing 'terminals' entry");
  }

  auto is_valid = TaskComposerGraph::isValid();
  if (!is_valid.first)
    throw std::runtime_error(is_valid.second);
}

TaskComposerNodeInfo TaskComposerGraph::runImpl(TaskComposerContext& context,
                                                OptionalTaskComposerExecutor executor) const
{
  if (terminals_.empty())
    throw std::runtime_error("TaskComposerGraph, with name '" + name_ + "' does not have terminals!");

  tesseract_common::Stopwatch stopwatch;
  stopwatch.start();

  if (!executor.has_value())
    throw std::runtime_error("TaskComposerGraph, the optional executor is null!");

  TaskComposerFuture::UPtr future = executor.value().get().run(*this, context.data_storage, context.dotgraph);
  future->wait();

  // Merge child context data into parent context
  context.task_infos.mergeInfoMap(std::move(future->context->task_infos));
  if (future->context->isAborted())
    context.abort(future->context->task_infos.getAbortingNode());

  TaskComposerNodeInfo info(*this);
  auto info_map = context.task_infos.getInfoMap();
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
    auto node_info = context.task_infos.getInfo(terminals_[i]);
    if (node_info.has_value())
    {
      stopwatch.stop();
      info.input_keys = input_keys_;
      info.output_keys = output_keys_;
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
  nodes_[uuid] = std::move(task_node);
  return uuid;
}

boost::uuids::uuid TaskComposerGraph::addNodePython(std::shared_ptr<TaskComposerNode> task_node)
{
  boost::uuids::uuid uuid = task_node->getUUID();
  task_node->parent_uuid_ = uuid_;
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

std::pair<bool, std::string> TaskComposerGraph::isValid() const
{
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

  return { true, "Task Composer Graph Valid" };
}

void TaskComposerGraph::renameInputKeys(const std::map<std::string, std::string>& input_keys)
{
  input_keys_.rename(input_keys);
  for (auto& node : nodes_)
    node.second->renameInputKeys(input_keys);
}

void TaskComposerGraph::renameOutputKeys(const std::map<std::string, std::string>& output_keys)
{
  output_keys_.rename(output_keys);
  for (auto& node : nodes_)
    node.second->renameOutputKeys(output_keys);
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
  os << "Inputs:\\l" << input_keys_;
  os << "Outputs:\\l" << output_keys_;
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
      auto it = results_map.find(node->getUUID());
      std::string color = (it != results_map.end() && it->second.color != "white") ? it->second.color : "blue";
      const std::string tmp = toString(node->uuid_, "node_");
      const TaskComposerKeys& input_keys = node->getInputKeys();
      const TaskComposerKeys& output_keys = node->getOutputKeys();
      os << "\n"
         << tmp << " [shape=box3d, nojustify=true label=\"Subgraph: " << node->name_ << "\\nUUID: " << node->uuid_str_
         << "\\l";
      os << "Inputs:\\l" << input_keys;
      os << "Outputs:\\l" << output_keys;
      os << "Abort Terminal: " << static_cast<const TaskComposerGraph&>(*node).abort_terminal_ << "\\l";
      os << "Conditional: " << ((node->isConditional()) ? "True" : "False") << "\\l";
      if (it != results_map.end())
        os << "Time: " << std::fixed << std::setprecision(3) << it->second.elapsed_time << "s\\l";

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
           << ", label=\"[" << std::to_string(i) << "]\""
           << "];\n";
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

bool TaskComposerGraph::operator==(const TaskComposerGraph& rhs) const
{
  bool equal = true;
  equal &= (nodes_.size() == rhs.nodes_.size());
  if (equal)
  {
    for (const auto& pair : nodes_)
    {
      auto it = rhs.nodes_.find(pair.first);
      equal &= (it != rhs.nodes_.end());
      if (equal)
        equal &= (*(pair.second) == *(it->second));
    }
  }
  equal &= (terminals_ == rhs.terminals_);
  equal &= (abort_terminal_ == rhs.abort_terminal_);
  equal &= TaskComposerNode::operator==(rhs);
  return equal;
}

// LCOV_EXCL_START
bool TaskComposerGraph::operator!=(const TaskComposerGraph& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void TaskComposerGraph::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("nodes", nodes_);
  ar& boost::serialization::make_nvp("terminals", terminals_);
  ar& boost::serialization::make_nvp("abort_terminal", abort_terminal_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNode);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerGraph)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerGraph)
