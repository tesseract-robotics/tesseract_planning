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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

namespace tesseract_planning
{
TaskComposerGraph::TaskComposerGraph(std::string name)
  : TaskComposerGraph(std::move(name), TaskComposerNodeType::GRAPH, false)
{
}

TaskComposerGraph::TaskComposerGraph(std::string name, TaskComposerNodeType type, bool conditional)
  : TaskComposerNode(std::move(name), type, conditional)
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
  : TaskComposerNode(std::move(name), type, config)
{
  std::unordered_map<std::string, boost::uuids::uuid> node_uuids;
  YAML::Node nodes = config["nodes"];
  if (!nodes.IsMap())
    throw std::runtime_error("Task Composer Graph '" + name_ + "' 'nodes' entry is not a map");

  for (auto node_it = nodes.begin(); node_it != nodes.end(); ++node_it)
  {
    auto node_name = node_it->first.as<std::string>();
    if (YAML::Node fn = node_it->second["class"])
    {
      tesseract_common::PluginInfo plugin_info;
      plugin_info.class_name = fn.as<std::string>();
      if (YAML::Node cn = node_it->second["config"])
        plugin_info.config = cn;

      TaskComposerNode::UPtr task_node = plugin_factory.createTaskComposerNode(node_name, plugin_info);
      if (task_node == nullptr)
        throw std::runtime_error("Task Composer Graph '" + name_ + "' failed to create node '" + node_name + "'");

      node_uuids[node_name] = addNode(std::move(task_node));
    }
    else if (YAML::Node tn = node_it->second["task"])
    {
      auto task_name = tn.as<std::string>();
      TaskComposerNode::UPtr task_node = plugin_factory.createTaskComposerNode(task_name);
      if (task_node == nullptr)
        throw std::runtime_error("Task Composer Graph '" + name_ + "' failed to create task '" + task_name +
                                     "' for node '" += node_name + "'");

      task_node->setName(node_name);

      if (YAML::Node tc = node_it->second["config"])
      {
        if (YAML::Node n = tc["conditional"])
          task_node->setConditional(n.as<bool>());

        if (YAML::Node n = tc["abort_terminal"])
        {
          if (task_node->getType() != TaskComposerNodeType::GRAPH ||
              task_node->getType() == TaskComposerNodeType::PIPELINE)
            static_cast<TaskComposerGraph&>(*task_node).setTerminalTriggerAbortByIndex(n.as<int>());
          else
            throw std::runtime_error("YAML entry 'abort_terminal' is only supported for GRAPH and PIPELINE types");
        }

        if (tc["input_remapping"])  // NOLINT
          throw std::runtime_error("TaskComposerGraph, input_remapping is no longer supported use 'remapping'");

        if (tc["output_remapping"])  // NOLINT
          throw std::runtime_error("TaskComposerGraph, output_remapping is no longer supported use 'remapping'");

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
}

boost::uuids::uuid TaskComposerGraph::addNode(TaskComposerNode::UPtr task_node)
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

std::map<boost::uuids::uuid, TaskComposerNode::ConstPtr> TaskComposerGraph::getNodes() const
{
  return std::map<boost::uuids::uuid, TaskComposerNode::ConstPtr>{ nodes_.begin(), nodes_.end() };
}

TaskComposerNode::ConstPtr TaskComposerGraph::getNodeByName(const std::string& name) const
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
    auto& n = nodes_.at(terminal);
    if (n->getType() == TaskComposerNodeType::TASK)
      static_cast<TaskComposerTask&>(*n).setTriggerAbort(true);
    else
      throw std::runtime_error("Tasks can only trigger abort!");
  }
  else
  {
    for (const auto& terminal : terminals_)
    {
      auto& n = nodes_.at(terminal);
      if (n->getType() == TaskComposerNodeType::TASK)
        static_cast<TaskComposerTask&>(*n).setTriggerAbort(false);
    }
  }
}

void TaskComposerGraph::setTerminalTriggerAbortByIndex(int terminal_index)
{
  if (terminal_index >= 0)
  {
    auto& n = nodes_.at(terminals_.at(static_cast<std::size_t>(terminal_index)));
    if (n->getType() == TaskComposerNodeType::TASK)
      static_cast<TaskComposerTask&>(*n).setTriggerAbort(true);
    else
      throw std::runtime_error("Tasks can only trigger abort!");
  }
  else
  {
    for (const auto& terminal : terminals_)
    {
      auto& n = nodes_.at(terminal);
      if (n->getType() == TaskComposerNodeType::TASK)
        static_cast<TaskComposerTask&>(*n).setTriggerAbort(false);
    }
  }
}

void TaskComposerGraph::renameInputKeys(const std::map<std::string, std::string>& input_keys)
{
  for (const auto& key : input_keys)
    std::replace(input_keys_.begin(), input_keys_.end(), key.first, key.second);

  for (auto& node : nodes_)
    node.second->renameInputKeys(input_keys);
}

void TaskComposerGraph::renameOutputKeys(const std::map<std::string, std::string>& output_keys)
{
  for (const auto& key : output_keys)
    std::replace(output_keys_.begin(), output_keys_.end(), key.first, key.second);

  for (auto& node : nodes_)
    node.second->renameOutputKeys(output_keys);
}

std::string TaskComposerGraph::dump(std::ostream& os,
                                    const TaskComposerNode* parent,
                                    const std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr>& results_map) const
{
  if (parent == nullptr)
    os << "digraph TaskComposer {\n";

  std::ostringstream sub_graphs;
  const std::string tmp = toString(uuid_);
  os << "subgraph cluster_" << tmp << " {\n color=black;\n label = \"" << name_ << "\\n(" << uuid_str_ << ")";
  os << "\\n Inputs: [";
  for (std::size_t i = 0; i < input_keys_.size(); ++i)
  {
    os << input_keys_[i];
    if (i < input_keys_.size() - 1)
      os << ", ";
  }
  os << "]";

  os << "\\n Outputs: [";
  for (std::size_t i = 0; i < output_keys_.size(); ++i)
  {
    os << output_keys_[i];
    if (i < output_keys_.size() - 1)
      os << ", ";
  }
  os << "]";
  os << "\\n Conditional: " << ((conditional_) ? "True" : "False");
  if (getType() == TaskComposerNodeType::PIPELINE || getType() == TaskComposerNodeType::GRAPH)
  {
    auto it = results_map.find(getUUID());
    if (it != results_map.end())
      os << "\\nTime: " << std::fixed << std::setprecision(3) << it->second->elapsed_time << "s";
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
      std::string color = (it != results_map.end() && it->second->color != "white") ? it->second->color : "blue";
      const std::string tmp = toString(node->uuid_, "node_");
      const std::vector<std::string>& input_keys = node->getInputKeys();
      const std::vector<std::string>& output_keys = node->getOutputKeys();
      os << std::endl << tmp << " [shape=box3d, label=\"Subgraph: " << node->name_ << "\\n(" << node->uuid_str_ << ")";
      os << "\\n Inputs: [";
      for (std::size_t i = 0; i < input_keys.size(); ++i)
      {
        os << input_keys[i];
        if (i < input_keys.size() - 1)
          os << ", ";
      }
      os << "]";

      os << "\\n Outputs: [";
      for (std::size_t i = 0; i < output_keys.size(); ++i)
      {
        os << output_keys[i];
        if (i < output_keys.size() - 1)
          os << ", ";
      }
      os << "]";

      os << "\\n Conditional: " << ((node->isConditional()) ? "True" : "False");
      if (it != results_map.end())
        os << "\\nTime: " << std::fixed << std::setprecision(3) << it->second->elapsed_time << "s";

      os << "\", margin=\"0.1\", color=" << color << "];\n";  // NOLINT
      node->dump(sub_graphs, this, results_map);
    }
  }

  if (type_ == TaskComposerNodeType::PIPELINE)
  {
    const auto& pipeline = static_cast<const TaskComposerPipeline&>(*this);
    if (pipeline.isConditional())
    {
      int return_value = -1;

      auto it = results_map.find(uuid_);
      if (it != results_map.end())
        return_value = it->second->return_value;

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
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNode);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerGraph)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerGraph)
