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
#include <boost/serialization/shared_ptr.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_serialize.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_graph.h>
#include <tesseract_task_composer/task_composer_plugin_factory.h>

namespace tesseract_planning
{
TaskComposerGraph::TaskComposerGraph(std::string name) : TaskComposerNode(std::move(name), TaskComposerNodeType::GRAPH)
{
}
TaskComposerGraph::TaskComposerGraph(std::string name,
                                     const YAML::Node& config,
                                     const TaskComposerPluginFactory& plugin_factory)
  : TaskComposerGraph(std::move(name))
{
  // Get input keys
  if (YAML::Node n = config["inputs"])
    input_keys_ = n.as<std::vector<std::string>>();

  if (YAML::Node n = config["outputs"])
    output_keys_ = n.as<std::vector<std::string>>();

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
      std::map<std::string, std::string> input_remapping;
      std::map<std::string, std::string> output_remapping;

      auto task_name = tn.as<std::string>();
      if (YAML::Node n = tn["input_remapping"])
        input_remapping = n.as<std::map<std::string, std::string>>();

      if (YAML::Node n = tn["output_remapping"])
        output_remapping = n.as<std::map<std::string, std::string>>();

      TaskComposerNode::UPtr task_node = plugin_factory.createTaskComposerNode(task_name);
      if (task_node == nullptr)
        throw std::runtime_error("Task Composer Graph '" + name_ + "' failed to create task '" + task_name +
                                     "' for node '" += node_name + "'");

      task_node->setName(node_name);
      if (!input_remapping.empty())
        task_node->renameInputKeys(input_remapping);

      if (!output_remapping.empty())
        task_node->renameOutputKeys(output_remapping);

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
      destinations = n.as<std::vector<std::string>>();
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
  os << "subgraph cluster_" << tmp << " {\n color=black;\n label = \"" << name_ << "\\n(" << uuid_str_ << ")\";";
  for (const auto& pair : nodes_)
  {
    const auto& node = pair.second;
    if (node->getType() == TaskComposerNodeType::TASK)
    {
      sub_graphs << node->dump(os, this, results_map);
    }
    else if (node->getType() == TaskComposerNodeType::GRAPH)
    {
      const std::string tmp = toString(node->uuid_, "node_");
      os << std::endl
         << tmp << " [shape=box3d, label=\"Subgraph: " << node->name_ << "\\n(" << node->uuid_str_
         << ")\", color=blue, margin=\"0.1\"];\n";
      node->dump(sub_graphs, this, results_map);
    }
  }

  for (const auto& edge : outbound_edges_)
  {
    os << "node_" << tmp << " -> " << toString(edge, "node_") << ";\n";
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
      equal &= (it == rhs.nodes_.end());
      if (equal)
        equal &= (*(pair.second) == *(it->second));
    }
  }
  equal &= TaskComposerNode::operator==(rhs);
  return equal;
}
bool TaskComposerGraph::operator!=(const TaskComposerGraph& rhs) const { return !operator==(rhs); }

template <class Archive>
void TaskComposerGraph::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("nodes", nodes_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNode);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskComposerGraph)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskComposerGraph)
