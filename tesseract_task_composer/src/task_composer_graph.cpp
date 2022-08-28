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

namespace tesseract_planning
{
TaskComposerGraph::TaskComposerGraph(std::string name) : TaskComposerNode(std::move(name), TaskComposerNodeType::GRAPH)
{
}

boost::uuids::uuid TaskComposerGraph::addNode(TaskComposerNode::UPtr task_node)
{
  boost::uuids::uuid uuid = task_node->getUUID();
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

void TaskComposerGraph::dump(std::ostream& os) const
{
  os << "digraph TaskComposer {\n";
  dumpHelper(os, *this);
  os << "}\n";
}

void TaskComposerGraph::dumpHelper(std::ostream& os, const TaskComposerGraph& /*parent*/) const
{
  std::ostringstream sub_graphs;
  const std::string tmp = toString(uuid_);
  os << "subgraph cluster_" << tmp << " {\n color=black;\n label = \"" << name_ << "\\n(" << uuid_str_ << ")\";";
  for (const auto& pair : nodes_)
  {
    const auto& node = pair.second;
    if (node->getType() == TaskComposerNodeType::TASK)
      node->dump(os);
    else if (node->getType() == TaskComposerNodeType::GRAPH)
    {
      const std::string tmp = toString(node->uuid_, "node_");
      os << std::endl
         << tmp << " [shape=box3d, label=\"Subgraph: " << node->name_ << "\\n(" << node->uuid_str_
         << ")\", color=blue, margin=\"0.1\"];\n";
      static_cast<const TaskComposerGraph&>(*node).dumpHelper(sub_graphs, *this);
    }
  }

  for (const auto& edge : outbound_edges_)
  {
    os << "node_" << tmp << " -> " << toString(edge, "node_") << ";\n";
  }

  os << "}\n";

  // Dump subgraphs outside this subgraph
  os << sub_graphs.str();
}

TaskComposerNode::UPtr TaskComposerGraph::clone() const
{
  auto clone_graph = std::make_unique<TaskComposerGraph>(name_);
  std::map<boost::uuids::uuid, boost::uuids::uuid> clone_map;
  for (const auto& node : nodes_)
    clone_map[node.first] = clone_graph->addNode(node.second->clone());

  for (const auto& node : nodes_)
  {
    std::vector<boost::uuids::uuid> cloned_edges;
    cloned_edges.reserve(node.second->outbound_edges_.size());
    for (const auto& edge : node.second->outbound_edges_)
      cloned_edges.push_back(clone_map[edge]);

    clone_graph->addEdges(clone_map[node.first], cloned_edges);
  }

  return clone_graph;
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
      if (equal == true)
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
