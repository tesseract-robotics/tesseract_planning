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
  node->edges_.insert(node->edges_.end(), destinations.begin(), destinations.end());
}

std::map<boost::uuids::uuid, TaskComposerNode::ConstPtr> TaskComposerGraph::getNodes() const
{
  return std::map<boost::uuids::uuid, TaskComposerNode::ConstPtr>{ nodes_.begin(), nodes_.end() };
}

int TaskComposerGraph::run(TaskComposerInput& /*input*/) const
{
  throw std::runtime_error("TaskComposerGraph, run is currently not implemented");
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
