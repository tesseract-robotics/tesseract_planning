/**
 * @file task_composer_graph.h
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_GRAPH_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_GRAPH_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_node.h>
#include <tesseract_task_composer/task_composer_node_info.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;
/**
 * @brief This class facilitates the composition of an arbitrary taskflow graph.
 * Tasks are nodes in the graph connected to each other in a configurable order by directed edges
 */
class TaskComposerGraph : public TaskComposerNode
{
public:
  using Ptr = std::shared_ptr<TaskComposerGraph>;
  using ConstPtr = std::shared_ptr<const TaskComposerGraph>;
  using UPtr = std::unique_ptr<TaskComposerGraph>;
  using ConstUPtr = std::unique_ptr<const TaskComposerGraph>;

  TaskComposerGraph(std::string name = "TaskComposerGraph");
  TaskComposerGraph(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& plugin_factory);
  ~TaskComposerGraph() override = default;
  TaskComposerGraph(const TaskComposerGraph&) = delete;
  TaskComposerGraph& operator=(const TaskComposerGraph&) = delete;
  TaskComposerGraph(TaskComposerGraph&&) = delete;
  TaskComposerGraph& operator=(TaskComposerGraph&&) = delete;

  /**
   * @brief Add a node to the pipeline
   * @return The node ID which should be used with adding edges
   */
  boost::uuids::uuid addNode(TaskComposerNode::UPtr task_node);

  /**
   * @brief Adds directed edges from a source node to destination nodes in the taskflow graph
   * @details If source is a non-conditional task, it is only relevant to provide one destination as the output of a
   * non-conditional tf::Task is void. If source is a conditional task, the order of the destinations should correspond
   * to the integer output of the conditional tf::Task. For example,if the output of the conditional tf::Task is 0, the
   * taskflow would transition to the node identified by the ID at index 0 in the destinations input. A leaf node (i.e.
   * node without defined edges) is always connected to the done callback. If the leaf node is a conditional node, it is
   * connected to the error task at index 0 and the done task at index 1
   * @param from The ID of the source node of the edge
   * @param dest A list of IDs of the destination nodes of the edges.
   */
  void addEdges(boost::uuids::uuid source, std::vector<boost::uuids::uuid> destinations);

  /** @brief Get the nodes associated with the pipeline */
  std::map<boost::uuids::uuid, TaskComposerNode::ConstPtr> getNodes() const;

  void renameInputKeys(const std::map<std::string, std::string>& input_keys) override;

  void renameOutputKeys(const std::map<std::string, std::string>& output_keys) override;

  void dump(std::ostream& os, const TaskComposerNodeInfo::UPtr& node_info = nullptr) const override;

  void dump(std::ostream& os, const std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr>& results_map) const;

  bool operator==(const TaskComposerGraph& rhs) const;
  bool operator!=(const TaskComposerGraph& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  void dumpHelper(std::ostream& os, const TaskComposerGraph& parent, const std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr>& results_map) const;

  std::map<boost::uuids::uuid, TaskComposerNode::Ptr> nodes_;
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TaskComposerGraph, "TaskComposerGraph")

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_GRAPH_H
