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
#include <map>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_node.h>

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
   * @brief Get the root node of the graph
   * @return The root node uuid
   */
  boost::uuids::uuid getRootNode() const;

  /**
   * @brief Add a node to the pipeline
   * @return The node ID which should be used with adding edges
   */
  boost::uuids::uuid addNode(std::unique_ptr<TaskComposerNode> task_node);

  /**
   * @brief Add a node to the pipeline
   * @warning This is only available for python buindings and should not be used in c++
   * @return The node ID which should be used with adding edges
   */
  boost::uuids::uuid addNodePython(std::shared_ptr<TaskComposerNode> task_node);

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

  /** @brief Get the nodes associated with the pipeline mapped by uuid */
  std::map<boost::uuids::uuid, std::shared_ptr<const TaskComposerNode>> getNodes() const;

  /**
   * @brief Get a node by name
   * @param name The name of the node to search for
   * @return The node with the name, otherwise nullptr
   */
  std::shared_ptr<const TaskComposerNode> getNodeByName(const std::string& name) const;

  /**
   *  @brief Set the terminals nodes
   *  @details must be called after all nodes and edges are setup.
   */
  void setTerminals(std::vector<boost::uuids::uuid> terminals);

  /** @brief Get the terminal nodes for the pipeline */
  std::vector<boost::uuids::uuid> getTerminals() const;

  /**
   * @brief Set if the pipeline should call abort if terminal node is reached
   * @details If null, it will not abort
   * @param terminal The terminal uuid
   */
  void setTerminalTriggerAbort(boost::uuids::uuid terminal);

  /**
   * @brief Set if the pipeline should call abort if terminal node is reached
   * @details If less than zero, it will not abort
   * @param terminal The terminal index
   */
  void setTerminalTriggerAbortByIndex(int terminal_index);

  /** Get the abort terminal uuid if set */
  boost::uuids::uuid getAbortTerminal() const;

  /** Get the abort terminal index if set */
  int getAbortTerminalIndex() const;

  /**
   * @brief Check if the current state of the graph is valid
   * @todo Replace return type with std::expected when upgraded to use c++23
   * @return True if valid otherwise false with a reason
   */
  virtual std::pair<bool, std::string> isValid() const;

  void renameInputKeys(const std::map<std::string, std::string>& input_keys) override;

  void renameOutputKeys(const std::map<std::string, std::string>& output_keys) override;

  std::string
  dump(std::ostream& os,
       const TaskComposerNode* parent = nullptr,
       const std::map<boost::uuids::uuid, tesseract_planning::TaskComposerNodeInfo>& results_map = {}) const override;

  bool operator==(const TaskComposerGraph& rhs) const;
  bool operator!=(const TaskComposerGraph& rhs) const;

protected:
  // These are protected and used by PIPELINE
  TaskComposerGraph(std::string name, TaskComposerNodeType type, bool conditional);
  TaskComposerGraph(std::string name,
                    TaskComposerNodeType type,
                    const YAML::Node& config,
                    const TaskComposerPluginFactory& plugin_factory);

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor = std::nullopt) const override;

  std::map<boost::uuids::uuid, TaskComposerNode::Ptr> nodes_;
  std::vector<boost::uuids::uuid> terminals_;
  int abort_terminal_{ -1 };

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TaskComposerGraph)

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_GRAPH_H
