/**
 * @file graph_taskflow.h
 * @brief Creates a directed graph taskflow
 *
 * @author Levi Armstrong
 * @date August 13. 2020
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

#ifndef TESSERACT_PROCESS_MANAGERS_GRAPH_TASKFLOW_H
#define TESSERACT_PROCESS_MANAGERS_GRAPH_TASKFLOW_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/taskflow_generator.h>
#include <tesseract_process_managers/core/task_generator.h>

namespace tesseract_planning
{
/**
 * @brief This class facilitates the composition of an arbitrary taskflow graph.
 * Tasks are nodes in the graph connected to each other in a configurable order by directed edges
 */
class GraphTaskflow : public TaskflowGenerator
{
public:
  using UPtr = std::unique_ptr<GraphTaskflow>;

  /** @brief ID for the Done node */
  const static int DONE_NODE = -1;
  /** @brief ID for the Error node */
  const static int ERROR_NODE = -2;

  GraphTaskflow(std::string name = "GraphTaskflow");
  ~GraphTaskflow() override = default;
  GraphTaskflow(const GraphTaskflow&) = delete;
  GraphTaskflow& operator=(const GraphTaskflow&) = delete;
  GraphTaskflow(GraphTaskflow&&) = delete;
  GraphTaskflow& operator=(GraphTaskflow&&) = delete;

  const std::string& getName() const override;

  TaskflowContainer generateTaskflow(TaskInput input, TaskflowVoidFn done_cb, TaskflowVoidFn error_cb) override;

  /**
   * @brief Add a node to the taskflow graph along with setting the process type.
   * @param process The process generator assigned to the node
   * @param is_conditional Flag indicating if this process is conditional
   * @return The node ID which should be used with adding edges
   */
  int addNode(TaskGenerator::UPtr process, bool is_conditional = false);

  /**
   * @brief Adds directed edges from a source node to desintation nodes in the taskflow graph
   * @details If source is a non-conditional task, it is only relevant to provide one destination as the output of a
   * non-conditional tf::Task is void. If source is a conditional task, the order of the destinations should correspond
   * to the integer output of the conditional tf::Task. For example,if the output of the conditional tf::Task is 0, the
   * taskflow would transition to the node identified by the ID at index 0 in the destinations input. A leaf node (i.e.
   * node without defined edges) is always connected to the done callback. If the leaf node is a conditional node, it is
   * connected to the error task at index 0 and the done task at index 1
   * @param from The ID of the source node of the edge
   * @param dest A list of IDs of the destination nodes of the edges.
   */
  void addEdges(int source, std::vector<int> destinations);

private:
  /** @brief Helper struct for holding relevant information about the nodes */
  struct Node
  {
    Node(TaskGenerator::UPtr process_, bool is_conditional_);

    TaskGenerator::UPtr process;
    const bool is_conditional;
    /** @brief IDs of nodes (i.e. tasks) that should run after this node */
    std::vector<int> edges;
  };

  std::vector<Node> nodes_;
  std::string name_;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_GRAPH_TASKFLOW_H
