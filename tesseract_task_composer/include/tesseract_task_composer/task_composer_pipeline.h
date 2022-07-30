/**
 * @file task_composer_pipeline.h
 * @brief A task pipeline
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PIPELINE_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PIPELINE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_node.h>

namespace tesseract_planning
{
/**
 * @brief This class facilitates the composition of an arbitrary taskflow graph.
 * Tasks are nodes in the graph connected to each other in a configurable order by directed edges
 */
class TaskComposerPipeline
{
public:
  using UPtr = std::unique_ptr<TaskComposerPipeline>;

  /** @brief ID for the Done node */
  const static int DONE_NODE = -1;
  /** @brief ID for the Error node */
  const static int ERROR_NODE = -2;

  TaskComposerPipeline(std::string name = "TaskComposerPipeline");
  virtual ~TaskComposerPipeline() = default;
  TaskComposerPipeline(const TaskComposerPipeline&) = delete;
  TaskComposerPipeline& operator=(const TaskComposerPipeline&) = delete;
  TaskComposerPipeline(TaskComposerPipeline&&) = delete;
  TaskComposerPipeline& operator=(TaskComposerPipeline&&) = delete;

  /** @brief Get the name of the pipeline */
  const std::string& getName() const;

  /**
   * @brief Add a node to the pipeline
   * @return The node ID which should be used with adding edges
   */
  int addNode(TaskComposerNode::UPtr task_node);

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
  void addEdges(int source, std::vector<int> destinations);

  /** @brief Get the nodes associated with the pipeline */
  std::vector<TaskComposerNode::ConstPtr> getNodes();

private:
  std::vector<TaskComposerNode::Ptr> nodes_;
  std::string name_;
};

}  // namespace tesseract_planning
#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PIPELINE_H
