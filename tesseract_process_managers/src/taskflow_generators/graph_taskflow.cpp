/**
 * @file graph_taskflow.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/utils.h>
#include <tesseract_process_managers/taskflow_generators/graph_taskflow.h>

namespace tesseract_planning
{
GraphTaskflow::GraphTaskflow(std::string name) : name_(std::move(name)) {}

GraphTaskflowNode::GraphTaskflowNode(TaskGenerator::UPtr process_, bool is_conditional_)
  : process(std::move(process_)), is_conditional(is_conditional_)
{
}

const std::string& GraphTaskflow::getName() const { return name_; }

TaskflowContainer GraphTaskflow::generateTaskflow(TaskInput input, TaskflowVoidFn done_cb, TaskflowVoidFn error_cb)
{
  // Create Taskflow and Container
  TaskflowContainer container;
  container.taskflow = std::make_unique<tf::Taskflow>(name_);

  // Add "Error" task
  auto error_fn = [=]() { failureTask(input, name_, "", error_cb); };
  container.outputs.push_back(container.taskflow->emplace(error_fn).name("Error Callback"));

  // Add "Done" task
  auto done_fn = [=]() { successTask(input, name_, "", done_cb); };
  container.outputs.push_back(container.taskflow->emplace(done_fn).name("Done Callback"));

  // Grab references to the error and done tasks for use later
  const tf::Task& error_task = container.outputs.at(0);
  const tf::Task& done_task = container.outputs.at(1);

  // Generate process tasks for each node using its process generator
  std::vector<tf::Task> tasks;
  tasks.reserve(nodes_.size());
  for (GraphTaskflowNode& node : nodes_)
  {
    if (node.is_conditional)
      tasks.push_back(node.process->generateConditionalTask(input, *(container.taskflow)));
    else
      tasks.push_back(node.process->generateTask(input, *(container.taskflow)));
  }

  for (std::size_t i = 0; i < nodes_.size(); ++i)
  {
    // Ensure the current task precedes the tasks that it is connected to
    const GraphTaskflowNode& node = nodes_[i];
    for (int idx : node.edges)
    {
      if (idx >= 0)
        tasks.at(i).precede(tasks.at(static_cast<std::size_t>(idx)));
      else if (idx == DONE_NODE)
        tasks.at(i).precede(done_task);
      else if (idx == ERROR_NODE)
        tasks.at(i).precede(error_task);
      else
        throw std::runtime_error("Invalid GraphTaskflow: Node specified with invalid edge");
    }

    // If no edges exist for the current node, make sure it precedes the done task (and error task if conditional)
    if (node.edges.empty())
    {
      // Make sure the 0th connection of a conditional task goes to the error task
      if (node.is_conditional)
      {
        tasks.at(i).precede(error_task);
      }
      tasks.at(i).precede(done_task);
    }
  }

  // Assumes the first node added is the input node
  container.input = tasks.front();
  return container;
}

int GraphTaskflow::addNode(TaskGenerator::UPtr process, bool is_conditional)
{
  nodes_.emplace_back(GraphTaskflowNode(std::move(process), is_conditional));
  return static_cast<int>(nodes_.size() - 1);
}

void GraphTaskflow::addEdges(int source, std::vector<int> destinations)
{
  GraphTaskflowNode& node = nodes_.at(static_cast<std::size_t>(source));
  if (destinations.size() > 1 && node.is_conditional)
    node.edges.insert(node.edges.end(), destinations.begin(), destinations.end());
  else if (destinations.size() == 1)
    node.edges.push_back(destinations.front());
  else
    throw std::runtime_error("Multiple edges can only be added to conditional nodes");
}
}  // namespace tesseract_planning
