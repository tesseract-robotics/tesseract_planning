/**
 * @file taskflow_visualization_utils.h
 * @brief Utils for visualizing taskflows
 *
 * @author Matthew Powelson
 * @date June 22. 2021
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_PROCESS_MANAGERS_TASKFLOW_VISUALIZATION_UTILS_H
#define TESSERACT_PROCESS_MANAGERS_TASKFLOW_VISUALIZATION_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_info.h>
#include <tesseract_process_managers/taskflow_generators/graph_taskflow.h>
#include <tesseract_process_managers/utils/task_info_statistics.h>

namespace tesseract_planning
{
/**
 * @brief Contains additional information about a task to be displayed
 */
struct TaskDisplayInfo
{
  std::string task_info;
  std::map<int, std::string> edge_info;
};

/**
 * @brief Utility for a custom graphviz dump for graph taskflows that allows the addition of addition information
 * @param os Stream to which the dot file is dumped
 * @param graph GraphTaskflow dumped to dot file
 * @param task_info_map Key: Task name, Value: Information about that task to add to the visualization
 */
inline void dump(std::ostream& os,
                 const GraphTaskflow::UPtr& graph,
                 const std::unordered_map<std::string, TaskDisplayInfo>& task_info_map =
                     std::unordered_map<std::string, TaskDisplayInfo>())
{
  os << "digraph Taskflow {\n";
  {
    os << "subgraph cluster"
       << " {\nlabel=\"Taskflow:\";\n";
    {
      const std::vector<GraphTaskflowNode>& nodes = graph->getNodes();
      for (std::size_t idx = 0; idx < nodes.size(); idx++)
      {
        const GraphTaskflowNode& node = nodes[idx];
        const std::string node_name = node.process->getName();
        TaskDisplayInfo task_display_info;
        if (task_info_map.find(node_name) != task_info_map.end())
          task_display_info = task_info_map.at(node_name);

        // Add the node
        os << "node_" + std::to_string(idx + 2);
        os << " [label=<" << node_name;
        if (!task_display_info.task_info.empty())
          os << "<BR /> <FONT POINT-SIZE=\"10\">" << task_display_info.task_info << "</FONT>";
        os << ">";

        if (node.is_conditional)
          os << " shape=diamond color=black fillcolor=aquamarine style=filled";
        os << " ];\n";

        // Add the edge
        int return_value = 0;
        for (const int& edge : node.edges)
        {
          os << "node_" + std::to_string(idx + 2);
          os << " -> ";
          os << "node_" + std::to_string(edge + 2);
          os << "[style=dashed label=\"" + std::to_string(return_value);
          if (!task_display_info.edge_info[return_value].empty())
            os << ": " << task_display_info.edge_info[return_value];
          os << "\" ];\n";
          return_value++;
        }
      }
    }
    os << "}\n";
  }
  os << "}\n";
}

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_task_info_H
