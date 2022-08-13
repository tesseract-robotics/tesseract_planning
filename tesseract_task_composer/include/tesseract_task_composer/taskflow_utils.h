#ifndef TESSERACT_TASK_COMPOSER_TASKFLOW_UTILS_H
#define TESSERACT_TASK_COMPOSER_TASKFLOW_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_graph.h>
#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_input.h>

namespace tesseract_planning
{
inline void convertToTaskflowRecursive(tf::Taskflow& taskflow,
                                       const TaskComposerGraph& task_composer,
                                       TaskComposerInput::Ptr task_input)
{
  // Generate process tasks for each node
  std::map<boost::uuids::uuid, tf::Task> tasks;
  const auto& nodes = task_composer.getNodes();
  //  tasks.reserve(nodes.size());
  for (auto& pair : nodes)
  {
    auto edges = pair.second->getEdges();
    if (pair.second->getType() == TaskComposerNodeType::TASK)
    {
      const auto& task = static_cast<const TaskComposerTask&>(*pair.second);
      if (edges.size() > 1 && task.isConditional())
        tasks[pair.first] = taskflow.emplace([node = pair.second, task_input] { return node->run(*task_input); })
                                .name(pair.second->getName());
      else
        tasks[pair.first] =
            taskflow.emplace([node = pair.second, task_input] { node->run(*task_input); }).name(pair.second->getName());
    }
    else if (pair.second->getType() == TaskComposerNodeType::GRAPH)
    {
      const auto& graph = static_cast<const TaskComposerGraph&>(*pair.second);
      tf::Taskflow sub_taskflow;
      convertToTaskflowRecursive(sub_taskflow, graph, task_input);
      tasks[pair.first] = taskflow.composed_of(sub_taskflow);
    }
    else
      throw std::runtime_error("convertToTaskflow, unsupported node type!");
  }

  for (const auto& pair : nodes)
  {
    // Ensure the current task precedes the tasks that it is connected to
    auto edges = pair.second->getEdges();
    for (const auto& e : edges)
      tasks[pair.first].precede(tasks[e]);
  }
}

inline std::unique_ptr<tf::Taskflow> convertToTaskflow(const TaskComposerGraph& task_composer,
                                                       TaskComposerInput::Ptr task_input)
{
  auto taskflow = std::make_unique<tf::Taskflow>(task_composer.getName());
  convertToTaskflowRecursive(*taskflow, task_composer, task_input);
  return taskflow;
}
}  // namespace tesseract_planning
#endif  // TESSERACT_TASK_COMPOSER_TASKFLOW_UTILS_H
