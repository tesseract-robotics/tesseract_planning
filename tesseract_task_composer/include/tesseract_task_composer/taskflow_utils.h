#ifndef TESSERACT_TASK_COMPOSER_TASKFLOW_UTILS_H
#define TESSERACT_TASK_COMPOSER_TASKFLOW_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_graph.h>
#include <tesseract_task_composer/task_composer_input.h>

namespace tesseract_planning
{
inline std::unique_ptr<tf::Taskflow> convertToTaskflow(TaskComposerGraph& task_composer,
                                                       TaskComposerInput::Ptr task_input)
{
  auto taskflow = std::make_unique<tf::Taskflow>(task_composer.getName());

  // Generate process tasks for each node
  std::vector<tf::Task> tasks;
  const auto& nodes = task_composer.getNodes();
  tasks.reserve(nodes.size());
  for (const auto& node : nodes)
  {
    auto edges = node->getEdges();
    if (edges.size() > 1 && node->getType() == TaskComposerNodeType::CONDITIONAL_TASK)
      tasks.push_back(taskflow->emplace([node, task_input] { return node->run(*task_input); }).name(node->getName()));
    else
      tasks.push_back(taskflow->emplace([node, task_input] { node->run(*task_input); }).name(node->getName()));
  }

  for (std::size_t i = 0; i < nodes.size(); ++i)
  {
    // Ensure the current task precedes the tasks that it is connected to
    const auto& node = nodes[i];
    auto edges = node->getEdges();
    for (int idx : edges)
    {
      if (idx >= 0)
        tasks.at(i).precede(tasks.at(static_cast<std::size_t>(idx)));
      else
        throw std::runtime_error("Invalid TaskComposerGraph: Node specified with invalid edge");
    }
  }

  return taskflow;
}
}  // namespace tesseract_planning
#endif  // TESSERACT_TASK_COMPOSER_TASKFLOW_UTILS_H
