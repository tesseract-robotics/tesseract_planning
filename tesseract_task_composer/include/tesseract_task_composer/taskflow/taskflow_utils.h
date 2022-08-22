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
struct TaskComposerTaskflowContainer
{
  using Ptr = std::shared_ptr<TaskComposerTaskflowContainer>;
  using ConstPtr = std::shared_ptr<const TaskComposerTaskflowContainer>;

  std::unique_ptr<tf::Taskflow> top;
  std::vector<std::unique_ptr<tf::Taskflow>> children;
};

inline TaskComposerTaskflowContainer::Ptr convertToTaskflow(const TaskComposerGraph& task_graph,
                                                            TaskComposerInput& task_input)
{
  auto tf_container = std::make_shared<TaskComposerTaskflowContainer>();
  tf_container->top = std::make_unique<tf::Taskflow>(task_graph.getName());

  // Generate process tasks for each node
  std::map<boost::uuids::uuid, tf::Task> tasks;
  const auto& nodes = task_graph.getNodes();
  for (auto& pair : nodes)
  {
    auto edges = pair.second->getOutboundEdges();
    if (pair.second->getType() == TaskComposerNodeType::TASK)
    {
      const auto& task = static_cast<const TaskComposerTask&>(*pair.second);
      if (edges.size() > 1 && task.isConditional())
        tasks[pair.first] =
            tf_container->top->emplace([node = pair.second, &task_input] { return node->run(task_input); })
                .name(pair.second->getName());
      else
        tasks[pair.first] = tf_container->top->emplace([node = pair.second, &task_input] { node->run(task_input); })
                                .name(pair.second->getName());
    }
    else if (pair.second->getType() == TaskComposerNodeType::GRAPH)
    {
      const auto& graph = static_cast<const TaskComposerGraph&>(*pair.second);
      TaskComposerTaskflowContainer::Ptr sub_tf_container = convertToTaskflow(graph, task_input);
      tasks[pair.first] = tf_container->top->composed_of(*sub_tf_container->top);
      tf_container->children.push_back(std::move(sub_tf_container->top));
      tf_container->children.insert(tf_container->children.end(),
                                    std::make_move_iterator(sub_tf_container->children.begin()),
                                    std::make_move_iterator(sub_tf_container->children.end()));
    }
    else
      throw std::runtime_error("convertToTaskflow, unsupported node type!");
  }

  for (const auto& pair : nodes)
  {
    // Ensure the current task precedes the tasks that it is connected to
    auto edges = pair.second->getOutboundEdges();
    for (const auto& e : edges)
      tasks[pair.first].precede(tasks[e]);
  }

  return tf_container;
}

inline TaskComposerTaskflowContainer::Ptr convertToTaskflow(const TaskComposerTask& task, TaskComposerInput& task_input)
{
  auto tf_container = std::make_shared<TaskComposerTaskflowContainer>();
  tf_container->top = std::make_unique<tf::Taskflow>(task.getName());
  tf_container->top->emplace([&task, &task_input] { return task.run(task_input); }).name(task.getName());
  return tf_container;
}
}  // namespace tesseract_planning
#endif  // TESSERACT_TASK_COMPOSER_TASKFLOW_UTILS_H
