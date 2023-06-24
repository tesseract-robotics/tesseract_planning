/**
 * @file taskflow_task_composer_executor.hcpp
 * @brief The tasflow executor implementation
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

#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_future.h>
#include <taskflow/taskflow.hpp>

namespace tesseract_planning
{
TaskflowTaskComposerExecutor::TaskflowTaskComposerExecutor(size_t num_threads)
  : TaskComposerExecutor("TaskflowExecutor")
  , num_threads_(num_threads)
  , executor_(std::make_unique<tf::Executor>(num_threads_))
{
}
TaskflowTaskComposerExecutor::TaskflowTaskComposerExecutor(std::string name, size_t num_threads)
  : TaskComposerExecutor(std::move(name))
  , num_threads_(num_threads)
  , executor_(std::make_unique<tf::Executor>(num_threads_))
{
}

TaskflowTaskComposerExecutor::TaskflowTaskComposerExecutor(std::string name, const YAML::Node& config)
  : TaskComposerExecutor(std::move(name)), num_threads_(std::thread::hardware_concurrency())
{
  try
  {
    if (YAML::Node n = config["threads"])
    {
      auto t = n.as<int>();
      if (t > 0)
        num_threads_ = static_cast<std::size_t>(t);
      else
        throw std::runtime_error("TaskflowTaskComposerExecutor: entry 'threads' must be greater than zero");
    }

    executor_ = std::make_unique<tf::Executor>(num_threads_);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("TaskflowTaskComposerExecutor: Failed to parse yaml config data! Details: " +
                             std::string(e.what()));
  }
}

TaskflowTaskComposerExecutor::~TaskflowTaskComposerExecutor() = default;

TaskComposerFuture::UPtr TaskflowTaskComposerExecutor::run(const TaskComposerNode& node, TaskComposerInput& task_input)
{
  std::shared_ptr<std::vector<std::unique_ptr<tf::Taskflow>>> taskflow;
  if (node.getType() == TaskComposerNodeType::TASK)
    taskflow = convertToTaskflow(static_cast<const TaskComposerTask&>(node), task_input, *this);
  else if (node.getType() == TaskComposerNodeType::PIPELINE)
    taskflow = convertToTaskflow(static_cast<const TaskComposerPipeline&>(node), task_input, *this);
  else if (node.getType() == TaskComposerNodeType::GRAPH)
    taskflow = convertToTaskflow(static_cast<const TaskComposerGraph&>(node), task_input, *this);
  else
    throw std::runtime_error("TaskComposerExecutor, unsupported node type!");

  //  std::ofstream out_data;
  //  out_data.open(tesseract_common::getTempPath() + "task_composer_example.dot");
  //  taskflow.top->dump(out_data);  // dump the graph including dynamic tasks
  //  out_data.close();

  std::shared_future<void> f = executor_->run(*(taskflow->front()));
  return std::make_unique<TaskflowTaskComposerFuture>(f, std::move(taskflow));
}

long TaskflowTaskComposerExecutor::getWorkerCount() const { return static_cast<long>(executor_->num_workers()); }

long TaskflowTaskComposerExecutor::getTaskCount() const { return static_cast<long>(executor_->num_topologies()); }

bool TaskflowTaskComposerExecutor::operator==(const TaskflowTaskComposerExecutor& rhs) const
{
  bool equal = true;
  equal &= (num_threads_ == rhs.num_threads_);
  equal &= TaskComposerExecutor::operator==(rhs);
  return equal;
}

bool TaskflowTaskComposerExecutor::operator!=(const TaskflowTaskComposerExecutor& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void TaskflowTaskComposerExecutor::save(Archive& ar, const unsigned int /*version*/) const
{
  ar& BOOST_SERIALIZATION_NVP(num_threads_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerExecutor);
}

template <class Archive>
void TaskflowTaskComposerExecutor::load(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(num_threads_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerExecutor);

  executor_ = std::make_unique<tf::Executor>(num_threads_);
}

template <class Archive>
void TaskflowTaskComposerExecutor::serialize(Archive& ar, const unsigned int version)
{
  boost::serialization::split_member(ar, *this, version);
}

std::shared_ptr<std::vector<std::unique_ptr<tf::Taskflow>>>
TaskflowTaskComposerExecutor::convertToTaskflow(const TaskComposerGraph& task_graph,
                                                TaskComposerInput& task_input,
                                                TaskComposerExecutor& task_executor)
{
  auto tf_container = std::make_shared<std::vector<std::unique_ptr<tf::Taskflow>>>();
  tf_container->emplace_back(std::make_unique<tf::Taskflow>(task_graph.getName()));

  // Generate process tasks for each node
  std::map<boost::uuids::uuid, tf::Task> tasks;
  const auto& nodes = task_graph.getNodes();
  for (const auto& pair : nodes)
  {
    auto edges = pair.second->getOutboundEdges();
    if (pair.second->getType() == TaskComposerNodeType::TASK)
    {
      auto task = std::static_pointer_cast<const TaskComposerTask>(pair.second);
      if (edges.size() > 1 && task->isConditional())
        tasks[pair.first] =
            tf_container->front()
                ->emplace([task, &task_input, &task_executor] { return task->run(task_input, task_executor); })
                .name(pair.second->getName());
      else
        tasks[pair.first] = tf_container->front()
                                ->emplace([task, &task_input, &task_executor] { task->run(task_input, task_executor); })
                                .name(pair.second->getName());
    }
    else if (pair.second->getType() == TaskComposerNodeType::PIPELINE)
    {
      auto pipeline = std::static_pointer_cast<const TaskComposerPipeline>(pair.second);
      if (edges.size() > 1 && pipeline->isConditional())
        tasks[pair.first] =
            tf_container->front()
                ->emplace([pipeline, &task_input, &task_executor] { return pipeline->run(task_input, task_executor); })
                .name(pair.second->getName());
      else
        tasks[pair.first] =
            tf_container->front()
                ->emplace([pipeline, &task_input, &task_executor] { pipeline->run(task_input, task_executor); })
                .name(pair.second->getName());
    }
    else if (pair.second->getType() == TaskComposerNodeType::GRAPH)
    {
      const auto& graph = static_cast<const TaskComposerGraph&>(*pair.second);

      // Must add a Node Info object for the graph
      auto info = std::make_unique<TaskComposerNodeInfo>(graph);
      info->color = "green";
      task_input.task_infos.addInfo(std::move(info));

      auto sub_tf_container = convertToTaskflow(graph, task_input, task_executor);
      tasks[pair.first] = tf_container->front()->composed_of(*sub_tf_container->front());
      tf_container->insert(tf_container->end(),
                           std::make_move_iterator(sub_tf_container->begin()),
                           std::make_move_iterator(sub_tf_container->end()));
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

std::shared_ptr<std::vector<std::unique_ptr<tf::Taskflow>>>
TaskflowTaskComposerExecutor::convertToTaskflow(const TaskComposerPipeline& task_pipeline,
                                                TaskComposerInput& task_input,
                                                TaskComposerExecutor& task_executor)
{
  auto tf_container = std::make_shared<std::vector<std::unique_ptr<tf::Taskflow>>>();
  tf_container->emplace_back(std::make_unique<tf::Taskflow>(task_pipeline.getName()));
  tf_container->front()
      ->emplace([&task_pipeline, &task_input, &task_executor] { return task_pipeline.run(task_input, task_executor); })
      .name(task_pipeline.getName());
  return tf_container;
}

std::shared_ptr<std::vector<std::unique_ptr<tf::Taskflow>>>
TaskflowTaskComposerExecutor::convertToTaskflow(const TaskComposerTask& task,
                                                TaskComposerInput& task_input,
                                                TaskComposerExecutor& task_executor)
{
  auto tf_container = std::make_shared<std::vector<std::unique_ptr<tf::Taskflow>>>();
  tf_container->emplace_back(std::make_unique<tf::Taskflow>(task.getName()));
  tf_container->front()
      ->emplace([&task, &task_input, &task_executor] { return task.run(task_input, task_executor); })
      .name(task.getName());
  return tf_container;
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskflowTaskComposerExecutor)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskflowTaskComposerExecutor)
