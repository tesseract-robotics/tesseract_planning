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
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/timer.h>
#include <taskflow/taskflow.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

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

void TaskflowTaskComposerExecutor::removeFuture(const boost::uuids::uuid& uuid)
{
  std::unique_lock<std::mutex> lock(futures_mutex_);
  futures_.erase(uuid);
}

TaskComposerFuture::UPtr TaskflowTaskComposerExecutor::run(const TaskComposerNode& node,
                                                           TaskComposerContext::Ptr context)
{
  auto taskflow = std::make_unique<tf::Taskflow>(node.getName());
  if (node.getType() == TaskComposerNodeType::TASK)
    convertToTaskflow(static_cast<const TaskComposerTask&>(node), *context, *this, taskflow.get());
  else if (node.getType() == TaskComposerNodeType::PIPELINE)
    convertToTaskflow(static_cast<const TaskComposerPipeline&>(node), *context, *this, taskflow.get());
  else if (node.getType() == TaskComposerNodeType::GRAPH)
    convertToTaskflow(static_cast<const TaskComposerGraph&>(node), *context, *this, taskflow.get(), nullptr);
  else
    throw std::runtime_error("TaskComposerExecutor, unsupported node type!");

  // Inorder to better support dynamic tasking within pipelines we store all futures internally
  // and cleanup when finished because the data cannot go out of scope.
  std::unique_lock<std::mutex> lock(futures_mutex_);
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  std::shared_future<void> f = executor_->run(*taskflow, [this, uuid]() { removeFuture(uuid); });
  auto future = std::make_unique<TaskflowTaskComposerFuture>(f, std::move(taskflow), std::move(context));
  futures_[uuid] = future->copy();
  return future;
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

tf::Task TaskflowTaskComposerExecutor::convertToTaskflow(const TaskComposerGraph& task_graph,
                                                         TaskComposerContext& task_context,
                                                         TaskComposerExecutor& task_executor,
                                                         tf::Taskflow* taskflow,
                                                         tf::Subflow* parent_sbf)
{
  auto fn = [&task_graph, &task_context, &task_executor](tf::Subflow& subflow) {
    tesseract_common::Timer timer;
    timer.start();

    // Node Info
    auto info = std::make_unique<TaskComposerNodeInfo>(task_graph);
    info->color = "green";
    info->input_keys = task_graph.getInputKeys();
    info->output_keys = task_graph.getOutputKeys();
    info->start_time = std::chrono::system_clock::now();

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
              subflow.emplace([task, &task_context, &task_executor] { return task->run(task_context, task_executor); })
                  .name(pair.second->getName());
        else
          tasks[pair.first] =
              subflow.emplace([task, &task_context, &task_executor] { task->run(task_context, task_executor); })
                  .name(pair.second->getName());
      }
      else if (pair.second->getType() == TaskComposerNodeType::PIPELINE)
      {
        auto pipeline = std::static_pointer_cast<const TaskComposerPipeline>(pair.second);
        if (edges.size() > 1 && pipeline->isConditional())
          tasks[pair.first] = subflow
                                  .emplace([pipeline, &task_context, &task_executor] {
                                    return pipeline->run(task_context, task_executor);
                                  })
                                  .name(pair.second->getName());
        else
          tasks[pair.first] =
              subflow.emplace([pipeline, &task_context, &task_executor] { pipeline->run(task_context, task_executor); })
                  .name(pair.second->getName());
      }
      else if (pair.second->getType() == TaskComposerNodeType::GRAPH)
      {
        const auto& graph = static_cast<const TaskComposerGraph&>(*pair.second);
        tasks[pair.first] = convertToTaskflow(graph, task_context, task_executor, nullptr, &subflow);
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
    subflow.join();
    timer.stop();
    info->elapsed_time = timer.elapsedSeconds();
    task_context.task_infos.addInfo(std::move(info));
  };

  if (parent_sbf != nullptr)
    return parent_sbf->emplace(fn).name(task_graph.getName());

  return taskflow->emplace(fn).name(task_graph.getName());
}

void TaskflowTaskComposerExecutor::convertToTaskflow(const TaskComposerPipeline& task_pipeline,
                                                     TaskComposerContext& task_context,
                                                     TaskComposerExecutor& task_executor,
                                                     tf::Taskflow* taskflow)
{
  taskflow
      ->emplace(
          [&task_pipeline, &task_context, &task_executor] { return task_pipeline.run(task_context, task_executor); })
      .name(task_pipeline.getName());
}

void TaskflowTaskComposerExecutor::convertToTaskflow(const TaskComposerTask& task,
                                                     TaskComposerContext& task_context,
                                                     TaskComposerExecutor& task_executor,
                                                     tf::Taskflow* taskflow)
{
  taskflow->emplace([&task, &task_context, &task_executor] { return task.run(task_context, task_executor); })
      .name(task.getName());
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskflowTaskComposerExecutor)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskflowTaskComposerExecutor)
