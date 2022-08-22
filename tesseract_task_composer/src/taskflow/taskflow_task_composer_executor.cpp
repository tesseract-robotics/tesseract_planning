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
#include <tesseract_task_composer/taskflow/taskflow_utils.h>
#include <taskflow/taskflow.hpp>

namespace tesseract_planning
{
TaskflowTaskComposerExecutor::TaskflowTaskComposerExecutor(std::string name, size_t num_threads)
  : TaskComposerExecutor(std::move(name))
  , num_threads_(num_threads)
  , executor_(std::make_unique<tf::Executor>(num_threads_))
{
}

TaskflowTaskComposerExecutor::~TaskflowTaskComposerExecutor() {}

TaskComposerFuture::UPtr TaskflowTaskComposerExecutor::run(const TaskComposerGraph& task_graph,
                                                           TaskComposerInput& task_input)
{
  TaskComposerTaskflowContainer::ConstPtr taskflow = convertToTaskflow(task_graph, task_input);
  std::shared_future<void> f = executor_->run(*(taskflow->top));

  //  std::ofstream out_data;
  //  out_data.open(tesseract_common::getTempPath() + "task_composer_example.dot");
  //  taskflow.top->dump(out_data);  // dump the graph including dynamic tasks
  //  out_data.close();

  return std::make_unique<TaskflowTaskComposerFuture>(f, std::move(taskflow));
}

TaskComposerFuture::UPtr TaskflowTaskComposerExecutor::run(const TaskComposerTask& task, TaskComposerInput& task_input)
{
  TaskComposerTaskflowContainer::ConstPtr taskflow = convertToTaskflow(task, task_input);
  std::shared_future<void> f = executor_->run(*(taskflow->top));

  //  std::ofstream out_data;
  //  out_data.open(tesseract_common::getTempPath() + "task_composer_example.dot");
  //  taskflow.top->dump(out_data);  // dump the graph including dynamic tasks
  //  out_data.close();

  return std::make_unique<TaskflowTaskComposerFuture>(f, std::move(taskflow));
}

long TaskflowTaskComposerExecutor::getWorkerCount() const { return static_cast<long>(executor_->num_workers()); }

long TaskflowTaskComposerExecutor::getTaskCount() const { return static_cast<long>(executor_->num_topologies()); }

bool TaskflowTaskComposerExecutor::operator==(const TaskflowTaskComposerExecutor& rhs) const
{
  bool equal = true;
  equal &= (num_threads_ == rhs.num_threads_);
  equal &= (executor_ == rhs.executor_);
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

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TaskflowTaskComposerExecutor)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TaskflowTaskComposerExecutor)
