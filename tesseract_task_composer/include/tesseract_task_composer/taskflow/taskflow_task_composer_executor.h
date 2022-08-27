/**
 * @file taskflow_task_composer_executor.h
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
#ifndef TESSERACT_TASK_COMPOSER_TASKFLOW_TASK_COMPOSER_EXECUTOR_H
#define TESSERACT_TASK_COMPOSER_TASKFLOW_TASK_COMPOSER_EXECUTOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_executor.h>
#include <tesseract_task_composer/task_composer_graph.h>
#include <tesseract_task_composer/task_composer_task.h>
#include <tesseract_task_composer/task_composer_input.h>

namespace tf
{
class Executor;
class Taskflow;
}  // namespace tf

namespace tesseract_planning
{
class TaskflowTaskComposerExecutor : public TaskComposerExecutor
{
public:
  using Ptr = std::shared_ptr<TaskflowTaskComposerExecutor>;
  using ConstPtr = std::shared_ptr<const TaskflowTaskComposerExecutor>;
  using UPtr = std::unique_ptr<TaskflowTaskComposerExecutor>;
  using ConstUPtr = std::unique_ptr<const TaskflowTaskComposerExecutor>;

  TaskflowTaskComposerExecutor(std::string name = "TaskflowTaskComposerExecutor",
                               size_t num_threads = std::thread::hardware_concurrency());
  ~TaskflowTaskComposerExecutor() override;

  TaskComposerFuture::UPtr run(const TaskComposerGraph& task_graph, TaskComposerInput& task_input) override final;

  TaskComposerFuture::UPtr run(const TaskComposerTask& task, TaskComposerInput& task_input) override final;

  long getWorkerCount() const override final;

  long getTaskCount() const override final;

  bool operator==(const TaskflowTaskComposerExecutor& rhs) const;
  bool operator!=(const TaskflowTaskComposerExecutor& rhs) const;

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void save(Archive& ar, const unsigned int version) const;  // NOLINT

  template <class Archive>
  void load(Archive& ar, const unsigned int version);  // NOLINT

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  std::size_t num_threads_;
  std::unique_ptr<tf::Executor> executor_;

  std::shared_ptr<std::vector<std::unique_ptr<tf::Taskflow>>> convertToTaskflow(const TaskComposerGraph& task_graph,
                                                                                TaskComposerInput& task_input,
                                                                                TaskComposerExecutor& task_executor);

  std::shared_ptr<std::vector<std::unique_ptr<tf::Taskflow>>> convertToTaskflow(const TaskComposerTask& task,
                                                                                TaskComposerInput& task_input,
                                                                                TaskComposerExecutor& task_executor);
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TaskflowTaskComposerExecutor, "TaskflowTaskComposerExecutor")

#endif  // TESSERACT_TASK_COMPOSER_TASKFLOW_TASK_COMPOSER_EXECUTOR_H
