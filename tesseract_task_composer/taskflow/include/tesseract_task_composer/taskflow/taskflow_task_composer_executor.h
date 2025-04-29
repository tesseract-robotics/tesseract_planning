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
#include <mutex>
#include <map>
#include <thread>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_executor.h>

namespace tf
{
class Executor;
class Taskflow;
class Subflow;
class Task;
}  // namespace tf

namespace YAML
{
class Node;
}

namespace boost::uuids
{
struct uuid;
}

namespace tesseract_planning
{
class TaskComposerPipeline;
class TaskComposerTask;
class TaskComposerGraph;

class TaskflowTaskComposerExecutor : public TaskComposerExecutor
{
public:
  using Ptr = std::shared_ptr<TaskflowTaskComposerExecutor>;
  using ConstPtr = std::shared_ptr<const TaskflowTaskComposerExecutor>;
  using UPtr = std::unique_ptr<TaskflowTaskComposerExecutor>;
  using ConstUPtr = std::unique_ptr<const TaskflowTaskComposerExecutor>;

  TaskflowTaskComposerExecutor(std::string name = "TaskflowExecutor",
                               size_t num_threads = std::thread::hardware_concurrency());
  TaskflowTaskComposerExecutor(std::string name, const YAML::Node& config);
  TaskflowTaskComposerExecutor(size_t num_threads);
  ~TaskflowTaskComposerExecutor() override;
  TaskflowTaskComposerExecutor(const TaskflowTaskComposerExecutor&) = delete;
  TaskflowTaskComposerExecutor& operator=(const TaskflowTaskComposerExecutor&) = delete;
  TaskflowTaskComposerExecutor(TaskflowTaskComposerExecutor&&) = delete;
  TaskflowTaskComposerExecutor& operator=(TaskflowTaskComposerExecutor&&) = delete;

  long getWorkerCount() const override final;

  long getTaskCount() const override final;

  bool operator==(const TaskflowTaskComposerExecutor& rhs) const;
  bool operator!=(const TaskflowTaskComposerExecutor& rhs) const;

private:
  std::size_t num_threads_;
  std::unique_ptr<tf::Executor> executor_;

  std::mutex futures_mutex_;
  std::map<boost::uuids::uuid, std::unique_ptr<TaskComposerFuture>> futures_;
  void removeFuture(const boost::uuids::uuid& uuid);

  std::unique_ptr<TaskComposerFuture> run(const TaskComposerNode& node,
                                          std::shared_ptr<TaskComposerContext> context) override final;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void save(Archive& ar, const unsigned int version) const;  // NOLINT

  template <class Archive>
  void load(Archive& ar, const unsigned int version);  // NOLINT

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TaskflowTaskComposerExecutor)

#endif  // TESSERACT_TASK_COMPOSER_TASKFLOW_TASK_COMPOSER_EXECUTOR_H
