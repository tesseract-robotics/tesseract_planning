/**
 * @file task_composer_executor.h
 * @brief The executor for executing task graphs
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_EXECUTOR_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_EXECUTOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_task_composer/core/task_composer_pipeline.h>
#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_problem.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_future.h>

namespace tesseract_planning
{
class TaskComposerExecutor
{
public:
  using Ptr = std::shared_ptr<TaskComposerExecutor>;
  using ConstPtr = std::shared_ptr<const TaskComposerExecutor>;
  using UPtr = std::unique_ptr<TaskComposerExecutor>;
  using ConstUPtr = std::unique_ptr<const TaskComposerExecutor>;

  TaskComposerExecutor(std::string name = "TaskComposerExecutor");
  virtual ~TaskComposerExecutor() = default;

  /** @brief Get the name of the executor */
  const std::string& getName() const;

  /**
   * @brief Execute the provided node
   * @param node The node to execute
   * @param problem The problem
   * @param data_storage The data storage object to leverage
   * @return The future associated with execution
   */
  TaskComposerFuture::UPtr run(const TaskComposerNode& node,
                               TaskComposerProblem::Ptr problem,
                               TaskComposerDataStorage::Ptr data_storage = std::make_shared<TaskComposerDataStorage>());

  /** @brief Queries the number of workers (example: number of threads) */
  virtual long getWorkerCount() const = 0;

  /** @brief Queries the number of running tasks at the time of this call */
  virtual long getTaskCount() const = 0;

  bool operator==(const TaskComposerExecutor& rhs) const;
  bool operator!=(const TaskComposerExecutor& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  std::string name_;

  /**
   * @brief Execute provided node provide the cotext
   * @details This should only be used for dynamic tasking
   * @param node The node to execute
   * @param context The context
   * @return The future associated with execution
   */
  virtual TaskComposerFuture::UPtr run(const TaskComposerNode& node, TaskComposerContext::Ptr context) = 0;
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TaskComposerExecutor, "TaskComposerExecutor")

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_EXECUTOR_H
