/**
 * @file task_composer_future.h
 * @brief A task composer future
 *
 * @author Levi Armstrong
 * @date August 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_FUTURE_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_FUTURE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <chrono>
#include <future>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_context.h>

namespace tesseract_planning
{
/**
 * @brief This contains the result for the task composer request
 * @details Also this must be copyable so recommend using shared future or something comparable
 * @details Must check the status before access the results to know if available.
 * @note This stores a shared future and is copy-able to allow access from multiple threads
 * @note This must not go out of scope until the process has finished
 */
class TaskComposerFuture
{
public:
  using Ptr = std::shared_ptr<TaskComposerFuture>;
  using ConstPtr = std::shared_ptr<const TaskComposerFuture>;
  using UPtr = std::unique_ptr<TaskComposerFuture>;
  using ConstUPtr = std::unique_ptr<const TaskComposerFuture>;

  TaskComposerFuture() = default;
  TaskComposerFuture(TaskComposerContext::Ptr context) : context(std::move(context)) {}
  virtual ~TaskComposerFuture() = default;

  TaskComposerContext::Ptr context;

  /** @brief Clear all content */
  virtual void clear() = 0;

  /** @brief Checks if the future has a shared state */
  virtual bool valid() const = 0;

  /**
   * @brief This checks if the tasks are finished
   * @return True if the tasks finished, otherwise false
   */
  virtual bool ready() const = 0;

  /** @brief Wait until the process has finished */
  virtual void wait() const = 0;

  /**
   * @brief Check if a process has finished for a given duration
   * @return The future status
   */
  virtual std::future_status waitFor(const std::chrono::duration<double>& duration) const = 0;

  /**
   * @brief Check if a process has finished up to a given time point
   * @return The future status
   */
  virtual std::future_status
  waitUntil(const std::chrono::time_point<std::chrono::high_resolution_clock>& abs) const = 0;

  /**
   * @brief Make a copy of the future
   * @return A copy, for example to allow access from multiple thread
   */
  virtual TaskComposerFuture::UPtr copy() const = 0;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_FUTURE_H
