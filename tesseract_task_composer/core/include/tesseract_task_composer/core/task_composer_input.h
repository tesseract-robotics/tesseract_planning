/**
 * @file task_composer_input.h
 * @brief The input data structure to the pipeline
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_INPUT_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_INPUT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <atomic>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_problem.h>

namespace tesseract_planning
{
/**
 * @brief This struct is passed as an input to each process in the decision tree
 *
 * Note that it does not have ownership of any of its members (except the pointer). This means that if a TaskInput
 * spawns a child that is a subset, it does not have to remain in scope as the references will still be valid
 */
struct TaskComposerInput
{
  using Ptr = std::shared_ptr<TaskComposerInput>;
  using ConstPtr = std::shared_ptr<const TaskComposerInput>;
  using UPtr = std::unique_ptr<TaskComposerInput>;
  using ConstUPtr = std::unique_ptr<const TaskComposerInput>;

  TaskComposerInput(TaskComposerProblem::UPtr problem);
  TaskComposerInput(const TaskComposerInput&) = delete;
  TaskComposerInput(TaskComposerInput&&) noexcept = delete;
  TaskComposerInput& operator=(const TaskComposerInput&) = delete;
  TaskComposerInput& operator=(TaskComposerInput&&) = delete;
  virtual ~TaskComposerInput() = default;

  /** @brief The problem */
  TaskComposerProblem::UPtr problem;

  /**
   * @brief The location data is stored and retrieved during execution
   * @details The problem input data is copied into this structure when constructed
   */
  TaskComposerDataStorage data_storage;

  /** @brief The location where task info is stored during execution */
  TaskComposerNodeInfoContainer task_infos;

  /** @brief Indicate if dotgraph should be provided */
  bool dotgraph{ false };

  /**
   * @brief Check if process has been aborted
   * @details This accesses the internal process interface class
   * @return True if aborted otherwise false;
   */
  bool isAborted() const;

  /**
   * @brief If it was not aborted then it was successful
   * @return True if successful, otherwise false
   */
  bool isSuccessful() const;

  /**
   * @brief Abort the process input
   * @note If calling within a node you must provide the uuid
   * @details This accesses the internal process interface class to abort the process
   */
  void abort(const boost::uuids::uuid& calling_node = boost::uuids::uuid());

  /**
   * @brief Abort the process input
   * @note This method should be used if calling abort from within an node
   * @param caller The node calling abort
   */
  void abort(const TaskComposerNode& caller);

  /** @brief Reset abort and data storage to constructed state */
  void reset();

  bool operator==(const TaskComposerInput& rhs) const;
  bool operator!=(const TaskComposerInput& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;

  TaskComposerInput() = default;  // Required for serialization

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  mutable std::atomic<bool> aborted_{ false };
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TaskComposerInput, "TaskComposerInput")

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_INPUT_H
