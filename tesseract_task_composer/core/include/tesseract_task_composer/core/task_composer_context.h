/**
 * @file task_composer_context.h
 * @brief The context data structure to the pipeline
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_CONTEXT_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_CONTEXT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <atomic>
#include <memory>
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
struct TaskComposerContext
{
  using Ptr = std::shared_ptr<TaskComposerContext>;
  using ConstPtr = std::shared_ptr<const TaskComposerContext>;
  using UPtr = std::unique_ptr<TaskComposerContext>;
  using ConstUPtr = std::unique_ptr<const TaskComposerContext>;

  TaskComposerContext() = default;  // Required for serialization
  TaskComposerContext(TaskComposerProblem::UPtr problem);
  TaskComposerContext(const TaskComposerContext&) = delete;
  TaskComposerContext(TaskComposerContext&&) noexcept = delete;
  TaskComposerContext& operator=(const TaskComposerContext&) = delete;
  TaskComposerContext& operator=(TaskComposerContext&&) = delete;
  virtual ~TaskComposerContext() = default;

  /** @brief The problem */
  TaskComposerProblem& getProblem();
  const TaskComposerProblem& getProblem() const;

  /**
   * @brief The location data is stored and retrieved during execution
   * @details The problem input data is copied into this structure when constructed
   */
  TaskComposerDataStorage& getDataStorage();
  const TaskComposerDataStorage& getDataStorage() const;

  /** @brief The location where task info is stored during execution */
  TaskComposerNodeInfoContainer& getTaskInfos();
  const TaskComposerNodeInfoContainer& getTaskInfos() const;

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

  /**
   * @brief Create a child context which is used for dynamic tasking
   * @details
   *   - Everything is shared between the parent and the child except abort_ and task_infos_.
   *   - Task infos should be merged into the parent by the developer
   */
  TaskComposerContext::UPtr createChild();

  bool operator==(const TaskComposerContext& rhs) const;
  bool operator!=(const TaskComposerContext& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  /** @brief The problem */
  TaskComposerProblem::Ptr problem_;

  /**
   * @brief The location data is stored and retrieved during execution
   * @details The problem input data is copied into this structure when constructed
   */
  TaskComposerDataStorage::Ptr data_storage_;

  /** @brief The location where task info is stored during execution */
  TaskComposerNodeInfoContainer task_infos_;

  mutable std::atomic<bool> aborted_{ false };
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TaskComposerContext, "TaskComposerContext")

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_CONTEXT_H
