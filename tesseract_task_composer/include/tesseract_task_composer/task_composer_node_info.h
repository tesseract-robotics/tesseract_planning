/**
 * @file task_composer_node_info.h
 * @brief Task composer node info
 *
 * @author Matthew Powelson
 * @date July 15. 2020
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_INFO_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_INFO_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <shared_mutex>
#include <map>
#include <boost/uuid/uuid.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_environment/environment.h>

namespace tesseract_planning
{
/** Stores information about a node */
class TaskComposerNodeInfo
{
public:
  using Ptr = std::shared_ptr<TaskComposerNodeInfo>;
  using ConstPtr = std::shared_ptr<const TaskComposerNodeInfo>;
  using UPtr = std::unique_ptr<TaskComposerNodeInfo>;
  using ConstUPtr = std::unique_ptr<const TaskComposerNodeInfo>;

  TaskComposerNodeInfo() = default;  // Required for serialization
  TaskComposerNodeInfo(boost::uuids::uuid uuid, std::string name = "");
  virtual ~TaskComposerNodeInfo() = default;
  TaskComposerNodeInfo(const TaskComposerNodeInfo&) = default;
  TaskComposerNodeInfo& operator=(const TaskComposerNodeInfo&) = default;
  TaskComposerNodeInfo(TaskComposerNodeInfo&&) = default;
  TaskComposerNodeInfo& operator=(TaskComposerNodeInfo&&) = default;

  /** @brief Value returned from the Task on completion */
  int return_value{ std::numeric_limits<int>::lowest() };

  /** @brief The task uuid */
  boost::uuids::uuid uuid;

  /** @brief The name */
  std::string name;

  std::string message;

  /** @brief elapsed_time Time spent in this task in seconds*/
  double elapsed_time{ 0 };

  /** @brief Instructions passed to task (optionally set) */
  InstructionPoly instructions_input;
  /** @brief Instructions after running the task (optionally set)*/
  InstructionPoly instructions_output;
  /** @brief Seed/Results passed into the task (optionally set) */
  InstructionPoly results_input;
  /** @brief Seed/Results after running the task (optionally set)*/
  InstructionPoly results_output;
  /** @brief The environment at the beginning of the task (optionally set)*/
  tesseract_environment::Environment::ConstPtr environment{ nullptr };

  bool operator==(const TaskComposerNodeInfo& rhs) const;
  bool operator!=(const TaskComposerNodeInfo& rhs) const;

  virtual TaskComposerNodeInfo::UPtr clone() const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/** @brief A threadsafe container for TaskComposerNodeInfo */
struct TaskComposerNodeInfoContainer
{
  using Ptr = std::shared_ptr<TaskComposerNodeInfoContainer>;
  using ConstPtr = std::shared_ptr<const TaskComposerNodeInfoContainer>;
  using UPtr = std::unique_ptr<TaskComposerNodeInfoContainer>;
  using ConstUPtr = std::unique_ptr<const TaskComposerNodeInfoContainer>;

  void addInfo(TaskComposerNodeInfo::UPtr info);

  /** @brief Get a copy of the task_info_map_ in case it gets resized*/
  std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr> getInfoMap() const;

  TaskComposerNodeInfo::UPtr operator[](boost::uuids::uuid key) const;

  bool operator==(const TaskComposerNodeInfoContainer& rhs) const;
  bool operator!=(const TaskComposerNodeInfoContainer& rhs) const;

private:
  mutable std::shared_mutex mutex_;
  std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr> info_map_;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TaskComposerNodeInfo, "TaskComposerNodeInfo")
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TaskComposerNodeInfoContainer, "TaskComposerNodeInfoContainer")

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_NODE_INFO_H
