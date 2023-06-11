/**
 * @file task_composer_task.h
 * @brief A task in the pipeline
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_TASK_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/task_composer_node.h>

namespace tesseract_planning
{
class TaskComposerExecutor;
class TaskComposerTask : public TaskComposerNode
{
public:
  using Ptr = std::shared_ptr<TaskComposerTask>;
  using ConstPtr = std::shared_ptr<const TaskComposerTask>;
  using UPtr = std::unique_ptr<TaskComposerTask>;
  using ConstUPtr = std::unique_ptr<const TaskComposerTask>;

  /** @brief Most task will not require a executor so making it optional */
  using OptionalTaskComposerExecutor = std::optional<std::reference_wrapper<TaskComposerExecutor>>;

  TaskComposerTask();
  explicit TaskComposerTask(std::string name, bool is_conditional);
  explicit TaskComposerTask(std::string name, const YAML::Node& config);
  ~TaskComposerTask() override = default;
  TaskComposerTask(const TaskComposerTask&) = delete;
  TaskComposerTask& operator=(const TaskComposerTask&) = delete;
  TaskComposerTask(TaskComposerTask&&) = delete;
  TaskComposerTask& operator=(TaskComposerTask&&) = delete;

  bool operator==(const TaskComposerTask& rhs) const;
  bool operator!=(const TaskComposerTask& rhs) const;

  int run(TaskComposerInput& input, OptionalTaskComposerExecutor executor = std::nullopt) const;

  bool isConditional() const;

  std::string dump(std::ostream& os,
                   const TaskComposerNode* parent = nullptr,
                   const std::map<boost::uuids::uuid, TaskComposerNodeInfo::UPtr>& results_map = {}) const override;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  bool is_conditional_{ true };

  virtual TaskComposerNodeInfo::UPtr runImpl(TaskComposerInput& input,
                                             OptionalTaskComposerExecutor executor = std::nullopt) const = 0;
};

}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TaskComposerTask, "TaskComposerTask")

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_TASK_H
