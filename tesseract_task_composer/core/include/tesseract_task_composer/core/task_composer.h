/**
 * @file task_composer.h
 * @brief A task graph
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_H

#include <tesseract_task_composer/task_composer_graph.h>
#include <tesseract_task_composer/task_composer_graph.h>

namespace tesseract_planning
{
class TaskComposer
{
public:
  using Ptr = std::shared_ptr<TaskComposer>;
  using ConstPtr = std::shared_ptr<const TaskComposer>;
  using UPtr = std::unique_ptr<TaskComposer>;
  using ConstUPtr = std::unique_ptr<const TaskComposer>;

  TaskComposer(std::string name = "TaskComposer");
  virtual ~TaskComposer() = default;
  TaskComposer(const TaskComposer&) = default;
  TaskComposer& operator=(const TaskComposer&) = default;
  TaskComposer(TaskComposer&&) = default;
  TaskComposer& operator=(TaskComposer&&) = default;

  TaskComposerGraph::UPtr generate(TaskComposerInput& input) const = 0;

  bool operator==(const TaskComposerGraph& rhs) const;
  bool operator!=(const TaskComposerGraph& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  std::vector<TaskComposerNode::Ptr> nodes_;
}
}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_H
