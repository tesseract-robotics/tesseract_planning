/**
 * @file task_composer_problem.h
 * @brief A task composer server problem
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
#ifndef TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PROBLEM_H
#define TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PROBLEM_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <boost/serialization/access.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_common/any_poly.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

namespace tesseract_planning
{
struct TaskComposerProblem
{
  using Ptr = std::shared_ptr<TaskComposerProblem>;
  using ConstPtr = std::shared_ptr<const TaskComposerProblem>;
  using UPtr = std::unique_ptr<TaskComposerProblem>;
  using ConstUPtr = std::unique_ptr<const TaskComposerProblem>;

  TaskComposerProblem(std::string name = "unset", bool dotgraph = false);

  TaskComposerProblem(const TaskComposerProblem&) = default;
  TaskComposerProblem& operator=(const TaskComposerProblem&) = default;
  TaskComposerProblem(TaskComposerProblem&&) = default;
  TaskComposerProblem& operator=(TaskComposerProblem&&) = default;
  virtual ~TaskComposerProblem() = default;

  /** @brief The name of the task to be ran for this problem */
  std::string name;

  /** @brief Indicate if dotgraph should be provided */
  bool dotgraph{ false };

  /** @brief The problem input */
  tesseract_common::AnyPoly input;

  /**
   * @brief Clone the planning problem
   * @return A clone of the planning problem
   */
  virtual TaskComposerProblem::UPtr clone() const;

  bool operator==(const TaskComposerProblem& rhs) const;
  bool operator!=(const TaskComposerProblem& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_planning::TaskComposerProblem, "TaskComposerProblem")

#endif  // TESSERACT_TASK_COMPOSER_TASK_COMPOSER_PROBLEM_H
