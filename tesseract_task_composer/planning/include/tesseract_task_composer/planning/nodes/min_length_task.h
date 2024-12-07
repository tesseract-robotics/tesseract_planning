/**
 * @file min_length_task.h
 * @brief Task for processing the input data so it meets a minimum length. Planners like trajopt need
 * at least 10 states in the trajectory to perform velocity, acceleration and jerk smoothing.
 *
 * @author Levi Armstrong
 * @date November 2. 2020
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
#ifndef TESSERACT_TASK_COMPOSER_MIN_LENGTH_TASK_H
#define TESSERACT_TASK_COMPOSER_MIN_LENGTH_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <tesseract_task_composer/planning/tesseract_task_composer_planning_nodes_export.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;
class TESSERACT_TASK_COMPOSER_PLANNING_NODES_EXPORT MinLengthTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INOUT_PROGRAM_PORT;
  static const std::string INPUT_ENVIRONMENT_PORT;
  static const std::string INPUT_PROFILES_PORT;

  using Ptr = std::shared_ptr<MinLengthTask>;
  using ConstPtr = std::shared_ptr<const MinLengthTask>;
  using UPtr = std::unique_ptr<MinLengthTask>;
  using ConstUPtr = std::unique_ptr<const MinLengthTask>;

  MinLengthTask();
  explicit MinLengthTask(std::string name,
                         std::string input_program_key,
                         std::string input_environment_key,
                         std::string input_profiles_key,
                         std::string output_program_key,
                         bool conditional = false);
  explicit MinLengthTask(std::string name, const YAML::Node& config, const TaskComposerPluginFactory& plugin_factory);
  ~MinLengthTask() override = default;
  MinLengthTask(const MinLengthTask&) = delete;
  MinLengthTask& operator=(const MinLengthTask&) = delete;
  MinLengthTask(MinLengthTask&&) = delete;
  MinLengthTask& operator=(MinLengthTask&&) = delete;

  bool operator==(const MinLengthTask& rhs) const;
  bool operator!=(const MinLengthTask& rhs) const;

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

  static TaskComposerNodePorts ports();

  std::unique_ptr<TaskComposerNodeInfo>
  runImpl(TaskComposerContext& context, OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::MinLengthTask)
#endif  // TESSERACT_TASK_COMPOSER_MIN_LENGTH_TASK_H
