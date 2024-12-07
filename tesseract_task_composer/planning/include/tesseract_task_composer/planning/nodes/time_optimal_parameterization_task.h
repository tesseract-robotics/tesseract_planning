/**
 * @file time_optimal_parameterization_task.h
 * @brief Perform TOTG
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Jan 22, 2021
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
#ifndef TESSERACT_TASK_COMPOSER_TIME_OPTIMAL_TRAJECTORY_GENERATION_TASK_H
#define TESSERACT_TASK_COMPOSER_TIME_OPTIMAL_TRAJECTORY_GENERATION_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <tesseract_task_composer/planning/tesseract_task_composer_planning_nodes_export.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;
class TESSERACT_TASK_COMPOSER_PLANNING_NODES_EXPORT TimeOptimalParameterizationTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INOUT_PROGRAM_PORT;
  static const std::string INPUT_ENVIRONMENT_PORT;
  static const std::string INPUT_PROFILES_PORT;

  // Optional
  static const std::string INPUT_MANIP_INFO_PORT;

  using Ptr = std::shared_ptr<TimeOptimalParameterizationTask>;
  using ConstPtr = std::shared_ptr<const TimeOptimalParameterizationTask>;
  using UPtr = std::unique_ptr<TimeOptimalParameterizationTask>;
  using ConstUPtr = std::unique_ptr<const TimeOptimalParameterizationTask>;

  TimeOptimalParameterizationTask();
  explicit TimeOptimalParameterizationTask(std::string name,
                                           std::string input_program_key,
                                           std::string input_environment_key,
                                           std::string input_profiles_key,
                                           std::string output_program_key,
                                           bool conditional = true);
  explicit TimeOptimalParameterizationTask(std::string name,
                                           const YAML::Node& config,
                                           const TaskComposerPluginFactory& /*plugin_factory*/);
  ~TimeOptimalParameterizationTask() override = default;
  TimeOptimalParameterizationTask(const TimeOptimalParameterizationTask&) = delete;
  TimeOptimalParameterizationTask& operator=(const TimeOptimalParameterizationTask&) = delete;
  TimeOptimalParameterizationTask(TimeOptimalParameterizationTask&&) = delete;
  TimeOptimalParameterizationTask& operator=(TimeOptimalParameterizationTask&&) = delete;

  bool operator==(const TimeOptimalParameterizationTask& rhs) const;
  bool operator!=(const TimeOptimalParameterizationTask& rhs) const;

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

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TimeOptimalParameterizationTask)
#endif  // TESSERACT_TASK_COMPOSER_TIME_OPTIMAL_TRAJECTORY_GENERATION_TASK_H
