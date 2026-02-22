/**
 * @file constant_tcp_speed_parameterization.h
 * @brief Constant TCP Speed Parameterization
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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
#ifndef TESSERACT_TASK_COMPOSER_PLANNING_NODES_CONSTANT_TCP_SPEED_PARAMETERIZATION_TASK_H
#define TESSERACT_TASK_COMPOSER_PLANNING_NODES_CONSTANT_TCP_SPEED_PARAMETERIZATION_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_task_composer/planning/tesseract_task_composer_planning_nodes_export.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

#include <tesseract_time_parameterization/kdl/constant_tcp_speed_parameterization.h>

namespace tesseract::task_composer
{
class TaskComposerPluginFactory;
class TESSERACT_TASK_COMPOSER_PLANNING_NODES_EXPORT ConstantTCPSpeedParameterizationTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INOUT_PROGRAM_PORT;
  static const std::string INPUT_ENVIRONMENT_PORT;
  static const std::string INPUT_PROFILES_PORT;

  using Ptr = std::shared_ptr<ConstantTCPSpeedParameterizationTask>;
  using ConstPtr = std::shared_ptr<const ConstantTCPSpeedParameterizationTask>;
  using UPtr = std::unique_ptr<ConstantTCPSpeedParameterizationTask>;
  using ConstUPtr = std::unique_ptr<const ConstantTCPSpeedParameterizationTask>;

  ConstantTCPSpeedParameterizationTask();
  explicit ConstantTCPSpeedParameterizationTask(std::string name,
                                                std::string input_program_key,
                                                std::string input_environment_key,
                                                std::string input_profiles_key,
                                                std::string output_program_key,
                                                bool conditional = true);
  explicit ConstantTCPSpeedParameterizationTask(std::string name,
                                                const YAML::Node& config,
                                                const TaskComposerPluginFactory& plugin_factory);
  ~ConstantTCPSpeedParameterizationTask() override = default;
  ConstantTCPSpeedParameterizationTask(const ConstantTCPSpeedParameterizationTask&) = delete;
  ConstantTCPSpeedParameterizationTask& operator=(const ConstantTCPSpeedParameterizationTask&) = delete;
  ConstantTCPSpeedParameterizationTask(ConstantTCPSpeedParameterizationTask&&) = delete;
  ConstantTCPSpeedParameterizationTask& operator=(ConstantTCPSpeedParameterizationTask&&) = delete;

private:
  tesseract::time_parameterization::ConstantTCPSpeedParameterization solver_;

  static TaskComposerNodePorts ports();

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract::task_composer

#endif  // TESSERACT_TASK_COMPOSER_PLANNING_NODES_CONSTANT_TCP_SPEED_PARAMETERIZATION_TASK_H
