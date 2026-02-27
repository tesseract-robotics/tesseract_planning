/**
 * @file upsample_trajectory_task.h
 *
 * @author Levi Armstrong
 * @date December 15, 2021
 *
 * @copyright Copyright (c) 2021, Levi Armstrong
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
#ifndef TESSERACT_TASK_COMPOSER_UPSAMPLE_TRAJECTORY_TASK_H
#define TESSERACT_TASK_COMPOSER_UPSAMPLE_TRAJECTORY_TASK_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_task_composer/planning/tesseract_task_composer_planning_nodes_export.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_command_language/fwd.h>

namespace tesseract::task_composer
{
class TaskComposerPluginFactory;
/**
 * @brief This is used to upsample the results trajectory based on the longest valid segment length.
 * @note This is primarily useful to run before running time parameterization, because motion planners
 * assume joint interpolated between states. If the points are spaced to fart apart the path between
 * two states may not be a straight line causing collision during execution.
 */
class TESSERACT_TASK_COMPOSER_PLANNING_NODES_EXPORT UpsampleTrajectoryTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INOUT_PROGRAM_PORT;
  static const std::string INPUT_PROFILES_PORT;

  using Ptr = std::shared_ptr<UpsampleTrajectoryTask>;
  using ConstPtr = std::shared_ptr<const UpsampleTrajectoryTask>;
  using UPtr = std::unique_ptr<UpsampleTrajectoryTask>;
  using ConstUPtr = std::unique_ptr<const UpsampleTrajectoryTask>;

  UpsampleTrajectoryTask();
  explicit UpsampleTrajectoryTask(std::string name,
                                  std::string input_program_key,
                                  std::string input_profiles_key,
                                  std::string output_program_key,
                                  bool conditional = false);
  explicit UpsampleTrajectoryTask(std::string name,
                                  const YAML::Node& config,
                                  const TaskComposerPluginFactory& plugin_factory);
  ~UpsampleTrajectoryTask() override = default;
  UpsampleTrajectoryTask(const UpsampleTrajectoryTask&) = delete;
  UpsampleTrajectoryTask& operator=(const UpsampleTrajectoryTask&) = delete;
  UpsampleTrajectoryTask(UpsampleTrajectoryTask&&) = delete;
  UpsampleTrajectoryTask& operator=(UpsampleTrajectoryTask&&) = delete;

private:
  void upsample(tesseract::command_language::CompositeInstruction& composite,
                const tesseract::command_language::CompositeInstruction& current_composite,
                tesseract::command_language::InstructionPoly& start_instruction,
                double longest_valid_segment_length) const;

  static TaskComposerNodePorts ports();

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract::task_composer

#endif  // TESSERACT_TASK_COMPOSER_UPSAMPLE_TRAJECTORY_TASK_H
