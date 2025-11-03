/**
 * @file format_planning_input_task.h
 *
 * @brief This is used to format a composite instruction for motion planning. C
 * Currently it reformats joint waypoint, state waypoint and cartesian seed such
 * that it aligns with the manipulator joint names ordering.
 *
 * @copyright Copyright (c) 2024, Levi Armstrong
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
#ifndef TESSERACT_TASK_COMPOSERFORMAT_PLANNING_INPUT_TASK_H
#define TESSERACT_TASK_COMPOSERFORMAT_PLANNING_INPUT_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_task_composer/planning/tesseract_task_composer_planning_nodes_export.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;

class TESSERACT_TASK_COMPOSER_PLANNING_NODES_EXPORT FormatPlanningInputTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INOUT_PROGRAM_PORT;
  static const std::string INPUT_ENVIRONMENT_PORT;

  using Ptr = std::shared_ptr<FormatPlanningInputTask>;
  using ConstPtr = std::shared_ptr<const FormatPlanningInputTask>;
  using UPtr = std::unique_ptr<FormatPlanningInputTask>;
  using ConstUPtr = std::unique_ptr<const FormatPlanningInputTask>;

  FormatPlanningInputTask();
  explicit FormatPlanningInputTask(std::string name,
                                   std::string input_program_key,
                                   std::string input_environment_key,
                                   std::string output_program_key,
                                   bool conditional = false);
  explicit FormatPlanningInputTask(std::string name,
                                   const YAML::Node& config,
                                   const TaskComposerPluginFactory& plugin_factory);
  ~FormatPlanningInputTask() override = default;
  FormatPlanningInputTask(const FormatPlanningInputTask&) = delete;
  FormatPlanningInputTask& operator=(const FormatPlanningInputTask&) = delete;
  FormatPlanningInputTask(FormatPlanningInputTask&&) = delete;
  FormatPlanningInputTask& operator=(FormatPlanningInputTask&&) = delete;

private:
  static TaskComposerNodePorts ports();

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor = std::nullopt) const override final;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSERFORMAT_PLANNING_INPUT_TASK_H
