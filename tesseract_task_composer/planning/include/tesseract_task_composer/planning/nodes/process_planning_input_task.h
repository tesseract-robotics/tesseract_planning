/**
 * @file process_planning_input_instruction.h
 *
 * @brief This cast the input instruction to a CompositeInstruciton and stores under the key in data_storage.
 *
 * @author Levi Armstrong
 * @date September 17. 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Plectix Robotics
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
#ifndef TESSERACT_TASK_COMPOSER_PROCESS_PLANNING_INPUT_TASK_H
#define TESSERACT_TASK_COMPOSER_PROCESS_PLANNING_INPUT_TASK_H

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
class TESSERACT_TASK_COMPOSER_PLANNING_NODES_EXPORT ProcessPlanningInputTask : public TaskComposerTask
{
public:
  // Requried
  static const std::string INPUT_PLANNING_INPUT_PORT;
  static const std::string OUTPUT_PROGRAM_PORT;

  using Ptr = std::shared_ptr<ProcessPlanningInputTask>;
  using ConstPtr = std::shared_ptr<const ProcessPlanningInputTask>;
  using UPtr = std::unique_ptr<ProcessPlanningInputTask>;
  using ConstUPtr = std::unique_ptr<const ProcessPlanningInputTask>;

  ProcessPlanningInputTask();
  explicit ProcessPlanningInputTask(std::string name,
                                    std::string input_key,
                                    std::string output_key,
                                    bool conditional = false);
  explicit ProcessPlanningInputTask(std::string name,
                                    const YAML::Node& config,
                                    const TaskComposerPluginFactory& plugin_factory);

  ~ProcessPlanningInputTask() override = default;
  ProcessPlanningInputTask(const ProcessPlanningInputTask&) = delete;
  ProcessPlanningInputTask& operator=(const ProcessPlanningInputTask&) = delete;
  ProcessPlanningInputTask(ProcessPlanningInputTask&&) = delete;
  ProcessPlanningInputTask& operator=(ProcessPlanningInputTask&&) = delete;

  bool operator==(const ProcessPlanningInputTask& rhs) const;
  bool operator!=(const ProcessPlanningInputTask& rhs) const;

private:
  static TaskComposerNodePorts ports();

  TaskComposerNodeInfo runImpl(TaskComposerContext& context,
                               OptionalTaskComposerExecutor executor = std::nullopt) const override final;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::ProcessPlanningInputTask)

#endif  // TESSERACT_TASK_COMPOSER_PROCESS_PLANNING_INPUT_TASK_H
