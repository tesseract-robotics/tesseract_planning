/**
 * @file check_input_task.cpp
 * @brief Task for checking input data structure
 *
 * @author Levi Armstrong
 * @date November 2. 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/process_planning_input_task.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
ProcessPlanningInputTask::ProcessPlanningInputTask() : TaskComposerTask("ProcessPlanningInputInstructionTask", false) {}

ProcessPlanningInputTask::ProcessPlanningInputTask(std::string name, std::string output_key, bool is_conditional)
  : TaskComposerTask(std::move(name), is_conditional)
{
  output_keys_ = { std::move(output_key) };
}

ProcessPlanningInputTask::ProcessPlanningInputTask(std::string name,
                                                   const YAML::Node& config,
                                                   const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
  if (!input_keys_.empty())
    throw std::runtime_error("ProcessPlanningInputTask, config should not have 'inputs' entry");

  if (output_keys_.empty())
    throw std::runtime_error("ProcessPlanningInputTask, config missing 'inputs' entry");

  if (output_keys_.size() != 1)
    throw std::runtime_error("ProcessPlanningInputTask, 'outputs' should only have one key");
}

TaskComposerNodeInfo::UPtr ProcessPlanningInputTask::runImpl(TaskComposerContext& context,
                                                             OptionalTaskComposerExecutor /*executor*/) const
{
  // Get the problem
  auto& problem = dynamic_cast<PlanningTaskComposerProblem&>(*context.problem);

  auto info = std::make_unique<TaskComposerNodeInfo>(*this);

  if (problem.input.isNull() || problem.input.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->color = "red";
    info->message = "Input is not a Composite Instruction, aborting...";
    info->return_value = 0;

    // Abort
    context.abort(uuid_);
    return info;
  }

  context.data_storage->setData(output_keys_[0], problem.input.as<CompositeInstruction>());

  info->color = "green";
  info->message = "Successful";
  info->return_value = 1;
  return info;
}

bool ProcessPlanningInputTask::operator==(const ProcessPlanningInputTask& rhs) const
{
  return (TaskComposerTask::operator==(rhs));
}
bool ProcessPlanningInputTask::operator!=(const ProcessPlanningInputTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void ProcessPlanningInputTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ProcessPlanningInputTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ProcessPlanningInputTask)
