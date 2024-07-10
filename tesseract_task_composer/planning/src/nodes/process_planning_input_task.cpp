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

#include <tesseract_common/serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/process_planning_input_task.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
// Requried
const std::string ProcessPlanningInputTask::INPUT_PLANNING_INPUT_PORT = "planning_input";
const std::string ProcessPlanningInputTask::OUTPUT_PROGRAM_PORT = "program";

ProcessPlanningInputTask::ProcessPlanningInputTask()
  : TaskComposerTask("ProcessPlanningInputInstructionTask", ProcessPlanningInputTask::ports(), false)
{
}

ProcessPlanningInputTask::ProcessPlanningInputTask(std::string name,
                                                   std::string input_key,
                                                   std::string output_key,
                                                   bool is_conditional)
  : TaskComposerTask(std::move(name), ProcessPlanningInputTask::ports(), is_conditional)
{
  input_keys_.add(INPUT_PLANNING_INPUT_PORT, std::move(input_key));
  output_keys_.add(OUTPUT_PROGRAM_PORT, std::move(output_key));
  validatePorts();
}

ProcessPlanningInputTask::ProcessPlanningInputTask(std::string name,
                                                   const YAML::Node& config,
                                                   const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), ProcessPlanningInputTask::ports(), config)
{
}

TaskComposerNodePorts ProcessPlanningInputTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INPUT_PLANNING_INPUT_PORT] = false;
  ports.output_required[OUTPUT_PROGRAM_PORT] = false;
  return ports;
}

std::unique_ptr<TaskComposerNodeInfo> ProcessPlanningInputTask::runImpl(TaskComposerContext& context,
                                                                        OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  auto planning_input_poly = getData(*context.data_storage, INPUT_PLANNING_INPUT_PORT);
  if (planning_input_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->color = "red";
    info->status_code = 0;
    info->status_message = "Input is not a Composite Instruction, aborting...";
    info->return_value = 0;

    // Abort
    context.abort(uuid_);
    return info;
  }

  setData(*context.data_storage, OUTPUT_PROGRAM_PORT, planning_input_poly.as<CompositeInstruction>());

  info->color = "green";
  info->status_code = 1;
  info->status_message = "Successful";
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

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ProcessPlanningInputTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ProcessPlanningInputTask)
