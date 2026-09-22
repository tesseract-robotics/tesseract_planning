/**
 * @file check_input_task.cpp
 * @brief Task for checking input data structure
 *
 * @author Levi Armstrong
 * @date November 2. 2021
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/task_composer/planning/nodes/process_planning_input_task.h>

#include <tesseract/task_composer/task_composer_context.h>
#include <tesseract/task_composer/task_composer_node_info.h>
#include <tesseract/task_composer/task_composer_data_storage.h>

#include <tesseract/command_language/composite_instruction.h>

namespace tesseract::task_composer
{
// Requried

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
  input_port_mappings_.set(INPUT_PLANNING_INPUT_PORT, std::move(input_key));
  output_port_mappings_.set(OUTPUT_PROGRAM_PORT, std::move(output_key));
  setPortMappings(input_port_mappings_, output_port_mappings_);
}

ProcessPlanningInputTask::ProcessPlanningInputTask(std::string name,
                                                   const YAML::Node& config,
                                                   const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), ProcessPlanningInputTask::ports(), config)
{
}

tesseract::common::PropertyTree ProcessPlanningInputTask::schema() { return TaskComposerTask::schema(ports()); }

const TaskComposerNodePorts& ProcessPlanningInputTask::ports()
{
  static const TaskComposerNodePorts ports = []() {
    TaskComposerNodePorts ports;
    ports.addRequiredInput(INPUT_PLANNING_INPUT_PORT);
    ports.addRequiredOutput(OUTPUT_PROGRAM_PORT);
    return ports;
  }();
  return ports;
}

TaskComposerNodeInfo ProcessPlanningInputTask::runImpl(TaskComposerContext& context,
                                                       OptionalTaskComposerExecutor /*executor*/) const
{
  TaskComposerNodeInfo info(*this);
  auto planning_input_poly = getData(context, INPUT_PLANNING_INPUT_PORT);
  if (planning_input_poly.getType() != std::type_index(typeid(tesseract::command_language::CompositeInstruction)))
  {
    info.color = "red";
    info.status_code = 0;
    info.status_message = "Input is not a Composite Instruction, aborting...";
    info.return_value = 0;

    // Abort
    context.abort(uuid_);
    return info;
  }

  setData(context, OUTPUT_PROGRAM_PORT, planning_input_poly.as<tesseract::command_language::CompositeInstruction>());

  info.color = "green";
  info.status_code = 1;
  info.status_message = "Successful";
  info.return_value = 1;
  return info;
}

}  // namespace tesseract::task_composer
