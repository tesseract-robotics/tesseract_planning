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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/serialization/map.hpp>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/format_planning_input_task.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_environment/environment.h>

namespace tesseract_planning
{
// Requried
const std::string FormatPlanningInputTask::INOUT_PROGRAM_PORT = "program";
const std::string FormatPlanningInputTask::INPUT_ENVIRONMENT_PORT = "environment";

FormatPlanningInputTask::FormatPlanningInputTask()
  : TaskComposerTask("FormatPlanningInputTask", FormatPlanningInputTask::ports(), false)
{
}

FormatPlanningInputTask::FormatPlanningInputTask(std::string name,
                                                 std::string input_program_key,
                                                 std::string input_environment_key,
                                                 std::string output_program_key,
                                                 bool is_conditional)
  : TaskComposerTask(std::move(name), FormatPlanningInputTask::ports(), is_conditional)
{
  input_keys_.add(INOUT_PROGRAM_PORT, std::move(input_program_key));
  input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
  output_keys_.add(INOUT_PROGRAM_PORT, std::move(output_program_key));

  validatePorts();
}

FormatPlanningInputTask::FormatPlanningInputTask(std::string name,
                                                 const YAML::Node& config,
                                                 const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), FormatPlanningInputTask::ports(), config)
{
}

TaskComposerNodePorts FormatPlanningInputTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_ENVIRONMENT_PORT] = TaskComposerNodePorts::SINGLE;
  ports.output_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  return ports;
}

TaskComposerNodeInfo FormatPlanningInputTask::runImpl(TaskComposerContext& context,
                                                      OptionalTaskComposerExecutor /*executor*/) const
{
  // --------------------
  // Check that inputs are valid
  // --------------------
  auto env_poly = getData(*context.data_storage, INPUT_ENVIRONMENT_PORT);
  if (env_poly.getType() != std::type_index(typeid(std::shared_ptr<const tesseract_environment::Environment>)))
  {
    TaskComposerNodeInfo info(*this);
    info.return_value = 0;
    info.status_code = 0;
    info.status_message = "Input data '" + input_keys_.get(INPUT_ENVIRONMENT_PORT) + "' is not correct type";
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    return info;
  }

  auto env = env_poly.as<std::shared_ptr<const tesseract_environment::Environment>>();

  auto input_data_poly = getData(*context.data_storage, INOUT_PROGRAM_PORT);
  if (input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    TaskComposerNodeInfo info(*this);
    info.return_value = 0;
    info.status_code = 0;
    info.status_message = "Input to FormatPlanningInputTask must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    return info;
  }

  auto& ci = input_data_poly.as<CompositeInstruction>();

  const bool formatting_required = formatProgram(ci, *env);
  setData(*context.data_storage, INOUT_PROGRAM_PORT, ci);

  TaskComposerNodeInfo info(*this);
  info.return_value = 1;
  info.status_code = 1;
  if (formatting_required)
  {
    info.color = "yellow";
    info.status_message = "Successful (Formatting Required)";
  }
  else
  {
    info.color = "green";
    info.status_message = "Successful";
  }
  return info;
}

bool FormatPlanningInputTask::operator==(const FormatPlanningInputTask& rhs) const
{
  return (TaskComposerNode::operator==(rhs));
}
bool FormatPlanningInputTask::operator!=(const FormatPlanningInputTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void FormatPlanningInputTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::FormatPlanningInputTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::FormatPlanningInputTask)
