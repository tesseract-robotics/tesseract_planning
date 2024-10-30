/**
 * @file format_as_input_task.h
 *
 * @brief This is used in the case where you run trajopt with collision as a cost and then you post check it for
 * collision and it fails. Then you run trajopt with collision as a constraint but the output from trajopt with
 * collision as a cost must be formated as input for trajopt with collision as a constraint planner.
 *
 * @author Levi Armstrong
 * @date April 6. 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Levi Armstrong
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
#include <boost/serialization/string.hpp>

#include <tesseract_common/joint_state.h>
#include <tesseract_common/serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/format_as_input_task.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

#include <tesseract_command_language/utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>

namespace tesseract_planning
{
// Requried
const std::string FormatAsInputTask::INPUT_PRE_PLANNING_PROGRAM_PORT = "pre_planning_program";
const std::string FormatAsInputTask::INPUT_POST_PLANNING_PROGRAM_PORT = "post_planning_program";
const std::string FormatAsInputTask::OUTPUT_PROGRAM_PORT = "program";

FormatAsInputTask::FormatAsInputTask() : TaskComposerTask("FormatAsInputTask", FormatAsInputTask::ports(), true) {}
FormatAsInputTask::FormatAsInputTask(std::string name,
                                     std::string input_pre_planning_program_key,
                                     std::string input_post_planning_program_key,
                                     std::string output_program_key,
                                     bool is_conditional)
  : TaskComposerTask(std::move(name), FormatAsInputTask::ports(), is_conditional)
{
  input_keys_.add(INPUT_PRE_PLANNING_PROGRAM_PORT, std::move(input_pre_planning_program_key));
  input_keys_.add(INPUT_POST_PLANNING_PROGRAM_PORT, std::move(input_post_planning_program_key));
  output_keys_.add(OUTPUT_PROGRAM_PORT, std::move(output_program_key));
  validatePorts();
}

FormatAsInputTask::FormatAsInputTask(std::string name,
                                     const YAML::Node& config,
                                     const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), FormatAsInputTask::ports(), config)
{
}

TaskComposerNodePorts FormatAsInputTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INPUT_PRE_PLANNING_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_POST_PLANNING_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;

  ports.output_required[OUTPUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;

  return ports;
}

std::unique_ptr<TaskComposerNodeInfo> FormatAsInputTask::runImpl(TaskComposerContext& context,
                                                                 OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;
  info->status_code = 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_formatted_data_poly = getData(*context.data_storage, INPUT_PRE_PLANNING_PROGRAM_PORT);
  if (input_formatted_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->status_message = "Input '" + input_keys_.get(INPUT_PRE_PLANNING_PROGRAM_PORT) +
                           "' instruction to FormatAsInputTask must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
    return info;
  }

  auto input_unformatted_data_poly = getData(*context.data_storage, INPUT_POST_PLANNING_PROGRAM_PORT);
  if (input_unformatted_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->status_message = "Input '" + input_keys_.get(INPUT_POST_PLANNING_PROGRAM_PORT) +
                           "' instruction to FormatAsInputTask must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
    return info;
  }

  auto& ci_formatted_data = input_formatted_data_poly.as<CompositeInstruction>();
  const auto& ci_unformatted_data = input_unformatted_data_poly.as<CompositeInstruction>();

  std::vector<std::reference_wrapper<InstructionPoly>> mi_formatted_data = ci_formatted_data.flatten();
  std::vector<std::reference_wrapper<const InstructionPoly>> mi_unformatted_data =
      ci_unformatted_data.flatten(&moveFilter);

  if (mi_formatted_data.size() != mi_unformatted_data.size())
  {
    info->status_message = "FormatAsInputTask, input programs are not same size";
    CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
    return info;
  }

  for (std::size_t i = 0; i < mi_formatted_data.size(); ++i)
  {
    auto& mi = mi_formatted_data[i].get().as<MoveInstructionPoly>();
    const auto& umi = mi_unformatted_data[i].get().as<MoveInstructionPoly>();

    if (mi.getWaypoint().isStateWaypoint())
      continue;

    if (mi.getWaypoint().isCartesianWaypoint())
    {
      auto& cwp = mi.getWaypoint().as<CartesianWaypointPoly>();
      cwp.setSeed(tesseract_common::JointState(getJointNames(umi.getWaypoint()), getJointPosition(umi.getWaypoint())));
    }
    else if (mi.getWaypoint().isJointWaypoint())
    {
      auto& jwp = mi.getWaypoint().as<JointWaypointPoly>();
      if (!jwp.isConstrained() || (jwp.isConstrained() && jwp.isToleranced()))
      {
        jwp.setNames(getJointNames(umi.getWaypoint()));
        jwp.setPosition(getJointPosition(umi.getWaypoint()));
      }
    }
    else
    {
      throw std::runtime_error("FormatAsInputTask, unsupported waypoint type!");
    }
  }

  setData(*context.data_storage, OUTPUT_PROGRAM_PORT, input_formatted_data_poly);

  info->color = "green";
  info->status_code = 1;
  info->status_message = "Successful";
  info->return_value = 1;
  return info;
}

bool FormatAsInputTask::operator==(const FormatAsInputTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool FormatAsInputTask::operator!=(const FormatAsInputTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void FormatAsInputTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::FormatAsInputTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::FormatAsInputTask)
