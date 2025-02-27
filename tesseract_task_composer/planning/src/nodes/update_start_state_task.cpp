/**
 * @file update_start_state_task.h
 *
 * @author Levi Armstrong
 * @date August 5, 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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

#include <tesseract_common/serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/update_start_state_task.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>

namespace tesseract_planning
{
// Requried
const std::string UpdateStartStateTask::INPUT_PREVIOUS_PROGRAM_PORT = "previous_program";
const std::string UpdateStartStateTask::INPUT_CURRENT_PROGRAM_PORT = "current_program";
const std::string UpdateStartStateTask::OUTPUT_PROGRAM_PORT = "program";

UpdateStartStateTask::UpdateStartStateTask(std::string name,
                                           std::string input_prev_key,
                                           std::string output_key,
                                           bool conditional)
  : TaskComposerTask(std::move(name), UpdateStartStateTask::ports(), conditional)
{
  input_keys_.add(INPUT_CURRENT_PROGRAM_PORT, uuid_str_);
  input_keys_.add(INPUT_PREVIOUS_PROGRAM_PORT, std::move(input_prev_key));
  output_keys_.add(OUTPUT_PROGRAM_PORT, std::move(output_key));
  validatePorts();
}

UpdateStartStateTask::UpdateStartStateTask(std::string name,
                                           std::string input_key,
                                           std::string input_prev_key,
                                           std::string output_key,
                                           bool conditional)
  : TaskComposerTask(std::move(name), UpdateStartStateTask::ports(), conditional)
{
  input_keys_.add(INPUT_CURRENT_PROGRAM_PORT, std::move(input_key));
  input_keys_.add(INPUT_PREVIOUS_PROGRAM_PORT, std::move(input_prev_key));
  output_keys_.add(OUTPUT_PROGRAM_PORT, std::move(output_key));
  validatePorts();
}

TaskComposerNodePorts UpdateStartStateTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INPUT_CURRENT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_PREVIOUS_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.output_required[OUTPUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  return ports;
}

TaskComposerNodeInfo UpdateStartStateTask::runImpl(TaskComposerContext& context,
                                                   OptionalTaskComposerExecutor /*executor*/) const
{
  TaskComposerNodeInfo info(*this);
  info.return_value = 0;
  info.status_code = 0;

  auto input_data_poly = getData(*context.data_storage, INPUT_CURRENT_PROGRAM_PORT);
  auto input_prev_data_poly = getData(*context.data_storage, INPUT_PREVIOUS_PROGRAM_PORT);

  // --------------------
  // Check that inputs are valid
  // --------------------
  if (input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info.status_message = "UpdateStartStateTask: Input data for key '" + input_keys_.get(INPUT_CURRENT_PROGRAM_PORT) +
                          "' must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    return info;
  }

  if (input_prev_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info.status_message = "UpdateStartStateTask: Input data for key '" + input_keys_.get(INPUT_PREVIOUS_PROGRAM_PORT) +
                          "' must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info.status_message.c_str());
    return info;
  }

  // Make a non-const copy of the input instructions to update the start/end
  auto& instructions = input_data_poly.as<CompositeInstruction>();
  auto* first_move_instruction = instructions.getFirstMoveInstruction();
  /** @todo Should the waypoint profile be updated to the path profile if it exists? **/

  // Update start instruction
  const auto* prev_last_move = input_prev_data_poly.as<CompositeInstruction>().getLastMoveInstruction();
  if (prev_last_move->getWaypoint().isCartesianWaypoint() || prev_last_move->getWaypoint().isJointWaypoint() ||
      prev_last_move->getWaypoint().isStateWaypoint())
    first_move_instruction->getWaypoint() = prev_last_move->getWaypoint();
  else
    throw std::runtime_error("Invalid waypoint type");

  // Store results
  setData(*context.data_storage, OUTPUT_PROGRAM_PORT, input_data_poly);

  info.color = "green";
  info.status_code = 1;
  info.status_message = "Successful";
  info.return_value = 1;
  return info;
}

bool UpdateStartStateTask::operator==(const UpdateStartStateTask& rhs) const
{
  return (TaskComposerTask::operator==(rhs));
}
bool UpdateStartStateTask::operator!=(const UpdateStartStateTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void UpdateStartStateTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::UpdateStartStateTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::UpdateStartStateTask)
