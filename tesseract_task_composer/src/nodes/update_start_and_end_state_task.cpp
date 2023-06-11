/**
 * @file transition_mux_task.h
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/nodes/update_start_and_end_state_task.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
UpdateStartAndEndStateTask::UpdateStartAndEndStateTask(std::string name,
                                                       std::string input_prev_key,
                                                       std::string input_next_key,
                                                       std::string output_key,
                                                       bool is_conditional)
  : TaskComposerTask(std::move(name), is_conditional)
{
  input_keys_.push_back(uuid_str_);
  input_keys_.push_back(std::move(input_prev_key));
  input_keys_.push_back(std::move(input_next_key));
  output_keys_.push_back(std::move(output_key));
}

UpdateStartAndEndStateTask::UpdateStartAndEndStateTask(std::string name,
                                                       std::string input_key,
                                                       std::string input_prev_key,
                                                       std::string input_next_key,
                                                       std::string output_key,
                                                       bool is_conditional)
  : TaskComposerTask(std::move(name), is_conditional)
{
  input_keys_.push_back(std::move(input_key));
  input_keys_.push_back(std::move(input_prev_key));
  input_keys_.push_back(std::move(input_next_key));
  output_keys_.push_back(std::move(output_key));
}

TaskComposerNodeInfo::UPtr UpdateStartAndEndStateTask::runImpl(TaskComposerInput& input,
                                                               OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this, input);
  if (info->isAborted())
    return info;

  info->return_value = 0;
  info->env = input.problem.env;
  tesseract_common::Timer timer;
  timer.start();

  auto input_data_poly = input.data_storage.getData(input_keys_[0]);
  auto input_prev_data_poly = input.data_storage.getData(input_keys_[1]);
  auto input_next_data_poly = input.data_storage.getData(input_keys_[2]);

  // --------------------
  // Check that inputs are valid
  // --------------------
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message =
        "UpdateStartAndEndStateTask: Input data for key '" + input_keys_[0] + "' must be a composite instruction";
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  if (input_prev_data_poly.isNull() || input_prev_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message =
        "UpdateStartAndEndStateTask: Input data for key '" + input_keys_[1] + "' must be a composite instruction";
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  if (input_next_data_poly.isNull() || input_next_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message =
        "UpdateStartAndEndStateTask: Input data for key '" + input_keys_[2] + "' must be a composite instruction";
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  // Make a non-const copy of the input instructions to update the start/end
  auto& instructions = input_data_poly.as<CompositeInstruction>();
  const auto* prev_last_move = input_prev_data_poly.as<CompositeInstruction>().getLastMoveInstruction();
  const auto* next_start_move = input_next_data_poly.as<CompositeInstruction>().getFirstMoveInstruction();

  // Update start instruction
  instructions.at(0) = (*prev_last_move);

  // Update end instruction
  if (next_start_move->getWaypoint().isCartesianWaypoint())
    instructions.getLastMoveInstruction()->assignCartesianWaypoint(
        next_start_move->getWaypoint().as<CartesianWaypointPoly>());
  else if (next_start_move->getWaypoint().isJointWaypoint())
    instructions.getLastMoveInstruction()->assignJointWaypoint(next_start_move->getWaypoint().as<JointWaypointPoly>());
  else if (next_start_move->getWaypoint().isStateWaypoint())
    instructions.getLastMoveInstruction()->assignStateWaypoint(next_start_move->getWaypoint().as<StateWaypointPoly>());
  else
    throw std::runtime_error("Invalid waypoint type");

  // Store results
  input.data_storage.setData(output_keys_[0], input_data_poly);

  info->successful = true;
  info->message = "Successful";
  info->return_value = 1;
  info->elapsed_time = timer.elapsedSeconds();
  CONSOLE_BRIDGE_logDebug("UpdateStartAndEndStateTask succeeded");
  return info;
}

bool UpdateStartAndEndStateTask::operator==(const UpdateStartAndEndStateTask& rhs) const
{
  bool equal = true;
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool UpdateStartAndEndStateTask::operator!=(const UpdateStartAndEndStateTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void UpdateStartAndEndStateTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::UpdateStartAndEndStateTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::UpdateStartAndEndStateTask)
