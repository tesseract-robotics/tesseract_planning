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
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/nodes/update_start_state_task.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
UpdateStartStateTask::UpdateStartStateTask(std::string input_prev_key,
                                           std::string output_key,
                                           bool is_conditional,
                                           std::string name)
  : TaskComposerTask(is_conditional, std::move(name))
{
  input_keys_.push_back(uuid_str_);
  input_keys_.push_back(std::move(input_prev_key));
  output_keys_.push_back(std::move(output_key));
}

UpdateStartStateTask::UpdateStartStateTask(std::string input_key,
                                           std::string input_prev_key,
                                           std::string output_key,
                                           bool is_conditional,
                                           std::string name)
  : TaskComposerTask(is_conditional, std::move(name))
{
  input_keys_.push_back(std::move(input_key));
  input_keys_.push_back(std::move(input_prev_key));
  output_keys_.push_back(std::move(output_key));
}

TaskComposerNodeInfo::UPtr UpdateStartStateTask::runImpl(TaskComposerInput& input,
                                                         OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(uuid_, name_);
  info->return_value = 0;

  if (input.isAborted())
  {
    info->message = "Aborted";
    return info;
  }

  tesseract_common::Timer timer;
  timer.start();

  auto input_data_poly = input.data_storage->getData(input_keys_[0]);
  auto input_prev_data_poly = input.data_storage->getData(input_keys_[1]);

  // --------------------
  // Check that inputs are valid
  // --------------------
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "UpdateStartStateTask: Input data for key '" + input_keys_[0] + "' must be a composite instruction";
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  if (input_prev_data_poly.isNull() || input_prev_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "UpdateStartStateTask: Input data for key '" + input_keys_[1] + "' must be a composite instruction";
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  // Make a non-const copy of the input instructions to update the start/end
  auto& instructions = input_data_poly.as<CompositeInstruction>();
  const auto* prev_last_move = input_prev_data_poly.as<CompositeInstruction>().getLastMoveInstruction();

  // Update start instruction
  instructions.setStartInstruction(*prev_last_move);
  instructions.getStartInstruction().setMoveType(MoveInstructionType::START);

  // Store results
  input.data_storage->setData(output_keys_[0], input_data_poly);
  info->message = "Successful";
  info->return_value = 1;
  info->elapsed_time = timer.elapsedSeconds();
  return info;
}

TaskComposerNode::UPtr UpdateStartStateTask::clone() const
{
  return std::make_unique<UpdateStartStateTask>(
      input_keys_[0], input_keys_[1], output_keys_[0], is_conditional_, name_);
}

bool UpdateStartStateTask::operator==(const UpdateStartStateTask& rhs) const
{
  bool equal = true;
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool UpdateStartStateTask::operator!=(const UpdateStartStateTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void UpdateStartStateTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::UpdateStartStateTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::UpdateStartStateTask)
