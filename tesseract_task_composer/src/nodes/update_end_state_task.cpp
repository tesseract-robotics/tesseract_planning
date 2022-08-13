/**
 * @file update_end_state_task.h
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

#include <tesseract_task_composer/nodes/update_end_state_task.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
UpdateEndStateTask::UpdateEndStateTask(std::string input_next_key,
                                       std::string output_key,
                                       bool is_conditional,
                                       std::string name)
  : TaskComposerTask(is_conditional, std::move(name))
  , input_key_(uuid_str_)
  , input_next_key_(std::move(input_next_key))
  , output_key_(std::move(output_key))
{
}

UpdateEndStateTask::UpdateEndStateTask(std::string input_key,
                                       std::string input_next_key,
                                       std::string output_key,
                                       bool is_conditional,
                                       std::string name)
  : TaskComposerTask(is_conditional, std::move(name))
  , input_key_(std::move(input_key))
  , input_next_key_(std::move(input_next_key))
  , output_key_(std::move(output_key))
{
}

int UpdateEndStateTask::run(TaskComposerInput& input) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_unique<UpdateEndStateTaskInfo>(uuid_, name_);
  info->return_value = 0;
  tesseract_common::Timer timer;
  timer.start();
  //  saveInputs(*info, input);

  auto input_data_poly = input.data_storage->getData(input_key_);
  auto input_next_data_poly = input.data_storage->getData(input_next_key_);

  // --------------------
  // Check that inputs are valid
  // --------------------
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "UpdateEndStateTask: Input data for key '" + input_key_ + "' must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    //    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  if (input_next_data_poly.isNull() || input_next_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "UpdateEndStateTask: Input data for key '" + input_next_key_ + "' must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    //    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  // Make a non-const copy of the input instructions to update the start/end
  CompositeInstruction& instructions = input_data_poly.as<CompositeInstruction>();
  const auto* next_start_move = input_next_data_poly.as<CompositeInstruction>().getFirstMoveInstruction();

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
  input.data_storage->setData(output_key_, input_data_poly);
  info->return_value = 1;
  info->message = "UpdateEndStateTask: Successful";
  //    saveOutputs(*info, input);
  info->elapsed_time = timer.elapsedSeconds();
  input.addTaskInfo(std::move(info));
  return 1;
}

bool UpdateEndStateTask::operator==(const UpdateEndStateTask& rhs) const
{
  bool equal = true;
  equal &= (input_key_ == rhs.input_key_);
  equal &= (input_next_key_ == rhs.input_next_key_);
  equal &= (output_key_ == rhs.output_key_);
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool UpdateEndStateTask::operator!=(const UpdateEndStateTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void UpdateEndStateTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(input_key_);
  ar& BOOST_SERIALIZATION_NVP(input_next_key_);
  ar& BOOST_SERIALIZATION_NVP(output_key_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

UpdateEndStateTaskInfo::UpdateEndStateTaskInfo(boost::uuids::uuid uuid, std::string name)
  : TaskComposerNodeInfo(uuid, std::move(name))
{
}

TaskComposerNodeInfo::UPtr UpdateEndStateTaskInfo::clone() const
{
  return std::make_unique<UpdateEndStateTaskInfo>(*this);
}

bool UpdateEndStateTaskInfo::operator==(const UpdateEndStateTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskComposerNodeInfo::operator==(rhs);
  return equal;
}
bool UpdateEndStateTaskInfo::operator!=(const UpdateEndStateTaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void UpdateEndStateTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNodeInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::UpdateEndStateTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::UpdateEndStateTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::UpdateEndStateTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::UpdateEndStateTaskInfo)
