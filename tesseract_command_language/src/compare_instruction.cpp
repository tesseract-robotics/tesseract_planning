/**
 * @file compare_instruction.cpp
 * @brief This contains the comparison operators for Instruction by recovering the type and comparing
 *
 * @author Levi Armstrong
 * @date February 24, 2021
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
#include <tesseract_command_language/compare_instruction.h>

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/wait_instruction.h>
#include <tesseract_command_language/timer_instruction.h>
#include <tesseract_command_language/set_tool_instruction.h>
#include <tesseract_command_language/set_analog_instruction.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/instruction_type.h>

#include <tesseract_command_language/core/null_instruction.h>

namespace tesseract_planning
{
bool operator==(const Instruction& lhs, const Instruction& rhs)
{
  if (lhs.getType() == rhs.getType())
  {
    switch (lhs.getType())
    {
      case static_cast<int>(InstructionType::NULL_INSTRUCTION):
      {
        return true;
      }
      case static_cast<int>(InstructionType::PLAN_INSTRUCTION):
      {
        return ((*lhs.cast_const<PlanInstruction>()) == (*rhs.cast_const<PlanInstruction>()));
      }
      case static_cast<int>(InstructionType::MOVE_INSTRUCTION):
      {
        return ((*lhs.cast_const<MoveInstruction>()) == (*rhs.cast_const<MoveInstruction>()));
      }
      case static_cast<int>(InstructionType::WAIT_INSTRUCTION):
      {
        return ((*lhs.cast_const<WaitInstruction>()) == (*rhs.cast_const<WaitInstruction>()));
      }
      case static_cast<int>(InstructionType::TIMER_INSTRUCTION):
      {
        return ((*lhs.cast_const<TimerInstruction>()) == (*rhs.cast_const<TimerInstruction>()));
      }
      case static_cast<int>(InstructionType::SET_TOOL_INSTRUCTION):
      {
        return ((*lhs.cast_const<SetToolInstruction>()) == (*rhs.cast_const<SetToolInstruction>()));
      }
      case static_cast<int>(InstructionType::SET_ANALOG_INSTRUCTION):
      {
        return ((*lhs.cast_const<SetAnalogInstruction>()) == (*rhs.cast_const<SetAnalogInstruction>()));
      }
      case static_cast<int>(InstructionType::COMPOSITE_INSTRUCTION):
      {
        return ((*lhs.cast_const<CompositeInstruction>()) == (*rhs.cast_const<CompositeInstruction>()));
      }
      default:
      {
        return false;
      }
    }
  }

  return false;
}

bool operator!=(const Instruction& lhs, const Instruction& rhs) { return !operator==(lhs, rhs); }
}  // namespace tesseract_planning
