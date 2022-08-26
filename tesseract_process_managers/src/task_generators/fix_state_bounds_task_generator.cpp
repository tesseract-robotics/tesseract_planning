/**
 * @file fix_state_bounds_task_generator.cpp
 * @brief Process generator that changes the plan instructions to make push them back within joint limits
 *
 * @author Matthew Powelson
 * @date August 31. 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <tesseract_common/timer.h>

#include <tesseract_process_managers/core/utils.h>
#include <tesseract_process_managers/task_generators/fix_state_bounds_task_generator.h>
#include <tesseract_process_managers/task_profiles/fix_state_bounds_profile.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_process_managers/task_generators/fix_state_bounds_task_generator.h>

namespace tesseract_planning
{
FixStateBoundsTaskGenerator::FixStateBoundsTaskGenerator(std::string name) : TaskGenerator(std::move(name)) {}

int FixStateBoundsTaskGenerator::conditionalProcess(TaskInput input, std::size_t unique_id) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_unique<FixStateBoundsTaskInfo>(unique_id, name_);
  info->return_value = 0;
  tesseract_common::Timer timer;
  timer.start();
  saveInputs(*info, input);

  // --------------------
  // Check that inputs are valid
  // --------------------
  const InstructionPoly* input_instruction = input.getInstruction();
  if (!input_instruction->isCompositeInstruction())
  {
    info->message = "Input instruction to FixStateBounds must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  const auto& ci = input_instruction->as<CompositeInstruction>();
  const tesseract_common::ManipulatorInfo& manip_info = input.manip_info;
  auto joint_group = input.env->getJointGroup(manip_info.manipulator);
  auto limits = joint_group->getLimits();

  // Get Composite Profile
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, input.composite_profile_remapping);
  auto cur_composite_profile =
      getProfile<FixStateBoundsProfile>(name_, profile, *input.profiles, std::make_shared<FixStateBoundsProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

  if (cur_composite_profile->mode == FixStateBoundsProfile::Settings::DISABLED)
  {
    info->return_value = 1;
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 1;
  }

  limits.joint_limits.col(0) = limits.joint_limits.col(0).array() + cur_composite_profile->lower_bounds_reduction;
  limits.joint_limits.col(1) = limits.joint_limits.col(1).array() - cur_composite_profile->upper_bounds_reduction;
  switch (cur_composite_profile->mode)
  {
    case FixStateBoundsProfile::Settings::START_ONLY:
    {
      const MoveInstructionPoly* instr_const_ptr = ci.getFirstMoveInstruction();
      if (instr_const_ptr != nullptr)
      {
        auto* mutable_instruction = const_cast<MoveInstructionPoly*>(instr_const_ptr);  // NOLINT
        if (!isWithinJointLimits(mutable_instruction->getWaypoint(), limits.joint_limits))
        {
          CONSOLE_BRIDGE_logInform("FixStateBoundsTaskGenerator is modifying the const input instructions");
          if (!clampToJointLimits(
                  mutable_instruction->getWaypoint(), limits.joint_limits, cur_composite_profile->max_deviation_global))
          {
            saveOutputs(*info, input);
            info->elapsed_time = timer.elapsedSeconds();
            input.addTaskInfo(std::move(info));
            return 0;
          }
        }
      }
    }
    break;
    case FixStateBoundsProfile::Settings::END_ONLY:
    {
      const MoveInstructionPoly* instr_const_ptr = ci.getLastMoveInstruction();
      if (instr_const_ptr != nullptr)
      {
        auto* mutable_instruction = const_cast<MoveInstructionPoly*>(instr_const_ptr);  // NOLINT
        if (!isWithinJointLimits(mutable_instruction->getWaypoint(), limits.joint_limits))
        {
          CONSOLE_BRIDGE_logInform("FixStateBoundsTaskGenerator is modifying the const input instructions");
          if (!clampToJointLimits(
                  mutable_instruction->getWaypoint(), limits.joint_limits, cur_composite_profile->max_deviation_global))
          {
            saveOutputs(*info, input);
            info->elapsed_time = timer.elapsedSeconds();
            input.addTaskInfo(std::move(info));
            return 0;
          }
        }
      }
    }
    break;
    case FixStateBoundsProfile::Settings::ALL:
    {
      auto flattened = ci.flatten(moveFilter);
      if (flattened.empty())
      {
        CONSOLE_BRIDGE_logWarn("FixStateBoundsTaskGenerator found no MoveInstructions to process");
        info->return_value = 1;
        saveOutputs(*info, input);
        info->elapsed_time = timer.elapsedSeconds();
        input.addTaskInfo(std::move(info));
        return 1;
      }

      bool inside_limits = true;
      for (const auto& instruction : flattened)
      {
        inside_limits &=
            isWithinJointLimits(instruction.get().as<MoveInstructionPoly>().getWaypoint(), limits.joint_limits);
      }
      if (inside_limits)
        break;

      CONSOLE_BRIDGE_logInform("FixStateBoundsTaskGenerator is modifying the const input instructions");
      for (const auto& instruction : flattened)
      {
        const InstructionPoly* instr_const_ptr = &instruction.get();
        auto* mutable_instruction = const_cast<InstructionPoly*>(instr_const_ptr);  // NOLINT
        auto& plan = mutable_instruction->as<MoveInstructionPoly>();
        if (!clampToJointLimits(plan.getWaypoint(), limits.joint_limits, cur_composite_profile->max_deviation_global))
        {
          saveOutputs(*info, input);
          info->elapsed_time = timer.elapsedSeconds();
          input.addTaskInfo(std::move(info));
          return 0;
        }
      }
    }
    break;
    case FixStateBoundsProfile::Settings::DISABLED:
      info->return_value = 1;
      saveOutputs(*info, input);
      info->elapsed_time = timer.elapsedSeconds();
      input.addTaskInfo(std::move(info));
      return 1;
  }

  CONSOLE_BRIDGE_logDebug("FixStateBoundsTaskGenerator succeeded");
  info->return_value = 1;
  saveOutputs(*info, input);
  info->elapsed_time = timer.elapsedSeconds();
  input.addTaskInfo(std::move(info));
  return 1;
}

void FixStateBoundsTaskGenerator::process(TaskInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

FixStateBoundsTaskInfo::FixStateBoundsTaskInfo(std::size_t unique_id, std::string name)
  : TaskInfo(unique_id, std::move(name))
{
}

TaskInfo::UPtr FixStateBoundsTaskInfo::clone() const { return std::make_unique<FixStateBoundsTaskInfo>(*this); }

bool FixStateBoundsTaskInfo::operator==(const FixStateBoundsTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskInfo::operator==(rhs);
  return equal;
}
bool FixStateBoundsTaskInfo::operator!=(const FixStateBoundsTaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void FixStateBoundsTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::FixStateBoundsTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::FixStateBoundsTaskInfo)
