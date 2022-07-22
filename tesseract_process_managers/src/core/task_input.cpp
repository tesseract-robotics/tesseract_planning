/**
 * @file task_input.cpp
 * @brief Process input
 *
 * @author Matthew Powelson
 * @date July 15. 2020
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
#include <vector>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/task_input.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/utils.h>

namespace tesseract_planning
{
static const tesseract_common::ManipulatorInfo EMPTY_MANIPULATOR_INFO;
static const PlannerProfileRemapping EMPTY_PROFILE_MAPPING;

TaskInput::TaskInput(tesseract_environment::Environment::ConstPtr env,
                     const InstructionPoly* instruction,
                     const tesseract_common::ManipulatorInfo& manip_info,
                     InstructionPoly* seed,
                     bool has_seed,
                     ProfileDictionary::ConstPtr profiles)
  : env(std::move(env))
  , manip_info(manip_info)
  , plan_profile_remapping(EMPTY_PROFILE_MAPPING)
  , composite_profile_remapping(EMPTY_PROFILE_MAPPING)
  , profiles(std::move(profiles))
  , has_seed(has_seed)
  , instruction_(instruction)
  , results_(seed)
{
}

TaskInput::TaskInput(tesseract_environment::Environment::ConstPtr env,
                     const InstructionPoly* instruction,
                     const tesseract_common::ManipulatorInfo& manip_info,
                     const PlannerProfileRemapping& plan_profile_remapping,
                     const PlannerProfileRemapping& composite_profile_remapping,
                     InstructionPoly* seed,
                     bool has_seed,
                     ProfileDictionary::ConstPtr profiles)
  : env(std::move(env))
  , manip_info(manip_info)
  , plan_profile_remapping(plan_profile_remapping)
  , composite_profile_remapping(composite_profile_remapping)
  , profiles(std::move(profiles))
  , has_seed(has_seed)
  , instruction_(instruction)
  , results_(seed)
{
}

TaskInput::TaskInput(tesseract_environment::Environment::ConstPtr env,
                     const InstructionPoly* instruction,
                     const PlannerProfileRemapping& plan_profile_remapping,
                     const PlannerProfileRemapping& composite_profile_remapping,
                     InstructionPoly* seed,
                     bool has_seed,
                     ProfileDictionary::ConstPtr profiles)
  : env(std::move(env))
  , manip_info(EMPTY_MANIPULATOR_INFO)
  , plan_profile_remapping(plan_profile_remapping)
  , composite_profile_remapping(composite_profile_remapping)
  , profiles(std::move(profiles))
  , has_seed(has_seed)
  , instruction_(instruction)
  , results_(seed)
{
}

TaskInput::TaskInput(tesseract_environment::Environment::ConstPtr env,
                     const InstructionPoly* instruction,
                     InstructionPoly* seed,
                     bool has_seed,
                     ProfileDictionary::ConstPtr profiles)
  : env(std::move(env))
  , manip_info(EMPTY_MANIPULATOR_INFO)
  , plan_profile_remapping(EMPTY_PROFILE_MAPPING)
  , composite_profile_remapping(EMPTY_PROFILE_MAPPING)
  , profiles(std::move(profiles))
  , has_seed(has_seed)
  , instruction_(instruction)
  , results_(seed)
{
}

TaskInput TaskInput::operator[](std::size_t index)
{
  TaskInput pi(*this);
  pi.instruction_indice_.push_back(index);

  return pi;
}

std::size_t TaskInput::size()
{
  const InstructionPoly* ci = instruction_;
  for (const auto& i : instruction_indice_)
  {
    if (ci->isCompositeInstruction())
    {
      const auto& composite = ci->as<CompositeInstruction>();
      ci = &(composite.at(i));
    }
    else
    {
      return 0;
    }
  }

  if (ci->isCompositeInstruction())
  {
    const auto& composite = ci->as<CompositeInstruction>();
    return composite.size();
  }

  return 0;
}

const InstructionPoly* TaskInput::getInstruction() const
{
  const InstructionPoly* ci = instruction_;
  for (const auto& i : instruction_indice_)
  {
    if (ci->isCompositeInstruction())
    {
      const auto& composite = ci->as<CompositeInstruction>();
      ci = &(composite.at(i));
    }
    else
    {
      return nullptr;
    }
  }
  return ci;
}

InstructionPoly* TaskInput::getResults()
{
  InstructionPoly* ci = results_;
  for (const auto& i : instruction_indice_)
  {
    if (ci->isCompositeInstruction())
    {
      auto& composite = ci->as<CompositeInstruction>();
      ci = &(composite.at(i));
    }
    else
    {
      return nullptr;
    }
  }
  return ci;
}

TaskflowInterface::Ptr TaskInput::getTaskInterface() { return interface_; }

bool TaskInput::isAborted() const { return interface_->isAborted(); }

void TaskInput::abort() { interface_->abort(); }

void TaskInput::setStartInstruction(InstructionPoly start)
{
  start_instruction_ = std::move(start);
  start_instruction_indice_.clear();
}

void TaskInput::setStartInstruction(std::vector<std::size_t> start)
{
  start_instruction_indice_ = std::move(start);
  start_instruction_ = InstructionPoly();
}

InstructionPoly TaskInput::getStartInstruction() const
{
  if (!start_instruction_.isNull())
    return start_instruction_;

  if (start_instruction_indice_.empty())
    return InstructionPoly();

  InstructionPoly* ci = results_;
  for (const auto& i : start_instruction_indice_)
  {
    if (ci->isCompositeInstruction())
    {
      auto& composite = ci->as<CompositeInstruction>();
      ci = &(composite.at(i));
    }
    else
    {
      return InstructionPoly();
    }
  }

  if (ci->isCompositeInstruction())
    return *ci->as<CompositeInstruction>().getLastMoveInstruction();

  return *ci;
}

void TaskInput::setEndInstruction(InstructionPoly end)
{
  end_instruction_ = std::move(end);
  end_instruction_indice_.clear();
}

void TaskInput::setEndInstruction(std::vector<std::size_t> end)
{
  end_instruction_indice_ = std::move(end);
  end_instruction_ = InstructionPoly();
}

InstructionPoly TaskInput::getEndInstruction() const
{
  if (!end_instruction_.isNull())
    return end_instruction_;

  if (end_instruction_indice_.empty())
    return InstructionPoly();

  InstructionPoly* ci = results_;
  for (const auto& i : end_instruction_indice_)
  {
    if (ci->isCompositeInstruction())
    {
      auto& composite = ci->as<CompositeInstruction>();
      ci = &(composite.at(i));
    }
    else
    {
      return InstructionPoly();
    }
  }

  if (ci->isCompositeInstruction())
  {
    /** @todo Should this get the first move instruction? */
    auto& composite = ci->as<CompositeInstruction>();
    if (composite.hasStartInstruction())
      return composite.getStartInstruction();

    return InstructionPoly();
  }

  return *ci;
}

void TaskInput::addTaskInfo(TaskInfo::UPtr task_info)
{
  interface_->getTaskInfoContainer()->addTaskInfo(std::move(task_info));
}

TaskInfo::UPtr TaskInput::getTaskInfo(const std::size_t& index) const
{
  return (*interface_->getTaskInfoContainer())[index];
}

std::map<std::size_t, TaskInfo::UPtr> TaskInput::getTaskInfoMap() const
{
  return interface_->getTaskInfoContainer()->getTaskInfoMap();
}
}  // namespace tesseract_planning
