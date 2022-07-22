/**
 * @file motion_planner_task_generator.cpp
 * @brief Perform motion planning
 *
 * @author Levi Armstrong
 * @date August 11. 2020
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
#include <tesseract_process_managers/task_generators/motion_planner_task_generator.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_motion_planners/core/planner.h>

namespace tesseract_planning
{
MotionPlannerTaskGenerator::MotionPlannerTaskGenerator(std::shared_ptr<MotionPlanner> planner)
  : TaskGenerator(planner->getName()), planner_(std::move(planner))
{
}

int MotionPlannerTaskGenerator::conditionalProcess(TaskInput input, std::size_t unique_id) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_unique<MotionPlannerTaskInfo>(unique_id, name_);
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
    info->message = "Input instructions to MotionPlannerTaskGenerator: " + name_ + " must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  InstructionPoly* input_results = input.getResults();
  if (!input_results->isCompositeInstruction())
  {
    info->message = "Input seed to MotionPlannerTaskGenerator: " + name_ + " must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  // Make a non-const copy of the input instructions to update the start/end
  CompositeInstruction instructions = input_instruction->as<CompositeInstruction>();
  assert(!(input.manip_info.empty() && input.manip_info.empty()));
  instructions.setManipulatorInfo(instructions.getManipulatorInfo().getCombined(input.manip_info));

  // If the start and end waypoints need to be updated prior to planning
  InstructionPoly start_instruction = input.getStartInstruction();
  InstructionPoly end_instruction = input.getEndInstruction();

  if (!start_instruction.isNull())
  {
    // add start
    if (start_instruction.isCompositeInstruction())
    {
      // if provided a composite instruction as the start instruction it will extract the last move instruction
      const auto& ci = start_instruction.as<CompositeInstruction>();
      const auto* lmi = ci.getLastMoveInstruction();
      assert(lmi != nullptr);
      instructions.setStartInstruction(*lmi);
      instructions.getStartInstruction().setMoveType(MoveInstructionType::START);
    }
    else
    {
      assert(start_instruction.isMoveInstruction());
      if (start_instruction.isMoveInstruction())
      {
        instructions.setStartInstruction(start_instruction.as<MoveInstructionPoly>());
        instructions.getStartInstruction().setMoveType(MoveInstructionType::START);
      }
    }
  }
  if (!end_instruction.isNull())
  {
    // add end
    if (end_instruction.isCompositeInstruction())
    {
      // if provided a composite instruction as the end instruction it will extract the first move instruction
      const auto& ci = end_instruction.as<CompositeInstruction>();
      const auto* fmi = ci.getFirstMoveInstruction();
      assert(fmi != nullptr);
      if (fmi->getWaypoint().isCartesianWaypoint())
        instructions.getLastMoveInstruction()->assignCartesianWaypoint(fmi->getWaypoint().as<CartesianWaypointPoly>());
      else if (fmi->getWaypoint().isJointWaypoint())
        instructions.getLastMoveInstruction()->assignJointWaypoint(fmi->getWaypoint().as<JointWaypointPoly>());
      else if (fmi->getWaypoint().isStateWaypoint())
        instructions.getLastMoveInstruction()->assignStateWaypoint(fmi->getWaypoint().as<StateWaypointPoly>());
      else
        throw std::runtime_error("Invalid waypoint type");
    }
    else
    {
      assert(end_instruction.isMoveInstruction());
      auto* lpi = instructions.getLastMoveInstruction();
      if (end_instruction.isMoveInstruction())
      {
        const auto& wp = end_instruction.as<MoveInstructionPoly>().getWaypoint();
        if (wp.isCartesianWaypoint())
          lpi->assignCartesianWaypoint(wp.as<CartesianWaypointPoly>());
        else if (wp.isJointWaypoint())
          lpi->assignJointWaypoint(wp.as<JointWaypointPoly>());
        else if (wp.isStateWaypoint())
          lpi->assignStateWaypoint(wp.as<StateWaypointPoly>());
        else
          throw std::runtime_error("Invalid waypoint type");
      }
    }
  }

  // It should always have a start instruction which required by the motion planners
  assert(instructions.hasStartInstruction());

  // --------------------
  // Fill out request
  // --------------------
  PlannerRequest request;
  request.seed = input_results->as<CompositeInstruction>();
  request.env_state = input.env->getState();
  request.env = input.env;
  request.instructions = instructions;
  request.profiles = input.profiles;
  request.plan_profile_remapping = input.plan_profile_remapping;
  request.composite_profile_remapping = input.composite_profile_remapping;

  // --------------------
  // Fill out response
  // --------------------
  PlannerResponse response;

  bool verbose = false;
  if (console_bridge::getLogLevel() == console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG)
    verbose = true;
  auto status = planner_->solve(request, response, verbose);

  // --------------------
  // Verify Success
  // --------------------
  if (status)
  {
    *input_results = response.results;
    CONSOLE_BRIDGE_logDebug("Motion Planner process succeeded");
    info->return_value = 1;
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 1;
  }

  CONSOLE_BRIDGE_logInform("%s motion planning failed (%s) for process input: %s",
                           planner_->getName().c_str(),
                           status.message().c_str(),
                           input_instruction->getDescription().c_str());
  info->message = status.message();
  saveOutputs(*info, input);
  info->elapsed_time = timer.elapsedSeconds();
  input.addTaskInfo(std::move(info));
  return 0;
}

void MotionPlannerTaskGenerator::process(TaskInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

MotionPlannerTaskInfo::MotionPlannerTaskInfo(std::size_t unique_id, std::string name)
  : TaskInfo(unique_id, std::move(name))
{
}

TaskInfo::UPtr MotionPlannerTaskInfo::clone() const { return std::make_unique<MotionPlannerTaskInfo>(*this); }

bool MotionPlannerTaskInfo::operator==(const MotionPlannerTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskInfo::operator==(rhs);
  return equal;
}
bool MotionPlannerTaskInfo::operator!=(const MotionPlannerTaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void MotionPlannerTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MotionPlannerTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::MotionPlannerTaskInfo)
