/**
 * @file upsample_trajectory_task.cpp
 *
 * @author Levi Armstrong
 * @date December 15, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Levi Armstrong
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

#include <tesseract_task_composer/planning/nodes/upsample_trajectory_task.h>
#include <tesseract_task_composer/planning/profiles/upsample_trajectory_profile.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/planner_utils.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>

namespace tesseract_planning
{
// Requried
const std::string UpsampleTrajectoryTask::INOUT_PROGRAM_PORT = "program";
const std::string UpsampleTrajectoryTask::INPUT_PROFILES_PORT = "profiles";

UpsampleTrajectoryTask::UpsampleTrajectoryTask()
  : TaskComposerTask("UpsampleTrajectoryTask", UpsampleTrajectoryTask::ports(), false)
{
}
UpsampleTrajectoryTask::UpsampleTrajectoryTask(std::string name,
                                               std::string input_program_key,
                                               std::string input_profiles_key,
                                               std::string output_program_key,
                                               bool conditional)
  : TaskComposerTask(std::move(name), UpsampleTrajectoryTask::ports(), conditional)
{
  input_keys_.add(INOUT_PROGRAM_PORT, std::move(input_program_key));
  input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
  output_keys_.add(INOUT_PROGRAM_PORT, std::move(output_program_key));
  validatePorts();
}

UpsampleTrajectoryTask::UpsampleTrajectoryTask(std::string name,
                                               const YAML::Node& config,
                                               const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), UpsampleTrajectoryTask::ports(), config)
{
}

TaskComposerNodePorts UpsampleTrajectoryTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_PROFILES_PORT] = TaskComposerNodePorts::SINGLE;
  ports.output_required[INOUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  return ports;
}

std::unique_ptr<TaskComposerNodeInfo> UpsampleTrajectoryTask::runImpl(TaskComposerContext& context,
                                                                      OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;
  info->status_code = 0;

  // Check that inputs are valid
  auto input_data_poly = getData(*context.data_storage, INOUT_PROGRAM_PORT);
  if (input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->status_message = "Input seed to UpsampleTrajectoryTask must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
    return info;
  }

  // Get Composite Profile
  auto profiles = getData(*context.data_storage, INPUT_PROFILES_PORT).as<std::shared_ptr<ProfileDictionary>>();
  const auto& ci = input_data_poly.as<CompositeInstruction>();
  auto cur_composite_profile = getProfile<UpsampleTrajectoryProfile>(
      ns_, ci.getProfile(ns_), *profiles, std::make_shared<UpsampleTrajectoryProfile>());

  assert(cur_composite_profile->longest_valid_segment_length > 0);
  InstructionPoly start_instruction;
  CompositeInstruction new_results{ ci };
  new_results.clear();

  upsample(new_results, ci, start_instruction, cur_composite_profile->longest_valid_segment_length);
  setData(*context.data_storage, INOUT_PROGRAM_PORT, new_results);

  info->color = "green";
  info->status_code = 1;
  info->status_message = "Successful";
  info->return_value = 1;
  return info;
}

void UpsampleTrajectoryTask::upsample(CompositeInstruction& composite,
                                      const CompositeInstruction& current_composite,
                                      InstructionPoly& start_instruction,
                                      double longest_valid_segment_length) const
{
  for (const InstructionPoly& i : current_composite)
  {
    if (i.isCompositeInstruction())
    {
      const auto& cc = i.as<CompositeInstruction>();
      CompositeInstruction new_cc(cc);
      new_cc.clear();

      upsample(new_cc, cc, start_instruction, longest_valid_segment_length);
      composite.push_back(new_cc);
    }
    else if (i.isMoveInstruction())
    {
      if (start_instruction.isNull())
      {
        start_instruction = i.as<MoveInstructionPoly>();
        composite.push_back(i);  // Prevents loss of very first waypoint when upsampling
        continue;
      }

      assert(start_instruction.isMoveInstruction());
      const auto& mi0 = start_instruction.as<MoveInstructionPoly>();
      const auto& mi1 = i.as<MoveInstructionPoly>();

      assert(mi0.getWaypoint().isStateWaypoint());
      assert(mi1.getWaypoint().isStateWaypoint());
      const auto& swp0 = mi0.getWaypoint().as<StateWaypointPoly>();
      const auto& swp1 = mi1.getWaypoint().as<StateWaypointPoly>();

      double dist = (swp1.getPosition() - swp0.getPosition()).norm();
      if (dist > longest_valid_segment_length)
      {
        long cnt = static_cast<long>(std::ceil(dist / longest_valid_segment_length)) + 1;

        // Linearly interpolate in joint space
        Eigen::MatrixXd states = interpolate(swp0.getPosition(), swp1.getPosition(), cnt);

        // Since this is filling out a new composite instruction and the start is the previous
        // instruction it is excluded when populated the composite instruction.
        for (long i = 1; i < states.cols(); ++i)
        {
          MoveInstructionPoly move_instruction(mi1);
          move_instruction.getWaypoint().as<StateWaypointPoly>().setPosition(states.col(i));
          composite.appendMoveInstruction(move_instruction);
        }
      }
      else
      {
        composite.push_back(i);
      }

      start_instruction = i;
    }
    else
    {
      assert(!i.isMoveInstruction());
      composite.push_back(i);
    }
  }
}

bool UpsampleTrajectoryTask::operator==(const UpsampleTrajectoryTask& rhs) const
{
  return (TaskComposerTask::operator==(rhs));
}
bool UpsampleTrajectoryTask::operator!=(const UpsampleTrajectoryTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void UpsampleTrajectoryTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::UpsampleTrajectoryTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::UpsampleTrajectoryTask)
