/**
 * @file upsample_trajectory_task_generator.cpp
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_process_managers/core/utils.h>
#include <tesseract_process_managers/task_generators/upsample_trajectory_task_generator.h>
#include <tesseract_process_managers/task_profiles/upsample_trajectory_profile.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
UpsampleTrajectoryTaskGenerator::UpsampleTrajectoryTaskGenerator(std::string name) : TaskGenerator(std::move(name)) {}

int UpsampleTrajectoryTaskGenerator::conditionalProcess(TaskInput input, std::size_t unique_id) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_unique<UpsampleTrajectoryTaskInfo>(unique_id, name_);
  info->return_value = 0;
  tesseract_common::Timer timer;
  timer.start();
  saveInputs(*info, input);

  // Check that inputs are valid
  InstructionPoly* input_results = input.getResults();
  if (!input_results->isCompositeInstruction())
  {
    CONSOLE_BRIDGE_logError("Input seed to UpsampleTrajectoryTaskGenerator must be a composite instruction");
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  // Get Composite Profile
  const auto& ci = input_results->as<CompositeInstruction>();
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, input.composite_profile_remapping);
  auto cur_composite_profile = getProfile<UpsampleTrajectoryProfile>(
      name_, profile, *input.profiles, std::make_shared<UpsampleTrajectoryProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

  assert(cur_composite_profile->longest_valid_segment_length > 0);
  auto& results = input_results->as<CompositeInstruction>();
  InstructionPoly start_instruction = results.getStartInstruction();
  CompositeInstruction new_results(results);
  new_results.clear();

  upsample(new_results, results, start_instruction, cur_composite_profile->longest_valid_segment_length);
  results = new_results;

  info->return_value = 1;
  saveOutputs(*info, input);
  info->elapsed_time = timer.elapsedSeconds();
  input.addTaskInfo(std::move(info));
  return 1;
}

void UpsampleTrajectoryTaskGenerator::process(TaskInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

void UpsampleTrajectoryTaskGenerator::upsample(CompositeInstruction& composite,
                                               const CompositeInstruction& current_composite,
                                               InstructionPoly& start_instruction,
                                               double longest_valid_segment_length) const
{
  for (const InstructionPoly& i : current_composite)
  {
    assert(!i.isMoveInstruction());
    if (i.isCompositeInstruction())
    {
      const auto& cc = i.as<CompositeInstruction>();
      CompositeInstruction new_cc(cc);
      new_cc.clear();

      upsample(new_cc, cc, start_instruction, longest_valid_segment_length);
      composite.appendInstruction(new_cc);
    }
    else if (i.isMoveInstruction())
    {
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
        composite.appendInstruction(i);
      }

      start_instruction = i;
    }
    else
    {
      assert(!i.isMoveInstruction());
      composite.appendInstruction(i);
    }
  }
}
UpsampleTrajectoryTaskInfo::UpsampleTrajectoryTaskInfo(std::size_t unique_id, std::string name)
  : TaskInfo(unique_id, std::move(name))
{
}

TaskInfo::UPtr UpsampleTrajectoryTaskInfo::clone() const { return std::make_unique<UpsampleTrajectoryTaskInfo>(*this); }

bool UpsampleTrajectoryTaskInfo::operator==(const UpsampleTrajectoryTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskInfo::operator==(rhs);
  return equal;
}
bool UpsampleTrajectoryTaskInfo::operator!=(const UpsampleTrajectoryTaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void UpsampleTrajectoryTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::UpsampleTrajectoryTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::UpsampleTrajectoryTaskInfo)
