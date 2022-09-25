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
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

//#include <tesseract_process_managers/core/utils.h>
#include <tesseract_task_composer/nodes/upsample_trajectory_task.h>
#include <tesseract_task_composer/profiles/upsample_trajectory_profile.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/core/interpolation.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
UpsampleTrajectoryTask::UpsampleTrajectoryTask(std::string input_key,
                                               std::string output_key,
                                               bool is_conditional,
                                               std::string name)
  : TaskComposerTask(is_conditional, std::move(name))
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));
}

TaskComposerNodeInfo::UPtr UpsampleTrajectoryTask::runImpl(TaskComposerInput& input,
                                                           OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;

  if (input.isAborted())
  {
    info->message = "Aborted";
    return info;
  }

  tesseract_common::Timer timer;
  timer.start();

  // Check that inputs are valid
  auto input_data_poly = input.data_storage.getData(input_keys_[0]);
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "Input seed to UpsampleTrajectoryTask must be a composite instruction";
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  // Get Composite Profile
  const auto& ci = input_data_poly.as<CompositeInstruction>();
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, input.problem.composite_profile_remapping);
  auto cur_composite_profile = getProfile<UpsampleTrajectoryProfile>(
      name_, profile, *input.profiles, std::make_shared<UpsampleTrajectoryProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

  assert(cur_composite_profile->longest_valid_segment_length > 0);
  InstructionPoly start_instruction = ci.getStartInstruction();
  CompositeInstruction new_results{ ci };
  new_results.clear();

  upsample(new_results, ci, start_instruction, cur_composite_profile->longest_valid_segment_length);
  input.data_storage.setData(output_keys_[0], new_results);

  info->message = "Successful";
  info->return_value = 1;
  info->elapsed_time = timer.elapsedSeconds();
  return info;
}

void UpsampleTrajectoryTask::upsample(CompositeInstruction& composite,
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

bool UpsampleTrajectoryTask::operator==(const UpsampleTrajectoryTask& rhs) const
{
  bool equal = true;
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool UpsampleTrajectoryTask::operator!=(const UpsampleTrajectoryTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void UpsampleTrajectoryTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::UpsampleTrajectoryTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::UpsampleTrajectoryTask)
