/**
 * @file seed_length_task_generator.cpp
 * @brief Process generator for processing the seed so it meets a minimum length. Planners like trajopt need
 * at least 10 states in the trajectory to perform velocity, acceleration and jerk smoothing.
 *
 * @author Levi Armstrong
 * @date November 2. 2020
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
#include <tesseract_process_managers/task_generators/seed_min_length_task_generator.h>
#include <tesseract_process_managers/task_profiles/seed_min_length_profile.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
SeedMinLengthTaskGenerator::SeedMinLengthTaskGenerator(std::string name) : TaskGenerator(std::move(name)) {}

int SeedMinLengthTaskGenerator::conditionalProcess(TaskInput input, std::size_t unique_id) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_unique<SeedMinLengthTaskInfo>(unique_id, name_);
  info->return_value = 0;
  tesseract_common::Timer timer;
  timer.start();
  saveInputs(*info, input);

  // Check that inputs are valid
  InstructionPoly* input_results = input.getResults();
  if (!input_results->isCompositeInstruction())
  {
    CONSOLE_BRIDGE_logError("Input seed to SeedMinLengthTaskGenerator must be a composite instruction");
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  // Get Composite Profile
  const auto& ci = input_results->as<CompositeInstruction>();
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, input.composite_profile_remapping);
  auto cur_composite_profile =
      getProfile<SeedMinLengthProfile>(name_, profile, *input.profiles, std::make_shared<SeedMinLengthProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

  auto& results = input_results->as<CompositeInstruction>();
  long cnt = results.getMoveInstructionCount();
  if (cnt >= cur_composite_profile->min_length)
  {
    info->return_value = 1;
    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 1;
  }

  InstructionPoly start_instruction = results.getStartInstruction();
  auto subdivisions =
      static_cast<int>(std::ceil(static_cast<double>(cur_composite_profile->min_length) / static_cast<double>(cnt))) +
      1;

  CompositeInstruction new_results(results);
  new_results.clear();

  subdivide(new_results, results, start_instruction, subdivisions);
  results = new_results;

  CONSOLE_BRIDGE_logDebug("Seed Min Length Process Generator Succeeded!");
  info->return_value = 1;
  saveOutputs(*info, input);
  info->elapsed_time = timer.elapsedSeconds();
  input.addTaskInfo(std::move(info));
  return 1;
}

void SeedMinLengthTaskGenerator::process(TaskInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

void SeedMinLengthTaskGenerator::subdivide(CompositeInstruction& composite,
                                           const CompositeInstruction& current_composite,
                                           InstructionPoly& start_instruction,
                                           int subdivisions) const
{
  for (const InstructionPoly& i : current_composite)
  {
    if (i.isCompositeInstruction())
    {
      const auto& cc = i.as<CompositeInstruction>();
      CompositeInstruction new_cc(cc);
      new_cc.clear();

      subdivide(new_cc, cc, start_instruction, subdivisions);
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

      // Linearly interpolate in joint space
      Eigen::MatrixXd states = interpolate(swp0.getPosition(), swp1.getPosition(), subdivisions);

      // Convert to MoveInstructions
      for (long i = 1; i < states.cols(); ++i)
      {
        MoveInstructionPoly move_instruction(mi1);
        move_instruction.getWaypoint().as<StateWaypointPoly>().getPosition() = states.col(i);
        composite.appendMoveInstruction(move_instruction);
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
SeedMinLengthTaskInfo::SeedMinLengthTaskInfo(std::size_t unique_id, std::string name)
  : TaskInfo(unique_id, std::move(name))
{
}

TaskInfo::UPtr SeedMinLengthTaskInfo::clone() const { return std::make_unique<SeedMinLengthTaskInfo>(*this); }

bool SeedMinLengthTaskInfo::operator==(const SeedMinLengthTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskInfo::operator==(rhs);
  return equal;
}
bool SeedMinLengthTaskInfo::operator!=(const SeedMinLengthTaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void SeedMinLengthTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::SeedMinLengthTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::SeedMinLengthTaskInfo)
