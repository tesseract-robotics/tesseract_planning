/**
 * @file seed_length_task_generator.cpp
 * @brief Process generator for processing the seed so it meets a minimum length. Planners like trajopt need
 * at least 10 states in the trajectory to perform velocity, accelleration and jerk smoothing.
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

#include <tesseract_process_managers/core/utils.h>
#include <tesseract_process_managers/task_generators/seed_min_length_task_generator.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>

namespace tesseract_planning
{
SeedMinLengthTaskGenerator::SeedMinLengthTaskGenerator(std::string name) : TaskGenerator(std::move(name)) {}

SeedMinLengthTaskGenerator::SeedMinLengthTaskGenerator(long min_length, std::string name)
  : TaskGenerator(std::move(name)), min_length_(min_length)
{
  if (min_length_ <= 1)
  {
    CONSOLE_BRIDGE_logWarn("SeedMinLengthTaskGenerator: The min length must be greater than 1, setting to default.");
    min_length_ = 10;
  }
}

int SeedMinLengthTaskGenerator::conditionalProcess(TaskInput input, std::size_t unique_id) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_shared<SeedMinLengthTaskInfo>(unique_id, name_);
  info->return_value = 0;
  input.addTaskInfo(info);
  saveInputs(info, input);

  // Check that inputs are valid
  Instruction* input_results = input.getResults();
  if (!isCompositeInstruction(*input_results))
  {
    CONSOLE_BRIDGE_logError("Input seed to SeedMinLengthTaskGenerator must be a composite instruction");
    saveOutputs(info, input);
    return 0;
  }

  CompositeInstruction& results = input_results->as<CompositeInstruction>();
  long cnt = getMoveInstructionCount(results);
  if (cnt >= min_length_)
  {
    info->return_value = 1;
    saveOutputs(info, input);
    return 1;
  }

  Instruction start_instruction = results.getStartInstruction();
  int subdivisions = static_cast<int>(std::ceil(static_cast<double>(min_length_) / static_cast<double>(cnt))) + 1;

  CompositeInstruction new_results(results);
  new_results.clear();

  subdivide(new_results, results, start_instruction, subdivisions);
  results = new_results;

  CONSOLE_BRIDGE_logDebug("Seed Min Length Process Generator Succeeded!");
  info->return_value = 1;
  saveOutputs(info, input);
  return 1;
}

void SeedMinLengthTaskGenerator::process(TaskInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

void SeedMinLengthTaskGenerator::subdivide(CompositeInstruction& composite,
                                           const CompositeInstruction& current_composite,
                                           Instruction& start_instruction,
                                           int subdivisions) const
{
  for (const Instruction& i : current_composite)
  {
    assert(!isPlanInstruction(i));
    if (isCompositeInstruction(i))
    {
      const CompositeInstruction& cc = i.as<CompositeInstruction>();
      CompositeInstruction new_cc(cc);
      new_cc.clear();

      subdivide(new_cc, cc, start_instruction, subdivisions);
      composite.push_back(new_cc);
    }
    else if (isMoveInstruction(i))
    {
      assert(isMoveInstruction(start_instruction));
      const MoveInstruction& mi0 = start_instruction.as<MoveInstruction>();
      const MoveInstruction& mi1 = i.as<MoveInstruction>();

      assert(isStateWaypoint(mi0.getWaypoint()));
      assert(isStateWaypoint(mi1.getWaypoint()));
      const StateWaypoint& swp0 = mi0.getWaypoint().as<StateWaypoint>();
      const StateWaypoint& swp1 = mi1.getWaypoint().as<StateWaypoint>();

      // Linearly interpolate in joint space
      Eigen::MatrixXd states = interpolate(swp0.position, swp1.position, subdivisions);

      // Convert to MoveInstructions
      for (long i = 1; i < states.cols(); ++i)
      {
        MoveInstruction move_instruction(mi1);
        move_instruction.getWaypoint().as<StateWaypoint>().position = states.col(i);
        composite.push_back(move_instruction);
      }

      start_instruction = i;
    }
    else
    {
      assert(!isPlanInstruction(i));
      composite.push_back(i);
    }
  }
}
SeedMinLengthTaskInfo::SeedMinLengthTaskInfo(std::size_t unique_id, std::string name)
  : TaskInfo(unique_id, std::move(name))
{
}
}  // namespace tesseract_planning
