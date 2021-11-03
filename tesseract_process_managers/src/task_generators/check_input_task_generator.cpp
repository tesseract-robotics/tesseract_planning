/**
 * @file check_input_task_generator.h
 * @brief Process generator for checking input data structure
 *
 * @author Levi Armstrong
 * @date November 2. 2021
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_process_managers/task_generators/check_input_task_generator.h>

namespace tesseract_planning
{
CheckInputTaskGenerator::CheckInputTaskGenerator(std::string name)
  : TaskGenerator(std::move(name)), fn_(tesseract_planning::CheckInputTaskGenerator::checkTaskInput)
{
}

CheckInputTaskGenerator::CheckInputTaskGenerator(CheckInputFn fn, std::string name)
  : TaskGenerator(std::move(name)), fn_(std::move(fn))
{
}

int CheckInputTaskGenerator::conditionalProcess(TaskInput input, std::size_t /*unique_id*/) const
{
  return ((fn_(input)) ? 1 : 0);
}

void CheckInputTaskGenerator::process(TaskInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

bool CheckInputTaskGenerator::checkTaskInput(const TaskInput& input)
{
  // Check Input
  if (!input.env)
  {
    CONSOLE_BRIDGE_logError("TaskInput env is a nullptr");
    return false;
  }

  // Check the overall input
  const Instruction* input_instruction = input.getInstruction();
  if (!isCompositeInstruction(*input_instruction))
  {
    CONSOLE_BRIDGE_logError("TaskInput Invalid: input.instructions should be a composite");
    return false;
  }

  return true;
}

CheckInputTaskInfo::CheckInputTaskInfo(std::size_t unique_id, std::string name) : TaskInfo(unique_id, std::move(name))
{
}
}  // namespace tesseract_planning
