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
#include <tesseract_process_managers/task_profiles/check_input_profile.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
CheckInputTaskGenerator::CheckInputTaskGenerator(std::string name) : TaskGenerator(std::move(name)) {}

int CheckInputTaskGenerator::conditionalProcess(TaskInput input, std::size_t /*unique_id*/) const
{
  // Get Composite Profile
  const InstructionPoly* input_instruction = input.getInstruction();
  const auto& ci = input_instruction->as<CompositeInstruction>();
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, input.composite_profile_remapping);
  auto cur_composite_profile =
      getProfile<CheckInputProfile>(name_, profile, *input.profiles, std::make_shared<CheckInputProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

  return ((cur_composite_profile->isValid(input)) ? 1 : 0);
}

void CheckInputTaskGenerator::process(TaskInput input, std::size_t unique_id) const
{
  conditionalProcess(input, unique_id);
}

CheckInputTaskInfo::CheckInputTaskInfo(std::size_t unique_id, std::string name) : TaskInfo(unique_id, std::move(name))
{
}

TaskInfo::UPtr CheckInputTaskInfo::clone() const { return std::make_unique<CheckInputTaskInfo>(*this); }

bool CheckInputTaskInfo::operator==(const CheckInputTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskInfo::operator==(rhs);
  return equal;
}
bool CheckInputTaskInfo::operator!=(const CheckInputTaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void CheckInputTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::CheckInputTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::CheckInputTaskInfo)
