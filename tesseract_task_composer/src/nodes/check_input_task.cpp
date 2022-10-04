/**
 * @file check_input_task.cpp
 * @brief Task for checking input data structure
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
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/timer.h>

#include <tesseract_task_composer/nodes/check_input_task.h>
#include <tesseract_task_composer/profiles/check_input_profile.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
CheckInputTask::CheckInputTask(std::vector<std::string> input_keys, bool is_conditional, std::string name)
  : TaskComposerTask(is_conditional, std::move(name))
{
  input_keys_ = std::move(input_keys);
}

CheckInputTask::CheckInputTask(std::string input_key, bool is_conditional, std::string name)
  : CheckInputTask(std::vector<std::string>({ std::move(input_key) }), is_conditional, std::move(name))
{
}

TaskComposerNodeInfo::UPtr CheckInputTask::runImpl(TaskComposerInput& input,
                                                   OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;

  if (input.isAborted())
  {
    info->message = "Aborted";
    return info;
  }

  // Get Composite Profile
  for (const auto& key : input_keys_)
  {
    auto input_data_poly = input.data_storage.getData(key);
    if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
    {
      info->message = "Input key '" + key + "' is missing";
      CONSOLE_BRIDGE_logError("%s", info->message.c_str());
      return info;
    }

    const auto& ci = input_data_poly.as<CompositeInstruction>();
    std::string profile = ci.getProfile();
    profile = getProfileString(name_, profile, input.problem.composite_profile_remapping);
    auto cur_composite_profile =
        getProfile<CheckInputProfile>(name_, profile, *input.profiles, std::make_shared<CheckInputProfile>());
    cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

    if (!cur_composite_profile->isValid(input))
    {
      info->message = "Validator failed";
      return info;
    }
  }

  info->env = input.problem.env;
  info->message = "Successful";
  info->return_value = 1;
  return info;
}

bool CheckInputTask::operator==(const CheckInputTask& rhs) const
{
  bool equal = true;
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool CheckInputTask::operator!=(const CheckInputTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void CheckInputTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::CheckInputTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::CheckInputTask)
