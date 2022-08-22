/**
 * @file profile_switch_task.h
 * @brief Task that returns a value based on the profile
 *
 * @author Matthew Powelson
 * @date October 26. 2020
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_task_composer/nodes/profile_switch_task.h>
#include <tesseract_task_composer/profiles/profile_switch_profile.h>
#include <tesseract_command_language/constants.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
ProfileSwitchTask::ProfileSwitchTask(std::string input_key, bool is_conditional, std::string name)
  : TaskComposerTask(is_conditional, std::move(name)), input_key_(std::move(input_key))
{
}

int ProfileSwitchTask::run(TaskComposerInput& input, OptionalTaskComposerExecutor /*executor*/) const
{
  if (input.isAborted())
    return 0;

  auto info = std::make_unique<ProfileSwitchTaskInfo>(uuid_, name_);
  info->return_value = 0;
  tesseract_common::Timer timer;
  timer.start();
  //  saveInputs(*info, input);

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = input.data_storage->getData(input_key_);
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    CONSOLE_BRIDGE_logError("Input instruction to ProfileSwitch must be a composite instruction. Returning 0");
    //    saveOutputs(*info, input);
    info->elapsed_time = timer.elapsedSeconds();
    input.addTaskInfo(std::move(info));
    return 0;
  }

  // Get Composite Profile
  const auto& ci = input_data_poly.as<CompositeInstruction>();
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, input.composite_profile_remapping);
  auto cur_composite_profile =
      getProfile<ProfileSwitchProfile>(name_, profile, *input.profiles, std::make_shared<ProfileSwitchProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.profile_overrides);

  // Return the value specified in the profile
  CONSOLE_BRIDGE_logDebug("ProfileSwitchProfile returning %d", cur_composite_profile->return_value);

  //  saveOutputs(*info, input);
  info->elapsed_time = timer.elapsedSeconds();
  input.addTaskInfo(std::move(info));
  return cur_composite_profile->return_value;
}

bool ProfileSwitchTask::operator==(const ProfileSwitchTask& rhs) const
{
  bool equal = true;
  equal &= (input_key_ == rhs.input_key_);
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool ProfileSwitchTask::operator!=(const ProfileSwitchTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void ProfileSwitchTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(input_key_);
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

ProfileSwitchTaskInfo::ProfileSwitchTaskInfo(boost::uuids::uuid uuid, std::string name)
  : TaskComposerNodeInfo(uuid, std::move(name))
{
}

TaskComposerNodeInfo::UPtr ProfileSwitchTaskInfo::clone() const
{
  return std::make_unique<ProfileSwitchTaskInfo>(*this);
}

bool ProfileSwitchTaskInfo::operator==(const ProfileSwitchTaskInfo& rhs) const
{
  bool equal = true;
  equal &= TaskComposerNodeInfo::operator==(rhs);
  return equal;
}
bool ProfileSwitchTaskInfo::operator!=(const ProfileSwitchTaskInfo& rhs) const { return !operator==(rhs); }

template <class Archive>
void ProfileSwitchTaskInfo::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerNodeInfo);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ProfileSwitchTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ProfileSwitchTask)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ProfileSwitchTaskInfo)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ProfileSwitchTaskInfo)
