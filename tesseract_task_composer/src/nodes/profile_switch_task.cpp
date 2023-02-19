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
ProfileSwitchTask::ProfileSwitchTask() : TaskComposerTask("ProfileSwitchTask", true) {}
ProfileSwitchTask::ProfileSwitchTask(std::string name, std::string input_key, bool is_conditional)
  : TaskComposerTask(std::move(name), is_conditional)
{
  input_keys_.push_back(std::move(input_key));
}

ProfileSwitchTask::ProfileSwitchTask(std::string name,
                                     const YAML::Node& config,
                                     const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), config)
{
  if (input_keys_.empty())
    throw std::runtime_error("ProfileSwitchTask, config missing 'inputs' entry");

  if (input_keys_.size() > 1)
    throw std::runtime_error("ProfileSwitchTask, config 'inputs' entry currently only supports one input key");

  if (!output_keys_.empty())
    throw std::runtime_error("ProfileSwitchTask, does not support 'outputs' entry");
}

TaskComposerNodeInfo::UPtr ProfileSwitchTask::runImpl(TaskComposerInput& input,
                                                      OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;
  info->env = input.problem.env;

  if (input.isAborted())
  {
    info->message = "Aborted";
    return info;
  }

  tesseract_common::Timer timer;
  timer.start();

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = input.data_storage.getData(input_keys_[0]);
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->message = "Input instruction to ProfileSwitch must be a composite instruction";
    info->elapsed_time = timer.elapsedSeconds();
    CONSOLE_BRIDGE_logError("%s", info->message.c_str());
    return info;
  }

  // Get Composite Profile
  const auto& ci = input_data_poly.as<CompositeInstruction>();
  std::string profile = ci.getProfile();
  profile = getProfileString(name_, profile, input.problem.composite_profile_remapping);
  auto cur_composite_profile =
      getProfile<ProfileSwitchProfile>(name_, profile, *input.profiles, std::make_shared<ProfileSwitchProfile>());
  cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

  // Return the value specified in the profile
  CONSOLE_BRIDGE_logDebug("ProfileSwitchProfile returning %d", cur_composite_profile->return_value);

  info->message = "Successful";
  info->return_value = cur_composite_profile->return_value;
  info->elapsed_time = timer.elapsedSeconds();
  return info;
}

bool ProfileSwitchTask::operator==(const ProfileSwitchTask& rhs) const
{
  bool equal = true;
  equal &= TaskComposerTask::operator==(rhs);
  return equal;
}
bool ProfileSwitchTask::operator!=(const ProfileSwitchTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void ProfileSwitchTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ProfileSwitchTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ProfileSwitchTask)
