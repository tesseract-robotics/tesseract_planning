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

#include <tesseract_common/serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/profile_switch_task.h>
#include <tesseract_task_composer/planning/profiles/profile_switch_profile.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>

#include <tesseract_command_language/constants.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
// Requried
const std::string ProfileSwitchTask::INPUT_PROGRAM_PORT = "program";
const std::string ProfileSwitchTask::INPUT_PROFILES_PORT = "profiles";

ProfileSwitchTask::ProfileSwitchTask() : TaskComposerTask("ProfileSwitchTask", ProfileSwitchTask::ports(), true) {}
ProfileSwitchTask::ProfileSwitchTask(std::string name,
                                     std::string input_program_key,
                                     std::string input_profiles_key,
                                     bool is_conditional)
  : TaskComposerTask(std::move(name), ProfileSwitchTask::ports(), is_conditional)
{
  input_keys_.add(INPUT_PROGRAM_PORT, std::move(input_program_key));
  input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
  validatePorts();
}

ProfileSwitchTask::ProfileSwitchTask(std::string name,
                                     const YAML::Node& config,
                                     const TaskComposerPluginFactory& /*plugin_factory*/)
  : TaskComposerTask(std::move(name), ProfileSwitchTask::ports(), config)
{
}

TaskComposerNodePorts ProfileSwitchTask::ports()
{
  TaskComposerNodePorts ports;
  ports.input_required[INPUT_PROGRAM_PORT] = TaskComposerNodePorts::SINGLE;
  ports.input_required[INPUT_PROFILES_PORT] = TaskComposerNodePorts::SINGLE;
  return ports;
}

std::unique_ptr<TaskComposerNodeInfo> ProfileSwitchTask::runImpl(TaskComposerContext& context,
                                                                 OptionalTaskComposerExecutor /*executor*/) const
{
  auto info = std::make_unique<TaskComposerNodeInfo>(*this);
  info->return_value = 0;
  info->status_code = 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = getData(*context.data_storage, INPUT_PROGRAM_PORT);
  if (input_data_poly.isNull() || input_data_poly.getType() != std::type_index(typeid(CompositeInstruction)))
  {
    info->status_message = "Input instruction to ProfileSwitch must be a composite instruction";
    CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
    return info;
  }

  // Get Composite Profile
  auto profiles = getData(*context.data_storage, INPUT_PROFILES_PORT).as<std::shared_ptr<ProfileDictionary>>();
  const auto& ci = input_data_poly.as<CompositeInstruction>();
  auto cur_composite_profile =
      getProfile<ProfileSwitchProfile>(ns_, ci.getProfile(ns_), *profiles, std::make_shared<ProfileSwitchProfile>());

  // Return the value specified in the profile
  CONSOLE_BRIDGE_logDebug("ProfileSwitchProfile returning %d", cur_composite_profile->return_value);

  info->color = "green";
  info->status_code = 1;
  info->status_message = "Successful";
  info->return_value = cur_composite_profile->return_value;
  return info;
}

bool ProfileSwitchTask::operator==(const ProfileSwitchTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool ProfileSwitchTask::operator!=(const ProfileSwitchTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void ProfileSwitchTask::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ProfileSwitchTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ProfileSwitchTask)
