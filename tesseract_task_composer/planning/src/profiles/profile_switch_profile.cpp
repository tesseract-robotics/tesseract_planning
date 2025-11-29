/**
 * @file profile_switch_profile.h
 * @brief Profile for task that returns a value based on the profile
 *
 * @author Matthew Powelson
 * @date October 26. 2020
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

#include <tesseract_task_composer/planning/profiles/profile_switch_profile.h>
#include <typeindex>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/profile_plugin_factory.h>

namespace tesseract_planning
{
ProfileSwitchProfile::ProfileSwitchProfile(int return_value)
  : Profile(createKey<ProfileSwitchProfile>()), return_value(return_value)
{
}
ProfileSwitchProfile::ProfileSwitchProfile(const YAML::Node& config,
                                           const tesseract_common::ProfilePluginFactory& /*plugin_factory*/)
  : ProfileSwitchProfile()
{
  try
  {
    return_value = config["return_value"].as<int>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("ProfileSwitchProfile: Failed to parse yaml config! Details: " + std::string(e.what()));
  }
}

bool ProfileSwitchProfile::operator==(const ProfileSwitchProfile& rhs) const
{
  return (return_value == rhs.return_value);
}

bool ProfileSwitchProfile::operator!=(const ProfileSwitchProfile& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_planning
