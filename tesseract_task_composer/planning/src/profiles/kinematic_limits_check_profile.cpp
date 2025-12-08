/**
 * @file kinematic_limits_check_profile.cpp
 * @brief Profile for kinematic limits check task
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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
#include <tesseract_task_composer/planning/profiles/kinematic_limits_check_profile.h>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/profile_plugin_factory.h>

namespace tesseract_planning
{
KinematicLimitsCheckProfile::KinematicLimitsCheckProfile(bool check_position,
                                                         bool check_velocity,
                                                         bool check_acceleration)
  : Profile(createKey<KinematicLimitsCheckProfile>())
  , check_position(check_position)
  , check_velocity(check_velocity)
  , check_acceleration(check_acceleration)
{
}

KinematicLimitsCheckProfile::KinematicLimitsCheckProfile(
    const YAML::Node& config,
    const tesseract_common::ProfilePluginFactory& /*plugin_factory*/)
  : KinematicLimitsCheckProfile()
{
  try
  {
    if (YAML::Node n = config["check_position"])
      check_position = n.as<bool>();
    if (YAML::Node n = config["check_velocity"])
      check_velocity = n.as<bool>();
    if (YAML::Node n = config["check_acceleration"])
      check_acceleration = n.as<bool>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("KinematicLimitsCheckProfile: Failed to parse yaml config! Details: " +
                             std::string(e.what()));
  }
}

bool KinematicLimitsCheckProfile::operator==(const KinematicLimitsCheckProfile& rhs) const
{
  bool equal = true;
  equal &= (check_position == rhs.check_position);
  equal &= (check_velocity == rhs.check_velocity);
  equal &= (check_acceleration == rhs.check_acceleration);
  return equal;
}

bool KinematicLimitsCheckProfile::operator!=(const KinematicLimitsCheckProfile& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_planning
