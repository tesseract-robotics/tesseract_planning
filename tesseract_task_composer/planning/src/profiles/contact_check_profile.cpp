/**
 * @file contact_check_profile.cpp
 * @brief Contact check trajectory profile
 *
 * @author Levi Armstrong
 * @date August 10. 2020
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
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract_collision/core/yaml_extensions.h>
#include <tesseract_common/profile_plugin_factory.h>

namespace tesseract_planning
{
ContactCheckProfile::ContactCheckProfile() : ContactCheckProfile(0.05, 0) {}

ContactCheckProfile::ContactCheckProfile(double longest_valid_segment_length, double contact_distance)
  : Profile(createKey<ContactCheckProfile>())
{
  contact_manager_config.default_margin = contact_distance;

  collision_check_config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
  collision_check_config.longest_valid_segment_length = longest_valid_segment_length;

  if (collision_check_config.longest_valid_segment_length <= 0)
  {
    CONSOLE_BRIDGE_logWarn("ContactCheckProfile: Invalid longest valid segment. Defaulting to 0.05");
    collision_check_config.longest_valid_segment_length = 0.05;
  }
}

ContactCheckProfile::ContactCheckProfile(const YAML::Node& config,
                                         const tesseract_common::ProfilePluginFactory& /*plugin_factory*/)
  : ContactCheckProfile()
{
  try
  {
    if (YAML::Node n = config["contact_manager_config"])
      contact_manager_config = n.as<tesseract_collision::ContactManagerConfig>();

    if (YAML::Node n = config["collision_check_config"])
      collision_check_config = n.as<tesseract_collision::CollisionCheckConfig>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("ContactCheckProfile: Failed to parse yaml config! Details: " + std::string(e.what()));
  }
}

bool ContactCheckProfile::operator==(const ContactCheckProfile& rhs) const
{
  bool equal = true;
  equal &= (contact_manager_config == rhs.contact_manager_config);
  equal &= (collision_check_config == rhs.collision_check_config);
  return equal;
}

bool ContactCheckProfile::operator!=(const ContactCheckProfile& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_planning
