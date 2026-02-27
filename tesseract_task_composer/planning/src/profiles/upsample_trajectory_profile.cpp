/**
 * @file upsample_trajectory_profile.cpp
 *
 * @author Levi Armstrong
 * @date December 15, 2021
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

#include <tesseract_task_composer/planning/profiles/upsample_trajectory_profile.h>
#include <typeindex>
#include <yaml-cpp/yaml.h>
#include <tesseract/common/profile_plugin_factory.h>
#include <tesseract/common/utils.h>

namespace tesseract::task_composer
{
UpsampleTrajectoryProfile::UpsampleTrajectoryProfile() : Profile(createKey<UpsampleTrajectoryProfile>()) {}

UpsampleTrajectoryProfile::UpsampleTrajectoryProfile(double longest_valid_segment_length)
  : Profile(createKey<UpsampleTrajectoryProfile>()), longest_valid_segment_length(longest_valid_segment_length)
{
}
UpsampleTrajectoryProfile::UpsampleTrajectoryProfile(const YAML::Node& config,
                                                     const tesseract::common::ProfilePluginFactory& /*plugin_factory*/)
  : UpsampleTrajectoryProfile()
{
  try
  {
    longest_valid_segment_length = config["longest_valid_segment_length"].as<double>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("UpsampleTrajectoryProfile: Failed to parse yaml config! Details: " +
                             std::string(e.what()));
  }
}

bool UpsampleTrajectoryProfile::operator==(const UpsampleTrajectoryProfile& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  return tesseract::common::almostEqualRelativeAndAbs(
      longest_valid_segment_length, rhs.longest_valid_segment_length, max_diff);
}

bool UpsampleTrajectoryProfile::operator!=(const UpsampleTrajectoryProfile& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::task_composer
