/**
 * @file fix_state_bounds_profile.cpp
 * @brief Profile for process that pushes plan instructions back within joint limits
 *
 * @author Matthew Powelson
 * @date August 31. 2020
 * @version TODO
 * @bug No known bugs
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
#include <tesseract_task_composer/planning/profiles/fix_state_bounds_profile.h>
#include <tesseract_task_composer/planning/yaml_extensions.h>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/profile_plugin_factory.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
FixStateBoundsProfile::FixStateBoundsProfile(Settings mode) : Profile(createKey<FixStateBoundsProfile>()), mode(mode) {}

FixStateBoundsProfile::FixStateBoundsProfile(const YAML::Node& config,
                                             const tesseract_common::ProfilePluginFactory& /*plugin_factory*/)
  : FixStateBoundsProfile()
{
  try
  {
    if (YAML::Node n = config["mode"])
      mode = n.as<Settings>();
    if (YAML::Node n = config["max_deviation_global"])
      max_deviation_global = n.as<double>();
    if (YAML::Node n = config["upper_bounds_reduction"])
      upper_bounds_reduction = n.as<double>();
    if (YAML::Node n = config["lower_bounds_reduction"])
      lower_bounds_reduction = n.as<double>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("FixStateBoundsProfile: Failed to parse yaml config! Details: " + std::string(e.what()));
  }
}

bool FixStateBoundsProfile::operator==(const FixStateBoundsProfile& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (mode == rhs.mode);
  equal &= tesseract_common::almostEqualRelativeAndAbs(max_deviation_global, rhs.max_deviation_global, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(upper_bounds_reduction, rhs.upper_bounds_reduction, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lower_bounds_reduction, rhs.lower_bounds_reduction, max_diff);
  return equal;
}

bool FixStateBoundsProfile::operator!=(const FixStateBoundsProfile& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_planning
