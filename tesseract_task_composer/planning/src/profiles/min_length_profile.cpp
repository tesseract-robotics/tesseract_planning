/**
 * @file min_length_profile.cpp
 * @brief Profile for task that processing the program so it meets a minimum length. Planners like trajopt
 * need at least the user defined number of states in the trajectory to perform velocity, acceleration and jerk
 * smoothing.
 *
 * @author Levi Armstrong
 * @date November 2. 2020
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

#include <tesseract_task_composer/planning/profiles/min_length_profile.h>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/profile_plugin_factory.h>

namespace tesseract::task_composer
{
MinLengthProfile::MinLengthProfile() : Profile(createKey<MinLengthProfile>()) {}
MinLengthProfile::MinLengthProfile(long min_length) : Profile(createKey<MinLengthProfile>()), min_length(min_length) {}
MinLengthProfile::MinLengthProfile(const YAML::Node& config,
                                   const tesseract::common::ProfilePluginFactory& /*plugin_factory*/)
  : MinLengthProfile()
{
  try
  {
    min_length = config["min_length"].as<long>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("MinLengthProfile: Failed to parse yaml config! Details: " + std::string(e.what()));
  }
}

bool MinLengthProfile::operator==(const MinLengthProfile& rhs) const { return (min_length == rhs.min_length); }

bool MinLengthProfile::operator!=(const MinLengthProfile& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::task_composer
