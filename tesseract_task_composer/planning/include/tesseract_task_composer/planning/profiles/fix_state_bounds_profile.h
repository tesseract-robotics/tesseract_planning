/**
 * @file fix_state_bounds_profile.h
 * @brief Profile for process that pushes plan instructions back within joint limits
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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
#ifndef TESSERACT_TASK_COMPOSER_FIX_STATE_BOUNDS_PROFILE_H
#define TESSERACT_TASK_COMPOSER_FIX_STATE_BOUNDS_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <limits>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/profile.h>
#include <tesseract_common/fwd.h>

namespace YAML
{
class Node;
}

namespace tesseract::task_composer
{
struct FixStateBoundsProfile : public tesseract::common::Profile
{
  using Ptr = std::shared_ptr<FixStateBoundsProfile>;
  using ConstPtr = std::shared_ptr<const FixStateBoundsProfile>;

  enum class Settings
  {
    START_ONLY,
    END_ONLY,
    ALL,
    DISABLED
  };

  FixStateBoundsProfile(Settings mode = Settings::ALL);
  FixStateBoundsProfile(const YAML::Node& config, const tesseract::common::ProfilePluginFactory& plugin_factory);

  /** @brief Sets which terms will be corrected  */
  Settings mode;

  /** @brief Maximum amount the process is allowed to correct. If deviation is further than this, it will fail */
  double max_deviation_global = std::numeric_limits<double>::max();

  /** @brief Amount to reduce the upper bounds before clamping limits. Should be > 1 */
  double upper_bounds_reduction{ std::numeric_limits<float>::epsilon() };

  /** @brief Amount to increase the lower bounds before clamping limits. Should be > 1 */
  double lower_bounds_reduction{ std::numeric_limits<float>::epsilon() };

  bool operator==(const FixStateBoundsProfile& rhs) const;
  bool operator!=(const FixStateBoundsProfile& rhs) const;
};
}  // namespace tesseract::task_composer

#endif  // TESSERACT_TASK_COMPOSER_FIX_STATE_BOUNDS_PROFILE_H
