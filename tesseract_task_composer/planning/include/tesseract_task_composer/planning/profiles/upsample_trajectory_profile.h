/**
 * @file upsample_trajectory_profile.h
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
#ifndef TESSERACT_TASK_COMPOSER_UPSAMPLE_TRAJECTORY_PROFILE_H
#define TESSERACT_TASK_COMPOSER_UPSAMPLE_TRAJECTORY_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/profile.h>
#include <tesseract_common/fwd.h>

namespace YAML
{
class Node;
}

namespace tesseract_planning
{
struct UpsampleTrajectoryProfile : public tesseract_common::Profile
{
  using Ptr = std::shared_ptr<UpsampleTrajectoryProfile>;
  using ConstPtr = std::shared_ptr<const UpsampleTrajectoryProfile>;

  UpsampleTrajectoryProfile();
  UpsampleTrajectoryProfile(double longest_valid_segment_length);
  UpsampleTrajectoryProfile(const YAML::Node& config, const tesseract_common::ProfilePluginFactory& plugin_factory);

  double longest_valid_segment_length{ 0.1 };

  bool operator==(const UpsampleTrajectoryProfile& rhs) const;
  bool operator!=(const UpsampleTrajectoryProfile& rhs) const;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_UPSAMPLE_TRAJECTORY_PROFILE_H
