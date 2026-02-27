/**
 * @file trajopt_collision_config.cpp
 * @brief TrajOpt collision configuration settings
 *
 * @author Tyler Marr
 * @date November 2, 2023
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

#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>
#include <tesseract/common/utils.h>

namespace tesseract::motion_planners
{
bool TrajOptCartesianWaypointConfig::operator==(const TrajOptCartesianWaypointConfig& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (enabled == rhs.enabled);
  equal &= (use_tolerance_override == rhs.use_tolerance_override);
  equal &= tesseract::common::almostEqualRelativeAndAbs(lower_tolerance, rhs.lower_tolerance, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(upper_tolerance, rhs.upper_tolerance, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(coeff, rhs.coeff, max_diff);
  return equal;
}

bool TrajOptCartesianWaypointConfig::operator!=(const TrajOptCartesianWaypointConfig& rhs) const
{
  return !operator==(rhs);
}

bool TrajOptJointWaypointConfig::operator==(const TrajOptJointWaypointConfig& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (enabled == rhs.enabled);
  equal &= (use_tolerance_override == rhs.use_tolerance_override);
  equal &= tesseract::common::almostEqualRelativeAndAbs(lower_tolerance, rhs.lower_tolerance, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(upper_tolerance, rhs.upper_tolerance, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(coeff, rhs.coeff, max_diff);
  return equal;
}

bool TrajOptJointWaypointConfig::operator!=(const TrajOptJointWaypointConfig& rhs) const { return !operator==(rhs); }
}  // namespace tesseract::motion_planners
