/**
 * @file trajopt_ifopt_waypoint_config.cpp
 * @brief TrajOpt Ifopt Waypoint configuration settings
 *
 * @author Tyler Marr
 * @date November 2, 2023
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

#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_waypoint_config.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
bool TrajOptIfoptCartesianWaypointConfig::operator==(const TrajOptIfoptCartesianWaypointConfig& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (enabled == rhs.enabled);
  equal &= (use_tolerance_override == rhs.use_tolerance_override);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lower_tolerance, rhs.lower_tolerance, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(upper_tolerance, rhs.upper_tolerance, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(coeff, rhs.coeff, max_diff);
  return equal;
}

bool TrajOptIfoptCartesianWaypointConfig::operator!=(const TrajOptIfoptCartesianWaypointConfig& rhs) const
{
  return !operator==(rhs);
}

bool TrajOptIfoptJointWaypointConfig::operator==(const TrajOptIfoptJointWaypointConfig& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (enabled == rhs.enabled);
  equal &= (use_tolerance_override == rhs.use_tolerance_override);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lower_tolerance, rhs.lower_tolerance, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(upper_tolerance, rhs.upper_tolerance, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(coeff, rhs.coeff, max_diff);
  return equal;
}

bool TrajOptIfoptJointWaypointConfig::operator!=(const TrajOptIfoptJointWaypointConfig& rhs) const
{
  return !operator==(rhs);
}
}  // namespace tesseract_planning
