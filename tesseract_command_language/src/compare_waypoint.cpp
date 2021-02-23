/**
 * @file compare_waypoint.cpp
 * @brief This contains the comparison operators for Waypoint by recovering the type and comparing
 *
 * @author Levi Armstrong
 * @date February 24, 2021
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_command_language/compare_waypoint.h>

#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/waypoint_type.h>

#include <tesseract_command_language/core/null_waypoint.h>

namespace tesseract_planning
{
bool operator==(const Waypoint& lhs, const Waypoint& rhs)
{
  if (lhs.getType() == rhs.getType())
  {
    if (lhs.getType() == static_cast<int>(WaypointType::NULL_WAYPOINT))
      return true;
    else if (lhs.getType() == static_cast<int>(WaypointType::JOINT_WAYPOINT))
      return ((*lhs.cast_const<JointWaypoint>()) == (*rhs.cast_const<JointWaypoint>()));
    else if (lhs.getType() == static_cast<int>(WaypointType::CARTESIAN_WAYPOINT))
      return ((*lhs.cast_const<CartesianWaypoint>()) == (*rhs.cast_const<CartesianWaypoint>()));
    else if (lhs.getType() == static_cast<int>(WaypointType::STATE_WAYPOINT))
      return ((*lhs.cast_const<StateWaypoint>()) == (*rhs.cast_const<StateWaypoint>()));
    else
      return false;
  }

  return false;
}

bool operator!=(const Waypoint& lhs, const Waypoint& rhs) { return !operator==(lhs, rhs); }

}  // namespace tesseract_planning
