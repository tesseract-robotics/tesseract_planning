/**
 * @file trajopt_collision_config.cpp
 * @brief TrajOpt collision configuration settings
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>
#include <iostream>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/eigen_serialization.h>

namespace tesseract_planning
{
template <class Archive>
void TrajOptCartesianWaypointConfig::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(enabled);
  ar& BOOST_SERIALIZATION_NVP(use_tolerance_override);
  ar& BOOST_SERIALIZATION_NVP(lower_tolerance);
  ar& BOOST_SERIALIZATION_NVP(upper_tolerance);
  ar& BOOST_SERIALIZATION_NVP(coeff);
}

template <class Archive>
void TrajOptJointWaypointConfig::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(enabled);
  ar& BOOST_SERIALIZATION_NVP(use_tolerance_override);
  ar& BOOST_SERIALIZATION_NVP(lower_tolerance);
  ar& BOOST_SERIALIZATION_NVP(upper_tolerance);
  ar& BOOST_SERIALIZATION_NVP(coeff);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptCartesianWaypointConfig)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptCartesianWaypointConfig)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptJointWaypointConfig)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptJointWaypointConfig)
