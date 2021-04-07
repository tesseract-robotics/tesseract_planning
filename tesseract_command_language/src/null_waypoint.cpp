/**
 * @file null_waypoint.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/null_waypoint.h>

namespace tesseract_planning
{
void NullWaypoint::print(const std::string& prefix) const { std::cout << prefix << "Null WP"; }

bool NullWaypoint::operator==(const NullWaypoint& /*rhs*/) const { return true; }
bool NullWaypoint::operator!=(const NullWaypoint& /*rhs*/) const { return false; }

template <class Archive>
void NullWaypoint::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}
}  // namespace tesseract_planning

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
template void tesseract_planning::NullWaypoint::serialize(boost::archive::xml_oarchive& ar, const unsigned int version);
template void tesseract_planning::NullWaypoint::serialize(boost::archive::xml_iarchive& ar, const unsigned int version);

TESSERACT_WAYPOINT_EXPORT_IMPLEMENT(tesseract_planning::NullWaypoint);
