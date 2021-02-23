/**
 * @file null_waypoint.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_NULL_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_NULL_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tinyxml2.h>
#include <string>
#include <iostream>
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
class NullWaypoint
{
public:
  NullWaypoint() = default;
  NullWaypoint(const tinyxml2::XMLElement& /*xml_element*/) {}

  int getType() const { return 0; }

  void print(const std::string& prefix = "") const { std::cout << prefix << "Null WP"; }

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const
  {
    tinyxml2::XMLElement* xml_waypoint = doc.NewElement("Waypoint");
    xml_waypoint->SetAttribute("type", std::to_string(getType()).c_str());

    tinyxml2::XMLElement* xml_null_waypoint = doc.NewElement("NullWaypoint");
    xml_waypoint->InsertEndChild(xml_null_waypoint);

    return xml_waypoint;
  }

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& /*ar*/, const unsigned int /*version*/)
  {
  }
};
}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_waypoint_type(NullWaypoint)
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_NULL_WAYPOINT_H
