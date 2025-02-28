/**
 * @file waypoint_poly_unit.hpp
 * @brief Collection of unit tests for WaypointPoly
 *
 * @author Levi Armstrong
 * @date June 12, 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Levi Armstrong
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
#ifndef TESSERACT_COMMAND_LANGUAGE_WAYPOINT_POLY_UNIT_HPP
#define TESSERACT_COMMAND_LANGUAGE_WAYPOINT_POLY_UNIT_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning::test_suite
{
void runWaypointInterfaceTest(WaypointPoly waypoint)
{
  const std::string name{ "tesseract_planning::test_suite::WaypointPoly" };
  EXPECT_NE(name, waypoint.getName());
  waypoint.setName(name);
  EXPECT_EQ(name, waypoint.getName());
  EXPECT_NO_THROW(waypoint.print());         // NOLINT
  EXPECT_NO_THROW(waypoint.print("test_"));  // NOLINT
}

/**
 * @brief This will test the serialization of any waypoint implementation
 * @param waypoint The waypoint to test serialization
 */
inline void runWaypointSerializationTest(const WaypointPoly& waypoint)
{
  const std::string filepath = tesseract_common::getTempPath() + "waypoint_poly_boost.xml";
  tesseract_common::Serialization::toArchiveFileXML<WaypointPoly>(waypoint, filepath);
  auto nwp = tesseract_common::Serialization::fromArchiveFileXML<WaypointPoly>(filepath);
  EXPECT_TRUE(waypoint == nwp);
}
}  // namespace tesseract_planning::test_suite

#endif  // TESSERACT_COMMAND_LANGUAGE_WAYPOINT_POLY_UNIT_HPP
