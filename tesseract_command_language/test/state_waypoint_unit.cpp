/**
 * @file state_waypoint_unit.cpp
 * @brief Contains unit tests for StateWaypoint
 *
 * @author Levi Armstrong
 * @date January 28, 2021
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/serialization.h>
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/state_waypoint.h>

using namespace tesseract_planning;

TEST(TesseractCommandLanguageStateWaypointUnit, boostSerialization)  // NOLINT
{
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3" };
  Eigen::VectorXd joint_values = Eigen::VectorXd::Constant(3, 1);

  StateWaypointPoly jw{ StateWaypoint(joint_names, joint_values) };

  WaypointPoly wp = jw;
  tesseract_common::Serialization::toArchiveFileXML<WaypointPoly>(wp, "/tmp/state_waypoint_boost.xml");

  auto nwp = tesseract_common::Serialization::fromArchiveFileXML<WaypointPoly>("/tmp/state_waypoint_boost.xml");

  EXPECT_TRUE(jw == nwp.as<StateWaypointPoly>());
}

inline void SerializeDeserializeTest(const StateWaypointPoly& wp)
{
  tesseract_common::Serialization::toArchiveFileXML<WaypointPoly>(wp,
                                                                  tesseract_common::getTempPath() + "state_waypoint_"
                                                                                                    "unit."
                                                                                                    "xml");
  auto deserialized =
      tesseract_common::Serialization::fromArchiveFileXML<WaypointPoly>(tesseract_common::getTempPath() + "state_"
                                                                                                          "waypoint_"
                                                                                                          "unit."
                                                                                                          "xml");
  EXPECT_TRUE(wp == deserialized.as<StateWaypointPoly>());
}

TEST(TesseractCommandLanguageStateWaypointUnit, equalityOperatorAndSerialization)  // NOLINT
{
  // Equal
  {
    StateWaypointPoly wp1{ StateWaypoint({ "j1", "j2", "j3" }, { 0.0, 0.0, 0.0 }) };
    const StateWaypointPoly wp2(wp1);  // NOLINT
    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }
  {
    StateWaypointPoly wp1{ StateWaypoint({ "j1", "j2", "j3" }, { 0, -1e6, 1e6 }) };
    StateWaypointPoly wp2(wp1);  // NOLINT
    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }
  {
    StateWaypointPoly wp1{ StateWaypoint({ "j1", "j2", "j3" }, { 0, -1e6, 1e6 }) };
    wp1.getVelocity().resize(3);
    wp1.getVelocity() << 1, 2, 3;
    wp1.getAcceleration().resize(3);
    wp1.getAcceleration() << -4, -5, -6;
    wp1.getEffort().resize(3);
    wp1.getEffort() << 4, 5, 6;
    wp1.setTime(5);
    StateWaypointPoly wp2(wp1);
    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }
  // Not equal
  {
    StateWaypointPoly wp1{ StateWaypoint({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
    StateWaypointPoly wp2{ StateWaypoint(std::vector<std::string>({ "j1" }), Eigen::VectorXd::Zero(1)) };
    EXPECT_FALSE(wp1 == wp2);
    EXPECT_FALSE(wp2 == wp1);
    EXPECT_TRUE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
    SerializeDeserializeTest(wp2);
  }
  {
    StateWaypointPoly wp1{ StateWaypoint({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
    StateWaypointPoly wp2{ StateWaypoint({ "j1", "j2", "j4" }, { 0, 0, 0 }) };
    EXPECT_FALSE(wp1 == wp2);
    EXPECT_FALSE(wp2 == wp1);
    EXPECT_TRUE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
    SerializeDeserializeTest(wp2);
  }
  {
    StateWaypointPoly wp1{ StateWaypoint({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
    StateWaypointPoly wp2{ StateWaypoint({ "j1", "j2", "j3" }, { 0.001, 0, 0 }) };
    EXPECT_FALSE(wp1 == wp2);
    EXPECT_FALSE(wp2 == wp1);
    EXPECT_TRUE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
    SerializeDeserializeTest(wp2);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
