/**
 * @file joint_waypoint_unit.cpp
 * @brief Contains unit tests for JointWaypoint
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
#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/serialization.h>
#include <tesseract_command_language/joint_waypoint.h>

using namespace tesseract_planning;

TEST(TesseractCommandLanguageJointWaypointUnit, isToleranced)  // NOLINT
{
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3" };
  Eigen::VectorXd joint_values = Eigen::VectorXd::Constant(3, 1);

  JointWaypoint jw(joint_names, joint_values);
  EXPECT_FALSE(jw.isToleranced());

  jw.upper_tolerance = Eigen::VectorXd::Constant(3, 5);
  jw.lower_tolerance = Eigen::VectorXd::Constant(3, -5);
  EXPECT_TRUE(jw.isToleranced());

  jw.upper_tolerance = Eigen::VectorXd::Constant(3, -5);
  jw.lower_tolerance = Eigen::VectorXd::Constant(3, -5);
  EXPECT_ANY_THROW(jw.isToleranced());  // NOLINT

  jw.upper_tolerance = Eigen::VectorXd::Constant(3, 5);
  jw.lower_tolerance = Eigen::VectorXd::Constant(3, 5);
  EXPECT_ANY_THROW(jw.isToleranced());  // NOLINT

  jw.upper_tolerance = Eigen::VectorXd::Constant(3, 0);
  jw.lower_tolerance = Eigen::VectorXd::Constant(3, 0);
  EXPECT_FALSE(jw.isToleranced());  // NOLINT
}

TEST(TesseractCommandLanguageJointWaypointUnit, boostSerialization)  // NOLINT
{
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3" };
  Eigen::VectorXd joint_values = Eigen::VectorXd::Constant(3, 1);

  JointWaypoint jw(joint_names, joint_values);

  Waypoint wp = jw;
  Serialization::toArchiveFileXML<Waypoint>(wp, "/tmp/joint_waypoint_boost.xml");

  auto nwp = Serialization::fromArchiveFileXML<Waypoint>("/tmp/joint_waypoint_boost.xml");

  EXPECT_TRUE(jw == nwp.as<JointWaypoint>());
}

inline void SerializeDeserializeTest(const JointWaypoint& wp)
{
  Serialization::toArchiveFileXML<Waypoint>(wp, tesseract_common::getTempPath() + "joint_waypoint_unit.xml");
  auto deserialized = Serialization::fromArchiveFileXML<Waypoint>(tesseract_common::getTempPath() + "joint_"
                                                                                                    "waypoint_unit."
                                                                                                    "xml");
  EXPECT_TRUE(wp == deserialized.as<JointWaypoint>());
}

TEST(TesseractCommandLanguageJointWaypointUnit, equalityOperatorAndSerialization)  // NOLINT
{
  // Equal
  {
    JointWaypoint wp1({ "j1", "j2", "j3" }, { 0, 0, 0 });
    const JointWaypoint wp2(wp1);  // NOLINT
    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }
  {
    JointWaypoint wp1({ "j1", "j2", "j3" }, { 0, -1e6, 1e6 });
    JointWaypoint wp2(wp1);  // NOLINT
    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }
  {
    JointWaypoint wp1({ "j1", "j2", "j3" }, { 0, -1e6, 1e6 });
    wp1.upper_tolerance.resize(3);
    wp1.upper_tolerance << 1, 2, 3;
    wp1.lower_tolerance.resize(3);
    wp1.lower_tolerance << -4, -5, -6;
    JointWaypoint wp2(wp1);
    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }
  // Not equal
  {
    JointWaypoint wp1({ "j1", "j2", "j3" }, { 0, 0, 0 });
    JointWaypoint wp2({ "j1" }, { 0 });
    EXPECT_FALSE(wp1 == wp2);
    EXPECT_FALSE(wp2 == wp1);
    EXPECT_TRUE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
    SerializeDeserializeTest(wp2);
  }
  {
    JointWaypoint wp1({ "j1", "j2", "j3" }, { 0, 0, 0 });
    JointWaypoint wp2({ "j1", "j2", "j4" }, { 0, 0, 0 });
    EXPECT_FALSE(wp1 == wp2);
    EXPECT_FALSE(wp2 == wp1);
    EXPECT_TRUE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
    SerializeDeserializeTest(wp2);
  }
  {
    JointWaypoint wp1({ "j1", "j2", "j3" }, { 0, 0, 0 });
    JointWaypoint wp2({ "j1", "j2", "j3" }, { 0.001, 0, 0 });
    EXPECT_FALSE(wp1 == wp2);
    EXPECT_FALSE(wp2 == wp1);
    EXPECT_TRUE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
    SerializeDeserializeTest(wp2);
  }
  {
    JointWaypoint wp1({ "j1", "j2", "j3" }, { 0, 0, 0 });
    JointWaypoint wp2(wp1);
    wp2.upper_tolerance.resize(3);
    wp2.upper_tolerance << 1, 2, 3;
    wp2.lower_tolerance.resize(3);
    wp2.lower_tolerance << -4, -5, -6;
    EXPECT_FALSE(wp1 == wp2);
    EXPECT_FALSE(wp2 == wp1);
    EXPECT_TRUE(wp2 != wp1);
    SerializeDeserializeTest(wp2);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
