/**
 * @file cartesian_waypoint_unit.cpp
 * @brief Contains unit tests for CartesianWaypoint
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date February 15, 2021
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
#include <tesseract_command_language/null_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>

using namespace tesseract_planning;

TEST(TesseractCommandLanguageCartesianWaypointUnit, isToleranced)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  CartesianWaypoint cw(pose);
  EXPECT_FALSE(cw.isToleranced());

  cw.upper_tolerance = Eigen::VectorXd::Constant(3, 5);
  cw.lower_tolerance = Eigen::VectorXd::Constant(3, -5);
  EXPECT_TRUE(cw.isToleranced());

  cw.upper_tolerance = Eigen::VectorXd::Constant(3, -5);
  cw.lower_tolerance = Eigen::VectorXd::Constant(3, -5);
  EXPECT_ANY_THROW(cw.isToleranced());

  cw.upper_tolerance = Eigen::VectorXd::Constant(3, 5);
  cw.lower_tolerance = Eigen::VectorXd::Constant(3, 5);
  EXPECT_ANY_THROW(cw.isToleranced());

  cw.upper_tolerance = Eigen::VectorXd::Constant(3, 0);
  cw.lower_tolerance = Eigen::VectorXd::Constant(3, 0);
  EXPECT_FALSE(cw.isToleranced());
}

TEST(TesseractCommandLanguageCartesianWaypointUnit, boostSerialization)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  CartesianWaypoint cw(pose);

  Waypoint wp{ NullWaypoint() };
  wp = cw;
  Serialization::toArchiveFileXML<Waypoint>(wp, "/tmp/cartesian_waypoint_boost.xml");

  Waypoint nwp = Serialization::fromArchiveFileXML<Waypoint>("/tmp/cartesian_waypoint_boost.xml");

  EXPECT_TRUE(cw == nwp.as<CartesianWaypoint>());
}

inline void SerializeDeserializeTest(const CartesianWaypoint& wp)
{
  Serialization::toArchiveFileXML<Waypoint>(wp, tesseract_common::getTempPath() + "cartesian_waypoint_unit.xml");
  Waypoint deserialized = Serialization::fromArchiveFileXML<Waypoint>(tesseract_common::getTempPath() + "cartesian_"
                                                                                                        "waypoint_unit."
                                                                                                        "xml");
  EXPECT_TRUE(wp == deserialized.as<CartesianWaypoint>());
}

TEST(TesseractCommandLanguageCartesianWaypointUnit, equalityOperatorAndSerialization)
{
  // Equal
  {
    CartesianWaypoint wp1(Eigen::Isometry3d::Identity());
    CartesianWaypoint wp2(wp1);
    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }
  {
    CartesianWaypoint wp1(Eigen::Isometry3d::Identity());
    wp1.waypoint.translate(Eigen::Vector3d(1e6, 0, 0));
    CartesianWaypoint wp2(wp1);
    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }
  {
    CartesianWaypoint wp1(Eigen::Isometry3d::Identity());
    wp1.upper_tolerance.resize(3);
    wp1.upper_tolerance << 1, 2, 3;
    wp1.lower_tolerance.resize(3);
    wp1.lower_tolerance << -4, -5, -6;
    CartesianWaypoint wp2(wp1);
    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }
  // Not equal
  {
    CartesianWaypoint wp1(Eigen::Isometry3d::Identity());
    CartesianWaypoint wp2(Eigen::Isometry3d::Identity());
    wp2.waypoint.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1)));
    EXPECT_FALSE(wp1 == wp2);
    EXPECT_FALSE(wp2 == wp1);
    EXPECT_TRUE(wp2 != wp1);
    SerializeDeserializeTest(wp2);
  }
  {
    CartesianWaypoint wp1(Eigen::Isometry3d::Identity());
    CartesianWaypoint wp2(Eigen::Isometry3d::Identity());
    wp2.waypoint.translate(Eigen::Vector3d(1e6, 0, 0));
    EXPECT_FALSE(wp1 == wp2);
    EXPECT_FALSE(wp2 == wp1);
    EXPECT_TRUE(wp2 != wp1);
    SerializeDeserializeTest(wp2);
  }
  {
    CartesianWaypoint wp1(Eigen::Isometry3d::Identity());
    CartesianWaypoint wp2(wp1);
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
