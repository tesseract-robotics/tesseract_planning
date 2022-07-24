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
#include <tesseract_common/serialization.h>
#include <tesseract_common/utils.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>

using namespace tesseract_planning;

TEST(TesseractCommandLanguageWaypointUnit, boostSerialization)  // NOLINT
{
  WaypointPoly null_wp;
  EXPECT_TRUE(null_wp.isNull());
  EXPECT_FALSE(null_wp.isCartesianWaypoint());
  EXPECT_FALSE(null_wp.isJointWaypoint());
  EXPECT_FALSE(null_wp.isStateWaypoint());

  tesseract_common::Serialization::toArchiveFileXML<WaypointPoly>(null_wp, "/tmp/waypoint_null_boost.xml");
  auto nnull_wp = tesseract_common::Serialization::fromArchiveFileXML<WaypointPoly>("/tmp/waypoint_null_boost.xml");

  EXPECT_TRUE(nnull_wp.isNull());
  EXPECT_FALSE(null_wp.isCartesianWaypoint());
  EXPECT_FALSE(null_wp.isJointWaypoint());
  EXPECT_FALSE(null_wp.isStateWaypoint());
}

inline void SerializeDeserializeTest(const WaypointPoly& wp)
{
  tesseract_common::Serialization::toArchiveFileXML<WaypointPoly>(wp,
                                                                  tesseract_common::getTempPath() + "waypoint_unit."
                                                                                                    "xml");
  auto deserialized =
      tesseract_common::Serialization::fromArchiveFileXML<WaypointPoly>(tesseract_common::getTempPath() + "waypoint_"
                                                                                                          "unit.xml");
  EXPECT_TRUE(wp == deserialized);
}

TEST(TesseractCommandLanguageWaypointUnit, equalityOperatorAndSerialization)  // NOLINT
{
  // Equal Null
  {
    WaypointPoly wp1;
    EXPECT_TRUE(wp1.isNull());
    EXPECT_FALSE(wp1.isCartesianWaypoint());
    EXPECT_FALSE(wp1.isJointWaypoint());
    EXPECT_FALSE(wp1.isStateWaypoint());

    WaypointPoly wp2(wp1);  // NOLINT
    EXPECT_TRUE(wp2.isNull());
    EXPECT_FALSE(wp2.isCartesianWaypoint());
    EXPECT_FALSE(wp2.isJointWaypoint());
    EXPECT_FALSE(wp2.isStateWaypoint());

    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }

  // Equal
  {
    WaypointPoly wp1{ CartesianWaypointPoly{ CartesianWaypoint(Eigen::Isometry3d::Identity()) } };
    EXPECT_FALSE(wp1.isNull());
    EXPECT_TRUE(wp1.isCartesianWaypoint());
    EXPECT_FALSE(wp1.isJointWaypoint());
    EXPECT_FALSE(wp1.isStateWaypoint());

    WaypointPoly wp2(wp1);  // NOLINT
    EXPECT_FALSE(wp1.isNull());
    EXPECT_TRUE(wp1.isCartesianWaypoint());
    EXPECT_FALSE(wp1.isJointWaypoint());
    EXPECT_FALSE(wp1.isStateWaypoint());

    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }

  // Equal
  {
    WaypointPoly wp1{ JointWaypointPoly{ JointWaypoint({ "a", "b" }, Eigen::VectorXd::Constant(2, 3)) } };
    EXPECT_FALSE(wp1.isNull());
    EXPECT_FALSE(wp1.isCartesianWaypoint());
    EXPECT_TRUE(wp1.isJointWaypoint());
    EXPECT_FALSE(wp1.isStateWaypoint());

    WaypointPoly wp2(wp1);  // NOLINT
    EXPECT_FALSE(wp1.isNull());
    EXPECT_FALSE(wp1.isCartesianWaypoint());
    EXPECT_TRUE(wp1.isJointWaypoint());
    EXPECT_FALSE(wp1.isStateWaypoint());

    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }

  // Equal
  {
    WaypointPoly wp1{ StateWaypointPoly{ StateWaypoint({ "a", "b" }, Eigen::VectorXd::Constant(2, 3)) } };
    EXPECT_FALSE(wp1.isNull());
    EXPECT_FALSE(wp1.isCartesianWaypoint());
    EXPECT_FALSE(wp1.isJointWaypoint());
    EXPECT_TRUE(wp1.isStateWaypoint());

    WaypointPoly wp2(wp1);  // NOLINT
    EXPECT_FALSE(wp1.isNull());
    EXPECT_FALSE(wp1.isCartesianWaypoint());
    EXPECT_FALSE(wp1.isJointWaypoint());
    EXPECT_TRUE(wp1.isStateWaypoint());

    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
