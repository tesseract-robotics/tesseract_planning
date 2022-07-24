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
#include <tesseract_command_language/cartesian_waypoint.h>

using namespace tesseract_planning;

TEST(TesseractCommandLanguageCartesianWaypointUnit, isToleranced)  // NOLINT
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  CartesianWaypointPoly cw{ CartesianWaypoint(pose) };
  EXPECT_FALSE(cw.isToleranced());

  cw.setUpperTolerance(Eigen::VectorXd::Constant(3, 5));
  cw.setLowerTolerance(Eigen::VectorXd::Constant(3, -5));
  EXPECT_TRUE(cw.isToleranced());

  cw.setUpperTolerance(Eigen::VectorXd::Constant(3, -5));
  cw.setLowerTolerance(Eigen::VectorXd::Constant(3, -5));
  EXPECT_ANY_THROW(cw.isToleranced());  // NOLINT

  cw.setUpperTolerance(Eigen::VectorXd::Constant(3, 5));
  cw.setLowerTolerance(Eigen::VectorXd::Constant(3, 5));
  EXPECT_ANY_THROW(cw.isToleranced());  // NOLINT

  cw.setUpperTolerance(Eigen::VectorXd::Constant(3, 0));
  cw.setLowerTolerance(Eigen::VectorXd::Constant(3, 0));
  EXPECT_FALSE(cw.isToleranced());

  EXPECT_TRUE(cw.getType() == std::type_index(typeid(CartesianWaypoint)));
  cw.print();
}

TEST(TesseractCommandLanguageCartesianWaypointUnit, boostSerialization)  // NOLINT
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  CartesianWaypoint cw(pose);

  CartesianWaypointPoly wp{ cw };
  tesseract_common::Serialization::toArchiveFileXML<CartesianWaypointPoly>(wp, "/tmp/cartesian_waypoint_boost.xml");

  auto nwp = tesseract_common::Serialization::fromArchiveFileXML<CartesianWaypointPoly>("/tmp/"
                                                                                        "cartesian_waypoint_boost.xml");

  EXPECT_TRUE(cw == nwp.as<CartesianWaypoint>());
}

inline void SerializeDeserializeTest(const CartesianWaypointPoly& wp)
{
  tesseract_common::Serialization::toArchiveFileXML<CartesianWaypointPoly>(wp,
                                                                           tesseract_common::getTempPath() + "cartesian"
                                                                                                             "_"
                                                                                                             "waypoint_"
                                                                                                             "unit."
                                                                                                             "xml");
  auto deserialized =
      tesseract_common::Serialization::fromArchiveFileXML<CartesianWaypointPoly>(tesseract_common::getTempPath() + "car"
                                                                                                                   "tes"
                                                                                                                   "ian"
                                                                                                                   "_"
                                                                                                                   "way"
                                                                                                                   "poi"
                                                                                                                   "nt_"
                                                                                                                   "uni"
                                                                                                                   "t."
                                                                                                                   "xm"
                                                                                                                   "l");
  EXPECT_TRUE(wp == deserialized);
}

TEST(TesseractCommandLanguageCartesianWaypointUnit, equalityOperatorAndSerialization)  // NOLINT
{
  // Equal
  {
    CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
    CartesianWaypointPoly wp2(wp1);  // NOLINT
    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }
  {
    CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
    wp1.getTransform().translate(Eigen::Vector3d(1e6, 0, 0));
    CartesianWaypointPoly wp2(wp1);
    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }
  {
    CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
    wp1.getUpperTolerance().resize(3);
    wp1.getUpperTolerance() << 1, 2, 3;
    wp1.getLowerTolerance().resize(3);
    wp1.getLowerTolerance() << -4, -5, -6;
    CartesianWaypointPoly wp2(wp1);
    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
    SerializeDeserializeTest(wp1);
  }
  // Not equal
  {
    CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
    CartesianWaypointPoly wp2{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
    wp2.getTransform().rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1)));
    EXPECT_FALSE(wp1 == wp2);
    EXPECT_FALSE(wp2 == wp1);
    EXPECT_TRUE(wp2 != wp1);
    SerializeDeserializeTest(wp2);
  }
  {
    CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
    CartesianWaypointPoly wp2{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
    wp2.getTransform().translate(Eigen::Vector3d(1e6, 0, 0));
    EXPECT_FALSE(wp1 == wp2);
    EXPECT_FALSE(wp2 == wp1);
    EXPECT_TRUE(wp2 != wp1);
    SerializeDeserializeTest(wp2);
  }
  {
    CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
    CartesianWaypointPoly wp2(wp1);
    wp2.getUpperTolerance().resize(3);
    wp2.getUpperTolerance() << 1, 2, 3;
    wp2.getLowerTolerance().resize(3);
    wp2.getLowerTolerance() << -4, -5, -6;
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
