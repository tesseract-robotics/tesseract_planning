/**
 * @file cartesian_waypoint_poly_unit.hpp
 * @brief Collection of unit tests for CartesianWaypointPoly
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
#ifndef TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_POLY_UNIT_HPP
#define TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_POLY_UNIT_HPP

#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/test_suite/waypoint_poly_unit.hpp>

namespace tesseract_planning::test_suite
{
template <typename T>
void runCartesianWaypointTest()
{
  runWaypointInterfaceTest<T>();

  {  // Cartesian WaypointPoly Interface Test
    const std::string name{ "tesseract_planning::test_suite::WaypointPoly" };
    CartesianWaypointPoly wp{ T() };
    EXPECT_FALSE(wp.isNull());
    EXPECT_NE(name, wp.getName());
    wp.setName(name);
    EXPECT_EQ(name, wp.getName());
    EXPECT_NO_THROW(wp.print());         // NOLINT
    EXPECT_NO_THROW(wp.print("test_"));  // NOLINT
    EXPECT_TRUE(wp.getType() == std::type_index(typeid(T)));

    WaypointPoly base = wp;
    EXPECT_TRUE(base.isCartesianWaypoint());
    EXPECT_FALSE(base.isJointWaypoint());
    EXPECT_FALSE(base.isStateWaypoint());
  }

  {  // Test default construction
    CartesianWaypointPoly wp{ T() };
    EXPECT_TRUE(wp.getTransform().isApprox(Eigen::Isometry3d::Identity()));
    EXPECT_TRUE(std::as_const(wp).getUpperTolerance().rows() == 0);
    EXPECT_TRUE(std::as_const(wp).getLowerTolerance().rows() == 0);
    EXPECT_FALSE(wp.hasSeed());
    EXPECT_FALSE(wp.isToleranced());
  }

  {   // Set/Get Transform
    { // Test construction providing pose
      Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  pose.translation() = Eigen::Vector3d(1, 2, 3);
  CartesianWaypointPoly wp{ T(pose) };
  EXPECT_TRUE(wp.getTransform().isApprox(pose));
  EXPECT_FALSE(wp.isToleranced());
}

{  // Test construction providing pose
  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  pose.translation() = Eigen::Vector3d(1, 2, 3);
  CartesianWaypointPoly wp{ T(pose, Eigen::VectorXd::Constant(3, -5), Eigen::VectorXd::Constant(3, 5)) };
  EXPECT_TRUE(wp.getTransform().isApprox(pose));
  EXPECT_TRUE(wp.getLowerTolerance().isApprox(Eigen::VectorXd::Constant(3, -5)));
  EXPECT_TRUE(wp.getUpperTolerance().isApprox(Eigen::VectorXd::Constant(3, 5)));
  EXPECT_TRUE(wp.isToleranced());
}

{  // Test set pose
  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  pose.translation() = Eigen::Vector3d(1, 2, 3);
  CartesianWaypointPoly wp{ T() };
  wp.setTransform(pose);
  EXPECT_TRUE(wp.getTransform().isApprox(pose));
  EXPECT_FALSE(wp.isToleranced());
}

{  // Test assigning pose
  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  pose.translation() = Eigen::Vector3d(1, 2, 3);
  CartesianWaypointPoly wp{ T() };
  wp.getTransform() = pose;
  EXPECT_TRUE(std::as_const(wp).getTransform().isApprox(pose));
  EXPECT_FALSE(wp.isToleranced());
}
}  // namespace tesseract_planning::test_suite

{  // Test Set Tolerances
  CartesianWaypointPoly wp{ T() };
  EXPECT_FALSE(wp.isToleranced());

  wp.setUpperTolerance(Eigen::VectorXd::Constant(3, 5));
  wp.setLowerTolerance(Eigen::VectorXd::Constant(3, -5));
  EXPECT_TRUE(wp.isToleranced());

  wp.setUpperTolerance(Eigen::VectorXd::Constant(3, -5));
  wp.setLowerTolerance(Eigen::VectorXd::Constant(3, -5));
  EXPECT_ANY_THROW(wp.isToleranced());  // NOLINT

  wp.setUpperTolerance(Eigen::VectorXd::Constant(3, 5));
  wp.setLowerTolerance(Eigen::VectorXd::Constant(3, 5));
  EXPECT_ANY_THROW(wp.isToleranced());  // NOLINT

  wp.setUpperTolerance(Eigen::VectorXd::Constant(3, 0));
  wp.setLowerTolerance(Eigen::VectorXd::Constant(3, 0));
  EXPECT_FALSE(wp.isToleranced());
}

{ // Test Equality and Serialization
  { CartesianWaypointPoly wp1{ T(Eigen::Isometry3d::Identity()) };
CartesianWaypointPoly wp2(wp1);  // NOLINT
EXPECT_TRUE(wp1 == wp2);
EXPECT_TRUE(wp2 == wp1);
EXPECT_FALSE(wp2 != wp1);
runWaypointSerializationTest(wp1);
}
{
  CartesianWaypointPoly wp1{ T(Eigen::Isometry3d::Identity()) };
  wp1.getTransform().translate(Eigen::Vector3d(1e6, 0, 0));
  CartesianWaypointPoly wp2(wp1);
  EXPECT_TRUE(wp1 == wp2);
  EXPECT_TRUE(wp2 == wp1);
  EXPECT_FALSE(wp2 != wp1);
  runWaypointSerializationTest(wp1);
}
{
  CartesianWaypointPoly wp1{ T(Eigen::Isometry3d::Identity()) };
  wp1.getUpperTolerance().resize(3);
  wp1.getUpperTolerance() << 1, 2, 3;
  wp1.getLowerTolerance().resize(3);
  wp1.getLowerTolerance() << -4, -5, -6;
  CartesianWaypointPoly wp2(wp1);
  EXPECT_TRUE(wp1 == wp2);
  EXPECT_TRUE(wp2 == wp1);
  EXPECT_FALSE(wp2 != wp1);
  runWaypointSerializationTest(wp1);
}
// Not equal
{
  CartesianWaypointPoly wp1{ T(Eigen::Isometry3d::Identity()) };
  CartesianWaypointPoly wp2{ T(Eigen::Isometry3d::Identity()) };
  wp2.getTransform().rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1)));
  EXPECT_FALSE(wp1 == wp2);
  EXPECT_FALSE(wp2 == wp1);
  EXPECT_TRUE(wp2 != wp1);
  runWaypointSerializationTest(wp2);
}
{
  CartesianWaypointPoly wp1{ T(Eigen::Isometry3d::Identity()) };
  CartesianWaypointPoly wp2{ T(Eigen::Isometry3d::Identity()) };
  wp2.getTransform().translate(Eigen::Vector3d(1e6, 0, 0));
  EXPECT_FALSE(wp1 == wp2);
  EXPECT_FALSE(wp2 == wp1);
  EXPECT_TRUE(wp2 != wp1);
  runWaypointSerializationTest(wp2);
}
{
  CartesianWaypointPoly wp1{ T(Eigen::Isometry3d::Identity()) };
  CartesianWaypointPoly wp2(wp1);
  wp2.getUpperTolerance().resize(3);
  wp2.getUpperTolerance() << 1, 2, 3;
  wp2.getLowerTolerance().resize(3);
  wp2.getLowerTolerance() << -4, -5, -6;
  EXPECT_FALSE(wp1 == wp2);
  EXPECT_FALSE(wp2 == wp1);
  EXPECT_TRUE(wp2 != wp1);
  runWaypointSerializationTest(wp2);
}
}

{  // Set/Get Seed
  tesseract_common::JointState seed_state;
  seed_state.joint_names = { "joint_1", "joint_2", "joint_3" };
  seed_state.position.resize(3);
  seed_state.position << .01, .02, .03;
  seed_state.velocity.resize(3);
  seed_state.velocity << .1, .2, .3;
  seed_state.acceleration.resize(3);
  seed_state.acceleration << 1, 2, 3;

  {  // Test default construction pose
    CartesianWaypointPoly wp{ T() };
    EXPECT_FALSE(wp.hasSeed());
    EXPECT_FALSE(std::as_const(wp).getSeed() == seed_state);
  }

  {  // Test assigning pose
    CartesianWaypointPoly wp{ T() };
    EXPECT_FALSE(wp.hasSeed());
    wp.setSeed(seed_state);
    EXPECT_TRUE(wp.hasSeed());
    EXPECT_TRUE(std::as_const(wp).getSeed() == seed_state);
  }

  {  // Test assigning pose
    CartesianWaypointPoly wp{ T() };
    EXPECT_FALSE(wp.hasSeed());
    wp.getSeed() = seed_state;
    EXPECT_TRUE(wp.hasSeed());
    EXPECT_TRUE(std::as_const(wp).getSeed() == seed_state);
  }

  {  // Test clear seed
    CartesianWaypointPoly wp{ T() };
    EXPECT_FALSE(wp.hasSeed());
    wp.getSeed() = seed_state;
    EXPECT_TRUE(wp.hasSeed());
    wp.clearSeed();
    EXPECT_FALSE(wp.hasSeed());
  }
}
}
}
#endif  // TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_POLY_UNIT_HPP
