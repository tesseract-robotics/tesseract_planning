/**
 * @file joint_waypoint_poly_unit.hpp
 * @brief Collection of unit tests for JointWaypointPoly
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
#ifndef TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_POLY_UNIT_HPP
#define TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_POLY_UNIT_HPP

#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/test_suite/waypoint_poly_unit.hpp>

namespace tesseract_planning::test_suite
{
template <typename T>
void runJointWaypointTest()
{
  runWaypointInterfaceTest<T>();

  {  // WaypointPoly Interface Test
    JointWaypointPoly wp{ T() };
    EXPECT_FALSE(wp.isNull());
    EXPECT_TRUE(wp.getType() == std::type_index(typeid(T)));
    WaypointPoly base = wp;
    EXPECT_FALSE(base.isCartesianWaypoint());
    EXPECT_TRUE(base.isJointWaypoint());
    EXPECT_FALSE(base.isStateWaypoint());
  }

  {  // Test default construction
    JointWaypointPoly wp{ T() };
    EXPECT_TRUE(wp.getNames().empty());
    EXPECT_TRUE(wp.getPosition().rows() == 0);
    EXPECT_TRUE(wp.getUpperTolerance().rows() == 0);
    EXPECT_TRUE(wp.getLowerTolerance().rows() == 0);
    EXPECT_TRUE(wp.isConstrained());
  }

  {  // Test is constrained
    JointWaypointPoly wp{ T() };
    EXPECT_TRUE(wp.isConstrained());
    wp.setIsConstrained(false);
    EXPECT_FALSE(wp.isConstrained());
  }

  {  // Set/Get Names
    const std::vector<std::string> names{ "j1", "j2", "j3" };
    {  // Test set
      JointWaypointPoly wp{ T() };
      wp.setNames(names);
      EXPECT_TRUE(wp.getNames() == names);
    }

    {  // Test assigning
      JointWaypointPoly wp{ T() };
      wp.getNames() = names;
      EXPECT_TRUE(wp.getNames() == names);
    }
  }

  {  // Set/Get Positions
    Eigen::VectorXd positions;
    positions.resize(3);
    positions << 1.0, 2.0, 3.0;

    {  // Test set
      JointWaypointPoly wp{ T() };
      wp.setPosition(positions);
      EXPECT_TRUE(wp.getPosition().isApprox(positions));
    }

    {  // Test assigning
      JointWaypointPoly wp{ T() };
      wp.getPosition() = positions;
      EXPECT_TRUE(wp.getPosition().isApprox(positions));
    }
  }

  {  // Test Set Tolerances
    JointWaypointPoly wp{ T() };
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

  {  // Test Equality and Serialization
    // Equal
    {
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0.0, 0.0, 0.0 }) };
      const JointWaypointPoly wp2(wp1);  // NOLINT
      EXPECT_TRUE(wp1 == wp2);
      EXPECT_TRUE(wp2 == wp1);
      EXPECT_FALSE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
    }
    {
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, -1e6, 1e6 }) };
      JointWaypointPoly wp2(wp1);  // NOLINT
      EXPECT_TRUE(wp1 == wp2);
      EXPECT_TRUE(wp2 == wp1);
      EXPECT_FALSE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
    }
    {
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, -1e6, 1e6 }) };
      wp1.getUpperTolerance().resize(3);
      wp1.getUpperTolerance() << 1, 2, 3;
      wp1.getLowerTolerance().resize(3);
      wp1.getLowerTolerance() << -4, -5, -6;
      JointWaypointPoly wp2(wp1);
      EXPECT_TRUE(wp1 == wp2);
      EXPECT_TRUE(wp2 == wp1);
      EXPECT_FALSE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
    }
    // Not equal
    {
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
      JointWaypointPoly wp2{ T(std::vector<std::string>({ "j1" }), Eigen::VectorXd::Zero(1)) };
      EXPECT_FALSE(wp1 == wp2);
      EXPECT_FALSE(wp2 == wp1);
      EXPECT_TRUE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
      runWaypointSerializationTest(wp2);
    }
    {
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
      JointWaypointPoly wp2{ T({ "j1", "j2", "j4" }, { 0, 0, 0 }) };
      EXPECT_FALSE(wp1 == wp2);
      EXPECT_FALSE(wp2 == wp1);
      EXPECT_TRUE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
      runWaypointSerializationTest(wp2);
    }
    {
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
      JointWaypointPoly wp2{ T({ "j1", "j2", "j3" }, { 0.001, 0, 0 }) };
      EXPECT_FALSE(wp1 == wp2);
      EXPECT_FALSE(wp2 == wp1);
      EXPECT_TRUE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
      runWaypointSerializationTest(wp2);
    }
    {
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
      JointWaypointPoly wp2(wp1);
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
}
}  // namespace tesseract_planning::test_suite
#endif  // TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_POLY_UNIT_HPP
