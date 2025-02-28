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
  runWaypointInterfaceTest(JointWaypointPoly(T()));

  {  // WaypointPoly Interface Test
    const std::string name{ "tesseract_planning::test_suite::WaypointPoly" };
    JointWaypointPoly wp{ T() };
    EXPECT_FALSE(wp.isNull());
    EXPECT_NE(name, wp.getName());
    wp.setName(name);
    EXPECT_EQ(name, wp.getName());
    EXPECT_NO_THROW(wp.print());         // NOLINT
    EXPECT_NO_THROW(wp.print("test_"));  // NOLINT
    EXPECT_TRUE(wp.getType() == std::type_index(typeid(T)));
    WaypointPoly base = wp;
    EXPECT_FALSE(base.isCartesianWaypoint());
    EXPECT_TRUE(base.isJointWaypoint());
    EXPECT_FALSE(base.isStateWaypoint());
  }

  {  // Test construction
    {
      JointWaypointPoly wp{ T() };
      EXPECT_TRUE(wp.getNames().empty());
      EXPECT_TRUE(wp.getPosition().rows() == 0);
      EXPECT_TRUE(std::as_const(wp).getUpperTolerance().rows() == 0);
      EXPECT_TRUE(std::as_const(wp).getLowerTolerance().rows() == 0);
      EXPECT_FALSE(wp.isConstrained());
    }
    {
      std::vector<std::string> names{ "j1", "j2", "j3" };
      Eigen::VectorXd positions = Eigen::VectorXd::Constant(3, 0.0);
      JointWaypointPoly wp{ T(names, positions) };
      EXPECT_EQ(wp.getNames(), names);
      EXPECT_TRUE(wp.getPosition().isApprox(positions));
      EXPECT_TRUE(std::as_const(wp).getUpperTolerance().rows() == 0);
      EXPECT_TRUE(std::as_const(wp).getLowerTolerance().rows() == 0);
      EXPECT_TRUE(wp.isConstrained());
    }
    {
      std::vector<std::string> names{ "j1", "j2", "j3" };
      Eigen::VectorXd positions = Eigen::VectorXd::Constant(3, 0.0);
      Eigen::VectorXd lower_tol = Eigen::VectorXd::Constant(3, -5);
      Eigen::VectorXd uppert_tol = Eigen::VectorXd::Constant(3, 5);
      JointWaypointPoly wp{ T(names, positions, lower_tol, uppert_tol) };
      EXPECT_EQ(wp.getNames(), names);
      EXPECT_TRUE(wp.getPosition().isApprox(positions));
      EXPECT_TRUE(wp.getLowerTolerance().isApprox(lower_tol));
      EXPECT_TRUE(wp.getUpperTolerance().isApprox(uppert_tol));
      EXPECT_TRUE(wp.isConstrained());
    }
    {
      std::vector<std::string> names{ "j1", "j2" };
      Eigen::VectorXd positions = Eigen::VectorXd::Constant(3, 0.0);
      EXPECT_ANY_THROW(JointWaypointPoly{ T(names, positions) });  // NOLINT
    }
    {
      std::vector<std::string> names{ "j1", "j2", "j3" };
      Eigen::VectorXd positions = Eigen::VectorXd::Constant(2, 0.0);
      EXPECT_ANY_THROW(JointWaypointPoly{ T(names, positions) });  // NOLINT
    }
    {
      std::vector<std::string> names{ "j1", "j2", "j3" };
      Eigen::VectorXd positions = Eigen::VectorXd::Constant(3, 0.0);
      Eigen::VectorXd lower_tol = Eigen::VectorXd::Constant(2, -5);
      Eigen::VectorXd uppert_tol = Eigen::VectorXd::Constant(3, 5);
      EXPECT_ANY_THROW(JointWaypointPoly{ T(names, positions, lower_tol, uppert_tol) });  // NOLINT
    }
    {
      std::vector<std::string> names{ "j1", "j2", "j3" };
      Eigen::VectorXd positions = Eigen::VectorXd::Constant(3, 0.0);
      Eigen::VectorXd lower_tol = Eigen::VectorXd::Constant(3, -5);
      Eigen::VectorXd uppert_tol = Eigen::VectorXd::Constant(2, 5);
      EXPECT_ANY_THROW(JointWaypointPoly{ T(names, positions, lower_tol, uppert_tol) });  // NOLINT
    }
  }  // namespace tesseract_planning::test_suite

  {  // Test is constrained
    JointWaypointPoly wp{ T() };
    EXPECT_FALSE(wp.isConstrained());
    wp.setIsConstrained(true);
    EXPECT_TRUE(wp.isConstrained());
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
      EXPECT_TRUE(std::as_const(wp).getNames() == names);
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
      EXPECT_TRUE(std::as_const(wp).getPosition().isApprox(positions));
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
    {
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0.0, 0.0, 0.0 }) };
      const JointWaypointPoly wp2(wp1);  // NOLINT
      EXPECT_TRUE(wp1.isConstrained());
      EXPECT_TRUE(wp2.isConstrained());
      EXPECT_TRUE(wp1 == wp2);
      EXPECT_TRUE(wp2 == wp1);
      EXPECT_FALSE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
    }
    {
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, -1e6, 1e6 }) };
      JointWaypointPoly wp2(wp1);  // NOLINT
      EXPECT_TRUE(wp1.isConstrained());
      EXPECT_TRUE(wp2.isConstrained());
      EXPECT_TRUE(wp1 == wp2);
      EXPECT_TRUE(wp2 == wp1);
      EXPECT_FALSE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
    }
    {
      std::vector<std::string> names{ "j1", "j2", "j3" };
      Eigen::VectorXd positions, upper_tol, lower_tol;
      positions.resize(3);
      upper_tol.resize(3);
      lower_tol.resize(3);
      positions << 0, -1e6, 1e6;
      upper_tol << 1, 2, 3;
      lower_tol << -4, -5, -6;

      JointWaypointPoly wp1{ T(names, positions, lower_tol, upper_tol) };
      JointWaypointPoly wp2(wp1);
      EXPECT_TRUE(wp1.isConstrained());
      EXPECT_TRUE(wp2.isConstrained());
      EXPECT_TRUE(wp1 == wp2);
      EXPECT_TRUE(wp2 == wp1);
      EXPECT_FALSE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
    }
    // Not equal
    {
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
      JointWaypointPoly wp2{ T(std::vector<std::string>({ "j1" }), Eigen::VectorXd::Zero(1)) };
      EXPECT_TRUE(wp1.isConstrained());
      EXPECT_TRUE(wp2.isConstrained());
      EXPECT_FALSE(wp1 == wp2);
      EXPECT_FALSE(wp2 == wp1);
      EXPECT_TRUE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
      runWaypointSerializationTest(wp2);
    }
    {
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
      JointWaypointPoly wp2{ T({ "j1", "j2", "j4" }, { 0, 0, 0 }) };
      EXPECT_TRUE(wp1.isConstrained());
      EXPECT_TRUE(wp2.isConstrained());
      EXPECT_FALSE(wp1 == wp2);
      EXPECT_FALSE(wp2 == wp1);
      EXPECT_TRUE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
      runWaypointSerializationTest(wp2);
    }
    {
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
      JointWaypointPoly wp2{ T({ "j1", "j2", "j3" }, { 0.001, 0, 0 }) };
      EXPECT_TRUE(wp1.isConstrained());
      EXPECT_TRUE(wp2.isConstrained());
      EXPECT_FALSE(wp1 == wp2);
      EXPECT_FALSE(wp2 == wp1);
      EXPECT_TRUE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
      runWaypointSerializationTest(wp2);
    }
    {
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, 0, 0 }, { 1, 2, 3 }, { -4, -5, -6 }) };
      JointWaypointPoly wp2(wp1);
      EXPECT_TRUE(wp1.isConstrained());
      EXPECT_TRUE(wp2.isConstrained());
      EXPECT_TRUE(wp1 == wp2);
      EXPECT_TRUE(wp2 == wp1);
      EXPECT_FALSE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
      runWaypointSerializationTest(wp2);
    }
    {
      Eigen::VectorXd upper_tol, lower_tol;
      upper_tol.resize(3);
      lower_tol.resize(3);
      upper_tol << 1, 2, 3;
      lower_tol << -4, -5, -6;
      JointWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
      JointWaypointPoly wp2(wp1);
      wp1.getUpperTolerance() = upper_tol;
      wp1.getLowerTolerance() = lower_tol;
      EXPECT_TRUE(wp1.isConstrained());
      EXPECT_TRUE(wp2.isConstrained());
      EXPECT_FALSE(wp1 == wp2);
      EXPECT_FALSE(wp2 == wp1);
      EXPECT_TRUE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
      runWaypointSerializationTest(wp2);
    }
  }
}
}  // namespace tesseract_planning::test_suite
#endif  // TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_POLY_UNIT_HPP
