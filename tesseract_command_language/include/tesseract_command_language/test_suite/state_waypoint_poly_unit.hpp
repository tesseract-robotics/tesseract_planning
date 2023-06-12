/**
 * @file state_waypoint_poly_unit.hpp
 * @brief Collection of unit tests for StateWaypointPoly
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
#ifndef TESSERACT_COMMAND_LANGUAGE_STATE_WAYPOINT_POLY_UNIT_HPP
#define TESSERACT_COMMAND_LANGUAGE_STATE_WAYPOINT_POLY_UNIT_HPP

#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/test_suite/waypoint_poly_unit.hpp>

namespace tesseract_planning::test_suite
{
template <typename T>
void runStateWaypointTest()
{
  runWaypointInterfaceTest<T>();

  {  // WaypointPoly Interface Test
    StateWaypointPoly wp{ T() };
    EXPECT_FALSE(wp.isNull());
    EXPECT_TRUE(wp.getType() == std::type_index(typeid(T)));
    WaypointPoly base = wp;
    EXPECT_FALSE(base.isCartesianWaypoint());
    EXPECT_FALSE(base.isJointWaypoint());
    EXPECT_TRUE(base.isStateWaypoint());
  }

  {  // Test default construction
    StateWaypointPoly wp{ T() };
    EXPECT_TRUE(wp.getNames().empty());
    EXPECT_TRUE(wp.getPosition().rows() == 0);
    EXPECT_TRUE(wp.getVelocity().rows() == 0);
    EXPECT_TRUE(wp.getAcceleration().rows() == 0);
    EXPECT_TRUE(wp.getEffort().rows() == 0);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(wp.getTime(), 0.0));
  }

  {  // Set/Get Names
    const std::vector<std::string> names{ "j1", "j2", "j3" };
    {  // Test set
      StateWaypointPoly wp{ T() };
      wp.setNames(names);
      EXPECT_TRUE(wp.getNames() == names);
    }

    {  // Test assigning
      StateWaypointPoly wp{ T() };
      wp.getNames() = names;
      EXPECT_TRUE(wp.getNames() == names);
    }
  }

  {  // Set/Get Positions
    Eigen::VectorXd data;
    data.resize(3);
    data << 1.0, 2.0, 3.0;

    {  // Test set
      StateWaypointPoly wp{ T() };
      wp.setPosition(data);
      EXPECT_TRUE(wp.getPosition().isApprox(data));
    }

    {  // Test assigning
      StateWaypointPoly wp{ T() };
      wp.getPosition() = data;
      EXPECT_TRUE(wp.getPosition().isApprox(data));
    }
  }

  {  // Set/Get Velocity
    Eigen::VectorXd data;
    data.resize(3);
    data << 1.0, 2.0, 3.0;

    {  // Test set
      StateWaypointPoly wp{ T() };
      wp.setVelocity(data);
      EXPECT_TRUE(wp.getVelocity().isApprox(data));
    }

    {  // Test assigning
      StateWaypointPoly wp{ T() };
      wp.getVelocity() = data;
      EXPECT_TRUE(wp.getVelocity().isApprox(data));
    }
  }

  {  // Set/Get Acceleration
    Eigen::VectorXd data;
    data.resize(3);
    data << 1.0, 2.0, 3.0;

    {  // Test set
      StateWaypointPoly wp{ T() };
      wp.setAcceleration(data);
      EXPECT_TRUE(wp.getAcceleration().isApprox(data));
    }

    {  // Test assigning
      StateWaypointPoly wp{ T() };
      wp.getAcceleration() = data;
      EXPECT_TRUE(wp.getAcceleration().isApprox(data));
    }
  }

  {  // Set/Get Effort
    Eigen::VectorXd data;
    data.resize(3);
    data << 1.0, 2.0, 3.0;

    {  // Test set
      StateWaypointPoly wp{ T() };
      wp.setEffort(data);
      EXPECT_TRUE(wp.getEffort().isApprox(data));
    }

    {  // Test assigning
      StateWaypointPoly wp{ T() };
      wp.getEffort() = data;
      EXPECT_TRUE(wp.getEffort().isApprox(data));
    }
  }

  {  // Set/Get Time
    double data{ 3 };
    StateWaypointPoly wp{ T() };
    wp.setTime(data);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(wp.getTime(), data));
  }

  {  // Test Equality and Serialization
    // Equal
    {
      StateWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0.0, 0.0, 0.0 }) };
      const StateWaypointPoly wp2(wp1);  // NOLINT
      EXPECT_TRUE(wp1 == wp2);
      EXPECT_TRUE(wp2 == wp1);
      EXPECT_FALSE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
    }
    {
      StateWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, -1e6, 1e6 }) };
      StateWaypointPoly wp2(wp1);  // NOLINT
      EXPECT_TRUE(wp1 == wp2);
      EXPECT_TRUE(wp2 == wp1);
      EXPECT_FALSE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
    }
    {
      StateWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, -1e6, 1e6 }) };
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
      runWaypointSerializationTest(wp1);
    }
    // Not equal
    {
      StateWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
      StateWaypointPoly wp2{ T(std::vector<std::string>({ "j1" }), Eigen::VectorXd::Zero(1)) };
      EXPECT_FALSE(wp1 == wp2);
      EXPECT_FALSE(wp2 == wp1);
      EXPECT_TRUE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
      runWaypointSerializationTest(wp2);
    }
    {
      StateWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
      StateWaypointPoly wp2{ T({ "j1", "j2", "j4" }, { 0, 0, 0 }) };
      EXPECT_FALSE(wp1 == wp2);
      EXPECT_FALSE(wp2 == wp1);
      EXPECT_TRUE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
      runWaypointSerializationTest(wp2);
    }
    {
      StateWaypointPoly wp1{ T({ "j1", "j2", "j3" }, { 0, 0, 0 }) };
      StateWaypointPoly wp2{ T({ "j1", "j2", "j3" }, { 0.001, 0, 0 }) };
      EXPECT_FALSE(wp1 == wp2);
      EXPECT_FALSE(wp2 == wp1);
      EXPECT_TRUE(wp2 != wp1);
      runWaypointSerializationTest(wp1);
      runWaypointSerializationTest(wp2);
    }
  }
}
}  // namespace tesseract_planning::test_suite

#endif  // TESSERACT_COMMAND_LANGUAGE_STATE_WAYPOINT_POLY_UNIT_HPP
