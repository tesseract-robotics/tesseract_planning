/**
 * @file command_language_unit.cpp
 * @brief Contains unit tests for this package
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

#include <tesseract_command_language/test_suite/cartesian_waypoint_poly_unit.hpp>
#include <tesseract_command_language/test_suite/joint_waypoint_poly_unit.hpp>
#include <tesseract_command_language/test_suite/state_waypoint_poly_unit.hpp>
#include <tesseract_command_language/test_suite/move_instruction_poly_unit.hpp>

#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>

#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>

TEST(TesseractCommandLanguageUnit, WaypointPolyTests)  // NOLINT
{
  {  // Null waypoint and serialization
    tesseract_planning::WaypointPoly null_wp;
    EXPECT_TRUE(null_wp.isNull());
    EXPECT_FALSE(null_wp.isCartesianWaypoint());
    EXPECT_FALSE(null_wp.isJointWaypoint());
    EXPECT_FALSE(null_wp.isStateWaypoint());

    tesseract_planning::test_suite::runWaypointSerializationTest(null_wp);
  }

  {  // Equality
    tesseract_planning::WaypointPoly wp1;
    EXPECT_TRUE(wp1.isNull());
    EXPECT_FALSE(wp1.isCartesianWaypoint());
    EXPECT_FALSE(wp1.isJointWaypoint());
    EXPECT_FALSE(wp1.isStateWaypoint());

    tesseract_planning::WaypointPoly wp2(wp1);  // NOLINT
    EXPECT_TRUE(wp2.isNull());
    EXPECT_FALSE(wp2.isCartesianWaypoint());
    EXPECT_FALSE(wp2.isJointWaypoint());
    EXPECT_FALSE(wp2.isStateWaypoint());

    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
  }
}

TEST(TesseractCommandLanguageUnit, CartesianWaypointPolyTests)  // NOLINT
{
  {  // Null waypoint and serialization
    tesseract_planning::CartesianWaypointPoly null_wp;
    EXPECT_TRUE(null_wp.isNull());
    tesseract_planning::test_suite::runWaypointSerializationTest(null_wp);
  }

  {  // Equality
    tesseract_planning::CartesianWaypointPoly wp1;
    EXPECT_TRUE(wp1.isNull());

    tesseract_planning::CartesianWaypointPoly wp2(wp1);  // NOLINT
    EXPECT_TRUE(wp2.isNull());

    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
  }
}

TEST(TesseractCommandLanguageUnit, JointWaypointPolyTests)  // NOLINT
{
  {  // Null waypoint and serialization
    tesseract_planning::JointWaypointPoly null_wp;
    EXPECT_TRUE(null_wp.isNull());
    tesseract_planning::test_suite::runWaypointSerializationTest(null_wp);
  }

  {  // Equality
    tesseract_planning::JointWaypointPoly wp1;
    EXPECT_TRUE(wp1.isNull());

    tesseract_planning::JointWaypointPoly wp2(wp1);  // NOLINT
    EXPECT_TRUE(wp2.isNull());

    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
  }
}

TEST(TesseractCommandLanguageUnit, StateWaypointPolyTests)  // NOLINT
{
  {  // Null waypoint and serialization
    tesseract_planning::StateWaypointPoly null_wp;
    EXPECT_TRUE(null_wp.isNull());
    tesseract_planning::test_suite::runWaypointSerializationTest(null_wp);
  }

  {  // Equality
    tesseract_planning::StateWaypointPoly wp1;
    EXPECT_TRUE(wp1.isNull());

    tesseract_planning::StateWaypointPoly wp2(wp1);  // NOLINT
    EXPECT_TRUE(wp2.isNull());

    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
  }
}

TEST(TesseractCommandLanguageUnit, CartesianWaypointTests)  // NOLINT
{
  tesseract_planning::test_suite::runCartesianWaypointTest<tesseract_planning::CartesianWaypoint>();
}

TEST(TesseractCommandLanguageUnit, JointWaypointTests)  // NOLINT
{
  tesseract_planning::test_suite::runJointWaypointTest<tesseract_planning::JointWaypoint>();
}

TEST(TesseractCommandLanguageUnit, StateWaypointTests)  // NOLINT
{
  tesseract_planning::test_suite::runStateWaypointTest<tesseract_planning::StateWaypoint>();
}

TEST(TesseractCommandLanguageUnit, MoveInstructionTests)  // NOLINT
{
  tesseract_planning::test_suite::runMoveInstructionTest<tesseract_planning::MoveInstruction>();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
