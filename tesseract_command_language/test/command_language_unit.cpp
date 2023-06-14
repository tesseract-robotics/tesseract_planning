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

#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/set_analog_instruction.h>
#include <tesseract_command_language/set_tool_instruction.h>
#include <tesseract_command_language/timer_instruction.h>
#include <tesseract_command_language/wait_instruction.h>

#include "command_language_test_program.hpp"

using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

TEST(TesseractCommandLanguageUnit, WaypointPolyTests)  // NOLINT
{
  {  // Null waypoint and serialization
    WaypointPoly null_wp;
    EXPECT_TRUE(null_wp.isNull());
    EXPECT_FALSE(null_wp.isCartesianWaypoint());
    EXPECT_FALSE(null_wp.isJointWaypoint());
    EXPECT_FALSE(null_wp.isStateWaypoint());

    test_suite::runWaypointSerializationTest(null_wp);
  }

  {  // Equality
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
  }
}

TEST(TesseractCommandLanguageUnit, CartesianWaypointPolyTests)  // NOLINT
{
  {  // Null waypoint and serialization
    CartesianWaypointPoly null_wp;
    EXPECT_TRUE(null_wp.isNull());
    test_suite::runWaypointSerializationTest(null_wp);
  }

  {  // Equality
    CartesianWaypointPoly wp1;
    EXPECT_TRUE(wp1.isNull());

    CartesianWaypointPoly wp2(wp1);  // NOLINT
    EXPECT_TRUE(wp2.isNull());

    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
  }
}

TEST(TesseractCommandLanguageUnit, JointWaypointPolyTests)  // NOLINT
{
  {  // Null waypoint and serialization
    JointWaypointPoly null_wp;
    EXPECT_TRUE(null_wp.isNull());
    test_suite::runWaypointSerializationTest(null_wp);
  }

  {  // Equality
    JointWaypointPoly wp1;
    EXPECT_TRUE(wp1.isNull());

    JointWaypointPoly wp2(wp1);  // NOLINT
    EXPECT_TRUE(wp2.isNull());

    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
  }
}

TEST(TesseractCommandLanguageUnit, StateWaypointPolyTests)  // NOLINT
{
  {  // Null waypoint and serialization
    StateWaypointPoly null_wp;
    EXPECT_TRUE(null_wp.isNull());
    test_suite::runWaypointSerializationTest(null_wp);
  }

  {  // Equality
    StateWaypointPoly wp1;
    EXPECT_TRUE(wp1.isNull());

    StateWaypointPoly wp2(wp1);  // NOLINT
    EXPECT_TRUE(wp2.isNull());

    EXPECT_TRUE(wp1 == wp2);
    EXPECT_TRUE(wp2 == wp1);
    EXPECT_FALSE(wp2 != wp1);
  }
}

TEST(TesseractCommandLanguageUnit, CartesianWaypointTests)  // NOLINT
{
  test_suite::runCartesianWaypointTest<CartesianWaypoint>();
}

TEST(TesseractCommandLanguageUnit, JointWaypointTests)  // NOLINT
{
  test_suite::runJointWaypointTest<JointWaypoint>();
}

TEST(TesseractCommandLanguageUnit, StateWaypointTests)  // NOLINT
{
  test_suite::runStateWaypointTest<StateWaypoint>();
}

TEST(TesseractCommandLanguageUnit, MoveInstructionTests)  // NOLINT
{
  test_suite::runMoveInstructionTest<MoveInstruction>();
}

TEST(TesseractCommandLanguageUnit, SetAnalogInstructionTests)  // NOLINT
{
  using T = SetAnalogInstruction;
  test_suite::runInstructionInterfaceTest<T>();

  {
    const std::string key{ "key" };
    const int index{ 5 };
    const double value{ 50 };
    T instr(key, index, value);
    EXPECT_EQ(instr.getKey(), key);
    EXPECT_EQ(instr.getIndex(), index);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(instr.getValue(), value));

    // Copy
    InstructionPoly poly{ instr };
    InstructionPoly copy(poly);
    EXPECT_EQ(copy.as<T>().getKey(), key);
    EXPECT_EQ(copy.as<T>().getIndex(), index);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(copy.as<T>().getValue(), value));
    EXPECT_TRUE(isSetAnalogInstruction(poly));
    EXPECT_TRUE(isSetAnalogInstruction(copy));
    test_suite::runInstructionSerializationTest(poly);
    test_suite::runInstructionSerializationTest(copy);

    // Equal
    EXPECT_TRUE(poly == copy);
    EXPECT_TRUE(copy == poly);
    EXPECT_FALSE(copy != poly);

    {  // Not Equal
      {
        InstructionPoly ne_poly{ T("ne", index, value) };
        EXPECT_FALSE(poly == ne_poly);
        EXPECT_FALSE(ne_poly == poly);
        EXPECT_TRUE(ne_poly != poly);
      }
      {
        InstructionPoly ne_poly{ T(key, 1, value) };
        EXPECT_FALSE(poly == ne_poly);
        EXPECT_FALSE(ne_poly == poly);
        EXPECT_TRUE(ne_poly != poly);
      }
      {
        InstructionPoly ne_poly{ T(key, index, 20) };
        EXPECT_FALSE(poly == ne_poly);
        EXPECT_FALSE(ne_poly == poly);
        EXPECT_TRUE(ne_poly != poly);
      }
    }
  }
}

TEST(TesseractCommandLanguageUnit, SetToolInstructionTests)  // NOLINT
{
  using T = SetToolInstruction;
  test_suite::runInstructionInterfaceTest<T>();

  {
    const int tool_id{ 5 };
    T instr(tool_id);
    EXPECT_EQ(instr.getTool(), tool_id);

    // Copy
    InstructionPoly poly{ instr };
    InstructionPoly copy(poly);
    EXPECT_EQ(copy.as<T>().getTool(), tool_id);
    EXPECT_TRUE(isSetToolInstruction(poly));
    EXPECT_TRUE(isSetToolInstruction(copy));
    test_suite::runInstructionSerializationTest(poly);
    test_suite::runInstructionSerializationTest(copy);

    // Equal
    EXPECT_TRUE(poly == copy);
    EXPECT_TRUE(copy == poly);
    EXPECT_FALSE(copy != poly);

    {  // Not Equal
      InstructionPoly ne_poly{ T(1) };
      EXPECT_FALSE(poly == ne_poly);
      EXPECT_FALSE(ne_poly == poly);
      EXPECT_TRUE(ne_poly != poly);
    }
  }
}

TEST(TesseractCommandLanguageUnit, TimerInstructionTests)  // NOLINT
{
  using T = TimerInstruction;

  test_suite::runInstructionInterfaceTest<T>();

  {
    const TimerInstructionType type{ TimerInstructionType::DIGITAL_OUTPUT_HIGH };
    const double time{ 50 };
    const int io{ 5 };
    T instr(type, time, io);
    EXPECT_EQ(instr.getTimerType(), type);
    EXPECT_EQ(instr.getTimerIO(), io);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(instr.getTimerTime(), time));

    // Copy
    InstructionPoly poly{ instr };
    InstructionPoly copy(poly);
    EXPECT_EQ(copy.as<T>().getTimerType(), type);
    EXPECT_EQ(copy.as<T>().getTimerIO(), io);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(copy.as<T>().getTimerTime(), time));
    EXPECT_TRUE(isTimerInstruction(poly));
    EXPECT_TRUE(isTimerInstruction(copy));
    test_suite::runInstructionSerializationTest(poly);
    test_suite::runInstructionSerializationTest(copy);

    // Equal
    EXPECT_TRUE(poly == copy);
    EXPECT_TRUE(copy == poly);
    EXPECT_FALSE(copy != poly);

    {  // Not Equal
      {
        InstructionPoly ne_poly{ poly };
        ne_poly.as<T>().setTimerType(TimerInstructionType::DIGITAL_OUTPUT_LOW);
        EXPECT_FALSE(poly == ne_poly);
        EXPECT_FALSE(ne_poly == poly);
        EXPECT_TRUE(ne_poly != poly);
      }
      {
        InstructionPoly ne_poly{ poly };
        ne_poly.as<T>().setTimerIO(1);
        EXPECT_FALSE(poly == ne_poly);
        EXPECT_FALSE(ne_poly == poly);
        EXPECT_TRUE(ne_poly != poly);
      }
      {
        InstructionPoly ne_poly{ poly };
        ne_poly.as<T>().setTimerTime(20);
        EXPECT_FALSE(poly == ne_poly);
        EXPECT_FALSE(ne_poly == poly);
        EXPECT_TRUE(ne_poly != poly);
      }
    }
  }
}

TEST(TesseractCommandLanguageUnit, WaitInstructionTests)  // NOLINT
{
  using T = WaitInstruction;

  test_suite::runInstructionInterfaceTest<T>();

  {
    const WaitInstructionType type{ WaitInstructionType::TIME };
    const double time{ 50 };
    const int io{ 5 };
    T instr(time);
    EXPECT_EQ(instr.getWaitType(), type);
    EXPECT_EQ(instr.getWaitIO(), -1);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(instr.getWaitTime(), time));

    T instr2(WaitInstructionType::DIGITAL_INPUT_HIGH, io);
    EXPECT_EQ(instr2.getWaitType(), WaitInstructionType::DIGITAL_INPUT_HIGH);
    EXPECT_EQ(instr2.getWaitIO(), io);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(instr2.getWaitTime(), 0.0));

    EXPECT_ANY_THROW((T{ type, io }));  // NOLINT

    // Copy
    InstructionPoly poly{ instr };
    InstructionPoly copy(poly);
    EXPECT_EQ(copy.as<T>().getWaitType(), type);
    EXPECT_EQ(copy.as<T>().getWaitIO(), -1);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(copy.as<T>().getWaitTime(), time));
    EXPECT_TRUE(isWaitInstruction(poly));
    EXPECT_TRUE(isWaitInstruction(copy));
    test_suite::runInstructionSerializationTest(poly);
    test_suite::runInstructionSerializationTest(copy);

    // Equal
    EXPECT_TRUE(poly == copy);
    EXPECT_TRUE(copy == poly);
    EXPECT_FALSE(copy != poly);

    // Copy
    InstructionPoly poly2{ instr2 };
    InstructionPoly copy2(poly2);
    EXPECT_EQ(copy2.as<T>().getWaitType(), WaitInstructionType::DIGITAL_INPUT_HIGH);
    EXPECT_EQ(copy2.as<T>().getWaitIO(), io);
    EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(copy2.as<T>().getWaitTime(), 0.0));
    EXPECT_TRUE(isWaitInstruction(poly2));
    EXPECT_TRUE(isWaitInstruction(copy2));
    test_suite::runInstructionSerializationTest(poly2);
    test_suite::runInstructionSerializationTest(copy2);

    // Equal
    EXPECT_TRUE(poly2 == copy2);
    EXPECT_TRUE(copy2 == poly2);
    EXPECT_FALSE(copy2 != poly2);

    {  // Not Equal
      {
        InstructionPoly ne_poly{ poly };
        ne_poly.as<T>().setWaitType(WaitInstructionType::DIGITAL_INPUT_HIGH);
        EXPECT_FALSE(poly == ne_poly);
        EXPECT_FALSE(ne_poly == poly);
        EXPECT_TRUE(ne_poly != poly);
      }
      {
        InstructionPoly ne_poly{ poly };
        ne_poly.as<T>().setWaitIO(1);
        EXPECT_FALSE(poly == ne_poly);
        EXPECT_FALSE(ne_poly == poly);
        EXPECT_TRUE(ne_poly != poly);
      }
      {
        InstructionPoly ne_poly{ poly };
        ne_poly.as<T>().setWaitTime(20);
        EXPECT_FALSE(poly == ne_poly);
        EXPECT_FALSE(ne_poly == poly);
        EXPECT_TRUE(ne_poly != poly);
      }
    }
  }
}

bool notMoveFilter(const InstructionPoly& instruction, const CompositeInstruction& /*composite*/)
{
  return !instruction.isMoveInstruction();
}

TEST(TesseractCommandLanguageUnit, CompositeInstructionTests)  // NOLINT
{
  using T = CompositeInstruction;

  test_suite::runInstructionInterfaceTest<T>(false);

  std::string profile{ "raster_program" };
  ManipulatorInfo manip_info("manipulator", "world", "tool0");
  CompositeInstructionOrder order{ CompositeInstructionOrder::ORDERED };
  T instr = getTestProgram(profile, order, manip_info);
  EXPECT_FALSE(instr.getUUID().is_nil());
  EXPECT_TRUE(instr.getParentUUID().is_nil());
  EXPECT_EQ(instr.getProfile(), profile);
  EXPECT_EQ(instr.getProfileOverrides(), nullptr);
  EXPECT_EQ(instr.getManipulatorInfo(), manip_info);
  EXPECT_EQ(std::as_const(instr).getManipulatorInfo(), manip_info);
  EXPECT_EQ(instr.getOrder(), order);
  EXPECT_NE(instr.size(), instr.getInstructionCount());  // Because of nested composites
  EXPECT_EQ(instr.size(), instr.getInstructions().size());
  EXPECT_EQ(instr.size(), std::as_const(instr).getInstructions().size());
  EXPECT_EQ(instr.size(), 12);
  EXPECT_EQ(instr.getInstructionCount(), 41);
  EXPECT_NE(instr.getFirstInstruction(), nullptr);
  EXPECT_NE(std::as_const(instr).getFirstInstruction(), nullptr);
  EXPECT_EQ(std::as_const(instr).getFirstInstruction(), instr.getFirstInstruction());
  EXPECT_NE(instr.getLastInstruction(), nullptr);
  EXPECT_NE(std::as_const(instr).getLastInstruction(), nullptr);
  EXPECT_EQ(std::as_const(instr).getLastInstruction(), instr.getLastInstruction());
  EXPECT_EQ(instr.getMoveInstructionCount(), 25);
  EXPECT_NE(instr.getFirstMoveInstruction(), nullptr);
  EXPECT_NE(std::as_const(instr).getFirstMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(instr).getFirstMoveInstruction(), instr.getFirstMoveInstruction());
  EXPECT_NE(instr.getLastMoveInstruction(), nullptr);
  EXPECT_NE(std::as_const(instr).getLastMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(instr).getLastMoveInstruction(), instr.getLastMoveInstruction());

  InstructionPoly poly{ instr };
  test_suite::runInstructionSerializationTest(poly);

  T insert_program(instr.begin(), instr.end());
  EXPECT_FALSE(insert_program.getUUID().is_nil());
  EXPECT_TRUE(insert_program.getParentUUID().is_nil());
  EXPECT_EQ(insert_program.getProfile(), DEFAULT_PROFILE_KEY);
  EXPECT_EQ(insert_program.getProfileOverrides(), nullptr);
  EXPECT_TRUE(insert_program.getManipulatorInfo().empty());
  EXPECT_EQ(insert_program.getOrder(), order);
  EXPECT_NE(insert_program.size(), insert_program.getInstructionCount());  // Because of nested composites
  EXPECT_EQ(insert_program.size(), insert_program.getInstructions().size());
  EXPECT_EQ(insert_program.size(), std::as_const(insert_program).getInstructions().size());
  EXPECT_EQ(insert_program.size(), 12);
  EXPECT_EQ(insert_program.getInstructionCount(), 41);
  EXPECT_NE(insert_program.getFirstInstruction(), nullptr);
  EXPECT_NE(std::as_const(insert_program).getFirstInstruction(), nullptr);
  EXPECT_EQ(std::as_const(insert_program).getFirstInstruction(), insert_program.getFirstInstruction());
  EXPECT_NE(insert_program.getLastInstruction(), nullptr);
  EXPECT_NE(std::as_const(insert_program).getLastInstruction(), nullptr);
  EXPECT_EQ(std::as_const(insert_program).getLastInstruction(), insert_program.getLastInstruction());
  EXPECT_EQ(insert_program.getMoveInstructionCount(), 25);
  EXPECT_NE(insert_program.getFirstMoveInstruction(), nullptr);
  EXPECT_NE(std::as_const(insert_program).getFirstMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(insert_program).getFirstMoveInstruction(), insert_program.getFirstMoveInstruction());
  EXPECT_NE(insert_program.getLastMoveInstruction(), nullptr);
  EXPECT_NE(std::as_const(insert_program).getLastMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(insert_program).getLastMoveInstruction(), insert_program.getLastMoveInstruction());
  poly = insert_program;
  test_suite::runInstructionSerializationTest(poly);

  // Not Equal
  EXPECT_FALSE(instr == insert_program);
  EXPECT_FALSE(insert_program == instr);
  EXPECT_TRUE(insert_program != instr);

  T assign_program(profile, order, manip_info);
  assign_program.setInstructions(instr.getInstructions());
  EXPECT_FALSE(assign_program.getUUID().is_nil());
  EXPECT_TRUE(assign_program.getParentUUID().is_nil());
  EXPECT_EQ(assign_program.getProfile(), profile);
  EXPECT_EQ(assign_program.getProfileOverrides(), nullptr);
  EXPECT_EQ(assign_program.getManipulatorInfo(), manip_info);
  EXPECT_EQ(std::as_const(assign_program).getManipulatorInfo(), manip_info);
  EXPECT_EQ(assign_program.getOrder(), order);
  EXPECT_NE(assign_program.size(), assign_program.getInstructionCount());  // Because of nested composites
  EXPECT_EQ(assign_program.size(), assign_program.getInstructions().size());
  EXPECT_EQ(assign_program.size(), std::as_const(assign_program).getInstructions().size());
  EXPECT_EQ(assign_program.size(), 12);
  EXPECT_EQ(assign_program.getInstructionCount(), 41);
  EXPECT_NE(assign_program.getFirstInstruction(), nullptr);
  EXPECT_NE(std::as_const(assign_program).getFirstInstruction(), nullptr);
  EXPECT_EQ(std::as_const(assign_program).getFirstInstruction(), assign_program.getFirstInstruction());
  EXPECT_NE(assign_program.getLastInstruction(), nullptr);
  EXPECT_NE(std::as_const(assign_program).getLastInstruction(), nullptr);
  EXPECT_EQ(std::as_const(assign_program).getLastInstruction(), assign_program.getLastInstruction());
  EXPECT_EQ(assign_program.getMoveInstructionCount(), 25);
  EXPECT_NE(assign_program.getFirstMoveInstruction(), nullptr);
  EXPECT_NE(std::as_const(assign_program).getFirstMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(assign_program).getFirstMoveInstruction(), assign_program.getFirstMoveInstruction());
  EXPECT_NE(assign_program.getLastMoveInstruction(), nullptr);
  EXPECT_NE(std::as_const(assign_program).getLastMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(assign_program).getLastMoveInstruction(), assign_program.getLastMoveInstruction());
  poly = assign_program;
  test_suite::runInstructionSerializationTest(poly);

  // Equal
  EXPECT_TRUE(instr == assign_program);
  EXPECT_TRUE(assign_program == instr);
  EXPECT_FALSE(assign_program != instr);

  T copy_program{ instr };
  EXPECT_FALSE(copy_program.getUUID().is_nil());
  EXPECT_TRUE(copy_program.getParentUUID().is_nil());
  EXPECT_EQ(copy_program.getProfile(), profile);
  EXPECT_EQ(copy_program.getProfileOverrides(), nullptr);
  EXPECT_EQ(copy_program.getManipulatorInfo(), manip_info);
  EXPECT_EQ(std::as_const(copy_program).getManipulatorInfo(), manip_info);
  EXPECT_EQ(copy_program.getOrder(), order);
  EXPECT_NE(copy_program.size(), copy_program.getInstructionCount());  // Because of nested composites
  EXPECT_EQ(copy_program.size(), copy_program.getInstructions().size());
  EXPECT_EQ(copy_program.size(), std::as_const(copy_program).getInstructions().size());
  EXPECT_EQ(copy_program.size(), 12);
  EXPECT_EQ(copy_program.getInstructionCount(), 41);
  EXPECT_NE(copy_program.getFirstInstruction(), nullptr);
  EXPECT_NE(std::as_const(copy_program).getFirstInstruction(), nullptr);
  EXPECT_EQ(std::as_const(copy_program).getFirstInstruction(), copy_program.getFirstInstruction());
  EXPECT_NE(copy_program.getLastInstruction(), nullptr);
  EXPECT_NE(std::as_const(copy_program).getLastInstruction(), nullptr);
  EXPECT_EQ(std::as_const(copy_program).getLastInstruction(), copy_program.getLastInstruction());
  EXPECT_EQ(copy_program.getMoveInstructionCount(), 25);
  EXPECT_NE(copy_program.getFirstMoveInstruction(), nullptr);
  EXPECT_NE(std::as_const(copy_program).getFirstMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(copy_program).getFirstMoveInstruction(), copy_program.getFirstMoveInstruction());
  EXPECT_NE(copy_program.getLastMoveInstruction(), nullptr);
  EXPECT_NE(std::as_const(copy_program).getLastMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(copy_program).getLastMoveInstruction(), copy_program.getLastMoveInstruction());
  poly = copy_program;
  test_suite::runInstructionSerializationTest(poly);

  // Equal
  EXPECT_TRUE(instr == copy_program);
  EXPECT_TRUE(copy_program == instr);
  EXPECT_FALSE(copy_program != instr);

  T empty_program(profile, order, manip_info);
  EXPECT_FALSE(empty_program.getUUID().is_nil());
  EXPECT_TRUE(empty_program.getParentUUID().is_nil());
  EXPECT_EQ(empty_program.getProfile(), profile);
  EXPECT_EQ(empty_program.getProfileOverrides(), nullptr);
  EXPECT_EQ(empty_program.getManipulatorInfo(), manip_info);
  EXPECT_EQ(std::as_const(empty_program).getManipulatorInfo(), manip_info);
  EXPECT_EQ(empty_program.getOrder(), order);
  EXPECT_EQ(empty_program.size(), empty_program.getInstructionCount());
  EXPECT_EQ(empty_program.size(), empty_program.getInstructions().size());
  EXPECT_EQ(empty_program.size(), std::as_const(empty_program).getInstructions().size());
  EXPECT_EQ(empty_program.size(), 0);
  EXPECT_EQ(empty_program.getInstructionCount(), 0);
  EXPECT_EQ(empty_program.getFirstInstruction(), nullptr);
  EXPECT_EQ(std::as_const(empty_program).getFirstInstruction(), nullptr);
  EXPECT_EQ(std::as_const(empty_program).getFirstInstruction(), empty_program.getFirstInstruction());
  EXPECT_EQ(empty_program.getLastInstruction(), nullptr);
  EXPECT_EQ(std::as_const(empty_program).getLastInstruction(), nullptr);
  EXPECT_EQ(std::as_const(empty_program).getLastInstruction(), empty_program.getLastInstruction());
  EXPECT_EQ(empty_program.getMoveInstructionCount(), 0);
  EXPECT_EQ(empty_program.getFirstMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(empty_program).getFirstMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(empty_program).getFirstMoveInstruction(), empty_program.getFirstMoveInstruction());
  EXPECT_EQ(empty_program.getLastMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(empty_program).getLastMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(empty_program).getLastMoveInstruction(), empty_program.getLastMoveInstruction());
  poly = empty_program;
  test_suite::runInstructionSerializationTest(poly);

  // Not Equal
  EXPECT_FALSE(instr == empty_program);
  EXPECT_FALSE(empty_program == instr);
  EXPECT_TRUE(empty_program != instr);

  auto profile_overrides = std::make_shared<ProfileDictionary>();
  T set_program;
  set_program.setProfile(profile);
  set_program.setManipulatorInfo(manip_info);
  set_program.setProfileOverrides(profile_overrides);
  EXPECT_FALSE(set_program.getUUID().is_nil());
  EXPECT_TRUE(set_program.getParentUUID().is_nil());
  EXPECT_EQ(set_program.getProfile(), profile);
  EXPECT_EQ(set_program.getProfileOverrides(), profile_overrides);
  EXPECT_EQ(set_program.getManipulatorInfo(), manip_info);
  EXPECT_EQ(std::as_const(set_program).getManipulatorInfo(), manip_info);
  EXPECT_EQ(set_program.getOrder(), order);
  EXPECT_EQ(set_program.size(), set_program.getInstructionCount());
  EXPECT_EQ(set_program.size(), set_program.getInstructions().size());
  EXPECT_EQ(set_program.size(), std::as_const(set_program).getInstructions().size());
  EXPECT_EQ(set_program.size(), 0);
  EXPECT_EQ(set_program.getInstructionCount(), 0);
  EXPECT_EQ(set_program.getFirstInstruction(), nullptr);
  EXPECT_EQ(std::as_const(set_program).getFirstInstruction(), nullptr);
  EXPECT_EQ(std::as_const(set_program).getFirstInstruction(), set_program.getFirstInstruction());
  EXPECT_EQ(set_program.getLastInstruction(), nullptr);
  EXPECT_EQ(std::as_const(set_program).getLastInstruction(), nullptr);
  EXPECT_EQ(std::as_const(set_program).getLastInstruction(), set_program.getLastInstruction());
  EXPECT_EQ(set_program.getMoveInstructionCount(), 0);
  EXPECT_EQ(set_program.getFirstMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(set_program).getFirstMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(set_program).getFirstMoveInstruction(), set_program.getFirstMoveInstruction());
  EXPECT_EQ(set_program.getLastMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(set_program).getLastMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(set_program).getLastMoveInstruction(), set_program.getLastMoveInstruction());
  poly = set_program;
  test_suite::runInstructionSerializationTest(poly);

  // Not Equal
  EXPECT_FALSE(instr == set_program);
  EXPECT_FALSE(set_program == instr);
  EXPECT_TRUE(set_program != instr);

  // Clear
  T clear_program{ instr };
  clear_program.clear();
  EXPECT_FALSE(clear_program.getUUID().is_nil());
  EXPECT_TRUE(clear_program.getParentUUID().is_nil());
  EXPECT_EQ(clear_program.getProfile(), profile);
  EXPECT_EQ(clear_program.getProfileOverrides(), nullptr);
  EXPECT_EQ(clear_program.getManipulatorInfo(), manip_info);
  EXPECT_EQ(std::as_const(clear_program).getManipulatorInfo(), manip_info);
  EXPECT_EQ(clear_program.getOrder(), order);
  EXPECT_EQ(clear_program.size(), clear_program.getInstructionCount());
  EXPECT_EQ(clear_program.size(), clear_program.getInstructions().size());
  EXPECT_EQ(clear_program.size(), std::as_const(clear_program).getInstructions().size());
  EXPECT_EQ(clear_program.size(), 0);
  EXPECT_EQ(clear_program.getInstructionCount(), 0);
  EXPECT_EQ(clear_program.getFirstInstruction(), nullptr);
  EXPECT_EQ(std::as_const(clear_program).getFirstInstruction(), nullptr);
  EXPECT_EQ(std::as_const(clear_program).getFirstInstruction(), clear_program.getFirstInstruction());
  EXPECT_EQ(clear_program.getLastInstruction(), nullptr);
  EXPECT_EQ(std::as_const(clear_program).getLastInstruction(), nullptr);
  EXPECT_EQ(std::as_const(clear_program).getLastInstruction(), clear_program.getLastInstruction());
  EXPECT_EQ(clear_program.getMoveInstructionCount(), 0);
  EXPECT_EQ(clear_program.getFirstMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(clear_program).getFirstMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(clear_program).getFirstMoveInstruction(), clear_program.getFirstMoveInstruction());
  EXPECT_EQ(clear_program.getLastMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(clear_program).getLastMoveInstruction(), nullptr);
  EXPECT_EQ(std::as_const(clear_program).getLastMoveInstruction(), clear_program.getLastMoveInstruction());
  poly = clear_program;
  test_suite::runInstructionSerializationTest(poly);

  // Not Equal
  EXPECT_FALSE(instr == clear_program);
  EXPECT_FALSE(clear_program == instr);
  EXPECT_TRUE(clear_program != instr);

  // Filters
  EXPECT_EQ(instr.getInstructionCount(moveFilter, false), 0);
  EXPECT_EQ(instr.getFirstInstruction(moveFilter, false), nullptr);
  EXPECT_EQ(std::as_const(instr).getFirstInstruction(moveFilter, false), nullptr);
  EXPECT_EQ(std::as_const(instr).getFirstInstruction(moveFilter, false), instr.getFirstInstruction(moveFilter, false));
  EXPECT_EQ(instr.getLastInstruction(moveFilter, false), nullptr);
  EXPECT_EQ(std::as_const(instr).getLastInstruction(moveFilter, false), nullptr);
  EXPECT_EQ(std::as_const(instr).getLastInstruction(moveFilter, false), instr.getLastInstruction(moveFilter, false));

  EXPECT_EQ(instr.getInstructionCount(notMoveFilter, false), 12);
  EXPECT_NE(instr.getFirstInstruction(notMoveFilter, false), nullptr);
  EXPECT_NE(std::as_const(instr).getFirstInstruction(notMoveFilter, false), nullptr);
  EXPECT_EQ(std::as_const(instr).getFirstInstruction(notMoveFilter, false),
            instr.getFirstInstruction(notMoveFilter, false));
  EXPECT_NE(instr.getLastInstruction(notMoveFilter, false), nullptr);
  EXPECT_NE(std::as_const(instr).getLastInstruction(notMoveFilter, false), nullptr);
  EXPECT_EQ(std::as_const(instr).getLastInstruction(notMoveFilter, false),
            instr.getLastInstruction(notMoveFilter, false));

  // Flatten
  auto mis = instr.flatten(moveFilter);
  T insert_mi_program(profile, order, manip_info);
  for (const auto& mi : mis)
    insert_mi_program.insertMoveInstruction(insert_mi_program.end(), mi.get().as<MoveInstructionPoly>());

  const auto& nmis = insert_mi_program.getInstructions();
  EXPECT_EQ(mis.size(), insert_mi_program.size());
  for (std::size_t i = 0; i < mis.size(); ++i)
  {
    EXPECT_EQ(mis.at(i).get(), nmis.at(i));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
