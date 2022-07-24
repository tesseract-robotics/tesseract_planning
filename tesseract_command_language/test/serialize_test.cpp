/**
 * @file serialize_test.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date August 20, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <tesseract_common/any.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/wait_instruction.h>
#include <tesseract_command_language/timer_instruction.h>
#include <tesseract_command_language/set_analog_instruction.h>
#include <tesseract_command_language/set_tool_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_common/utils.h>

using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

CompositeInstruction getProgram()
{
  CompositeInstruction program(
      "raster_program", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "world", "tool0"));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  StateWaypointPoly wp0{ StateWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
  MoveInstruction start_instruction(wp0, MoveInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Define raster poses
  CartesianWaypointPoly wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  CartesianWaypointPoly wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.2, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  CartesianWaypointPoly wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.1, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  CartesianWaypointPoly wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.0, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  CartesianWaypointPoly wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.1, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  CartesianWaypointPoly wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.2, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  JointWaypointPoly wp7 = JointWaypoint(joint_names, Eigen::VectorXd::Ones(6));

  // Define raster move instruction
  MoveInstruction plan_c0(wp2, MoveInstructionType::LINEAR, "RASTER");
  MoveInstruction plan_c1(wp3, MoveInstructionType::LINEAR, "RASTER", "RASTER");
  MoveInstruction plan_c2(wp4, MoveInstructionType::LINEAR, "RASTER");
  MoveInstruction plan_c3(wp5, MoveInstructionType::LINEAR, "RASTER", "RASTER");
  MoveInstruction plan_c4(wp6, MoveInstructionType::LINEAR, "RASTER");
  MoveInstruction plan_c5(wp7, MoveInstructionType::LINEAR, "RASTER", "RASTER");

  MoveInstruction plan_f0(wp1, MoveInstructionType::FREESPACE, "freespace_profile");
  plan_f0.setDescription("from_start_plan");
  CompositeInstruction from_start;
  from_start.setDescription("from_start");
  from_start.appendMoveInstruction(plan_f0);
  program.appendInstruction(from_start);

  {
    CompositeInstruction raster_segment;
    raster_segment.setDescription("raster_segment");
    raster_segment.appendMoveInstruction(plan_c0);
    raster_segment.appendMoveInstruction(plan_c1);
    raster_segment.appendMoveInstruction(plan_c2);
    raster_segment.appendMoveInstruction(plan_c3);
    raster_segment.appendMoveInstruction(plan_c4);
    raster_segment.appendMoveInstruction(plan_c5);
    program.appendInstruction(raster_segment);
  }

  {
    MoveInstruction plan_f1(wp1, MoveInstructionType::FREESPACE, "freespace_profile");
    plan_f1.setDescription("transition_from_end_plan");
    CompositeInstruction transition_from_end;
    transition_from_end.setDescription("transition_from_end");
    transition_from_end.appendMoveInstruction(plan_f1);
    CompositeInstruction transition_from_start;
    transition_from_start.setDescription("transition_from_start");
    transition_from_start.appendMoveInstruction(plan_f1);

    CompositeInstruction transitions(DEFAULT_PROFILE_KEY, CompositeInstructionOrder::UNORDERED);
    transitions.setDescription("transitions");
    transitions.appendInstruction(transition_from_start);
    transitions.appendInstruction(transition_from_end);
    program.appendInstruction(transitions);
  }

  {
    CompositeInstruction raster_segment;
    raster_segment.setDescription("raster_segment");
    raster_segment.appendMoveInstruction(plan_c0);
    raster_segment.appendMoveInstruction(plan_c1);
    raster_segment.appendMoveInstruction(plan_c2);
    raster_segment.appendMoveInstruction(plan_c3);
    raster_segment.appendMoveInstruction(plan_c4);
    raster_segment.appendMoveInstruction(plan_c5);
    program.appendInstruction(raster_segment);
  }

  {
    MoveInstruction plan_f1(wp1, MoveInstructionType::FREESPACE, "freespace_profile");
    plan_f1.setDescription("transition_from_end_plan");
    CompositeInstruction transition_from_end;
    transition_from_end.setDescription("transition_from_end");
    transition_from_end.appendMoveInstruction(plan_f1);
    CompositeInstruction transition_from_start;
    transition_from_start.setDescription("transition_from_start");
    transition_from_start.appendMoveInstruction(plan_f1);

    CompositeInstruction transitions(DEFAULT_PROFILE_KEY, CompositeInstructionOrder::UNORDERED);
    transitions.setDescription("transitions");
    transitions.appendInstruction(transition_from_start);
    transitions.appendInstruction(transition_from_end);
    program.appendInstruction(transitions);
  }

  {
    CompositeInstruction raster_segment;
    raster_segment.setDescription("raster_segment");
    raster_segment.appendMoveInstruction(plan_c0);
    raster_segment.appendMoveInstruction(plan_c1);
    raster_segment.appendMoveInstruction(plan_c2);
    raster_segment.appendMoveInstruction(plan_c3);
    raster_segment.appendMoveInstruction(plan_c4);
    raster_segment.appendMoveInstruction(plan_c5);
    program.appendInstruction(raster_segment);
  }

  MoveInstruction plan_f2(wp1, MoveInstructionType::FREESPACE, "freespace_profile");
  plan_f2.setDescription("to_end_plan");
  CompositeInstruction to_end;
  to_end.setDescription("to_end");
  to_end.appendMoveInstruction(plan_f2);
  program.appendInstruction(to_end);

  // Add a wait instruction
  WaitInstruction wait_instruction_time(1.5);
  program.appendInstruction(wait_instruction_time);

  WaitInstruction wait_instruction_io(WaitInstructionType::DIGITAL_INPUT_LOW, 10);
  program.appendInstruction(wait_instruction_io);

  // Add a timer instruction
  TimerInstruction timer_instruction(TimerInstructionType::DIGITAL_OUTPUT_LOW, 3.1, 5);
  program.appendInstruction(timer_instruction);

  // Add a set tool instruction
  SetToolInstruction set_tool_instruction(5);
  program.appendInstruction(set_tool_instruction);

  // Add a set tool instruction
  SetAnalogInstruction set_analog_instruction("R", 0, 1.5);
  program.appendInstruction(set_analog_instruction);

  return program;
}

TEST(TesseractCommandLanguageSerializeUnit, serializationCompositeInstruction)  // NOLINT
{
  InstructionPoly program = getProgram();
  {  // Archive program to file
    std::string file_path = tesseract_common::getTempPath() + "composite_instruction_boost.xml";
    EXPECT_TRUE(tesseract_common::Serialization::toArchiveFileXML<InstructionPoly>(program, file_path));
    auto nprogram = tesseract_common::Serialization::fromArchiveFileXML<InstructionPoly>(file_path);
    EXPECT_TRUE(program == nprogram);
  }

  {  // Archive program to string
    std::string program_string =
        tesseract_common::Serialization::toArchiveStringXML<InstructionPoly>(program, "program");
    EXPECT_FALSE(program_string.empty());
    auto nprogram = tesseract_common::Serialization::fromArchiveStringXML<InstructionPoly>(program_string);
    EXPECT_TRUE(program == nprogram);
  }
}

TEST(TesseractCommandLanguageSerializeUnit, TypeErasureInTypeErasure)  // NOLINT
{
  tesseract_planning::InstructionPoly instruction{ SetToolInstruction(5) };
  tesseract_common::Any any_type;
  any_type = instruction;
  EXPECT_EQ(any_type.getType(), std::type_index(typeid(tesseract_planning::InstructionPoly)));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
