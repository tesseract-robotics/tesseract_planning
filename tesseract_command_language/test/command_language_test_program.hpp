#ifndef TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_TEST_PROGRAM_HPP
#define TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_TEST_PROGRAM_HPP

#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/set_analog_instruction.h>
#include <tesseract_command_language/set_tool_instruction.h>
#include <tesseract_command_language/timer_instruction.h>
#include <tesseract_command_language/wait_instruction.h>

namespace tesseract_planning
{
inline CompositeInstruction getTestProgram(std::string profile,
                                           CompositeInstructionOrder order,
                                           tesseract_common::ManipulatorInfo manipulator_info)
{
  CompositeInstruction program(std::move(profile), order, std::move(manipulator_info));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  StateWaypointPoly wp0{ StateWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
  JointWaypointPoly wp00{ JointWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
  MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "freespace_profile");
  MoveInstruction end_instruction(wp00, MoveInstructionType::FREESPACE, "freespace_profile");
  start_instruction.setDescription("Start Instruction");
  end_instruction.setDescription("End Instruction");

  tesseract_common::JointState seed_state;
  seed_state.joint_names = joint_names;
  seed_state.position = Eigen::VectorXd::Zero(6);

  // Define raster poses
  CartesianWaypointPoly wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  wp1.setSeed(seed_state);
  CartesianWaypointPoly wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.2, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  CartesianWaypointPoly wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.1, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  wp3.setSeed(seed_state);
  CartesianWaypointPoly wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.0, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  CartesianWaypointPoly wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.1, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  wp5.setSeed(seed_state);
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
  from_start.appendMoveInstruction(start_instruction);
  from_start.appendMoveInstruction(plan_f0);
  program.push_back(from_start);

  {
    CompositeInstruction raster_segment;
    raster_segment.setDescription("raster_segment");
    raster_segment.appendMoveInstruction(plan_c0);
    raster_segment.appendMoveInstruction(plan_c1);
    raster_segment.appendMoveInstruction(plan_c2);
    raster_segment.appendMoveInstruction(plan_c3);
    raster_segment.appendMoveInstruction(plan_c4);
    raster_segment.appendMoveInstruction(plan_c5);
    program.push_back(raster_segment);
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
    transitions.push_back(transition_from_start);
    transitions.push_back(transition_from_end);
    program.push_back(transitions);
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
    program.push_back(raster_segment);
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
    transitions.push_back(transition_from_start);
    transitions.push_back(transition_from_end);
    program.push_back(transitions);
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
    program.push_back(raster_segment);
  }

  CompositeInstruction to_end;
  to_end.setDescription("to_end");
  to_end.appendMoveInstruction(end_instruction);
  program.push_back(to_end);

  // Add a wait instruction
  WaitInstruction wait_instruction_time(1.5);
  program.push_back(wait_instruction_time);

  WaitInstruction wait_instruction_io(WaitInstructionType::DIGITAL_INPUT_LOW, 10);
  program.push_back(wait_instruction_io);

  // Add a timer instruction
  TimerInstruction timer_instruction(TimerInstructionType::DIGITAL_OUTPUT_LOW, 3.1, 5);
  program.push_back(timer_instruction);

  // Add a set tool instruction
  SetToolInstruction set_tool_instruction(5);
  program.push_back(set_tool_instruction);

  // Add a set tool instruction
  SetAnalogInstruction set_analog_instruction("R", 0, 1.5);
  program.push_back(set_analog_instruction);

  return program;
}
}  // namespace tesseract_planning
#endif  // TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_TEST_PROGRAM_HPP
