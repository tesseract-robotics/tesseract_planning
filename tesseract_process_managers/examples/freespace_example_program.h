#ifndef TESSERACT_PROCESS_MANAGER_EXAMPLE_PROGRAM_H
#define TESSERACT_PROCESS_MANAGER_EXAMPLE_PROGRAM_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
using tesseract_common::ManipulatorInfo;

inline CompositeInstruction freespaceExampleProgramIIWA(
    const Eigen::Isometry3d& goal = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.2, 0.2, 1.0),
    const std::string& composite_profile = DEFAULT_PROFILE_KEY,
    const std::string& freespace_profile = DEFAULT_PROFILE_KEY)
{
  CompositeInstruction program(
      composite_profile, CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool0"));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                           "joint_a5", "joint_a6", "joint_a7" };
  StateWaypointPoly wp1{ StateWaypoint(joint_names, Eigen::VectorXd::Zero(7)) };
  MoveInstruction start_instruction(wp1, MoveInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Define target pose
  CartesianWaypointPoly wp2{ CartesianWaypoint(goal) };
  MoveInstruction plan_f0(wp2, MoveInstructionType::FREESPACE, freespace_profile);
  plan_f0.setDescription("freespace_motion");
  program.appendMoveInstruction(plan_f0);

  JointWaypointPoly wp3{ JointWaypoint(joint_names, Eigen::VectorXd::Zero(7)) };
  MoveInstruction plan_f1(wp3, MoveInstructionType::FREESPACE);
  program.appendMoveInstruction(plan_f1);

  return program;
}

inline CompositeInstruction freespaceExampleProgramABB(
    const Eigen::Isometry3d& goal = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.2, 0.2, 1.0),
    const std::string& composite_profile = DEFAULT_PROFILE_KEY,
    const std::string& freespace_profile = DEFAULT_PROFILE_KEY)
{
  CompositeInstruction program(
      composite_profile, CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool0"));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  StateWaypointPoly wp1{ StateWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
  MoveInstruction start_instruction(wp1, MoveInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Define target pose
  CartesianWaypointPoly wp2{ CartesianWaypoint(goal) };
  MoveInstruction plan_f0(wp2, MoveInstructionType::FREESPACE, freespace_profile);
  plan_f0.setDescription("freespace_motion");
  program.appendMoveInstruction(plan_f0);

  JointWaypointPoly wp3{ JointWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
  MoveInstruction plan_f1(wp3, MoveInstructionType::FREESPACE);
  program.appendMoveInstruction(plan_f1);

  return program;
}

}  // namespace tesseract_planning

#endif
