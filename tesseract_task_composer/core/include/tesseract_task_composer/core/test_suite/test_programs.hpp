/**
 * @file test_programs.h
 *
 * @author Levi Armstrong
 * @date June 26, 2023
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
#ifndef TESSERACT_TASK_COMPOSER_TEST_PROGRAMS_HPP
#define TESSERACT_TASK_COMPOSER_TEST_PROGRAMS_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning::test_suite
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
  MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, freespace_profile);
  start_instruction.setDescription("Start Instruction");

  // Define target pose
  CartesianWaypointPoly wp2{ CartesianWaypoint(goal) };
  MoveInstruction plan_f0(wp2, MoveInstructionType::FREESPACE, freespace_profile);
  plan_f0.setDescription("freespace_motion");
  program.appendMoveInstruction(start_instruction);
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
  MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, freespace_profile);
  start_instruction.setDescription("Start Instruction");

  // Define target pose
  CartesianWaypointPoly wp2{ CartesianWaypoint(goal) };
  MoveInstruction plan_f0(wp2, MoveInstructionType::FREESPACE, freespace_profile);
  plan_f0.setDescription("freespace_motion");
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f0);

  JointWaypointPoly wp3{ JointWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
  MoveInstruction plan_f1(wp3, MoveInstructionType::FREESPACE);
  program.appendMoveInstruction(plan_f1);

  return program;
}

inline CompositeInstruction
jointInterpolatedExampleSolutionIIWA(bool use_joint_waypoint = false,
                                     const std::string& composite_profile = DEFAULT_PROFILE_KEY,
                                     const std::string& freespace_profile = DEFAULT_PROFILE_KEY)
{
  CompositeInstruction program(
      composite_profile, CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool0"));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                           "joint_a5", "joint_a6", "joint_a7" };
  Eigen::VectorXd start_state = Eigen::VectorXd::Zero(7);
  start_state(0) = -M_PI_4;
  Eigen::VectorXd end_state = Eigen::VectorXd::Zero(7);
  end_state(0) = M_PI_4;

  if (use_joint_waypoint)
  {
    JointWaypointPoly wp1{ JointWaypoint(joint_names, start_state) };
    MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, freespace_profile);
    start_instruction.setDescription("Start Instruction");

    JointWaypointPoly wp2{ JointWaypoint(joint_names, end_state) };
    MoveInstruction end_instruction(wp2, MoveInstructionType::FREESPACE);
    end_instruction.setDescription("End Instruction");

    program.appendMoveInstruction(start_instruction);
    program.appendMoveInstruction(end_instruction);
  }
  else
  {
    StateWaypointPoly wp1{ StateWaypoint(joint_names, start_state) };
    MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, freespace_profile);
    start_instruction.setDescription("Start Instruction");

    StateWaypointPoly wp2{ StateWaypoint(joint_names, end_state) };
    MoveInstruction end_instruction(wp2, MoveInstructionType::FREESPACE);
    end_instruction.setDescription("End Instruction");

    program.appendMoveInstruction(start_instruction);
    program.appendMoveInstruction(end_instruction);
  }

  return program;
}

inline CompositeInstruction
jointInterpolateExampleProgramABB(bool use_joint_waypoint = false,
                                  const std::string& composite_profile = DEFAULT_PROFILE_KEY,
                                  const std::string& freespace_profile = DEFAULT_PROFILE_KEY)
{
  CompositeInstruction program(
      composite_profile, CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool0"));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  Eigen::VectorXd start_state = Eigen::VectorXd::Zero(6);
  start_state(0) = -M_PI_4;
  Eigen::VectorXd end_state = Eigen::VectorXd::Zero(6);
  end_state(0) = M_PI_4;

  if (use_joint_waypoint)
  {
    JointWaypointPoly wp1{ JointWaypoint(joint_names, start_state) };
    MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, freespace_profile);
    start_instruction.setDescription("Start Instruction");

    JointWaypointPoly wp2{ JointWaypoint(joint_names, end_state) };
    MoveInstruction end_instruction(wp2, MoveInstructionType::FREESPACE);
    end_instruction.setDescription("End Instruction");

    program.appendMoveInstruction(start_instruction);
    program.appendMoveInstruction(end_instruction);
  }
  else
  {
    StateWaypointPoly wp1{ StateWaypoint(joint_names, start_state) };
    MoveInstruction start_instruction(wp1, MoveInstructionType::FREESPACE, freespace_profile);
    start_instruction.setDescription("Start Instruction");

    StateWaypointPoly wp2{ StateWaypoint(joint_names, end_state) };
    MoveInstruction end_instruction(wp2, MoveInstructionType::FREESPACE);
    end_instruction.setDescription("End Instruction");

    program.appendMoveInstruction(start_instruction);
    program.appendMoveInstruction(end_instruction);
  }

  return program;
}

inline CompositeInstruction rasterExampleProgram(const std::string& freespace_profile = DEFAULT_PROFILE_KEY,
                                                 const std::string& process_profile = "PROCESS")
{
  CompositeInstruction program(
      DEFAULT_PROFILE_KEY, CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool0"));

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  StateWaypointPoly swp1{ StateWaypoint(joint_names, Eigen::VectorXd::Zero(6)) };
  MoveInstruction start_instruction(swp1, MoveInstructionType::FREESPACE, freespace_profile);
  start_instruction.setDescription("Start");

  CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                               Eigen::Quaterniond(0, 0, -1.0, 0)) };

  // Define from start composite instruction
  MoveInstruction plan_f0(wp1, MoveInstructionType::FREESPACE, freespace_profile);
  plan_f0.setDescription("from_start_plan");
  CompositeInstruction from_start(freespace_profile);
  from_start.setDescription("from_start");
  from_start.appendMoveInstruction(start_instruction);
  from_start.appendMoveInstruction(plan_f0);
  program.push_back(from_start);

  //
  for (int i = 0; i < 4; ++i)
  {
    double x = 0.8 + (i * 0.1);
    CartesianWaypointPoly wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.3, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.2, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.1, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.0, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.1, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.2, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.3, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));

    CompositeInstruction raster_segment(process_profile);
    raster_segment.setDescription("Raster #" + std::to_string(i + 1));
    if (i == 0 || i == 2)
    {
      raster_segment.appendMoveInstruction(MoveInstruction(wp2, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp3, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp4, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp5, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp6, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp7, MoveInstructionType::LINEAR, process_profile));
    }
    else
    {
      raster_segment.appendMoveInstruction(MoveInstruction(wp6, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp5, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp4, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp3, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp2, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp1, MoveInstructionType::LINEAR, process_profile));
    }
    program.push_back(raster_segment);

    // Add transition
    if (i == 0 || i == 2)
    {
      CartesianWaypointPoly wp7 =
          CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8 + ((i + 1) * 0.1), 0.3, 0.8) *
                            Eigen::Quaterniond(0, 0, -1.0, 0));

      MoveInstruction plan_f1(wp7, MoveInstructionType::FREESPACE, freespace_profile);
      plan_f1.setDescription("transition_from_end_plan");

      CompositeInstruction transition(freespace_profile);
      transition.setDescription("Transition #" + std::to_string(i + 1));
      transition.appendMoveInstruction(plan_f1);
      program.push_back(transition);
    }
    else if (i == 1)
    {
      CartesianWaypointPoly wp1 =
          CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8 + ((i + 1) * 0.1), -0.3, 0.8) *
                            Eigen::Quaterniond(0, 0, -1.0, 0));

      MoveInstruction plan_f1(wp1, MoveInstructionType::FREESPACE, freespace_profile);
      plan_f1.setDescription("transition_from_end_plan");

      CompositeInstruction transition(freespace_profile);
      transition.setDescription("Transition #" + std::to_string(i + 1));
      transition.appendMoveInstruction(plan_f1);
      program.push_back(transition);
    }
  }

  MoveInstruction plan_f2(swp1, MoveInstructionType::FREESPACE, freespace_profile);
  plan_f2.setDescription("to_end_plan");
  CompositeInstruction to_end(freespace_profile);
  to_end.setDescription("to_end");
  to_end.appendMoveInstruction(plan_f2);
  program.push_back(to_end);

  return program;
}

inline CompositeInstruction rasterOnlyExampleProgram(const std::string& freespace_profile = DEFAULT_PROFILE_KEY,
                                                     const std::string& process_profile = "PROCESS")
{
  CompositeInstruction program(
      DEFAULT_PROFILE_KEY, CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool0"));

  for (int i = 0; i < 4; ++i)
  {
    double x = 0.8 + (i * 0.1);
    CartesianWaypointPoly wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.3, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.2, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, -0.1, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.0, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.1, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.2, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));
    CartesianWaypointPoly wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(x, 0.3, 0.8) *
                                                  Eigen::Quaterniond(0, 0, -1.0, 0));

    CompositeInstruction raster_segment(process_profile);
    raster_segment.setDescription("Raster #" + std::to_string(i + 1));
    if (i == 0 || i == 2)
    {
      raster_segment.appendMoveInstruction(MoveInstruction(wp1, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp2, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp3, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp4, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp5, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp6, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp7, MoveInstructionType::LINEAR, process_profile));
    }
    else
    {
      raster_segment.appendMoveInstruction(MoveInstruction(wp6, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp5, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp4, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp3, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp2, MoveInstructionType::LINEAR, process_profile));
      raster_segment.appendMoveInstruction(MoveInstruction(wp1, MoveInstructionType::LINEAR, process_profile));
    }
    program.push_back(raster_segment);

    // Add transition
    if (i == 0 || i == 2)
    {
      CartesianWaypointPoly wp7 =
          CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8 + ((i + 1) * 0.1), 0.3, 0.8) *
                            Eigen::Quaterniond(0, 0, -1.0, 0));

      MoveInstruction plan_f1(wp7, MoveInstructionType::FREESPACE, freespace_profile);
      plan_f1.setDescription("transition_from_end_plan");

      CompositeInstruction transition(freespace_profile);
      transition.setDescription("Transition #" + std::to_string(i + 1));
      transition.appendMoveInstruction(plan_f1);
      program.push_back(transition);
    }
    else if (i == 1)
    {
      CartesianWaypointPoly wp1 =
          CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8 + ((i + 1) * 0.1), -0.3, 0.8) *
                            Eigen::Quaterniond(0, 0, -1.0, 0));

      MoveInstruction plan_f1(wp1, MoveInstructionType::FREESPACE, freespace_profile);
      plan_f1.setDescription("transition_from_end_plan");

      CompositeInstruction transition(freespace_profile);
      transition.setDescription("Transition #" + std::to_string(i + 1));
      transition.appendMoveInstruction(plan_f1);
      program.push_back(transition);
    }
  }

  return program;
}
}  // namespace tesseract_planning::test_suite

#endif  // TESSERACT_TASK_COMPOSER_TEST_PROGRAMS_HPP
