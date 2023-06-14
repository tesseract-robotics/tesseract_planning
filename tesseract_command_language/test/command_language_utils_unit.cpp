/**
 * @file utils_test.cpp
 * @brief
 *
 * @author Matthew Powelson
 * @date June 15, 2020
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
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/utils.h>
#include "command_language_test_program.hpp"

using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

static const bool DEBUG = false;

TEST(TesseractCommandLanguageUtilsUnit, flatten)  // NOLINT
{
  // Create a composite
  CompositeInstruction composite;
  composite.setDescription("flatten Unit: Composite");
  std::size_t i_max = 4;
  std::size_t j_max = 3;
  std::size_t k_max = 2;

  for (std::size_t i = 0; i < i_max; i++)
  {
    CompositeInstruction sub_composite;
    sub_composite.setDescription("sub_composite_" + std::to_string(i));
    for (std::size_t j = 0; j < j_max; j++)
    {
      CompositeInstruction sub_sub_composite;
      sub_sub_composite.setDescription("sub_sub_composite_" + std::to_string(j));
      for (std::size_t k = 0; k < k_max; k++)
      {
        CartesianWaypointPoly wp{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
        MoveInstruction instruction(wp, MoveInstructionType::LINEAR);
        instruction.setDescription("instruction_" + std::to_string(i) + "_" + std::to_string(j) + "_" +
                                   std::to_string(k));
        sub_sub_composite.appendMoveInstruction(instruction);
      }
      sub_composite.push_back(sub_sub_composite);
    }
    composite.push_back(sub_composite);
  }

  // flatten(composite);
  {
    if (DEBUG)
      composite.print();

    // flatten the composite
    std::vector<std::reference_wrapper<InstructionPoly>> flattened = composite.flatten();
    EXPECT_EQ(flattened.size(), i_max * j_max * k_max);

    // Now change something in the flattened composite
    int num_composites = 0;
    for (std::size_t i = 0; i < flattened.size(); i++)
    {
      if (flattened[i].get().isCompositeInstruction())
        num_composites++;
      flattened[i].get().setDescription("test_" + std::to_string(i));
    }
    EXPECT_EQ(num_composites, 0);

    if (DEBUG)
      composite.print();

    // Now make sure the original changed
    std::size_t cumulative = 0;
    for (std::size_t i = 0; i < i_max; i++)
    {
      for (std::size_t j = 0; j < j_max; j++)
      {
        for (std::size_t k = 0; k < k_max; k++)
        {
          EXPECT_EQ("test_" + std::to_string(cumulative),
                    composite[i].as<CompositeInstruction>().at(j).as<CompositeInstruction>().at(k).getDescription());
          cumulative++;
        }
      }
    }
  }

  // flatten(composite, true);
  {
    if (DEBUG)
      composite.print();

    // flatten the composite keeping the composite instructions
    flattenFilterFn filter = [](const InstructionPoly&, const CompositeInstruction&) { return true; };
    std::vector<std::reference_wrapper<InstructionPoly>> flattened = composite.flatten(filter);
    EXPECT_EQ(flattened.size(), i_max * j_max * k_max + 16);  // Add 16 for the composite instructions

    // Now change something in the flattened composite
    int num_composites = 0;
    for (std::size_t i = 0; i < flattened.size(); i++)
    {
      if (flattened[i].get().isCompositeInstruction())
        num_composites++;
      flattened[i].get().setDescription("test_" + std::to_string(i));
    }
    EXPECT_EQ(num_composites, 16);

    if (DEBUG)
    {
      std::cout << "----- flattened -----" << std::endl;
      for (auto& i : flattened)
        i.get().print();
      std::cout << "----- Composite -----" << std::endl;
      composite.print();
    }

    // Now make sure the original changed
    std::size_t cumulative = 0;
    for (std::size_t i = 0; i < i_max; i++)
    {
      EXPECT_EQ("test_" + std::to_string(cumulative), composite[i].as<CompositeInstruction>().getDescription());
      cumulative++;
      for (std::size_t j = 0; j < j_max; j++)
      {
        EXPECT_EQ("test_" + std::to_string(cumulative),
                  composite[i].as<CompositeInstruction>().at(j).as<CompositeInstruction>().getDescription());
        cumulative++;
        for (std::size_t k = 0; k < k_max; k++)
        {
          EXPECT_EQ("test_" + std::to_string(cumulative),
                    composite[i].as<CompositeInstruction>().at(j).as<CompositeInstruction>().at(k).getDescription());
          cumulative++;
        }
      }
    }
  }
}

TEST(TesseractCommandLanguageUtilsUnit, toJointTrajectoryTests)  // NOLINT
{
  std::string profile{ "raster_program" };
  ManipulatorInfo manip_info("manipulator", "world", "tool0");
  CompositeInstructionOrder order{ CompositeInstructionOrder::ORDERED };
  CompositeInstruction program = getTestProgram(profile, order, manip_info);
  InstructionPoly instr{ program };
  tesseract_common::JointTrajectory jt = toJointTrajectory(instr);
  EXPECT_EQ(jt.size(), 16);

  InstructionPoly error_poly{ MoveInstruction() };
  EXPECT_ANY_THROW(toJointTrajectory(error_poly));  // NOLINT
}

TEST(TesseractCommandLanguageUtilsUnit, getJointPositionTests)  // NOLINT
{
  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  StateWaypointPoly wp0{ StateWaypoint(joint_names, Eigen::VectorXd::Constant(6, 3)) };
  JointWaypointPoly wp00{ JointWaypoint(joint_names, Eigen::VectorXd::Constant(6, 5)) };
  MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "freespace_profile");
  MoveInstruction end_instruction(wp00, MoveInstructionType::FREESPACE, "freespace_profile");
  start_instruction.setDescription("Start Instruction");
  end_instruction.setDescription("End Instruction");

  tesseract_common::JointState seed_state;
  seed_state.joint_names = joint_names;
  seed_state.position = Eigen::VectorXd::Constant(6, 10);

  // Define raster poses
  CartesianWaypointPoly wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  wp1.setSeed(seed_state);
  CartesianWaypointPoly wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.2, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));

  WaypointPoly wp0_poly{ wp0 };
  const Eigen::VectorXd& p0 = getJointPosition(wp0_poly);
  EXPECT_TRUE(p0.isApprox(wp0.getPosition()));

  WaypointPoly wp00_poly{ wp00 };
  const Eigen::VectorXd& p00 = getJointPosition(wp00_poly);
  EXPECT_TRUE(p00.isApprox(wp00.getPosition()));

  WaypointPoly wp1_poly{ wp1 };
  const Eigen::VectorXd& p1 = getJointPosition(wp1_poly);
  EXPECT_TRUE(p1.isApprox(wp1.getSeed().position));

  WaypointPoly wp2_poly{ wp2 };
  EXPECT_ANY_THROW(getJointPosition(wp2_poly));  // NOLINT

  WaypointPoly error_poly;
  EXPECT_ANY_THROW(getJointPosition(error_poly));  // NOLINT
}

TEST(TesseractCommandLanguageUtilsUnit, getJointPositionFormatedTests)  // NOLINT
{
  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2" };
  std::vector<std::string> format_joint_names = { "joint_2", "joint_1" };
  Eigen::VectorXd position0 = Eigen::Vector2d(1, 2);
  Eigen::VectorXd position00 = Eigen::Vector2d(3, 4);
  Eigen::VectorXd format_position0 = Eigen::Vector2d(2, 1);
  Eigen::VectorXd format_position00 = Eigen::Vector2d(4, 3);
  StateWaypointPoly wp0{ StateWaypoint(joint_names, position0) };
  JointWaypointPoly wp00{ JointWaypoint(joint_names, position00) };
  MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "freespace_profile");
  MoveInstruction end_instruction(wp00, MoveInstructionType::FREESPACE, "freespace_profile");
  start_instruction.setDescription("Start Instruction");
  end_instruction.setDescription("End Instruction");

  Eigen::VectorXd seed_position = Eigen::Vector2d(5, 6);
  Eigen::VectorXd format_seed_position = Eigen::Vector2d(6, 5);
  tesseract_common::JointState seed_state;
  seed_state.joint_names = joint_names;
  seed_state.position = seed_position;

  // Define raster poses
  CartesianWaypointPoly wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  wp1.setSeed(seed_state);
  CartesianWaypointPoly wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.2, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));

  WaypointPoly wp0_poly{ wp0 };
  const Eigen::VectorXd& p0 = getJointPosition(format_joint_names, wp0_poly);
  EXPECT_TRUE(p0.isApprox(format_position0));

  WaypointPoly wp00_poly{ wp00 };
  const Eigen::VectorXd& p00 = getJointPosition(format_joint_names, wp00_poly);
  EXPECT_TRUE(p00.isApprox(format_position00));

  WaypointPoly wp1_poly{ wp1 };
  const Eigen::VectorXd& p1 = getJointPosition(format_joint_names, wp1_poly);
  EXPECT_TRUE(p1.isApprox(format_seed_position));

  const Eigen::VectorXd& p1u = getJointPosition(joint_names, wp1_poly);
  EXPECT_TRUE(p1u.isApprox(seed_position));

  // Format is not correct size or invalid joint name
  EXPECT_ANY_THROW(getJointPosition({ "joint_1" }, wp0_poly));             // NOLINT
  EXPECT_ANY_THROW(getJointPosition({ "joint_3", "joint_1" }, wp0_poly));  // NOLINT

  WaypointPoly wp2_poly{ wp2 };
  EXPECT_ANY_THROW(getJointPosition(format_joint_names, wp2_poly));  // NOLINT

  WaypointPoly error_poly;
  EXPECT_ANY_THROW(getJointPosition(format_joint_names, error_poly));  // NOLINT
}

TEST(TesseractCommandLanguageUtilsUnit, formatJointPositionTests)  // NOLINT
{
  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2" };
  std::vector<std::string> format_joint_names = { "joint_2", "joint_1" };
  Eigen::VectorXd position0 = Eigen::Vector2d(1, 2);
  Eigen::VectorXd position00 = Eigen::Vector2d(3, 4);
  Eigen::VectorXd format_position0 = Eigen::Vector2d(2, 1);
  Eigen::VectorXd format_position00 = Eigen::Vector2d(4, 3);
  StateWaypointPoly wp0{ StateWaypoint(joint_names, position0) };
  JointWaypointPoly wp00{ JointWaypoint(joint_names, position00) };
  MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "freespace_profile");
  MoveInstruction end_instruction(wp00, MoveInstructionType::FREESPACE, "freespace_profile");
  start_instruction.setDescription("Start Instruction");
  end_instruction.setDescription("End Instruction");

  Eigen::VectorXd seed_position = Eigen::Vector2d(5, 6);
  Eigen::VectorXd format_seed_position = Eigen::Vector2d(6, 5);
  tesseract_common::JointState seed_state;
  seed_state.joint_names = joint_names;
  seed_state.position = seed_position;

  // Define raster poses
  CartesianWaypointPoly wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  wp1.setSeed(seed_state);
  CartesianWaypointPoly wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.2, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));

  WaypointPoly wp0_poly{ wp0 };
  EXPECT_TRUE(formatJointPosition(format_joint_names, wp0_poly));
  EXPECT_TRUE(wp0_poly.as<StateWaypointPoly>().getPosition().isApprox(format_position0));

  WaypointPoly wp00_poly{ wp00 };
  EXPECT_TRUE(formatJointPosition(format_joint_names, wp00_poly));
  EXPECT_TRUE(wp00_poly.as<JointWaypointPoly>().getPosition().isApprox(format_position00));

  WaypointPoly wp1_poly{ wp1 };
  EXPECT_TRUE(formatJointPosition(format_joint_names, wp1_poly));
  EXPECT_TRUE(wp1_poly.as<CartesianWaypointPoly>().getSeed().position.isApprox(format_seed_position));

  WaypointPoly wp1u_poly{ wp1 };
  EXPECT_FALSE(formatJointPosition(joint_names, wp1u_poly));
  EXPECT_TRUE(wp1u_poly.as<CartesianWaypointPoly>().getSeed().position.isApprox(seed_position));

  // Format is not correct size or invalid joint name
  EXPECT_ANY_THROW(formatJointPosition({ "joint_1" }, wp0_poly));             // NOLINT
  EXPECT_ANY_THROW(formatJointPosition({ "joint_3", "joint_1" }, wp0_poly));  // NOLINT

  WaypointPoly wp2_poly{ wp2 };
  EXPECT_ANY_THROW(formatJointPosition(format_joint_names, wp2_poly));  // NOLINT

  WaypointPoly error_poly;
  EXPECT_ANY_THROW(formatJointPosition(format_joint_names, error_poly));  // NOLINT
}

TEST(TesseractCommandLanguageUtilsUnit, checkJointPositionFormatTests)  // NOLINT
{
  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2" };
  std::vector<std::string> format_joint_names = { "joint_2", "joint_1" };
  Eigen::VectorXd position0 = Eigen::Vector2d(1, 2);
  Eigen::VectorXd position00 = Eigen::Vector2d(3, 4);
  Eigen::VectorXd format_position0 = Eigen::Vector2d(2, 1);
  Eigen::VectorXd format_position00 = Eigen::Vector2d(4, 3);
  StateWaypointPoly wp0{ StateWaypoint(joint_names, position0) };
  JointWaypointPoly wp00{ JointWaypoint(joint_names, position00) };
  MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "freespace_profile");
  MoveInstruction end_instruction(wp00, MoveInstructionType::FREESPACE, "freespace_profile");
  start_instruction.setDescription("Start Instruction");
  end_instruction.setDescription("End Instruction");

  Eigen::VectorXd seed_position = Eigen::Vector2d(5, 6);
  Eigen::VectorXd format_seed_position = Eigen::Vector2d(6, 5);
  tesseract_common::JointState seed_state;
  seed_state.joint_names = joint_names;
  seed_state.position = seed_position;

  // Define raster poses
  CartesianWaypointPoly wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  wp1.setSeed(seed_state);
  CartesianWaypointPoly wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.2, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));

  WaypointPoly wp0_poly{ wp0 };
  EXPECT_TRUE(checkJointPositionFormat(joint_names, wp0_poly));
  EXPECT_FALSE(checkJointPositionFormat(format_joint_names, wp0_poly));

  WaypointPoly wp00_poly{ wp00 };
  EXPECT_TRUE(checkJointPositionFormat(joint_names, wp00_poly));
  EXPECT_FALSE(checkJointPositionFormat(format_joint_names, wp00_poly));

  WaypointPoly wp1_poly{ wp1 };
  EXPECT_TRUE(checkJointPositionFormat(joint_names, wp1_poly));
  EXPECT_FALSE(checkJointPositionFormat(format_joint_names, wp1_poly));

  // Format is not correct size or invalid joint name
  EXPECT_FALSE(checkJointPositionFormat({ "joint_1" }, wp0_poly));
  EXPECT_FALSE(checkJointPositionFormat({ "joint_3", "joint_1" }, wp0_poly));

  WaypointPoly wp2_poly{ wp2 };
  EXPECT_ANY_THROW(checkJointPositionFormat(joint_names, wp2_poly));         // NOLINT
  EXPECT_ANY_THROW(checkJointPositionFormat(format_joint_names, wp2_poly));  // NOLINT

  WaypointPoly error_poly;
  EXPECT_ANY_THROW(checkJointPositionFormat(joint_names, error_poly));         // NOLINT
  EXPECT_ANY_THROW(checkJointPositionFormat(format_joint_names, error_poly));  // NOLINT
}

TEST(TesseractCommandLanguageUtilsUnit, getJointNamesTests)  // NOLINT
{
  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  StateWaypointPoly wp0{ StateWaypoint(joint_names, Eigen::VectorXd::Constant(6, 3)) };
  JointWaypointPoly wp00{ JointWaypoint(joint_names, Eigen::VectorXd::Constant(6, 5)) };
  MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "freespace_profile");
  MoveInstruction end_instruction(wp00, MoveInstructionType::FREESPACE, "freespace_profile");
  start_instruction.setDescription("Start Instruction");
  end_instruction.setDescription("End Instruction");

  tesseract_common::JointState seed_state;
  seed_state.joint_names = joint_names;
  seed_state.position = Eigen::VectorXd::Constant(6, 10);

  // Define raster poses
  CartesianWaypointPoly wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  wp1.setSeed(seed_state);
  CartesianWaypointPoly wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.2, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));

  WaypointPoly wp0_poly{ wp0 };
  const std::vector<std::string>& n0 = getJointNames(wp0_poly);
  EXPECT_EQ(n0, wp0.getNames());

  WaypointPoly wp00_poly{ wp00 };
  const std::vector<std::string>& n00 = getJointNames(wp00_poly);
  EXPECT_EQ(n00, wp00.getNames());

  WaypointPoly wp1_poly{ wp1 };
  const std::vector<std::string>& n1 = getJointNames(wp1_poly);
  EXPECT_EQ(n1, wp1.getSeed().joint_names);

  WaypointPoly wp2_poly{ wp2 };
  EXPECT_ANY_THROW(getJointNames(wp2_poly));  // NOLINT

  WaypointPoly error_poly;
  EXPECT_ANY_THROW(getJointNames(error_poly));  // NOLINT
}

TEST(TesseractCommandLanguageUtilsUnit, setJointPositionTests)  // NOLINT
{
  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  StateWaypointPoly wp0{ StateWaypoint(joint_names, Eigen::VectorXd::Constant(6, 3)) };
  JointWaypointPoly wp00{ JointWaypoint(joint_names, Eigen::VectorXd::Constant(6, 5)) };
  MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "freespace_profile");
  MoveInstruction end_instruction(wp00, MoveInstructionType::FREESPACE, "freespace_profile");
  start_instruction.setDescription("Start Instruction");
  end_instruction.setDescription("End Instruction");

  Eigen::VectorXd set_position = Eigen::VectorXd::Constant(6, 1);

  tesseract_common::JointState seed_state;
  seed_state.joint_names = joint_names;
  seed_state.position = Eigen::VectorXd::Constant(6, 10);

  // Define raster poses
  CartesianWaypointPoly wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));
  wp1.setSeed(seed_state);
  CartesianWaypointPoly wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.2, 0.8) *
                                                Eigen::Quaterniond(0, 0, -1.0, 0));

  WaypointPoly wp0_poly{ wp0 };
  EXPECT_TRUE(setJointPosition(wp0_poly, set_position));
  EXPECT_TRUE(set_position.isApprox(wp0_poly.as<StateWaypointPoly>().getPosition()));

  WaypointPoly wp00_poly{ wp00 };
  EXPECT_TRUE(setJointPosition(wp00_poly, set_position));
  EXPECT_TRUE(set_position.isApprox(wp00_poly.as<JointWaypointPoly>().getPosition()));

  WaypointPoly wp1_poly{ wp1 };
  EXPECT_TRUE(setJointPosition(wp1_poly, set_position));
  EXPECT_TRUE(set_position.isApprox(wp1_poly.as<CartesianWaypointPoly>().getSeed().position));

  WaypointPoly wp2_poly{ wp2 };
  EXPECT_FALSE(setJointPosition(wp2_poly, set_position));

  WaypointPoly error_poly;
  EXPECT_FALSE(setJointPosition(error_poly, set_position));
}

TEST(TesseractCommandLanguageUtilsUnit, isWithinJointLimits)  // NOLINT
{
  Eigen::MatrixX2d limits(3, 2);
  limits << 0, 2, 0, 2, 0, 2;
  std::vector<std::string> joint_names = { "1", "2", "3" };
  Eigen::VectorXd values(3);

  // Within limits
  {
    values << 1, 1, 1;
    JointWaypointPoly jp{ JointWaypoint(joint_names, values) };
    WaypointPoly tmp(jp);
    EXPECT_TRUE(isWithinJointLimits(tmp, limits));
  }
  // Above limits
  {
    values << 1, 1, 3;
    JointWaypointPoly jp{ JointWaypoint(joint_names, values) };
    WaypointPoly tmp(jp);
    EXPECT_FALSE(isWithinJointLimits(tmp, limits));
  }
  // Below limits
  {
    values << 1, -1, 1;
    JointWaypointPoly jp{ JointWaypoint(joint_names, values) };
    WaypointPoly tmp(jp);
    EXPECT_FALSE(isWithinJointLimits(tmp, limits));
  }
  // Cartesian Waypoint
  {
    CartesianWaypointPoly jp{ CartesianWaypoint() };
    WaypointPoly tmp(jp);
    EXPECT_ANY_THROW(isWithinJointLimits(tmp, limits));  // NOLINT
  }
}

TEST(TesseractCommandLanguageUtilsUnit, clampToJointLimits)  // NOLINT
{
  Eigen::MatrixX2d limits(3, 2);
  limits << 0, 2, 0, 2, 0, 2;
  std::vector<std::string> joint_names = { "1", "2", "3" };
  Eigen::VectorXd values(3);

  // Within limits
  {
    values << 1, 1, 1;
    JointWaypointPoly jp{ JointWaypoint(joint_names, values) };
    WaypointPoly tmp(jp);
    EXPECT_TRUE(clampToJointLimits(tmp, limits));
    EXPECT_TRUE(tmp.as<JointWaypointPoly>().getPosition().isApprox(values, 1e-5));
  }
  // Above limits
  {
    values << 1, 1, 3;
    JointWaypointPoly jp{ JointWaypoint(joint_names, values) };
    WaypointPoly tmp(jp);
    EXPECT_TRUE(clampToJointLimits(tmp, limits));
    EXPECT_FALSE(tmp.as<JointWaypointPoly>().getPosition().isApprox(values, 1e-5));
    EXPECT_DOUBLE_EQ(2, tmp.as<JointWaypointPoly>().getPosition()[2]);
  }
  // Below limits
  {
    values << 1, -1, 1;
    JointWaypointPoly jp{ JointWaypoint(joint_names, values) };
    WaypointPoly tmp(jp);
    EXPECT_TRUE(clampToJointLimits(tmp, limits));
    EXPECT_FALSE(tmp.as<JointWaypointPoly>().getPosition().isApprox(values, 1e-5));
    EXPECT_DOUBLE_EQ(0, tmp.as<JointWaypointPoly>().getPosition()[1]);
  }
  // Above limits with max deviation
  {
    values << 1, 1, 2.05;
    JointWaypointPoly jp{ JointWaypoint(joint_names, values) };
    WaypointPoly tmp(jp);
    // Outside max deviation
    EXPECT_FALSE(clampToJointLimits(tmp, limits, 0.01));
    EXPECT_TRUE(tmp.as<JointWaypointPoly>().getPosition().isApprox(values, 1e-5));
    // Inside max deviation
    EXPECT_TRUE(clampToJointLimits(tmp, limits, 0.1));
    EXPECT_FALSE(tmp.as<JointWaypointPoly>().getPosition().isApprox(values, 1e-5));
    EXPECT_DOUBLE_EQ(2, tmp.as<JointWaypointPoly>().getPosition()[2]);
  }
  // Below limits with max deviation
  {
    values << 1, -0.05, 1;
    JointWaypointPoly jp{ JointWaypoint(joint_names, values) };
    WaypointPoly tmp(jp);
    // Outside max deviation
    EXPECT_FALSE(clampToJointLimits(tmp, limits, 0.01));
    EXPECT_TRUE(tmp.as<JointWaypointPoly>().getPosition().isApprox(values, 1e-5));
    // Inside max deviation
    EXPECT_TRUE(clampToJointLimits(tmp, limits, 0.1));
    EXPECT_FALSE(tmp.as<JointWaypointPoly>().getPosition().isApprox(values, 1e-5));
    EXPECT_DOUBLE_EQ(0, tmp.as<JointWaypointPoly>().getPosition()[1]);
  }
  // Type with no joint values
  {
    CartesianWaypointPoly jp{ CartesianWaypoint() };
    WaypointPoly tmp(jp);
    EXPECT_ANY_THROW(clampToJointLimits(tmp, limits));  // NOLINT
  }
}

TEST(TesseractCommandLanguageUtilsUnit, toDelimitedFile)  // NOLINT
{
  CompositeInstruction composite;
  composite.setDescription("To Delimited File: Composite");

  std::vector<std::string> joint_names = { "1", "2", "3" };
  Eigen::VectorXd values = Eigen::VectorXd::Constant(3, 5);
  {
    JointWaypointPoly jwp{ JointWaypoint(joint_names, values) };
    composite.appendMoveInstruction(MoveInstruction(jwp, MoveInstructionType::FREESPACE));
  }
  {
    values = Eigen::VectorXd::Constant(3, 10);
    JointWaypointPoly jwp{ JointWaypoint(joint_names, values) };
    composite.appendMoveInstruction(MoveInstruction(jwp, MoveInstructionType::FREESPACE));
  }
  {
    values = Eigen::VectorXd::Constant(3, 15);
    JointWaypointPoly jwp{ JointWaypoint(joint_names, values) };
    composite.appendMoveInstruction(MoveInstruction(jwp, MoveInstructionType::FREESPACE));
  }

  std::string path = tesseract_common::getTempPath() + "to_delimited_file.csv";
  EXPECT_TRUE(toDelimitedFile(composite, path));

  std::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  std::string check = "1,2,3\n5,5,5\n10,10,10\n15,15,15\n";
  std::cout << buffer.str() << std::endl;
  EXPECT_EQ(check, buffer.str());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
