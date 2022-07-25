/**
 * @file move_instruction_unit.cpp
 * @brief Contains unit tests for MoveInstruction
 *
 * @author Levi Armstrong
 * @date February 15, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/state_waypoint.h>

using namespace tesseract_planning;

TEST(TesseractCommandLanguageMoveInstructionUnit, constructor)  // NOLINT
{
  Eigen::VectorXd jv = Eigen::VectorXd::Ones(6);
  std::vector<std::string> jn = { "j1", "j2", "j3", "j4", "j5", "j6" };
  StateWaypointPoly swp(StateWaypoint(jn, jv));

  // Minimum arguments
  {
    MoveInstruction instr(swp, MoveInstructionType::START);
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::START);
    EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_TRUE(instr.getPathProfile().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
  }

  {
    MoveInstruction instr(swp, MoveInstructionType::FREESPACE);
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::FREESPACE);
    EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_TRUE(instr.getPathProfile().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
  }

  {
    MoveInstruction instr(swp, MoveInstructionType::LINEAR);
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::LINEAR);
    EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_EQ(instr.getPathProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
  }

  // With plan profile
  {
    MoveInstruction instr(swp, MoveInstructionType::START, "TEST_PROFILE");
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::START);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_TRUE(instr.getPathProfile().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
  }

  {
    MoveInstruction instr(swp, MoveInstructionType::FREESPACE, "TEST_PROFILE");
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::FREESPACE);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_TRUE(instr.getPathProfile().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
  }

  {
    MoveInstruction instr(swp, MoveInstructionType::LINEAR, "TEST_PROFILE");
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::LINEAR);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PROFILE");
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
  }

  // With plan and path profile
  {
    MoveInstruction instr(swp, MoveInstructionType::START, "TEST_PROFILE", "TEST_PATH_PROFILE");
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::START);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
  }

  {
    MoveInstruction instr(swp, MoveInstructionType::FREESPACE, "TEST_PROFILE", "TEST_PATH_PROFILE");
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::FREESPACE);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
  }

  {
    MoveInstruction instr(swp, MoveInstructionType::LINEAR, "TEST_PROFILE", "TEST_PATH_PROFILE");
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::LINEAR);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
  }
}

TEST(TesseractCommandLanguageMoveInstructionUnit, setters)  // NOLINT
{
  Eigen::VectorXd jv = Eigen::VectorXd::Ones(6);
  std::vector<std::string> jn = { "j1", "j2", "j3", "j4", "j5", "j6" };
  StateWaypointPoly swp(StateWaypoint(jn, jv));

  MoveInstruction instr(swp, MoveInstructionType::START);
  EXPECT_EQ(instr.getWaypoint(), swp);
  EXPECT_EQ(instr.getMoveType(), MoveInstructionType::START);
  EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
  EXPECT_TRUE(instr.getPathProfile().empty());
  EXPECT_FALSE(instr.getDescription().empty());
  EXPECT_FALSE(instr.getUUID().is_nil());
  EXPECT_TRUE(instr.getParentUUID().is_nil());

  StateWaypoint test_swp(jn, 5 * jv);
  instr.assignStateWaypoint(test_swp);
  EXPECT_EQ(instr.getWaypoint().as<StateWaypointPoly>().as<StateWaypoint>(), test_swp);

  instr.setMoveType(MoveInstructionType::LINEAR);
  EXPECT_EQ(instr.getMoveType(), MoveInstructionType::LINEAR);

  instr.setProfile("TEST_PROFILE");
  EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");

  instr.setPathProfile("TEST_PATH_PROFILE");
  EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");

  instr.setDescription("This is a test.");
  EXPECT_EQ(instr.getDescription(), "This is a test.");

  boost::uuids::uuid uuid = instr.getUUID();
  instr.regenerateUUID();
  EXPECT_FALSE(uuid == instr.getUUID());

  instr.setParentUUID(uuid);
  EXPECT_FALSE(instr.getParentUUID().is_nil());
  EXPECT_TRUE(uuid == instr.getParentUUID());
}

TEST(TesseractCommandLanguageMoveInstructionUnit, UUID)  // NOLINT
{
  Eigen::VectorXd jv = Eigen::VectorXd::Ones(6);
  std::vector<std::string> jn = { "j1", "j2", "j3", "j4", "j5", "j6" };
  StateWaypointPoly swp(StateWaypoint(jn, jv));

  MoveInstruction instr(swp, MoveInstructionType::START);
  EXPECT_EQ(instr.getWaypoint(), swp);
  EXPECT_EQ(instr.getMoveType(), MoveInstructionType::START);
  EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
  EXPECT_TRUE(instr.getPathProfile().empty());
  EXPECT_FALSE(instr.getDescription().empty());
  EXPECT_FALSE(instr.getUUID().is_nil());

  MoveInstruction assign = instr;
  EXPECT_TRUE(assign == instr);
  EXPECT_TRUE(assign.getUUID() == instr.getUUID());

  MoveInstruction copy(instr);
  EXPECT_TRUE(copy == instr);
  EXPECT_TRUE(assign.getUUID() == instr.getUUID());

  boost::uuids::uuid uuid = instr.getUUID();
  instr.regenerateUUID();
  EXPECT_FALSE(uuid == instr.getUUID());
}

TEST(TesseractCommandLanguageMoveInstructionUnit, boostSerialization)  // NOLINT
{
  Eigen::VectorXd jv = Eigen::VectorXd::Ones(6);
  std::vector<std::string> jn = { "j1", "j2", "j3", "j4", "j5", "j6" };
  StateWaypointPoly swp(StateWaypoint(jn, jv));

  MoveInstruction instr(swp, MoveInstructionType::START);
  instr.setMoveType(MoveInstructionType::LINEAR);
  instr.setProfile("TEST_PROFILE");
  instr.setPathProfile("TEST_PATH_PROFILE");
  instr.setDescription("This is a test.");
  boost::uuids::uuid uuid = instr.getUUID();

  tesseract_common::Serialization::toArchiveFileXML<MoveInstruction>(instr, "/tmp/move_instruction_boost.xml");

  auto ninstr = tesseract_common::Serialization::fromArchiveFileXML<MoveInstruction>("/tmp/move_instruction_boost.xml");

  EXPECT_TRUE(ninstr == instr);
  EXPECT_EQ(ninstr.getWaypoint(), swp);
  EXPECT_EQ(ninstr.getMoveType(), MoveInstructionType::LINEAR);
  EXPECT_EQ(ninstr.getProfile(), "TEST_PROFILE");
  EXPECT_EQ(ninstr.getPathProfile(), "TEST_PATH_PROFILE");
  EXPECT_EQ(ninstr.getDescription(), "This is a test.");
  EXPECT_EQ(ninstr.getUUID(), uuid);
}

TEST(TesseractCommandLanguageMoveInstructionPolyUnit, boostSerialization)  // NOLINT
{
  Eigen::VectorXd jv = Eigen::VectorXd::Ones(6);
  std::vector<std::string> jn = { "j1", "j2", "j3", "j4", "j5", "j6" };
  StateWaypointPoly swp(StateWaypoint(jn, jv));

  MoveInstruction instr(swp, MoveInstructionType::START);
  instr.setMoveType(MoveInstructionType::LINEAR);
  instr.setProfile("TEST_PROFILE");
  instr.setPathProfile("TEST_PATH_PROFILE");
  instr.setDescription("This is a test.");
  boost::uuids::uuid uuid = instr.getUUID();

  MoveInstructionPoly instr_poly(instr);

  tesseract_common::Serialization::toArchiveFileXML<MoveInstructionPoly>(instr_poly, "/tmp/move_instruction_boost.xml");

  auto ninstr = tesseract_common::Serialization::fromArchiveFileXML<MoveInstructionPoly>("/tmp/"
                                                                                         "move_instruction_boost.xml");
  EXPECT_TRUE(ninstr == instr_poly);
  EXPECT_EQ(ninstr.getWaypoint(), swp);
  EXPECT_EQ(ninstr.getMoveType(), MoveInstructionType::LINEAR);
  EXPECT_EQ(ninstr.getProfile(), "TEST_PROFILE");
  EXPECT_EQ(ninstr.getPathProfile(), "TEST_PATH_PROFILE");
  EXPECT_EQ(ninstr.getDescription(), "This is a test.");
  EXPECT_EQ(ninstr.getUUID(), uuid);
  EXPECT_FALSE(ninstr.getUUID().is_nil());
  EXPECT_TRUE(ninstr.getParentUUID().is_nil());

  MoveInstructionPoly child_instr_poly = instr_poly.createChild();
  boost::uuids::uuid child_uuid = child_instr_poly.getUUID();

  tesseract_common::Serialization::toArchiveFileXML<MoveInstructionPoly>(child_instr_poly,
                                                                         "/tmp/child_move_instruction_boost.xml");

  auto child_ninstr = tesseract_common::Serialization::fromArchiveFileXML<MoveInstructionPoly>("/tmp/"
                                                                                               "child_move_instruction_"
                                                                                               "boost.xml");
  EXPECT_TRUE(child_ninstr == instr_poly);
  EXPECT_EQ(child_ninstr.getWaypoint(), swp);
  EXPECT_EQ(child_ninstr.getMoveType(), MoveInstructionType::LINEAR);
  EXPECT_EQ(child_ninstr.getProfile(), "TEST_PROFILE");
  EXPECT_EQ(child_ninstr.getPathProfile(), "TEST_PATH_PROFILE");
  EXPECT_EQ(child_ninstr.getDescription(), "This is a test.");
  EXPECT_EQ(child_ninstr.getUUID(), child_uuid);
  EXPECT_EQ(child_ninstr.getParentUUID(), uuid);
  EXPECT_NE(child_ninstr.getParentUUID(), child_ninstr.getUUID());
  EXPECT_FALSE(child_ninstr.getUUID().is_nil());
  EXPECT_FALSE(child_ninstr.getParentUUID().is_nil());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
