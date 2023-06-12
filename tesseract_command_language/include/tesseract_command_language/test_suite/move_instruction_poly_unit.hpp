/**
 * @file move_instruction_poly_unit.hpp
 * @brief Collection of unit tests for MoveInstructionPoly
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
#ifndef TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_POLY_UNIT_HPP
#define TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_POLY_UNIT_HPP

#include <tesseract_command_language/test_suite/instruction_poly_unit.hpp>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/constants.h>

namespace tesseract_planning::test_suite
{
namespace details
{
template <typename T>
void runMoveInstructionConstructorTest()
{
  Eigen::VectorXd jv = Eigen::VectorXd::Ones(6);
  std::vector<std::string> jn = { "j1", "j2", "j3", "j4", "j5", "j6" };
  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  pose.translation() = Eigen::Vector3d(1, 2, 3);

  StateWaypointPoly swp{ StateWaypoint(jn, jv) };
  JointWaypointPoly jwp{ JointWaypoint(jn, jv) };
  CartesianWaypointPoly cwp{ CartesianWaypoint(pose) };

  // Minimum arguments
  {
    MoveInstructionPoly instr{ T(swp, MoveInstructionType::CIRCULAR) };
    EXPECT_FALSE(instr.isNull());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::CIRCULAR);
    EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_EQ(instr.getPathProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_TRUE(instr.getProfileOverrides() == nullptr);
    EXPECT_TRUE(instr.getPathProfileOverrides() == nullptr);
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isCircular());

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(jwp, MoveInstructionType::FREESPACE) };
    EXPECT_FALSE(instr.isNull());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), jwp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::FREESPACE);
    EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_TRUE(instr.getPathProfile().empty());
    EXPECT_TRUE(instr.getProfileOverrides() == nullptr);
    EXPECT_TRUE(instr.getPathProfileOverrides() == nullptr);
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isFreespace());

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(cwp, MoveInstructionType::LINEAR) };
    EXPECT_FALSE(instr.isNull());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), cwp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::LINEAR);
    EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_EQ(instr.getPathProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_TRUE(instr.getProfileOverrides() == nullptr);
    EXPECT_TRUE(instr.getPathProfileOverrides() == nullptr);
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isLinear());

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  // With plan profile
  {
    MoveInstructionPoly instr{ T(swp, MoveInstructionType::CIRCULAR, "TEST_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::CIRCULAR);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PROFILE");
    EXPECT_TRUE(instr.getProfileOverrides() == nullptr);
    EXPECT_TRUE(instr.getPathProfileOverrides() == nullptr);
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isCircular());

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(jwp, MoveInstructionType::FREESPACE, "TEST_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), jwp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::FREESPACE);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_TRUE(instr.getPathProfile().empty());
    EXPECT_TRUE(instr.getProfileOverrides() == nullptr);
    EXPECT_TRUE(instr.getPathProfileOverrides() == nullptr);
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isFreespace());

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(cwp, MoveInstructionType::LINEAR, "TEST_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), cwp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::LINEAR);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PROFILE");
    EXPECT_TRUE(instr.getProfileOverrides() == nullptr);
    EXPECT_TRUE(instr.getPathProfileOverrides() == nullptr);
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isLinear());

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  // With plan and path profile
  {
    MoveInstructionPoly instr{ T(swp, MoveInstructionType::CIRCULAR, "TEST_PROFILE", "TEST_PATH_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::CIRCULAR);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
    EXPECT_TRUE(instr.getProfileOverrides() == nullptr);
    EXPECT_TRUE(instr.getPathProfileOverrides() == nullptr);
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isCircular());

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(jwp, MoveInstructionType::FREESPACE, "TEST_PROFILE", "TEST_PATH_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), jwp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::FREESPACE);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
    EXPECT_TRUE(instr.getProfileOverrides() == nullptr);
    EXPECT_TRUE(instr.getPathProfileOverrides() == nullptr);
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isFreespace());

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(cwp, MoveInstructionType::LINEAR, "TEST_PROFILE", "TEST_PATH_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), cwp);
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::LINEAR);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
    EXPECT_TRUE(instr.getProfileOverrides() == nullptr);
    EXPECT_TRUE(instr.getPathProfileOverrides() == nullptr);
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isLinear());

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }
}

template <typename T>
void runMoveInstructionSettersTest()
{
  Eigen::VectorXd jv = Eigen::VectorXd::Ones(6);
  std::vector<std::string> jn = { "j1", "j2", "j3", "j4", "j5", "j6" };
  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  pose.translation() = Eigen::Vector3d(1, 2, 3);

  StateWaypointPoly swp{ StateWaypoint(jn, jv) };
  JointWaypointPoly jwp{ JointWaypoint(jn, jv) };
  CartesianWaypointPoly cwp{ CartesianWaypoint(pose) };

  MoveInstructionPoly instr{ T(swp, MoveInstructionType::FREESPACE) };
  EXPECT_EQ(instr.getWaypoint(), swp);
  EXPECT_EQ(instr.getMoveType(), MoveInstructionType::FREESPACE);
  EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
  EXPECT_TRUE(instr.getPathProfile().empty());
  EXPECT_FALSE(instr.getDescription().empty());
  EXPECT_FALSE(instr.getUUID().is_nil());
  EXPECT_TRUE(instr.getParentUUID().is_nil());

  StateWaypoint test_swp(jn, 5 * jv);
  instr.assignStateWaypoint(test_swp);
  EXPECT_EQ(instr.getWaypoint().as<StateWaypointPoly>().as<StateWaypoint>(), test_swp);

  instr.assignJointWaypoint(jwp);
  EXPECT_EQ(instr.getWaypoint().as<JointWaypointPoly>().as<JointWaypoint>(), jwp.as<JointWaypoint>());

  instr.assignCartesianWaypoint(cwp);
  EXPECT_EQ(instr.getWaypoint().as<CartesianWaypointPoly>().as<CartesianWaypoint>(), cwp.as<CartesianWaypoint>());

  instr.setMoveType(MoveInstructionType::LINEAR);
  EXPECT_EQ(instr.getMoveType(), MoveInstructionType::LINEAR);

  instr.setProfile("TEST_PROFILE");
  EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");

  instr.setPathProfile("TEST_PATH_PROFILE");
  EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
}

}  // namespace details

template <typename T>
void runMoveInstructionTest()
{
  runInstructionInterfaceTest<T>();

  details::runMoveInstructionConstructorTest<T>();
  details::runMoveInstructionSettersTest<T>();
}
}  // namespace tesseract_planning::test_suite

#endif  // TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_POLY_UNIT_HPP
