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

#include <tesseract_common/manipulator_info.h>

namespace tesseract_planning::test_suite
{
namespace details
{
template <typename T>
void runMoveInstructionInterfaceTest()
{
  MoveInstructionPoly inst{ T() };
  auto uuid = inst.getUUID();
  EXPECT_TRUE(uuid.is_nil());
  EXPECT_TRUE(inst.getParentUUID().is_nil());

  auto set_uuid = boost::uuids::random_generator()();
  inst.setUUID(set_uuid);
  EXPECT_FALSE(inst.getUUID().is_nil());
  EXPECT_EQ(inst.getUUID(), set_uuid);
  EXPECT_ANY_THROW(inst.setUUID(boost::uuids::uuid{}));  // NOLINT
  EXPECT_FALSE(inst.getUUID().is_nil());
  EXPECT_EQ(inst.getUUID(), set_uuid);

  inst.regenerateUUID();
  auto reg_uuid = inst.getUUID();
  EXPECT_FALSE(reg_uuid.is_nil());
  EXPECT_NE(inst.getUUID(), uuid);

  inst.setParentUUID(reg_uuid);
  EXPECT_FALSE(inst.getParentUUID().is_nil());
  EXPECT_EQ(inst.getParentUUID(), reg_uuid);

  const std::string desc{ "tesseract_planning::test_suite::MoveInstruction::Description" };
  EXPECT_NE(inst.getDescription(), desc);
  inst.setDescription(desc);
  EXPECT_EQ(inst.getDescription(), desc);

  EXPECT_NO_THROW(inst.print());         // NOLINT
  EXPECT_NO_THROW(inst.print("test_"));  // NOLINT

  InstructionPoly assign = inst;
  EXPECT_TRUE(assign == inst);
  EXPECT_TRUE(assign.getUUID() == inst.getUUID());

  InstructionPoly copy(inst);
  EXPECT_TRUE(copy == inst);
  EXPECT_TRUE(assign.getUUID() == inst.getUUID());
}

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
    MoveInstructionPoly instr{ T(WaypointPoly{ swp }, MoveInstructionType::CIRCULAR) };
    EXPECT_FALSE(instr.isNull());
    EXPECT_FALSE(instr.isChild());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_TRUE(instr.getWaypoint().isStateWaypoint());
    EXPECT_TRUE(instr.getManipulatorInfo().empty());
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::CIRCULAR);
    EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_EQ(instr.getPathProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_TRUE(instr.getProfileOverrides().empty());
    EXPECT_TRUE(instr.getPathProfileOverrides().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isCircular());

    MoveInstructionPoly copy{ instr };
    EXPECT_TRUE(instr == copy);
    EXPECT_TRUE(instr == copy);
    EXPECT_FALSE(instr != copy);

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(swp, MoveInstructionType::CIRCULAR) };
    EXPECT_FALSE(instr.isNull());
    EXPECT_FALSE(instr.isChild());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_TRUE(instr.getWaypoint().isStateWaypoint());
    EXPECT_TRUE(instr.getManipulatorInfo().empty());
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::CIRCULAR);
    EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_EQ(instr.getPathProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_TRUE(instr.getProfileOverrides().empty());
    EXPECT_TRUE(instr.getPathProfileOverrides().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isCircular());

    MoveInstructionPoly copy{ instr };
    EXPECT_TRUE(instr == copy);
    EXPECT_TRUE(instr == copy);
    EXPECT_FALSE(instr != copy);

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(jwp, MoveInstructionType::FREESPACE) };
    EXPECT_FALSE(instr.isNull());
    EXPECT_FALSE(instr.isChild());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), jwp);
    EXPECT_TRUE(instr.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(instr.getManipulatorInfo().empty());
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::FREESPACE);
    EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_TRUE(instr.getPathProfile().empty());
    EXPECT_TRUE(instr.getProfileOverrides().empty());
    EXPECT_TRUE(instr.getPathProfileOverrides().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isFreespace());

    MoveInstructionPoly copy{ instr };
    EXPECT_TRUE(instr == copy);
    EXPECT_TRUE(instr == copy);
    EXPECT_FALSE(instr != copy);

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(cwp, MoveInstructionType::LINEAR) };
    EXPECT_FALSE(instr.isNull());
    EXPECT_FALSE(instr.isChild());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), cwp);
    EXPECT_TRUE(instr.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(instr.getManipulatorInfo().empty());
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::LINEAR);
    EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_EQ(instr.getPathProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_TRUE(instr.getProfileOverrides().empty());
    EXPECT_TRUE(instr.getPathProfileOverrides().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isLinear());

    MoveInstructionPoly copy{ instr };
    EXPECT_TRUE(instr == copy);
    EXPECT_TRUE(instr == copy);
    EXPECT_FALSE(instr != copy);

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  // With plan profile
  {
    MoveInstructionPoly instr{ T(swp, MoveInstructionType::CIRCULAR, "TEST_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_FALSE(instr.isChild());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_TRUE(instr.getWaypoint().isStateWaypoint());
    EXPECT_TRUE(instr.getManipulatorInfo().empty());
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::CIRCULAR);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PROFILE");
    EXPECT_TRUE(instr.getProfileOverrides().empty());
    EXPECT_TRUE(instr.getPathProfileOverrides().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isCircular());

    MoveInstructionPoly copy{ instr };
    EXPECT_TRUE(instr == copy);
    EXPECT_TRUE(instr == copy);
    EXPECT_FALSE(instr != copy);

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(WaypointPoly{ jwp }, MoveInstructionType::FREESPACE, "TEST_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_FALSE(instr.isChild());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), jwp);
    EXPECT_TRUE(instr.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(instr.getManipulatorInfo().empty());
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::FREESPACE);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_TRUE(instr.getPathProfile().empty());
    EXPECT_TRUE(instr.getProfileOverrides().empty());
    EXPECT_TRUE(instr.getPathProfileOverrides().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isFreespace());

    MoveInstructionPoly copy{ instr };
    EXPECT_TRUE(instr == copy);
    EXPECT_TRUE(instr == copy);
    EXPECT_FALSE(instr != copy);

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(jwp, MoveInstructionType::FREESPACE, "TEST_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_FALSE(instr.isChild());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), jwp);
    EXPECT_TRUE(instr.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(instr.getManipulatorInfo().empty());
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::FREESPACE);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_TRUE(instr.getPathProfile().empty());
    EXPECT_TRUE(instr.getProfileOverrides().empty());
    EXPECT_TRUE(instr.getPathProfileOverrides().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isFreespace());

    MoveInstructionPoly copy{ instr };
    EXPECT_TRUE(instr == copy);
    EXPECT_TRUE(instr == copy);
    EXPECT_FALSE(instr != copy);

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(cwp, MoveInstructionType::LINEAR, "TEST_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_FALSE(instr.isChild());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), cwp);
    EXPECT_TRUE(instr.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(instr.getManipulatorInfo().empty());
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::LINEAR);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PROFILE");
    EXPECT_TRUE(instr.getProfileOverrides().empty());
    EXPECT_TRUE(instr.getPathProfileOverrides().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isLinear());

    MoveInstructionPoly copy{ instr };
    EXPECT_TRUE(instr == copy);
    EXPECT_TRUE(instr == copy);
    EXPECT_FALSE(instr != copy);

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  // With plan and path profile
  {
    MoveInstructionPoly instr{ T(swp, MoveInstructionType::CIRCULAR, "TEST_PROFILE", "TEST_PATH_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_FALSE(instr.isChild());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), swp);
    EXPECT_TRUE(instr.getWaypoint().isStateWaypoint());
    EXPECT_TRUE(instr.getManipulatorInfo().empty());
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::CIRCULAR);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
    EXPECT_TRUE(instr.getProfileOverrides().empty());
    EXPECT_TRUE(instr.getPathProfileOverrides().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isCircular());

    MoveInstructionPoly copy{ instr };
    EXPECT_TRUE(instr == copy);
    EXPECT_TRUE(instr == copy);
    EXPECT_FALSE(instr != copy);

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(jwp, MoveInstructionType::FREESPACE, "TEST_PROFILE", "TEST_PATH_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_FALSE(instr.isChild());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), jwp);
    EXPECT_TRUE(instr.getWaypoint().isJointWaypoint());
    EXPECT_TRUE(instr.getManipulatorInfo().empty());
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::FREESPACE);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
    EXPECT_TRUE(instr.getProfileOverrides().empty());
    EXPECT_TRUE(instr.getPathProfileOverrides().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isFreespace());

    MoveInstructionPoly copy{ instr };
    EXPECT_TRUE(instr == copy);
    EXPECT_TRUE(instr == copy);
    EXPECT_FALSE(instr != copy);

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(
        WaypointPoly{ cwp }, MoveInstructionType::LINEAR, "TEST_PROFILE", "TEST_PATH_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_FALSE(instr.isChild());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), cwp);
    EXPECT_TRUE(instr.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(instr.getManipulatorInfo().empty());
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::LINEAR);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
    EXPECT_TRUE(instr.getProfileOverrides().empty());
    EXPECT_TRUE(instr.getPathProfileOverrides().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isLinear());

    MoveInstructionPoly copy{ instr };
    EXPECT_TRUE(instr == copy);
    EXPECT_TRUE(instr == copy);
    EXPECT_FALSE(instr != copy);

    InstructionPoly base = instr;
    EXPECT_TRUE(base.isMoveInstruction());
    EXPECT_FALSE(base.isCompositeInstruction());

    runInstructionSerializationTest(instr);
  }

  {
    MoveInstructionPoly instr{ T(cwp, MoveInstructionType::LINEAR, "TEST_PROFILE", "TEST_PATH_PROFILE") };
    EXPECT_FALSE(instr.isNull());
    EXPECT_FALSE(instr.isChild());
    EXPECT_TRUE(instr.getType() == std::type_index(typeid(T)));
    EXPECT_EQ(instr.getWaypoint(), cwp);
    EXPECT_TRUE(instr.getWaypoint().isCartesianWaypoint());
    EXPECT_TRUE(instr.getManipulatorInfo().empty());
    EXPECT_EQ(instr.getMoveType(), MoveInstructionType::LINEAR);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
    EXPECT_TRUE(instr.getProfileOverrides().empty());
    EXPECT_TRUE(instr.getPathProfileOverrides().empty());
    EXPECT_FALSE(instr.getDescription().empty());
    EXPECT_FALSE(instr.getUUID().is_nil());
    EXPECT_TRUE(instr.getParentUUID().is_nil());
    EXPECT_TRUE(instr.isLinear());

    MoveInstructionPoly copy{ instr };
    EXPECT_TRUE(instr == copy);
    EXPECT_TRUE(instr == copy);
    EXPECT_FALSE(instr != copy);

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
  instr.getWaypoint() = StateWaypointPoly(test_swp);
  EXPECT_EQ(instr.getWaypoint().as<StateWaypointPoly>().as<StateWaypoint>(), test_swp);

  instr.getWaypoint() = JointWaypointPoly(jwp);
  EXPECT_EQ(instr.getWaypoint().as<JointWaypointPoly>().as<JointWaypoint>(), jwp.as<JointWaypoint>());

  instr.getWaypoint() = CartesianWaypointPoly(cwp);
  EXPECT_EQ(instr.getWaypoint().as<CartesianWaypointPoly>().as<CartesianWaypoint>(), cwp.as<CartesianWaypoint>());

  instr.setMoveType(MoveInstructionType::LINEAR);
  EXPECT_EQ(instr.getMoveType(), MoveInstructionType::LINEAR);

  instr.setProfile("TEST_PROFILE");
  EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");

  instr.setPathProfile("TEST_PATH_PROFILE");
  EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");

  // Create arbitrary profile overrides under arbitrary namespaces
  const std::string ns1 = "ns1";
  const std::string ns1_profile = "profile1";
  const std::string ns2 = "ns2";
  const std::string ns2_profile = "profile2";
  {
    ProfileOverrides overrides;
    overrides[ns1] = ns1_profile;
    overrides[ns2] = ns2_profile;
    instr.setProfileOverrides(overrides);
    instr.setPathProfileOverrides(overrides);
  }

  // Profile Overrides
  EXPECT_EQ(instr.getProfile(ns1), ns1_profile);
  EXPECT_EQ(instr.getProfile(ns2), ns2_profile);
  EXPECT_EQ(instr.getProfile("nonexistent_ns"), "TEST_PROFILE");

  // Path Profile Overrides
  EXPECT_EQ(instr.getPathProfile(ns1), ns1_profile);
  EXPECT_EQ(instr.getPathProfile(ns2), ns2_profile);
  EXPECT_EQ(instr.getPathProfile("nonexistent_ns"), "TEST_PATH_PROFILE");

  EXPECT_TRUE(instr.getManipulatorInfo().empty());
  tesseract_common::ManipulatorInfo manip_info("manip", "base_link", "tool0");
  instr.setManipulatorInfo(manip_info);
  EXPECT_FALSE(instr.getManipulatorInfo().empty());
  EXPECT_TRUE(instr.getManipulatorInfo() == manip_info);

  tesseract_common::ManipulatorInfo manip_info2("manip2", "base_link2", "tool02");
  instr.getManipulatorInfo() = manip_info2;
  EXPECT_FALSE(instr.getManipulatorInfo().empty());
  EXPECT_TRUE(instr.getManipulatorInfo() == manip_info2);
  EXPECT_FALSE(instr.getManipulatorInfo() == manip_info);

  StateWaypointPoly cswp = instr.createStateWaypoint();
  EXPECT_TRUE(cswp.getType() == swp.getType());

  JointWaypointPoly cjwp = instr.createJointWaypoint();
  EXPECT_TRUE(cjwp.getType() == jwp.getType());

  CartesianWaypointPoly ccwp = instr.createCartesianWaypoint();
  EXPECT_TRUE(ccwp.getType() == cwp.getType());

  MoveInstructionPoly child = instr.createChild();
  EXPECT_TRUE(child.getType() == std::type_index(typeid(T)));
  EXPECT_TRUE(child.getUUID() != instr.getUUID());
  EXPECT_TRUE(child.getParentUUID() == instr.getUUID());
  EXPECT_TRUE(child.getManipulatorInfo() == instr.getManipulatorInfo());
  EXPECT_TRUE(child.getProfile() == instr.getProfile());
  EXPECT_TRUE(child.getPathProfile() == instr.getPathProfile());
  EXPECT_TRUE(child.getProfileOverrides() == instr.getProfileOverrides());
  EXPECT_TRUE(child.getPathProfileOverrides() == instr.getPathProfileOverrides());
  EXPECT_TRUE(child.getMoveType() == instr.getMoveType());
  EXPECT_TRUE(child.getWaypoint() == instr.getWaypoint());

  instr.getWaypoint().setName("CWP");
  MoveInstructionPoly child2 = instr.createChild();
  EXPECT_TRUE(child2.getWaypoint().getType() == instr.getWaypoint().getType());
}

}  // namespace details

template <typename T>
void runMoveInstructionTest()
{
  runInstructionInterfaceTest(MoveInstructionPoly(T()));

  details::runMoveInstructionInterfaceTest<T>();
  details::runMoveInstructionConstructorTest<T>();
  details::runMoveInstructionSettersTest<T>();
}
}  // namespace tesseract_planning::test_suite

#endif  // TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_POLY_UNIT_HPP
