/**
 * @file cartesian_waypoint_poly_unit.hpp
 * @brief Collection of unit tests for CartesianWaypointPoly
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
#ifndef TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_POLY_UNIT_HPP
#define TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_POLY_UNIT_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/uuid/random_generator.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning::test_suite
{
void runInstructionInterfaceTest(InstructionPoly inst, bool expect_uuid_null = true)
{
  auto uuid = inst.getUUID();
  EXPECT_EQ(uuid.is_nil(), expect_uuid_null);
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

  const std::string desc{ "tesseract_planning::test_suite::Instruction::Description" };
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

/**
 * @brief This will test the serialization of any waypoint implementation
 * @param waypoint The waypoint to test serialization
 */
inline void runInstructionSerializationTest(const InstructionPoly& inst)
{
  const std::string filepath = tesseract_common::getTempPath() + "instruction_poly_boost.xml";
  tesseract_common::Serialization::toArchiveFileXML<InstructionPoly>(inst, filepath);
  auto ninst = tesseract_common::Serialization::fromArchiveFileXML<InstructionPoly>(filepath);
  EXPECT_TRUE(inst == ninst);
}
}  // namespace tesseract_planning::test_suite

#endif  // TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_POLY_UNIT_HPP
