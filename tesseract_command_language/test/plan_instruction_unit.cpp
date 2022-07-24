/**
 * @file plan_instruction_unit.cpp
 * @brief Contains unit tests for PlanInstruction
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
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>

using namespace tesseract_planning;

TEST(TesseractCommandLanguagePlanInstructionUnit, constructor)  // NOLINT
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(1, 2, 3);
  CartesianWaypoint wp(pose);

  // Minimum arguments
  {
    PlanInstruction instr(wp, PlanInstructionType::START);
    EXPECT_EQ(instr.getWaypoint(), wp);
    EXPECT_EQ(instr.getPlanType(), PlanInstructionType::START);
    EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_TRUE(instr.getPathProfile().empty());
    EXPECT_FALSE(instr.getDescription().empty());
  }

  {
    PlanInstruction instr(wp, PlanInstructionType::FREESPACE);
    EXPECT_EQ(instr.getWaypoint(), wp);
    EXPECT_EQ(instr.getPlanType(), PlanInstructionType::FREESPACE);
    EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_TRUE(instr.getPathProfile().empty());
    EXPECT_FALSE(instr.getDescription().empty());
  }

  {
    PlanInstruction instr(wp, PlanInstructionType::LINEAR);
    EXPECT_EQ(instr.getWaypoint(), wp);
    EXPECT_EQ(instr.getPlanType(), PlanInstructionType::LINEAR);
    EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_EQ(instr.getPathProfile(), DEFAULT_PROFILE_KEY);
    EXPECT_FALSE(instr.getDescription().empty());
  }

  // With plan profile
  {
    PlanInstruction instr(wp, PlanInstructionType::START, "TEST_PROFILE");
    EXPECT_EQ(instr.getWaypoint(), wp);
    EXPECT_EQ(instr.getPlanType(), PlanInstructionType::START);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_TRUE(instr.getPathProfile().empty());
    EXPECT_FALSE(instr.getDescription().empty());
  }

  {
    PlanInstruction instr(wp, PlanInstructionType::FREESPACE, "TEST_PROFILE");
    EXPECT_EQ(instr.getWaypoint(), wp);
    EXPECT_EQ(instr.getPlanType(), PlanInstructionType::FREESPACE);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_TRUE(instr.getPathProfile().empty());
    EXPECT_FALSE(instr.getDescription().empty());
  }

  {
    PlanInstruction instr(wp, PlanInstructionType::LINEAR, "TEST_PROFILE");
    EXPECT_EQ(instr.getWaypoint(), wp);
    EXPECT_EQ(instr.getPlanType(), PlanInstructionType::LINEAR);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PROFILE");
    EXPECT_FALSE(instr.getDescription().empty());
  }

  // With plan and path profile
  {
    PlanInstruction instr(wp, PlanInstructionType::START, "TEST_PROFILE", "TEST_PATH_PROFILE");
    EXPECT_EQ(instr.getWaypoint(), wp);
    EXPECT_EQ(instr.getPlanType(), PlanInstructionType::START);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
    EXPECT_FALSE(instr.getDescription().empty());
  }

  {
    PlanInstruction instr(wp, PlanInstructionType::FREESPACE, "TEST_PROFILE", "TEST_PATH_PROFILE");
    EXPECT_EQ(instr.getWaypoint(), wp);
    EXPECT_EQ(instr.getPlanType(), PlanInstructionType::FREESPACE);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
    EXPECT_FALSE(instr.getDescription().empty());
  }

  {
    PlanInstruction instr(wp, PlanInstructionType::LINEAR, "TEST_PROFILE", "TEST_PATH_PROFILE");
    EXPECT_EQ(instr.getWaypoint(), wp);
    EXPECT_EQ(instr.getPlanType(), PlanInstructionType::LINEAR);
    EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");
    EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");
    EXPECT_FALSE(instr.getDescription().empty());
  }
}

TEST(TesseractCommandLanguagePlanInstructionUnit, setters)  // NOLINT
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(1, 2, 3);
  CartesianWaypoint cwp(pose);

  PlanInstruction instr(cwp, PlanInstructionType::START);
  EXPECT_EQ(instr.getWaypoint(), cwp);
  EXPECT_EQ(instr.getPlanType(), PlanInstructionType::START);
  EXPECT_EQ(instr.getProfile(), DEFAULT_PROFILE_KEY);
  EXPECT_TRUE(instr.getPathProfile().empty());
  EXPECT_FALSE(instr.getDescription().empty());

  Eigen::VectorXd jv = Eigen::VectorXd::Ones(6);
  std::vector<std::string> jn = { "j1", "j2", "j3", "j4", "j5", "j6" };
  JointWaypoint jwp(jn, jv);
  instr.setWaypoint(jwp);
  EXPECT_EQ(instr.getWaypoint(), jwp);

  instr.setPlanType(PlanInstructionType::LINEAR);
  EXPECT_EQ(instr.getPlanType(), PlanInstructionType::LINEAR);

  instr.setProfile("TEST_PROFILE");
  EXPECT_EQ(instr.getProfile(), "TEST_PROFILE");

  instr.setPathProfile("TEST_PATH_PROFILE");
  EXPECT_EQ(instr.getPathProfile(), "TEST_PATH_PROFILE");

  instr.setDescription("This is a test.");
  EXPECT_EQ(instr.getDescription(), "This is a test.");
}

TEST(TesseractCommandLanguagePlanInstructionUnit, boostSerialization)  // NOLINT
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(1, 2, 3);
  CartesianWaypoint cwp(pose);

  PlanInstruction instr(cwp, PlanInstructionType::START);
  instr.setPlanType(PlanInstructionType::LINEAR);
  instr.setProfile("TEST_PROFILE");
  instr.setPathProfile("TEST_PATH_PROFILE");
  instr.setDescription("This is a test.");

  tesseract_common::Serialization::toArchiveFileXML<Instruction>(instr, "/tmp/plan_instruction_boost.xml");

  auto ninstr = tesseract_common::Serialization::fromArchiveFileXML<Instruction>("/tmp/plan_instruction_boost.xml")
                    .as<PlanInstruction>();

  EXPECT_TRUE(instr == ninstr);
  EXPECT_EQ(ninstr.getWaypoint(), cwp);
  EXPECT_EQ(ninstr.getPlanType(), PlanInstructionType::LINEAR);
  EXPECT_EQ(ninstr.getProfile(), "TEST_PROFILE");
  EXPECT_EQ(ninstr.getPathProfile(), "TEST_PATH_PROFILE");
  EXPECT_EQ(ninstr.getDescription(), "This is a test.");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
