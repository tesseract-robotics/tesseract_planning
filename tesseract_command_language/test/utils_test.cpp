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
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/utils.h>

using namespace tesseract_planning;

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
      sub_composite.appendInstruction(sub_sub_composite);
    }
    composite.appendInstruction(sub_composite);
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
    flattenFilterFn filter = [](const InstructionPoly&, const CompositeInstruction&, bool) { return true; };
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

TEST(TesseractCommandLanguageUtilsUnit, flattenToPattern)  // NOLINT
{
  // Create a composite
  CompositeInstruction composite;
  composite.setDescription("flattenToPattern Unit: Composite");
  CompositeInstruction pattern;
  std::size_t i_max = 4;
  std::size_t j_max = 3;
  std::size_t k_max = 2;

  // These loops create a nested set of composites. Note that the pattern does not have the last nesting
  for (std::size_t i = 0; i < i_max; i++)
  {
    CompositeInstruction sub_composite;
    sub_composite.setDescription("sub_composite_" + std::to_string(i));
    CompositeInstruction sub_pattern;
    sub_pattern.setDescription("sub_pattern_" + std::to_string(i));

    for (std::size_t j = 0; j < j_max; j++)
    {
      CompositeInstruction sub_sub_composite;
      sub_sub_composite.setDescription("sub_sub_composite_" + std::to_string(j));

      CartesianWaypointPoly wp{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
      MoveInstruction pattern_instruction(wp, MoveInstructionType::LINEAR);
      pattern_instruction.setDescription("pattern_instruction_" + std::to_string(i) + "_" + std::to_string(j));
      sub_pattern.appendMoveInstruction(pattern_instruction);
      for (std::size_t k = 0; k < k_max; k++)
      {
        CartesianWaypointPoly wp{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
        MoveInstruction instruction(wp, MoveInstructionType::LINEAR);
        instruction.setDescription("instruction_" + std::to_string(i) + "_" + std::to_string(j) + "_" +
                                   std::to_string(k));
        sub_sub_composite.appendMoveInstruction(instruction);
      }
      sub_composite.appendInstruction(sub_sub_composite);
    }
    composite.appendInstruction(sub_composite);
    pattern.appendInstruction(sub_pattern);
  }

  // flattenToPattern(composite, pattern)
  {
    if (DEBUG)
    {
      std::cout << "Composite: " << std::endl;
      composite.print();
      std::cout << "Pattern: " << std::endl;
      pattern.print();
    }
    // flatten the composite
    std::vector<std::reference_wrapper<InstructionPoly>> flattened = composite.flattenToPattern(pattern);
    EXPECT_EQ(flattened.size(), i_max * j_max);

    // Now change something in the flattened composite
    int num_composites = 0;
    for (std::size_t i = 0; i < flattened.size(); i++)
    {
      if (flattened[i].get().isCompositeInstruction())
        num_composites++;
      flattened[i].get().setDescription("test_" + std::to_string(i));
    }
    EXPECT_EQ(num_composites, 12);

    if (DEBUG)
    {
      std::cout << "flattened Composite: " << std::endl;
      for (const auto& i : flattened)
        i.get().print();
    }

    // Now make sure the original changed
    std::size_t cumulative = 0;
    for (std::size_t i = 0; i < i_max; i++)
    {
      for (std::size_t j = 0; j < j_max; j++)
      {
        EXPECT_EQ("test_" + std::to_string(cumulative), composite[i].as<CompositeInstruction>().at(j).getDescription());
        cumulative++;
      }
    }
  }

  // flattenToPattern(composte, pattern, true)
  {
    if (DEBUG)
    {
      std::cout << "Composite after changing description: " << std::endl;
      composite.print();
    }

    if (DEBUG)
    {
      std::cout << "Composite: " << std::endl;
      composite.print();
      std::cout << "Pattern: " << std::endl;
      pattern.print();
    }
    // flatten the composite
    flattenFilterFn filter = [](const InstructionPoly&, const CompositeInstruction&, bool) { return true; };
    std::vector<std::reference_wrapper<InstructionPoly>> flattened = composite.flattenToPattern(pattern, filter);
    EXPECT_EQ(flattened.size(),
              i_max * j_max + 4);  // Add 4 for the extra composite instructions that would be flattened

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
      std::cout << "flattened Composite: " << std::endl;
      for (const auto& i : flattened)
        i.get().print();
    }

    // Now make sure the original changed
    std::size_t cumulative = 0;
    for (std::size_t i = 0; i < i_max; i++)
    {
      EXPECT_EQ("test_" + std::to_string(cumulative), composite[i].as<CompositeInstruction>().getDescription());
      cumulative++;
      for (std::size_t j = 0; j < j_max; j++)
      {
        EXPECT_EQ("test_" + std::to_string(cumulative), composite[i].as<CompositeInstruction>().at(j).getDescription());
        cumulative++;
      }
    }

    if (DEBUG)
    {
      std::cout << "Composite after changing description: " << std::endl;
      composite.print();
    }
  }
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

TEST(TesseractCommandLanguageUtilsUnit, generateSkeletonSeed)  // NOLINT
{
  // Create a composite
  CompositeInstruction composite;
  composite.setDescription("generateSkeletonSeed: Composite");
  composite.setProfile("COMPOSITE_PROFILE");
  std::size_t i_max = 4;

  for (std::size_t i = 0; i < i_max; i++)
  {
    CartesianWaypointPoly wp{ CartesianWaypoint(Eigen::Isometry3d::Identity()) };
    MoveInstruction instruction(wp, MoveInstructionType::LINEAR);
    instruction.setDescription("MoveInstruction");
    instruction.setProfile("CART_PROFILE");
    composite.appendMoveInstruction(instruction);
  }

  // generateSkeletonSeed
  auto skeleton = generateSkeletonSeed(composite);

  // Check that high level composite is correct
  EXPECT_EQ(skeleton.getProfile(), composite.getProfile());
  EXPECT_EQ(skeleton.getOrder(), composite.getOrder());
  EXPECT_EQ(skeleton.getDescription(), composite.getDescription());
  EXPECT_EQ(skeleton.getManipulatorInfo(), composite.getManipulatorInfo());
  // TODO: Test startInstruction

  // Check that each MoveInstruction has been turned into a CompositeInstruction
  // Check that CompositeInstructions are recursively handled (TODO)
  // Check that non-MoveInstructions are passed through (TODO)
  ASSERT_EQ(skeleton.size(), composite.size());
  for (std::size_t i = 0; i < i_max; i++)
  {
    const auto& skeleton_i = skeleton[i];
    const auto& composite_i = composite[i];
    if (composite_i.isMoveInstruction())
    {
      ASSERT_TRUE(skeleton_i.isCompositeInstruction());
      const auto& cast = skeleton_i.as<CompositeInstruction>();

      EXPECT_EQ(cast.getProfile(), composite_i.as<MoveInstructionPoly>().getProfile());
      EXPECT_EQ(cast.getOrder(), CompositeInstructionOrder::ORDERED);
      EXPECT_EQ(cast.getDescription(), "MoveInstruction");
      EXPECT_EQ(cast.getManipulatorInfo(), composite_i.as<MoveInstructionPoly>().getManipulatorInfo());
    }
    else
    {
      EXPECT_EQ(skeleton_i.getType(), composite_i.getType());
    }
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
