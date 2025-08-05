/**
 * @file simple_planner_yaml_conversions_tests.cpp
 * @brief This contains unit test for the tesseract simple planner yaml constructors
 *
 * @author Samantha Smith
 * @date August 25, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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

#include "simple_planner_test_utils.hpp"
#include <tesseract_common/profile_plugin_factory.h>

#include <tesseract_common/types.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_no_ik_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_assign_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_assign_no_ik_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_move_profile.h>

#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <yaml-cpp/yaml.h>

using namespace tesseract_planning;

class TesseractPlanningSimplePlannerYAMLConversionsUnit : public TesseractPlanningSimplePlannerUnit
{
};

TEST_F(TesseractPlanningSimplePlannerYAMLConversionsUnit, SimplePlannerFixedSizeAssignMoveProfileYAML)  // NOLINT
{
  { // Constructor
    const std::string yaml_string = R"(config:
                                        freespace_steps: 6
                                        linear_steps: 5
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerFixedSizeAssignMoveProfile profile(n["config"], plugin_factory);
    EXPECT_EQ(profile.freespace_steps, 6);
    EXPECT_EQ(profile.linear_steps, 5);
  }

  { // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerFixedSizeAssignMoveProfile profile(n["config"], plugin_factory);
    EXPECT_EQ(profile.freespace_steps, 10);
    EXPECT_EQ(profile.linear_steps, 10);
  }
}

TEST_F(TesseractPlanningSimplePlannerYAMLConversionsUnit, SimplePlannerFixedSizeAssignNoIKMoveProfileYAML)  // NOLINT
{
  { // Constructor
    const std::string yaml_string = R"(config:
                                        freespace_steps: 6
                                        linear_steps: 5
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerFixedSizeAssignNoIKMoveProfile profile(n["config"], plugin_factory);
    EXPECT_EQ(profile.freespace_steps, 6);
    EXPECT_EQ(profile.linear_steps, 5);
  }

  { // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerFixedSizeAssignNoIKMoveProfile profile(n["config"], plugin_factory);
    EXPECT_EQ(profile.freespace_steps, 10);
    EXPECT_EQ(profile.linear_steps, 10);
  }
}

TEST_F(TesseractPlanningSimplePlannerYAMLConversionsUnit, SimplePlannerFixedSizeMoveProfileYAML)  // NOLINT
{
  { // Constructor
    const std::string yaml_string = R"(config:
                                        freespace_steps: 6
                                        linear_steps: 5
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerFixedSizeMoveProfile profile(n["config"], plugin_factory);
    EXPECT_EQ(profile.freespace_steps, 6);
    EXPECT_EQ(profile.linear_steps, 5);
  }

  { // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerFixedSizeMoveProfile profile(n["config"], plugin_factory);
    EXPECT_EQ(profile.freespace_steps, 10);
    EXPECT_EQ(profile.linear_steps, 10);
  }
}

TEST_F(TesseractPlanningSimplePlannerYAMLConversionsUnit, SimplePlannerLVSAssignMoveProfileYAML)  // NOLINT
{
  { // Constructor
    const std::string yaml_string = R"(config:
                                        state_longest_valid_segment_length: 0.5
                                        translation_longest_valid_segment_length: 0.2
                                        rotation_longest_valid_segment_length: 0.4
                                        min_steps: 6
                                        max_steps: 5
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerLVSAssignMoveProfile profile(n["config"], plugin_factory);
    EXPECT_NEAR(profile.state_longest_valid_segment_length, 0.5, 1e-6);
    EXPECT_NEAR(profile.translation_longest_valid_segment_length, 0.2, 1e-6);
    EXPECT_NEAR(profile.rotation_longest_valid_segment_length, 0.4, 1e-6);
    EXPECT_EQ(profile.min_steps, 6);
    EXPECT_EQ(profile.max_steps, 5);
  }

  { // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerLVSAssignMoveProfile profile(n["config"], plugin_factory);
    EXPECT_NEAR(profile.state_longest_valid_segment_length, 5 * M_PI / 180, 1e-6);
    EXPECT_NEAR(profile.translation_longest_valid_segment_length, 0.1, 1e-6);
    EXPECT_NEAR(profile.rotation_longest_valid_segment_length, 5 * M_PI / 180, 1e-6);
    EXPECT_EQ(profile.min_steps, 1);
    EXPECT_EQ(profile.max_steps, std::numeric_limits<int>::max());
  }
}

TEST_F(TesseractPlanningSimplePlannerYAMLConversionsUnit, SimplePlannerLVSAssignNoIKMoveProfileYAML)  // NOLINT
{
  { // Constructor
    const std::string yaml_string = R"(config:
                                        state_longest_valid_segment_length: 0.5
                                        translation_longest_valid_segment_length: 0.2
                                        rotation_longest_valid_segment_length: 0.4
                                        min_steps: 6
                                        max_steps: 5
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerLVSAssignNoIKMoveProfile profile(n["config"], plugin_factory);
    EXPECT_NEAR(profile.state_longest_valid_segment_length, 0.5, 1e-6);
    EXPECT_NEAR(profile.translation_longest_valid_segment_length, 0.2, 1e-6);
    EXPECT_NEAR(profile.rotation_longest_valid_segment_length, 0.4, 1e-6);
    EXPECT_EQ(profile.min_steps, 6);
    EXPECT_EQ(profile.max_steps, 5);
  }

  { // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerLVSAssignNoIKMoveProfile profile(n["config"], plugin_factory);
    EXPECT_NEAR(profile.state_longest_valid_segment_length, 5 * M_PI / 180, 1e-6);
    EXPECT_NEAR(profile.translation_longest_valid_segment_length, 0.1, 1e-6);
    EXPECT_NEAR(profile.rotation_longest_valid_segment_length, 5 * M_PI / 180, 1e-6);
    EXPECT_EQ(profile.min_steps, 1);
    EXPECT_EQ(profile.max_steps, std::numeric_limits<int>::max());
  }
}

TEST_F(TesseractPlanningSimplePlannerYAMLConversionsUnit, SimplePlannerLVSMoveProfileYAML)  // NOLINT
{
  { // Constructor
    const std::string yaml_string = R"(config:
                                        state_longest_valid_segment_length: 0.5
                                        translation_longest_valid_segment_length: 0.2
                                        rotation_longest_valid_segment_length: 0.4
                                        min_steps: 6
                                        max_steps: 5
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerLVSMoveProfile profile(n["config"], plugin_factory);
    EXPECT_NEAR(profile.state_longest_valid_segment_length, 0.5, 1e-6);
    EXPECT_NEAR(profile.translation_longest_valid_segment_length, 0.2, 1e-6);
    EXPECT_NEAR(profile.rotation_longest_valid_segment_length, 0.4, 1e-6);
    EXPECT_EQ(profile.min_steps, 6);
    EXPECT_EQ(profile.max_steps, 5);
  }

  { // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerLVSMoveProfile profile(n["config"], plugin_factory);
    EXPECT_NEAR(profile.state_longest_valid_segment_length, 5 * M_PI / 180, 1e-6);
    EXPECT_NEAR(profile.translation_longest_valid_segment_length, 0.1, 1e-6);
    EXPECT_NEAR(profile.rotation_longest_valid_segment_length, 5 * M_PI / 180, 1e-6);
    EXPECT_EQ(profile.min_steps, 1);
    EXPECT_EQ(profile.max_steps, std::numeric_limits<int>::max());
  }
}

TEST_F(TesseractPlanningSimplePlannerYAMLConversionsUnit, SimplePlannerLVSNoIKMoveProfileYAML)  // NOLINT
{
  { // Constructor
    const std::string yaml_string = R"(config:
                                        state_longest_valid_segment_length: 0.5
                                        translation_longest_valid_segment_length: 0.2
                                        rotation_longest_valid_segment_length: 0.4
                                        min_steps: 6
                                        max_steps: 5
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerLVSNoIKMoveProfile profile(n["config"], plugin_factory);
    EXPECT_NEAR(profile.state_longest_valid_segment_length, 0.5, 1e-6);
    EXPECT_NEAR(profile.translation_longest_valid_segment_length, 0.2, 1e-6);
    EXPECT_NEAR(profile.rotation_longest_valid_segment_length, 0.4, 1e-6);
    EXPECT_EQ(profile.min_steps, 6);
    EXPECT_EQ(profile.max_steps, 5);
  }

  { // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    SimplePlannerLVSNoIKMoveProfile profile(n["config"], plugin_factory);
    EXPECT_NEAR(profile.state_longest_valid_segment_length, 5 * M_PI / 180, 1e-6);
    EXPECT_NEAR(profile.translation_longest_valid_segment_length, 0.1, 1e-6);
    EXPECT_NEAR(profile.rotation_longest_valid_segment_length, 5 * M_PI / 180, 1e-6);
    EXPECT_EQ(profile.min_steps, 1);
    EXPECT_EQ(profile.max_steps, std::numeric_limits<int>::max());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
