/**
 * @file trajopt_planner_yaml_conversions_tests.cpp
 * @brief This contains unit test for the tesseract trajopt planner yaml constructors
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

#include <tesseract_common/profile_plugin_factory.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_motion_planners/core/types.h>

#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <yaml-cpp/yaml.h>
#include <tesseract_motion_planners/trajopt/yaml_extensions.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>



using namespace tesseract_planning;
using namespace trajopt_common;

class TesseractPlanningTrajoptYAMLConversionsUnit : public ::testing::Test
{};

TEST_F(TesseractPlanningTrajoptYAMLConversionsUnit, EigenVectorFixedSizeYAMLConversion)  // NOLINT
{
  { // Encode test
    Eigen::Matrix<double, 3, 1> vec;
    vec << 1.1, 2.2, 3.3;

    YAML::Node node = YAML::convert<Eigen::Matrix<double, 3, 1>>::encode(vec);

    ASSERT_TRUE(node.IsSequence());
    EXPECT_EQ(node.size(), 3);
    EXPECT_NEAR(node[0].as<double>(), 1.1, 1e-6);
    EXPECT_NEAR(node[1].as<double>(), 2.2, 1e-6);
    EXPECT_NEAR(node[2].as<double>(), 3.3, 1e-6);
  }

  { // Decode test
    const std::string yaml_string = R"(
      - 4.4
      - 5.5
      - 6.6
    )";

    YAML::Node node = YAML::Load(yaml_string);

    Eigen::Matrix<double, 3, 1> vec;
    bool success = YAML::convert<Eigen::Matrix<double, 3, 1>>::decode(node, vec);

    EXPECT_TRUE(success);
    EXPECT_NEAR(vec(0), 4.4, 1e-6);
    EXPECT_NEAR(vec(1), 5.5, 1e-6);
    EXPECT_NEAR(vec(2), 6.6, 1e-6);
  }
}

TEST_F(TesseractPlanningTrajoptYAMLConversionsUnit, TrajOptCollisionConfigYAMLConversion)  // NOLINT
{
  const std::string yaml_string = R"(config:
                                      collision_margin_buffer: 0.5
                                      max_num_cnt: 5
                                )";
  { // decode

    TrajOptCollisionConfig config;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<TrajOptCollisionConfig>::decode(n["config"], config);
    EXPECT_TRUE(success);
    EXPECT_NEAR(config.collision_margin_buffer, 0.5, 1e-6);
    EXPECT_EQ(config.max_num_cnt, 5);
  }

  { // encode
    TrajOptCollisionConfig config;
    config.collision_margin_buffer = 0.5;
    config.max_num_cnt = 5;
    YAML::Node output_n = YAML::convert<TrajOptCollisionConfig>::encode(config);
    EXPECT_NEAR(output_n["collision_margin_buffer"].as<double>(), 0.5, 1e-6);
    EXPECT_EQ(output_n["max_num_cnt"].as<int>(), 5);
  }
}

TEST_F(TesseractPlanningTrajoptYAMLConversionsUnit, TrajOptCartesianWaypointConfigYAMLConversion)  // NOLINT
{
  const std::string yaml_string = R"(config:
                                      enabled: false
                                      use_tolerance_override: true
                                      lower_tolerance:
                                        - 1.0
                                        - 2.0
                                        - 3.0
                                        - 4.0
                                        - 5.0
                                        - 6.0
                                      upper_tolerance:
                                        - 1.0
                                        - 2.0
                                        - 3.0
                                        - 4.0
                                        - 5.0
                                        - 6.0
                                      coeff:
                                        - 1.0
                                        - 2.0
                                        - 3.0
                                        - 4.0
                                        - 5.0
                                        - 6.0
                                )";
  
  { // decode

    TrajOptCartesianWaypointConfig config;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<TrajOptCartesianWaypointConfig>::decode(n["config"], config);
    EXPECT_TRUE(success);
    EXPECT_EQ(config.enabled, false);
    EXPECT_EQ(config.use_tolerance_override, true);
    for (int i = 0; i < 6; i ++) {
      EXPECT_NEAR(config.lower_tolerance[i], (double)(i+1), 1e-6);
      EXPECT_NEAR(config.upper_tolerance[i], (double)(i+1), 1e-6);
      EXPECT_NEAR(config.coeff[i], (double)(i+1), 1e-6);
    }
  }

  { // encode
    TrajOptCartesianWaypointConfig config;
    config.enabled = false;
    config.use_tolerance_override = true;
    config.lower_tolerance = Eigen::VectorXd::Ones(6);
    config.upper_tolerance = Eigen::VectorXd::Ones(6);
    config.coeff = Eigen::VectorXd::Ones(6);
    YAML::Node output_n = YAML::convert<TrajOptCartesianWaypointConfig>::encode(config);
    EXPECT_EQ(output_n["enabled"].as<bool>(), false);
    EXPECT_EQ(output_n["use_tolerance_override"].as<bool>(), true);
    YAML::Node lower_tol_node = output_n["lower_tolerance"];
    YAML::Node upper_tol_node = output_n["upper_tolerance"];
    YAML::Node coeff_node = output_n["coeff"];

    ASSERT_EQ(lower_tol_node.size(), 6);
    ASSERT_EQ(upper_tol_node.size(), 6);
    ASSERT_EQ(coeff_node.size(), 6);

    for (int i = 0; i < 6; ++i) {
      EXPECT_NEAR(config.lower_tolerance[i], lower_tol_node[i].as<double>(), 1e-6);
      EXPECT_NEAR(config.upper_tolerance[i], upper_tol_node[i].as<double>(), 1e-6);
      EXPECT_NEAR(config.coeff[i], coeff_node[i].as<double>(), 1e-6);
    }
  }
}

TEST_F(TesseractPlanningTrajoptYAMLConversionsUnit, TrajOptJointWaypointConfigYAMLConversion)  // NOLINT
{
  const std::string yaml_string = R"(config:
                                      enabled: false
                                      use_tolerance_override: true
                                      lower_tolerance:
                                        - 1.0
                                        - 2.0
                                        - 3.0
                                        - 4.0
                                        - 5.0
                                        - 6.0
                                      upper_tolerance:
                                        - 1.0
                                        - 2.0
                                        - 3.0
                                        - 4.0
                                        - 5.0
                                        - 6.0
                                      coeff:
                                        - 1.0
                                        - 2.0
                                        - 3.0
                                        - 4.0
                                        - 5.0
                                        - 6.0
                                )";
  
  { // decode

    TrajOptJointWaypointConfig config;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<TrajOptJointWaypointConfig>::decode(n["config"], config);
    EXPECT_TRUE(success);
    EXPECT_EQ(config.enabled, false);
    EXPECT_EQ(config.use_tolerance_override, true);
    for (int i = 0; i < 6; i ++) {
      EXPECT_NEAR(config.lower_tolerance[i], (double)(i+1), 1e-6);
      EXPECT_NEAR(config.upper_tolerance[i], (double)(i+1), 1e-6);
      EXPECT_NEAR(config.coeff[i], (double)(i+1), 1e-6);
    }
  }

  { // encode
    TrajOptJointWaypointConfig config;
    config.enabled = false;
    config.use_tolerance_override = true;
    config.lower_tolerance = Eigen::VectorXd::Ones(6);
    config.upper_tolerance = Eigen::VectorXd::Ones(6);
    config.coeff = Eigen::VectorXd::Ones(6);
    YAML::Node output_n = YAML::convert<TrajOptJointWaypointConfig>::encode(config);
    EXPECT_EQ(output_n["enabled"].as<bool>(), false);
    EXPECT_EQ(output_n["use_tolerance_override"].as<bool>(), true);
    YAML::Node lower_tol_node = output_n["lower_tolerance"];
    YAML::Node upper_tol_node = output_n["upper_tolerance"];
    YAML::Node coeff_node = output_n["coeff"];

    ASSERT_EQ(lower_tol_node.size(), 6);
    ASSERT_EQ(upper_tol_node.size(), 6);
    ASSERT_EQ(coeff_node.size(), 6);

    for (int i = 0; i < 6; ++i) {
      EXPECT_NEAR(config.lower_tolerance[i], lower_tol_node[i].as<double>(), 1e-6);
      EXPECT_NEAR(config.upper_tolerance[i], upper_tol_node[i].as<double>(), 1e-6);
      EXPECT_NEAR(config.coeff[i], coeff_node[i].as<double>(), 1e-6);
    }
  }
}

TEST_F(TesseractPlanningTrajoptYAMLConversionsUnit, TrajOptDefaultCompositeProfileYAMLConversion)  // NOLINT
{
  { // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptDefaultCompositeProfile profile(n["config"], plugin_factory);
    TrajOptDefaultCompositeProfile def_constructor;
    EXPECT_EQ(profile.smooth_velocities, def_constructor.smooth_velocities);
    EXPECT_EQ(profile.smooth_accelerations, def_constructor.smooth_accelerations);
    EXPECT_EQ(profile.smooth_jerks, def_constructor.smooth_jerks);
    EXPECT_EQ(profile.avoid_singularity, def_constructor.avoid_singularity);
  }

    { // Constructor
    const std::string yaml_string = R"(config:
                                        smooth_velocities: false
                                        smooth_accelerations: true
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptDefaultCompositeProfile profile(n["config"], plugin_factory);
    TrajOptDefaultCompositeProfile def_constructor;
    EXPECT_EQ(profile.smooth_velocities, false);
    EXPECT_EQ(profile.smooth_accelerations, true);
    EXPECT_EQ(profile.smooth_jerks, def_constructor.smooth_jerks);
    EXPECT_EQ(profile.avoid_singularity, def_constructor.avoid_singularity);
  }
}


TEST_F(TesseractPlanningTrajoptYAMLConversionsUnit, TrajOptDefaultMoveProfileYAMLConversion)  // NOLINT
{
  { // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptDefaultMoveProfile profile(n["config"], plugin_factory);
    TrajOptDefaultMoveProfile def_constructor;
    EXPECT_EQ(profile.cartesian_cost_config.enabled, def_constructor.cartesian_cost_config.enabled);
    EXPECT_EQ(profile.cartesian_constraint_config.enabled, def_constructor.cartesian_constraint_config.enabled);
    EXPECT_EQ(profile.joint_cost_config.enabled, def_constructor.joint_cost_config.enabled);
    EXPECT_EQ(profile.joint_constraint_config.enabled, def_constructor.joint_constraint_config.enabled);
  }

  { // Constructor
    const std::string yaml_string = R"(config:
                                        cartesian_cost_config:
                                          enabled: true
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptDefaultMoveProfile profile(n["config"], plugin_factory);
    TrajOptDefaultMoveProfile def_constructor;
    EXPECT_EQ(profile.cartesian_cost_config.enabled, true);
    EXPECT_EQ(profile.cartesian_constraint_config.enabled, def_constructor.cartesian_constraint_config.enabled);
    EXPECT_EQ(profile.joint_cost_config.enabled, def_constructor.joint_cost_config.enabled);
    EXPECT_EQ(profile.joint_constraint_config.enabled, def_constructor.joint_constraint_config.enabled);
  }

}

TEST_F(TesseractPlanningTrajoptYAMLConversionsUnit, TrajOptOSQPSolverProfileYAMLConversion)  // NOLINT
{
  { // Constructor
    const std::string yaml_string = R"(config:
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptOSQPSolverProfile profile(n["config"], plugin_factory);
    TrajOptOSQPSolverProfile def_constructor;
    EXPECT_EQ(profile.update_workspace, def_constructor.update_workspace);
    EXPECT_EQ(profile.settings.rho, def_constructor.settings.rho);
  }

  { // Constructor
    const std::string yaml_string = R"(config:
                                        update_workspace: false
                                    )";
    YAML::Node n = YAML::Load(yaml_string);
    tesseract_common::ProfilePluginFactory plugin_factory;
    TrajOptOSQPSolverProfile profile(n["config"], plugin_factory);
    TrajOptOSQPSolverProfile def_constructor;
    EXPECT_EQ(profile.update_workspace, false);
    EXPECT_EQ(profile.settings.rho, def_constructor.settings.rho);
  }

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
