/**
 * @file ompl_yaml_conversions_tests.cpp
 * @brief This contains unit test for the tesseract ompl planner yaml conversions
 *
 * @author Samantha Smith
 * @date July 29, 2025
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>

#include <ompl/util/RandomNumbers.h>

#include <functional>
#include <cmath>
#include <gtest/gtest.h>
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_common/profile_dictionary.h>

#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>

#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_environment/commands/add_link_command.h>

#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_move_profile.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/simple/interpolation.h>

#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_motion_planners/ompl/yaml_extensions.h>

using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_environment;
using namespace tesseract_geometry;
using namespace tesseract_kinematics;
using namespace tesseract_planning;

class OMPLYAMLTestFixture : public ::testing::Test
{
public:
  OMPLYAMLTestFixture() {}
  using ::testing::Test::Test;
};

TEST(OMPLYAMLTestFixture, OMPLYAMLSBLConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    SBLConfigurator configurator;

    EXPECT_NEAR(configurator.range, 0, 1e-6);
  }

  const std::string yaml_string = R"(range: 1.0
                                    )";
  {  // decode
    SBLConfigurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<SBLConfigurator>::decode(n, configurator);
    EXPECT_TRUE(success);
    EXPECT_NEAR(configurator.range, 1.0, 1e-6);
  }

  {  // encode
    SBLConfigurator configurator;
    configurator.range = 1.0;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<SBLConfigurator>::encode(configurator);
    EXPECT_NEAR(output_n["range"].as<double>(), 1.0, 1e-6);
  }
}

TEST(OMPLYAMLTestFixture, OMPLYAMLESTConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    ESTConfigurator configurator;

    EXPECT_NEAR(configurator.range, 0, 1e-6);
    EXPECT_NEAR(configurator.goal_bias, 0.05, 1e-6);
  }

  const std::string yaml_string = R"(
                                    range: 1.0
                                    goal_bias: 0.04
                                    )";
  {  // decode
    ESTConfigurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<ESTConfigurator>::decode(n, configurator);
    EXPECT_TRUE(success);
    EXPECT_NEAR(configurator.range, 1.0, 1e-6);
    EXPECT_NEAR(configurator.goal_bias, 0.04, 1e-6);
  }

  {  // encode
    ESTConfigurator configurator;
    configurator.range = 1.0;
    configurator.goal_bias = 0.04;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<ESTConfigurator>::encode(configurator);
    EXPECT_NEAR(output_n["range"].as<double>(), 1.0, 1e-6);
    EXPECT_NEAR(output_n["goal_bias"].as<double>(), 0.04, 1e-6);
  }
}

TEST(OMPLYAMLTestFixture, OMPLYAMLLBKPIECE1ConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    LBKPIECE1Configurator configurator;

    EXPECT_NEAR(configurator.range, 0, 1e-6);
    EXPECT_NEAR(configurator.border_fraction, 0.9, 1e-6);
    EXPECT_NEAR(configurator.min_valid_path_fraction, 0.5, 1e-6);
  }

  const std::string yaml_string = R"(
                                    range: 1.0
                                    border_fraction: 0.04
                                    min_valid_path_fraction: 0.05
                                    )";
  {  // decode
    LBKPIECE1Configurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<LBKPIECE1Configurator>::decode(n, configurator);
    EXPECT_TRUE(success);
    EXPECT_NEAR(configurator.range, 1.0, 1e-6);
    EXPECT_NEAR(configurator.border_fraction, 0.04, 1e-6);
    EXPECT_NEAR(configurator.min_valid_path_fraction, 0.05, 1e-6);
  }

  {  // encode
    LBKPIECE1Configurator configurator;
    configurator.range = 1.0;
    configurator.border_fraction = 0.04;
    configurator.min_valid_path_fraction = 0.05;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<LBKPIECE1Configurator>::encode(configurator);
    EXPECT_NEAR(output_n["range"].as<double>(), 1.0, 1e-6);
    EXPECT_NEAR(output_n["border_fraction"].as<double>(), 0.04, 1e-6);
    EXPECT_NEAR(output_n["min_valid_path_fraction"].as<double>(), 0.05, 1e-6);
  }
}

TEST(OMPLYAMLTestFixture, OMPLYAMLBKPIECE1ConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    BKPIECE1Configurator configurator;

    EXPECT_NEAR(configurator.range, 0, 1e-6);
    EXPECT_NEAR(configurator.border_fraction, 0.9, 1e-6);
    EXPECT_NEAR(configurator.failed_expansion_score_factor, 0.5, 1e-6);
    EXPECT_NEAR(configurator.min_valid_path_fraction, 0.5, 1e-6);
  }

  const std::string yaml_string = R"(
                                    range: 1.0
                                    border_fraction: 0.04
                                    failed_expansion_score_factor: 0.6
                                    min_valid_path_fraction: 0.05
                                    )";
  {  // decode
    BKPIECE1Configurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<BKPIECE1Configurator>::decode(n, configurator);
    EXPECT_TRUE(success);
    EXPECT_NEAR(configurator.range, 1.0, 1e-6);
    EXPECT_NEAR(configurator.border_fraction, 0.04, 1e-6);
    EXPECT_NEAR(configurator.failed_expansion_score_factor, 0.6, 1e-6);
    EXPECT_NEAR(configurator.min_valid_path_fraction, 0.05, 1e-6);
  }

  {  // encode
    BKPIECE1Configurator configurator;
    configurator.range = 1.0;
    configurator.border_fraction = 0.04;
    configurator.failed_expansion_score_factor = 0.6;
    configurator.min_valid_path_fraction = 0.05;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<BKPIECE1Configurator>::encode(configurator);
    EXPECT_NEAR(output_n["range"].as<double>(), 1.0, 1e-6);
    EXPECT_NEAR(output_n["border_fraction"].as<double>(), 0.04, 1e-6);
    EXPECT_NEAR(output_n["failed_expansion_score_factor"].as<double>(), 0.6, 1e-6);
    EXPECT_NEAR(output_n["min_valid_path_fraction"].as<double>(), 0.05, 1e-6);
  }
}

TEST(OMPLYAMLTestFixture, OMPLYAMLKPIECE1ConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    KPIECE1Configurator configurator;

    EXPECT_NEAR(configurator.range, 0, 1e-6);
    EXPECT_NEAR(configurator.goal_bias, 0.05, 1e-6);
    EXPECT_NEAR(configurator.border_fraction, 0.9, 1e-6);
    EXPECT_NEAR(configurator.failed_expansion_score_factor, 0.5, 1e-6);
    EXPECT_NEAR(configurator.min_valid_path_fraction, 0.5, 1e-6);
  }

  const std::string yaml_string = R"(
                                    range: 1.0
                                    goal_bias: 0.04
                                    border_fraction: 0.04
                                    failed_expansion_score_factor: 0.6
                                    min_valid_path_fraction: 0.05
                                    )";
  {  // decode
    KPIECE1Configurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<KPIECE1Configurator>::decode(n, configurator);
    EXPECT_TRUE(success);
    EXPECT_NEAR(configurator.range, 1.0, 1e-6);
    EXPECT_NEAR(configurator.goal_bias, 0.04, 1e-6);
    EXPECT_NEAR(configurator.border_fraction, 0.04, 1e-6);
    EXPECT_NEAR(configurator.failed_expansion_score_factor, 0.6, 1e-6);
    EXPECT_NEAR(configurator.min_valid_path_fraction, 0.05, 1e-6);
  }

  {  // encode
    KPIECE1Configurator configurator;
    configurator.range = 1.0;
    configurator.goal_bias = 0.04;
    configurator.border_fraction = 0.04;
    configurator.failed_expansion_score_factor = 0.6;
    configurator.min_valid_path_fraction = 0.05;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<KPIECE1Configurator>::encode(configurator);
    EXPECT_NEAR(output_n["range"].as<double>(), 1.0, 1e-6);
    EXPECT_NEAR(output_n["goal_bias"].as<double>(), 0.04, 1e-6);
    EXPECT_NEAR(output_n["border_fraction"].as<double>(), 0.04, 1e-6);
    EXPECT_NEAR(output_n["failed_expansion_score_factor"].as<double>(), 0.6, 1e-6);
    EXPECT_NEAR(output_n["min_valid_path_fraction"].as<double>(), 0.05, 1e-6);
  }
}

TEST(OMPLYAMLTestFixture, OMPLYAMLBiTRRTConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    BiTRRTConfigurator configurator;

    EXPECT_NEAR(configurator.range, 0, 1e-6);
    EXPECT_NEAR(configurator.temp_change_factor, 0.1, 1e-6);
    EXPECT_EQ(configurator.cost_threshold, std::numeric_limits<double>::infinity());
    EXPECT_NEAR(configurator.init_temperature, 100.0, 1e-6);
    EXPECT_NEAR(configurator.frontier_threshold, 0.0, 1e-6);
    EXPECT_NEAR(configurator.frontier_node_ratio, 0.1, 1e-6);
  }

  const std::string yaml_string = R"(
                                    range: 1.0
                                    temp_change_factor: 0.04
                                    cost_threshold: 0.04
                                    init_temperature: 0.6
                                    frontier_threshold: 0.05
                                    frontier_node_ratio: 0.5
                                    )";
  {  // decode
    BiTRRTConfigurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<BiTRRTConfigurator>::decode(n, configurator);
    EXPECT_TRUE(success);
    EXPECT_NEAR(configurator.range, 1.0, 1e-6);
    EXPECT_NEAR(configurator.temp_change_factor, 0.04, 1e-6);
    EXPECT_NEAR(configurator.cost_threshold, 0.04, 1e-6);
    EXPECT_NEAR(configurator.init_temperature, 0.6, 1e-6);
    EXPECT_NEAR(configurator.frontier_threshold, 0.05, 1e-6);
    EXPECT_NEAR(configurator.frontier_node_ratio, 0.5, 1e-6);
  }

  {  // encode
    BiTRRTConfigurator configurator;
    configurator.range = 1.0;
    configurator.temp_change_factor = 0.04;
    configurator.cost_threshold = 0.04;
    configurator.init_temperature = 0.6;
    configurator.frontier_threshold = 0.05;
    configurator.frontier_node_ratio = 0.5;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<BiTRRTConfigurator>::encode(configurator);
    EXPECT_NEAR(output_n["range"].as<double>(), 1.0, 1e-6);
    EXPECT_NEAR(output_n["temp_change_factor"].as<double>(), 0.04, 1e-6);
    EXPECT_NEAR(output_n["cost_threshold"].as<double>(), 0.04, 1e-6);
    EXPECT_NEAR(output_n["init_temperature"].as<double>(), 0.6, 1e-6);
    EXPECT_NEAR(output_n["frontier_threshold"].as<double>(), 0.05, 1e-6);
    EXPECT_NEAR(output_n["frontier_node_ratio"].as<double>(), 0.5, 1e-6);
  }
}

TEST(OMPLYAMLTestFixture, OMPLYAMLRRTConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    RRTConfigurator configurator;

    EXPECT_NEAR(configurator.range, 0, 1e-6);
    EXPECT_NEAR(configurator.goal_bias, 0.05, 1e-6);
  }

  const std::string yaml_string = R"(
                                    range: 1.0
                                    goal_bias: 0.04
                                    )";
  {  // decode
    RRTConfigurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<RRTConfigurator>::decode(n, configurator);
    EXPECT_TRUE(success);
    EXPECT_NEAR(configurator.range, 1.0, 1e-6);
    EXPECT_NEAR(configurator.goal_bias, 0.04, 1e-6);
  }

  {  // encode
    RRTConfigurator configurator;
    configurator.range = 1.0;
    configurator.goal_bias = 0.04;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<RRTConfigurator>::encode(configurator);
    EXPECT_NEAR(output_n["range"].as<double>(), 1.0, 1e-6);
    EXPECT_NEAR(output_n["goal_bias"].as<double>(), 0.04, 1e-6);
  }
}

TEST(OMPLYAMLTestFixture, OMPLYAMLRRTConnectConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    RRTConnectConfigurator configurator;

    EXPECT_NEAR(configurator.range, 0, 1e-6);
  }

  const std::string yaml_string = R"(range: 1.0
                                    )";
  {  // decode
    RRTConnectConfigurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<RRTConnectConfigurator>::decode(n, configurator);
    EXPECT_TRUE(success);
    EXPECT_NEAR(configurator.range, 1.0, 1e-6);
  }

  {  // encode
    RRTConnectConfigurator configurator;
    configurator.range = 1.0;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<RRTConnectConfigurator>::encode(configurator);
    EXPECT_NEAR(output_n["range"].as<double>(), 1.0, 1e-6);
  }
}

TEST(OMPLYAMLTestFixture, OMPLYAMLRRTstarConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    RRTstarConfigurator configurator;

    EXPECT_NEAR(configurator.range, 0, 1e-6);
    EXPECT_NEAR(configurator.goal_bias, 0.05, 1e-6);
    EXPECT_EQ(configurator.delay_collision_checking, true);
  }

  const std::string yaml_string = R"(
                                    range: 1.0
                                    goal_bias: 0.04
                                    delay_collision_checking: false
                                    )";
  {  // decode
    RRTstarConfigurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<RRTstarConfigurator>::decode(n, configurator);
    EXPECT_TRUE(success);
    EXPECT_NEAR(configurator.range, 1.0, 1e-6);
    EXPECT_NEAR(configurator.goal_bias, 0.04, 1e-6);
    EXPECT_EQ(configurator.delay_collision_checking, false);
  }

  {  // encode
    RRTstarConfigurator configurator;
    configurator.range = 1.0;
    configurator.goal_bias = 0.04;
    configurator.delay_collision_checking = false;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<RRTstarConfigurator>::encode(configurator);
    EXPECT_NEAR(output_n["range"].as<double>(), 1.0, 1e-6);
    EXPECT_NEAR(output_n["goal_bias"].as<double>(), 0.04, 1e-6);
    EXPECT_EQ(output_n["delay_collision_checking"].as<bool>(), false);
  }
}

TEST(OMPLYAMLTestFixture, OMPLYAMLTRRTConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    TRRTConfigurator configurator;

    EXPECT_NEAR(configurator.range, 0, 1e-6);
    EXPECT_NEAR(configurator.temp_change_factor, 2.0, 1e-6);
    EXPECT_NEAR(configurator.goal_bias, 0.05, 1e-6);
    EXPECT_NEAR(configurator.init_temperature, 10e-6, 1e-6);
    EXPECT_NEAR(configurator.frontier_threshold, 0.0, 1e-6);
    EXPECT_NEAR(configurator.frontier_node_ratio, 0.1, 1e-6);
  }

  const std::string yaml_string = R"(
                                    range: 1.0
                                    goal_bias: 0.03
                                    temp_change_factor: 0.04
                                    init_temperature: 0.6
                                    frontier_threshold: 0.05
                                    frontier_node_ratio: 0.5
                                    )";
  {  // decode
    TRRTConfigurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<TRRTConfigurator>::decode(n, configurator);
    EXPECT_TRUE(success);
    EXPECT_NEAR(configurator.range, 1.0, 1e-6);
    EXPECT_NEAR(configurator.goal_bias, 0.03, 1e-6);
    EXPECT_NEAR(configurator.temp_change_factor, 0.04, 1e-6);
    EXPECT_NEAR(configurator.init_temperature, 0.6, 1e-6);
    EXPECT_NEAR(configurator.frontier_threshold, 0.05, 1e-6);
    EXPECT_NEAR(configurator.frontier_node_ratio, 0.5, 1e-6);
  }

  {  // encode
    TRRTConfigurator configurator;
    configurator.range = 1.0;
    configurator.goal_bias = 0.03;
    configurator.temp_change_factor = 0.04;
    configurator.init_temperature = 0.6;
    configurator.frontier_threshold = 0.05;
    configurator.frontier_node_ratio = 0.5;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<TRRTConfigurator>::encode(configurator);
    EXPECT_NEAR(output_n["range"].as<double>(), 1.0, 1e-6);
    EXPECT_NEAR(output_n["goal_bias"].as<double>(), 0.03, 1e-6);
    EXPECT_NEAR(output_n["goal_bias"].as<double>(), 0.03, 1e-6);
    EXPECT_NEAR(output_n["temp_change_factor"].as<double>(), 0.04, 1e-6);
    EXPECT_NEAR(output_n["init_temperature"].as<double>(), 0.6, 1e-6);
    EXPECT_NEAR(output_n["frontier_threshold"].as<double>(), 0.05, 1e-6);
    EXPECT_NEAR(output_n["frontier_node_ratio"].as<double>(), 0.5, 1e-6);
  }
}

TEST(OMPLYAMLTestFixture, OMPLYAMLPRMConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    PRMConfigurator configurator;

    EXPECT_EQ(configurator.max_nearest_neighbors, 10);
  }

  const std::string yaml_string = R"(max_nearest_neighbors: 1
                                    )";
  {  // decode
    PRMConfigurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<PRMConfigurator>::decode(n, configurator);
    EXPECT_TRUE(success);
    EXPECT_EQ(configurator.max_nearest_neighbors, 1);
  }

  {  // encode
    PRMConfigurator configurator;
    configurator.max_nearest_neighbors = 1;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<PRMConfigurator>::encode(configurator);
    EXPECT_EQ(output_n["max_nearest_neighbors"].as<int>(), 1);
  }
}

TEST(OMPLYAMLTestFixture, OMPLYAMLPRMstarConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    PRMstarConfigurator configurator;
  }

  const std::string yaml_string = R"()";
  {  // decode
    PRMstarConfigurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<PRMstarConfigurator>::decode(n, configurator);
    EXPECT_TRUE(success);
  }

  {  // encode
    PRMstarConfigurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<PRMstarConfigurator>::encode(configurator);
  }
}

TEST(OMPLYAMLTestFixture, OMPLYAMLLazyPRMstarConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    LazyPRMstarConfigurator configurator;
  }

  const std::string yaml_string = R"()";
  {  // decode
    LazyPRMstarConfigurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<LazyPRMstarConfigurator>::decode(n, configurator);
    EXPECT_TRUE(success);
  }

  {  // encode
    LazyPRMstarConfigurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<LazyPRMstarConfigurator>::encode(configurator);
  }
}

TEST(OMPLYAMLTestFixture, OMPLYAMLSPARSConfiguratorConversionsUnit)  // NOLINT
{
  {  // Constructor
    SPARSConfigurator configurator;

    EXPECT_EQ(configurator.max_failures, 1000);
    EXPECT_NEAR(configurator.dense_delta_fraction, 0.001, 1e-6);
    EXPECT_NEAR(configurator.sparse_delta_fraction, 0.25, 1e-6);
    EXPECT_NEAR(configurator.stretch_factor, 3, 1e-6);
  }

  const std::string yaml_string = R"(
                                     max_failures: 1
                                     dense_delta_fraction: 0.5
                                     sparse_delta_fraction: 0.4
                                     stretch_factor: 0.3
                                    )";
  {  // decode
    SPARSConfigurator configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<SPARSConfigurator>::decode(n, configurator);
    EXPECT_TRUE(success);
    EXPECT_EQ(configurator.max_failures, 1);
    EXPECT_NEAR(configurator.dense_delta_fraction, 0.5, 1e-6);
    EXPECT_NEAR(configurator.sparse_delta_fraction, 0.4, 1e-6);
    EXPECT_NEAR(configurator.stretch_factor, 0.3, 1e-6);
  }

  {  // encode
    SPARSConfigurator configurator;
    configurator.max_failures = 1;
    configurator.dense_delta_fraction = 0.5;
    configurator.sparse_delta_fraction = 0.4;
    configurator.stretch_factor = 0.3;

    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<SPARSConfigurator>::encode(configurator);
    EXPECT_EQ(output_n["max_failures"].as<int>(), 1);
    EXPECT_NEAR(output_n["dense_delta_fraction"].as<double>(), 0.5, 1e-6);
    EXPECT_NEAR(output_n["sparse_delta_fraction"].as<double>(), 0.4, 1e-6);
    EXPECT_NEAR(output_n["stretch_factor"].as<double>(), 0.3, 1e-6);
  }
}

bool containsType(OMPLPlannerType type,
                  std::vector<std::shared_ptr<const tesseract_planning::OMPLPlannerConfigurator>> planners)
{
  for (const auto& planner : planners)
  {
    if (planner->getType() == type)
    {
      return true;
    }
  }

  return false;
}

bool containsPlanner(std::string planner_type, YAML::Node n)
{
  return std::any_of(n.begin(), n.end(), [planner_type](const YAML::Node& n) { return n[planner_type]; });
}

TEST(OMPLYAMLTestFixture, OMPLYAMLOMPLSolverConfigConversionsUnit)  // NOLINT
{
  {  // Constructor
    OMPLSolverConfig configurator;

    EXPECT_NEAR(configurator.planning_time, 5.0, 1e-6);
    EXPECT_EQ(configurator.max_solutions, 10);
    EXPECT_EQ(configurator.simplify, false);
    EXPECT_EQ(configurator.optimize, true);
    EXPECT_EQ(configurator.planners.size(), 0);
  }

  const std::string yaml_string = R"(
                                     planning_time: 2.0
                                     max_solutions: 4
                                     simplify: true
                                     optimize: false
                                     planners:
                                       - SBLConfigurator: {}
                                       - ESTConfigurator: {}
                                       - LBKPIECE1Configurator: {}
                                       - BKPIECE1Configurator: {}
                                       - KPIECE1Configurator: {}
                                       - BiTRRTConfigurator: {}
                                       - RRTConfigurator: {}
                                       - RRTConnectConfigurator: {}
                                       - RRTstarConfigurator: {}
                                       - TRRTConfigurator: {}
                                       - PRMConfigurator: {}
                                       - PRMstarConfigurator: {}
                                       - LazyPRMstarConfigurator: {}
                                       - SPARSConfigurator: {}
                                    )";

  {  // decode
    OMPLSolverConfig configurator;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<OMPLSolverConfig>::decode(n, configurator);
    EXPECT_TRUE(success);
    EXPECT_NEAR(configurator.planning_time, 2.0, 1e-6);
    EXPECT_EQ(configurator.max_solutions, 4);
    EXPECT_EQ(configurator.simplify, true);
    EXPECT_EQ(configurator.optimize, false);
    EXPECT_EQ(containsType(OMPLPlannerType::SBL, configurator.planners), true);
    EXPECT_EQ(containsType(OMPLPlannerType::EST, configurator.planners), true);
    EXPECT_EQ(containsType(OMPLPlannerType::LBKPIECE1, configurator.planners), true);
    EXPECT_EQ(containsType(OMPLPlannerType::BKPIECE1, configurator.planners), true);
    EXPECT_EQ(containsType(OMPLPlannerType::KPIECE1, configurator.planners), true);
    EXPECT_EQ(containsType(OMPLPlannerType::BiTRRT, configurator.planners), true);
    EXPECT_EQ(containsType(OMPLPlannerType::RRT, configurator.planners), true);
    EXPECT_EQ(containsType(OMPLPlannerType::RRTConnect, configurator.planners), true);
    EXPECT_EQ(containsType(OMPLPlannerType::RRTstar, configurator.planners), true);
    EXPECT_EQ(containsType(OMPLPlannerType::TRRT, configurator.planners), true);
    EXPECT_EQ(containsType(OMPLPlannerType::PRM, configurator.planners), true);
    EXPECT_EQ(containsType(OMPLPlannerType::PRMstar, configurator.planners), true);
    EXPECT_EQ(containsType(OMPLPlannerType::LazyPRMstar, configurator.planners), true);
    EXPECT_EQ(containsType(OMPLPlannerType::SPARS, configurator.planners), true);
  }

  {  // encode
    OMPLSolverConfig configurator;
    configurator.planning_time = 2.0;
    configurator.max_solutions = 4;
    configurator.simplify = true;
    configurator.optimize = false;
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::SBLConfigurator>()));
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::ESTConfigurator>()));
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::LBKPIECE1Configurator>()));
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::BKPIECE1Configurator>()));
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::KPIECE1Configurator>()));
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::BiTRRTConfigurator>()));
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::RRTConfigurator>()));
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::RRTConnectConfigurator>()));
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::RRTstarConfigurator>()));
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::TRRTConfigurator>()));
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::PRMConfigurator>()));
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::PRMstarConfigurator>()));
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::LazyPRMstarConfigurator>()));
    configurator.planners.push_back(std::static_pointer_cast<const tesseract_planning::OMPLPlannerConfigurator>(
        std::make_shared<tesseract_planning::SPARSConfigurator>()));

    YAML::Node n = YAML::Load(yaml_string);
    YAML::Node output_n = YAML::convert<OMPLSolverConfig>::encode(configurator);
    EXPECT_NEAR(output_n["planning_time"].as<double>(), 2.0, 1e-6);
    EXPECT_EQ(output_n["max_solutions"].as<int>(), 4);
    EXPECT_EQ(output_n["simplify"].as<bool>(), true);
    EXPECT_EQ(output_n["optimize"].as<bool>(), false);
    EXPECT_TRUE(containsPlanner("SBLConfigurator", output_n["planners"]));
    EXPECT_TRUE(containsPlanner("ESTConfigurator", output_n["planners"]));
    EXPECT_TRUE(containsPlanner("LBKPIECE1Configurator", output_n["planners"]));
    EXPECT_TRUE(containsPlanner("BKPIECE1Configurator", output_n["planners"]));
    EXPECT_TRUE(containsPlanner("KPIECE1Configurator", output_n["planners"]));
    EXPECT_TRUE(containsPlanner("BiTRRTConfigurator", output_n["planners"]));
    EXPECT_TRUE(containsPlanner("RRTConfigurator", output_n["planners"]));
    EXPECT_TRUE(containsPlanner("RRTConnectConfigurator", output_n["planners"]));
    EXPECT_TRUE(containsPlanner("RRTstarConfigurator", output_n["planners"]));
    EXPECT_TRUE(containsPlanner("TRRTConfigurator", output_n["planners"]));
    EXPECT_TRUE(containsPlanner("PRMConfigurator", output_n["planners"]));
    EXPECT_TRUE(containsPlanner("PRMstarConfigurator", output_n["planners"]));
    EXPECT_TRUE(containsPlanner("LazyPRMstarConfigurator", output_n["planners"]));
    EXPECT_TRUE(containsPlanner("SPARSConfigurator", output_n["planners"]));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
