/**
 * @file trajopt_yaml_conversions_tests.cpp
 * @brief This contains unit test for TrajOpt YAML conversions
 *
 * @author Tyler Marr
 * @date August 8, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Tyler Marr, Confinity Robotics
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
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/yaml_extensions.h>
#include <tesseract_collision/core/yaml_extensions.h>
#include <tesseract_motion_planners/trajopt/yaml_extensions.h>
#include <trajopt_common/yaml_extensions.h>

#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>
#include <trajopt_common/collision_types.h>

using namespace tesseract_planning;

class TrajOptYAMLTestFixture : public ::testing::Test
{
public:
  TrajOptYAMLTestFixture() = default;
  using ::testing::Test::Test;
};

TEST(TrajOptYAMLTestFixture, TrajOptCartesianWaypointConfigConversionsUnit)  // NOLINT
{
  {  // Constructor defaults
    TrajOptCartesianWaypointConfig c;
    EXPECT_TRUE(c.enabled);
    EXPECT_FALSE(c.use_tolerance_override);
    EXPECT_EQ(c.lower_tolerance.size(), 6);
    EXPECT_EQ(c.upper_tolerance.size(), 6);
    EXPECT_EQ(c.coeff.size(), 6);
    for (int i = 0; i < 6; ++i)
    {
      EXPECT_NEAR(c.lower_tolerance[i], 0.0, 1e-8);
      EXPECT_NEAR(c.upper_tolerance[i], 0.0, 1e-8);
      EXPECT_NEAR(c.coeff[i], 5.0, 1e-8);
    }
  }

  const std::string yaml_string = R"(
    enabled: false
    use_tolerance_override: true
    lower_tolerance: [-0.1, -0.2, -0.3, -0.01, -0.02, -0.03]
    upper_tolerance: [0.1, 0.2, 0.3, 0.01, 0.02, 0.03]
    coeff: [1, 2, 3, 4, 5, 6]
  )";

  {  // decode
    TrajOptCartesianWaypointConfig c;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<TrajOptCartesianWaypointConfig>::decode(n, c);
    EXPECT_TRUE(success);
    EXPECT_FALSE(c.enabled);
    EXPECT_TRUE(c.use_tolerance_override);
    for (int i = 0; i < 6; ++i)
    {
      EXPECT_NEAR(c.lower_tolerance[i], (i < 3 ? -(i + 1) / 10.0 : -(i - 2) / 100.0), 1e-8);
      EXPECT_NEAR(c.upper_tolerance[i], (i < 3 ? (i + 1) / 10.0 : (i - 2) / 100.0), 1e-8);
      EXPECT_NEAR(c.coeff[i], static_cast<double>(i + 1), 1e-8);
    }
  }

  {  // encode
    TrajOptCartesianWaypointConfig c;
    c.enabled = false;
    c.use_tolerance_override = true;
    for (int i = 0; i < 6; ++i)
    {
      c.lower_tolerance[i] = (i < 3 ? -(i + 1) / 10.0 : -(i - 2) / 100.0);
      c.upper_tolerance[i] = (i < 3 ? (i + 1) / 10.0 : (i - 2) / 100.0);
      c.coeff[i] = static_cast<double>(i + 1);
    }
    YAML::Node out = YAML::convert<TrajOptCartesianWaypointConfig>::encode(c);
    EXPECT_FALSE(out["enabled"].as<bool>());
    EXPECT_TRUE(out["use_tolerance_override"].as<bool>());
    ASSERT_TRUE(out["lower_tolerance"].IsSequence());
    ASSERT_TRUE(out["upper_tolerance"].IsSequence());
    ASSERT_TRUE(out["coeff"].IsSequence());
    for (int i = 0; i < 6; ++i)
    {
      EXPECT_NEAR(out["lower_tolerance"][i].as<double>(), c.lower_tolerance[i], 1e-8);
      EXPECT_NEAR(out["upper_tolerance"][i].as<double>(), c.upper_tolerance[i], 1e-8);
      EXPECT_NEAR(out["coeff"][i].as<double>(), c.coeff[i], 1e-8);
    }
  }
}

TEST(TrajOptYAMLTestFixture, TrajOptJointWaypointConfigConversionsUnit)  // NOLINT
{
  {  // Constructor defaults
    TrajOptJointWaypointConfig c;
    EXPECT_TRUE(c.enabled);
    EXPECT_FALSE(c.use_tolerance_override);
    EXPECT_EQ(c.lower_tolerance.size(), 0);
    EXPECT_EQ(c.upper_tolerance.size(), 0);
    EXPECT_EQ(c.coeff.size(), 1);
    EXPECT_NEAR(c.coeff[0], 5.0, 1e-8);
  }

  const std::string yaml_string = R"(
    enabled: false
    use_tolerance_override: true
    lower_tolerance: [-0.1, -0.2, -0.3]
    upper_tolerance: [0.1, 0.2, 0.3]
    coeff: [5, 6, 7]
  )";

  {  // decode
    TrajOptJointWaypointConfig c;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<TrajOptJointWaypointConfig>::decode(n, c);
    EXPECT_TRUE(success);
    EXPECT_FALSE(c.enabled);
    EXPECT_TRUE(c.use_tolerance_override);
    ASSERT_EQ(c.lower_tolerance.size(), 3);
    ASSERT_EQ(c.upper_tolerance.size(), 3);
    ASSERT_EQ(c.coeff.size(), 3);
    for (int i = 0; i < 3; ++i)
    {
      EXPECT_NEAR(c.lower_tolerance[i], -(i + 1) / 10.0, 1e-8);
      EXPECT_NEAR(c.upper_tolerance[i], (i + 1) / 10.0, 1e-8);
      EXPECT_NEAR(c.coeff[i], 5.0 + static_cast<double>(i), 1e-8);
    }
  }

  {  // encode
    TrajOptJointWaypointConfig c;
    c.enabled = false;
    c.use_tolerance_override = true;
    c.lower_tolerance = Eigen::Vector3d(-0.1, -0.2, -0.3);
    c.upper_tolerance = Eigen::Vector3d(0.1, 0.2, 0.3);
    c.coeff = (Eigen::VectorXd(3) << 5.0, 6.0, 7.0).finished();

    YAML::Node out = YAML::convert<TrajOptJointWaypointConfig>::encode(c);
    EXPECT_FALSE(out["enabled"].as<bool>());
    EXPECT_TRUE(out["use_tolerance_override"].as<bool>());
    ASSERT_TRUE(out["lower_tolerance"].IsSequence());
    ASSERT_TRUE(out["upper_tolerance"].IsSequence());
    ASSERT_TRUE(out["coeff"].IsSequence());
    for (int i = 0; i < 3; ++i)
    {
      EXPECT_NEAR(out["lower_tolerance"][i].as<double>(), c.lower_tolerance[i], 1e-8);
      EXPECT_NEAR(out["upper_tolerance"][i].as<double>(), c.upper_tolerance[i], 1e-8);
      EXPECT_NEAR(out["coeff"][i].as<double>(), c.coeff[i], 1e-8);
    }
  }
}

TEST(TrajOptYAMLTestFixture, CollisionCoeffDataConversionsUnit)  // NOLINT
{
  const std::string yaml_string = R"(
    default_coeff: 2.5
    unique_pairs:
      - { pair: [link_a, link_b], coeff: 0.0 }
      - { pair: [link_c, link_d], coeff: 1.5 }
  )";

  {  // decode
    trajopt_common::CollisionCoeffData d;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<trajopt_common::CollisionCoeffData>::decode(n, d);
    EXPECT_TRUE(success);

    // default for unknown
    EXPECT_NEAR(d.getCollisionCoeff("foo", "bar"), 2.5, 1e-8);
    // specified
    EXPECT_NEAR(d.getCollisionCoeff("link_c", "link_d"), 1.5, 1e-8);
    EXPECT_NEAR(d.getCollisionCoeff("link_a", "link_b"), 0.0, 1e-8);

    const auto& zeros = d.getPairsWithZeroCoeff();
    EXPECT_FALSE(zeros.empty());
    bool has_zero_pair = false;
    for (const auto& p : zeros)
      if ((p.first == std::string("link_a") && p.second == std::string("link_b")) ||
          (p.first == std::string("link_b") && p.second == std::string("link_a")))
        has_zero_pair = true;
    EXPECT_TRUE(has_zero_pair);
  }

  {  // encode (only zero coeff pairs encoded)
    trajopt_common::CollisionCoeffData d(3.0);
    d.setCollisionCoeff("link_a", "link_b", 0.0);
    d.setCollisionCoeff("link_c", "link_d", 1.5);

    YAML::Node out = YAML::convert<trajopt_common::CollisionCoeffData>::encode(d);
    if (out["unique_pairs"])  // optional
    {
      auto up = out["unique_pairs"];
      ASSERT_TRUE(up.IsSequence());
      ASSERT_EQ(up.size(), 1u);
      EXPECT_EQ(up[0]["pair"][0].as<std::string>(), "link_a");
      EXPECT_EQ(up[0]["pair"][1].as<std::string>(), "link_b");
      EXPECT_NEAR(up[0]["coeff"].as<double>(), 0.0, 1e-8);
    }
  }
}

TEST(TrajOptYAMLTestFixture, TrajOptCollisionConfigConversionsUnit)  // NOLINT
{
  const std::string yaml_string = R"(
    enabled: false
    contact_manager_config:
      default_margin: 0.02
    collision_check_config:
      type: DISCRETE
      longest_valid_segment_length: 0.01
      check_program_mode: ALL
    collision_coeff_data:
      default_coeff: 3.0
      unique_pairs:
        - { pair: [l0, l1], coeff: 0.0 }
    collision_margin_buffer: 0.05
    max_num_cnt: 10
    collision_scale: 0.5
  )";

  {  // decode
    trajopt_common::TrajOptCollisionConfig c;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<trajopt_common::TrajOptCollisionConfig>::decode(n, c);
    EXPECT_TRUE(success);

    EXPECT_FALSE(c.enabled);
    // default_margin scaled by 0.5
    EXPECT_TRUE(c.contact_manager_config.default_margin.has_value());
    EXPECT_NEAR(c.contact_manager_config.default_margin.value(), 0.01, 1e-8);
    EXPECT_NEAR(c.collision_check_config.longest_valid_segment_length, 0.01, 1e-8);
    EXPECT_NEAR(c.collision_margin_buffer, 0.05, 1e-8);
    EXPECT_EQ(c.max_num_cnt, 10);

    // coeff data
    EXPECT_NEAR(c.collision_coeff_data.getCollisionCoeff("foo", "bar"), 3.0, 1e-8);
    EXPECT_NEAR(c.collision_coeff_data.getCollisionCoeff("l0", "l1"), 0.0, 1e-8);
  }

  {  // encode
    trajopt_common::TrajOptCollisionConfig c;
    c.enabled = false;
    c.contact_manager_config.default_margin = 0.01;
    c.collision_check_config.longest_valid_segment_length = 0.02;
    c.collision_coeff_data = trajopt_common::CollisionCoeffData(2.0);
    c.collision_coeff_data.setCollisionCoeff("l0", "l1", 0.0);
    c.collision_margin_buffer = 0.1;
    c.max_num_cnt = 5;

    YAML::Node out = YAML::convert<trajopt_common::TrajOptCollisionConfig>::encode(c);
    EXPECT_FALSE(out["enabled"].as<bool>());
    EXPECT_NEAR(out["contact_manager_config"]["default_margin"].as<double>(), 0.01, 1e-8);
    EXPECT_NEAR(out["collision_check_config"]["longest_valid_segment_length"].as<double>(), 0.02, 1e-8);
    EXPECT_NEAR(out["collision_margin_buffer"].as<double>(), 0.1, 1e-8);
    EXPECT_EQ(out["max_num_cnt"].as<int>(), 5);

    auto up = out["collision_coeff_data"]["unique_pairs"];
    ASSERT_TRUE(up.IsSequence());
    ASSERT_EQ(up.size(), 1u);
    EXPECT_EQ(up[0]["pair"][0].as<std::string>(), "l0");
    EXPECT_EQ(up[0]["pair"][1].as<std::string>(), "l1");
    EXPECT_NEAR(up[0]["coeff"].as<double>(), 0.0, 1e-8);
  }
}

TEST(TrajOptYAMLTestFixture, TrajOptCartesianWaypointConfigRoundTripUnit)  // NOLINT
{
  TrajOptCartesianWaypointConfig c;
  c.enabled = false;
  c.use_tolerance_override = true;
  for (int i = 0; i < 6; ++i)
  {
    c.lower_tolerance[i] = (i + 1) * -0.01;
    c.upper_tolerance[i] = (i + 1) * 0.01;
    c.coeff[i] = static_cast<double>(i) + 0.5;
  }

  YAML::Node n = YAML::convert<TrajOptCartesianWaypointConfig>::encode(c);
  TrajOptCartesianWaypointConfig c2;
  ASSERT_TRUE(YAML::convert<TrajOptCartesianWaypointConfig>::decode(n, c2));

  EXPECT_EQ(c2.enabled, c.enabled);
  EXPECT_EQ(c2.use_tolerance_override, c.use_tolerance_override);
  for (int i = 0; i < 6; ++i)
  {
    EXPECT_NEAR(c2.lower_tolerance[i], c.lower_tolerance[i], 1e-12);
    EXPECT_NEAR(c2.upper_tolerance[i], c.upper_tolerance[i], 1e-12);
    EXPECT_NEAR(c2.coeff[i], c.coeff[i], 1e-12);
  }
}

TEST(TrajOptYAMLTestFixture, TrajOptCartesianWaypointConfigSize1ToSize6ExpansionUnit)  // NOLINT
{
  // Test that a size-1 YAML sequence gets properly expanded to size-6 during decoding
  const std::string yaml_string_size1 = R"(
    enabled: true
    use_tolerance_override: false
    lower_tolerance: [-0.05]  # Size 1 - should expand to fill all 6 elements
    upper_tolerance: [0.05]   # Size 1 - should expand to fill all 6 elements  
    coeff: [2.5]             # Size 1 - should expand to fill all 6 elements
  )";

  TrajOptCartesianWaypointConfig c;
  YAML::Node n = YAML::Load(yaml_string_size1);

  // Decode should expand size-1 sequences to size-6 with repeated values
  ASSERT_TRUE(YAML::convert<TrajOptCartesianWaypointConfig>::decode(n, c));

  EXPECT_EQ(c.enabled, true);
  EXPECT_EQ(c.use_tolerance_override, false);
  ASSERT_EQ(c.lower_tolerance.size(), 6);
  ASSERT_EQ(c.upper_tolerance.size(), 6);
  ASSERT_EQ(c.coeff.size(), 6);

  // All elements should have the repeated value from the size-1 input
  for (int i = 0; i < 6; ++i)
  {
    EXPECT_NEAR(c.lower_tolerance[i], -0.05, 1e-12);
    EXPECT_NEAR(c.upper_tolerance[i], 0.05, 1e-12);
    EXPECT_NEAR(c.coeff[i], 2.5, 1e-12);
  }

  // Round-trip test: encode should produce size-6 sequences
  YAML::Node n_encoded = YAML::convert<TrajOptCartesianWaypointConfig>::encode(c);

  ASSERT_TRUE(n_encoded["lower_tolerance"].IsSequence());
  ASSERT_TRUE(n_encoded["upper_tolerance"].IsSequence());
  ASSERT_TRUE(n_encoded["coeff"].IsSequence());
  ASSERT_EQ(n_encoded["lower_tolerance"].size(), 6u);
  ASSERT_EQ(n_encoded["upper_tolerance"].size(), 6u);
  ASSERT_EQ(n_encoded["coeff"].size(), 6u);

  for (int i = 0; i < 6; ++i)
  {
    EXPECT_NEAR(n_encoded["lower_tolerance"][i].as<double>(), -0.05, 1e-12);
    EXPECT_NEAR(n_encoded["upper_tolerance"][i].as<double>(), 0.05, 1e-12);
    EXPECT_NEAR(n_encoded["coeff"][i].as<double>(), 2.5, 1e-12);
  }
}

TEST(TrajOptYAMLTestFixture, TrajOptCartesianWaypointConfigInvalidSizeFailureUnit)  // NOLINT
{
  // Test that YAML with invalid sizes (not 1 or 6) fails to decode
  const std::string yaml_string_invalid_size3 = R"(
    enabled: true
    use_tolerance_override: false
    lower_tolerance: [-0.1, -0.2, -0.3]  # Invalid size: 3 instead of 6 or 1
    upper_tolerance: [0.1, 0.2, 0.3, 0.01, 0.02, 0.03]  # Correct size: 6
    coeff: [1, 2, 3, 4, 5, 6]  # Correct size: 6
  )";

  TrajOptCartesianWaypointConfig c;
  YAML::Node n = YAML::Load(yaml_string_invalid_size3);

  // Decode should fail because lower_tolerance has invalid size (3)
  EXPECT_FALSE(YAML::convert<TrajOptCartesianWaypointConfig>::decode(n, c));

  const std::string yaml_string_invalid_size4 = R"(
    enabled: true
    use_tolerance_override: false
    lower_tolerance: [-0.1, -0.2, -0.3, -0.01, -0.02, -0.03]  # Correct size: 6
    upper_tolerance: [0.1, 0.2, 0.3, 0.01]  # Invalid size: 4 instead of 6 or 1
    coeff: [1, 2, 3, 4, 5, 6]  # Correct size: 6
  )";

  YAML::Node n2 = YAML::Load(yaml_string_invalid_size4);

  // Decode should fail because upper_tolerance has invalid size (4)
  EXPECT_FALSE(YAML::convert<TrajOptCartesianWaypointConfig>::decode(n2, c));

  const std::string yaml_string_invalid_size2 = R"(
    enabled: true
    use_tolerance_override: false
    lower_tolerance: [-0.1, -0.2, -0.3, -0.01, -0.02, -0.03]  # Correct size: 6
    upper_tolerance: [0.1, 0.2, 0.3, 0.01, 0.02, 0.03]  # Correct size: 6
    coeff: [1, 2]  # Invalid size: 2 instead of 6 or 1
  )";

  YAML::Node n3 = YAML::Load(yaml_string_invalid_size2);

  // Decode should fail because coeff has invalid size (2)
  EXPECT_FALSE(YAML::convert<TrajOptCartesianWaypointConfig>::decode(n3, c));
}

TEST(TrajOptYAMLTestFixture, TrajOptJointWaypointConfigRoundTripUnit)  // NOLINT
{
  TrajOptJointWaypointConfig c;
  c.enabled = false;
  c.use_tolerance_override = true;
  c.lower_tolerance = (Eigen::VectorXd(4) << -0.1, -0.2, -0.3, -0.4).finished();
  c.upper_tolerance = (Eigen::VectorXd(4) << 0.1, 0.2, 0.3, 0.4).finished();
  c.coeff = (Eigen::VectorXd(4) << 1.0, 2.0, 3.0, 4.0).finished();

  YAML::Node n = YAML::convert<TrajOptJointWaypointConfig>::encode(c);
  TrajOptJointWaypointConfig c2;
  ASSERT_TRUE(YAML::convert<TrajOptJointWaypointConfig>::decode(n, c2));

  EXPECT_EQ(c2.enabled, c.enabled);
  EXPECT_EQ(c2.use_tolerance_override, c.use_tolerance_override);
  ASSERT_EQ(c2.lower_tolerance.size(), c.lower_tolerance.size());
  ASSERT_EQ(c2.upper_tolerance.size(), c.upper_tolerance.size());
  ASSERT_EQ(c2.coeff.size(), c.coeff.size());
  for (int i = 0; i < c.coeff.size(); ++i)
  {
    EXPECT_NEAR(c2.lower_tolerance[i], c.lower_tolerance[i], 1e-12);
    EXPECT_NEAR(c2.upper_tolerance[i], c.upper_tolerance[i], 1e-12);
    EXPECT_NEAR(c2.coeff[i], c.coeff[i], 1e-12);
  }
}

TEST(TrajOptYAMLTestFixture, CollisionCoeffDataRoundTripUnit)  // NOLINT
{
  trajopt_common::CollisionCoeffData d_in(2.5);
  // Only zero coeff pairs are preserved by encode; include two to test set handling
  d_in.setCollisionCoeff("a", "b", 0.0);
  d_in.setCollisionCoeff("c", "d", 0.0);

  YAML::Node n = YAML::convert<trajopt_common::CollisionCoeffData>::encode(d_in);
  trajopt_common::CollisionCoeffData d_out;
  ASSERT_TRUE(YAML::convert<trajopt_common::CollisionCoeffData>::decode(n, d_out));

  // default coeff should match
  EXPECT_NEAR(d_out.getCollisionCoeff("x", "y"), 2.5, 1e-12);
  // zero pairs should be preserved irrespective of order
  EXPECT_NEAR(d_out.getCollisionCoeff("a", "b"), 0.0, 1e-12);
  EXPECT_NEAR(d_out.getCollisionCoeff("b", "a"), 0.0, 1e-12);
  EXPECT_NEAR(d_out.getCollisionCoeff("c", "d"), 0.0, 1e-12);
  EXPECT_NEAR(d_out.getCollisionCoeff("d", "c"), 0.0, 1e-12);
}

TEST(TrajOptYAMLTestFixture, TrajOptCollisionConfigRoundTripUnit)  // NOLINT
{
  trajopt_common::TrajOptCollisionConfig c_in;
  c_in.enabled = false;
  c_in.contact_manager_config.default_margin = 0.02;
  c_in.collision_check_config.longest_valid_segment_length = 0.01;
  c_in.collision_check_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  c_in.collision_check_config.check_program_mode = tesseract_collision::CollisionCheckProgramType::ALL;
  c_in.collision_coeff_data = trajopt_common::CollisionCoeffData(3.3);
  c_in.collision_coeff_data.setCollisionCoeff("l0", "l1", 0.0);
  c_in.collision_margin_buffer = 0.05;
  c_in.max_num_cnt = 7;

  YAML::Node n = YAML::convert<trajopt_common::TrajOptCollisionConfig>::encode(c_in);
  trajopt_common::TrajOptCollisionConfig c_out;
  ASSERT_TRUE(YAML::convert<trajopt_common::TrajOptCollisionConfig>::decode(n, c_out));

  EXPECT_EQ(c_out.enabled, c_in.enabled);
  ASSERT_TRUE(c_out.contact_manager_config.default_margin.has_value());
  EXPECT_NEAR(
      c_out.contact_manager_config.default_margin.value(), c_in.contact_manager_config.default_margin.value(), 1e-12);
  EXPECT_NEAR(c_out.collision_check_config.longest_valid_segment_length,
              c_in.collision_check_config.longest_valid_segment_length,
              1e-12);
  EXPECT_EQ(c_out.collision_check_config.type, c_in.collision_check_config.type);
  EXPECT_EQ(c_out.collision_check_config.check_program_mode, c_in.collision_check_config.check_program_mode);
  EXPECT_NEAR(c_out.collision_coeff_data.getCollisionCoeff("x", "y"), 3.3, 1e-12);
  EXPECT_NEAR(c_out.collision_coeff_data.getCollisionCoeff("l0", "l1"), 0.0, 1e-12);
  EXPECT_NEAR(c_out.collision_margin_buffer, c_in.collision_margin_buffer, 1e-12);
  EXPECT_EQ(c_out.max_num_cnt, c_in.max_num_cnt);
}
