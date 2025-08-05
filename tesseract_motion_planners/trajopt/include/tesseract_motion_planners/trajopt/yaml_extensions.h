/**
 * @file yaml_extensions.h
 * @brief YAML Type conversions for trajopt types
 *
 * @author Samantha Smith
 * @date August 5, 2025
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
#ifndef TESSERACT_MOTION_PLANNING_YAML_EXTENSIONS_H
#define TESSERACT_MOTION_PLANNING_YAML_EXTENSIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
#include <set>
#include <vector>
#include <osqp.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/plugin_info.h>
#include <tesseract_common/profile_plugin_factory.h>
#include <tesseract_collision/core/fwd.h>
#include <tesseract_collision/core/types.h>

#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/core/utils.h>

#include <trajopt_sco/osqp_interface.hpp>



namespace YAML
{
//=========================== Eigen::VectorXd ===========================

template <>
struct convert<Eigen::VectorXd>
{
  static Node encode(const Eigen::VectorXd& rhs)
  {
    Node node;
    for (int i = 0; i < rhs.size(); ++i)
      node.push_back(rhs(i));
    return node;
  }

  static bool decode(const Node& node, Eigen::VectorXd& rhs)
  {
    if (!node.IsSequence())
      return false;

    rhs.resize(node.size());
    for (std::size_t i = 0; i < node.size(); ++i)
      rhs(static_cast<Eigen::Index>(i)) = node[i].as<double>();

    return true;
  }
};

//=========================== Eigen::Matrix<Scalar, Rows, 1> ===========================
template <typename Scalar, int Rows>
struct convert<Eigen::Matrix<Scalar, Rows, 1>>
{
  static Node encode(const Eigen::Matrix<Scalar, Rows, 1>& rhs)
  {
    Node node;
    for (int i = 0; i < rhs.rows(); ++i)
      node.push_back(rhs(i));
    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix<Scalar, Rows, 1>& rhs)
  {
    if (!node.IsSequence() || node.size() != static_cast<std::size_t>(Rows))
      return false;

    for (int i = 0; i < Rows; ++i)
      rhs(i) = node[i].as<Scalar>();

    return true;
  }
};

//=========================== TrajOptCollisionConfig ===========================
template <>
struct convert<trajopt_common::TrajOptCollisionConfig>
{
  static Node encode(const trajopt_common::TrajOptCollisionConfig& rhs)
  {
    Node node;
    node["collision_coeff_data"] = rhs.collision_coeff_data;
    node["collision_margin_buffer"] = rhs.collision_margin_buffer;
    node["max_num_cnt"] = rhs.max_num_cnt;
    return node;
  }

  static bool decode(const Node& node, trajopt_common::TrajOptCollisionConfig& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["collision_coeff_data"])
      rhs.collision_coeff_data = n.as<trajopt_common::CollisionCoeffData>();
    if (const YAML::Node& n = node["collision_margin_buffer"])
      rhs.collision_margin_buffer = n.as<double>();
    if (const YAML::Node& n = node["max_num_cnt"])
      rhs.max_num_cnt = n.as<int>();
    return true;
  }
};

//=========================== CollisionCoeffData ===========================
template <>
struct convert<trajopt_common::CollisionCoeffData>
{
  static Node encode(const trajopt_common::CollisionCoeffData& rhs)
  {
    Node node;
    // only contains private variables
    return node;
  }

  static bool decode(const Node& node, trajopt_common::CollisionCoeffData& rhs)
  {
    // Check for required entries
    return true;
  }
};

//=========================== TrajOptCartesianWaypointConfig ===========================
template <>
struct convert<tesseract_planning::TrajOptCartesianWaypointConfig>
{
  static Node encode(const tesseract_planning::TrajOptCartesianWaypointConfig& rhs)
  {
    Node node;
    node["enabled"] = rhs.enabled;
    node["use_tolerance_override"] = rhs.use_tolerance_override;
    node["lower_tolerance"] = rhs.lower_tolerance;
    node["upper_tolerance"] = rhs.upper_tolerance;
    node["coeff"] = rhs.coeff;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::TrajOptCartesianWaypointConfig& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["enabled"])
      rhs.enabled = n.as<bool>();
    if (const YAML::Node& n = node["use_tolerance_override"])
      rhs.use_tolerance_override = n.as<bool>();
    if (const YAML::Node& n = node["lower_tolerance"])
      rhs.lower_tolerance = n.as<Eigen::Matrix<double, 6, 1>>();
    if (const YAML::Node& n = node["upper_tolerance"])
      rhs.upper_tolerance = n.as<Eigen::Matrix<double, 6, 1>>();
    if (const YAML::Node& n = node["coeff"])
      rhs.coeff = n.as<Eigen::Matrix<double, 6, 1>>();
    return true;
  }
};

//=========================== TrajOptJointWaypointConfig ===========================
template <>
struct convert<tesseract_planning::TrajOptJointWaypointConfig>
{
  static Node encode(const tesseract_planning::TrajOptJointWaypointConfig& rhs)
  {
    Node node;
    node["enabled"] = rhs.enabled;
    node["use_tolerance_override"] = rhs.use_tolerance_override;
    node["lower_tolerance"] = rhs.lower_tolerance;
    node["upper_tolerance"] = rhs.upper_tolerance;
    node["coeff"] = rhs.coeff;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::TrajOptJointWaypointConfig& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["enabled"])
      rhs.enabled = n.as<bool>();
    if (const YAML::Node& n = node["use_tolerance_override"])
      rhs.use_tolerance_override = n.as<bool>();
    if (const YAML::Node& n = node["lower_tolerance"])
      rhs.lower_tolerance = n.as<Eigen::VectorXd>();
    if (const YAML::Node& n = node["upper_tolerance"])
      rhs.upper_tolerance = n.as<Eigen::VectorXd>();
    if (const YAML::Node& n = node["coeff"])
      rhs.coeff = n.as<Eigen::VectorXd>();
    return true;
  }
};


}  // namespace YAML

#endif  // TESSERACT_MOTION_PLANNING_YAML_EXTENSIONS_H