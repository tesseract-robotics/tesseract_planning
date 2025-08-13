/**
 * @file yaml_extensions.h
 * @brief YAML Type conversions
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

#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_YAML_EXTENSIONS_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_YAML_EXTENSIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/yaml_extensions.h>
#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>

namespace YAML
{
//=========================== TrajOpt Cartesian Waypoint Config ===========================
template <>
struct convert<tesseract_planning::TrajOptCartesianWaypointConfig>
{
  static Node encode(const tesseract_planning::TrajOptCartesianWaypointConfig& rhs)
  {
    Node node;
    node["enabled"] = rhs.enabled;
    node["use_tolerance_override"] = rhs.use_tolerance_override;

    // For fixed-size vectors, just encode them directly as they are already size 6
    node["lower_tolerance"] = rhs.lower_tolerance;
    node["upper_tolerance"] = rhs.upper_tolerance;
    node["coeff"] = rhs.coeff;

    return node;
  }

  static bool decode(const Node& node, tesseract_planning::TrajOptCartesianWaypointConfig& rhs)
  {
    if (!node.IsMap())
      return false;

    if (const YAML::Node& n = node["enabled"])
      rhs.enabled = n.as<bool>();
    if (const YAML::Node& n = node["use_tolerance_override"])
      rhs.use_tolerance_override = n.as<bool>();

    // Helper lambda to decode sequences with size-1-to-6 expansion
    auto decode_with_expansion = [](const YAML::Node& n, Eigen::Matrix<double, 6, 1>& target) -> bool {
      if (!n || !n.IsSequence())
        return true;  // Optional field

      if (n.size() == 6)
      {
        // Size 6: direct assignment
        target = n.as<Eigen::Matrix<double, 6, 1>>();
        return true;
      }
      else if (n.size() == 1)
      {
        // Size 1: expand to fill all 6 elements
        double value = n[0].as<double>();
        target.fill(value);
        return true;
      }
      else
      {
        // Invalid size
        return false;
      }
    };

    // Decode with size-1-to-6 expansion support
    if (const YAML::Node& n = node["lower_tolerance"])
    {
      if (!decode_with_expansion(n, rhs.lower_tolerance))
        return false;
    }
    if (const YAML::Node& n = node["upper_tolerance"])
    {
      if (!decode_with_expansion(n, rhs.upper_tolerance))
        return false;
    }
    if (const YAML::Node& n = node["coeff"])
    {
      if (!decode_with_expansion(n, rhs.coeff))
        return false;
    }

    return true;
  }
};

//=========================== TrajOpt Joint Waypoint Config ===========================
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
    if (!node.IsMap())
      return false;

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

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_YAML_EXTENSIONS_H