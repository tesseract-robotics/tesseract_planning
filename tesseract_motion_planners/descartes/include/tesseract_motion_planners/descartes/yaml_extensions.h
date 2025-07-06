/**
 * @file yaml_extensions.h
 * @brief YAML Type conversions
 *
 * @author Levi Armstrong
 * @date April 30, 2025
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_YAML_EXTENSIONS_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_YAML_EXTENSIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/profile/descartes_default_move_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_ladder_graph_solver_profile.h>

#include <tesseract_common/yaml_extensions.h>
#include <tesseract_collision/core/yaml_extensions.h>

namespace YAML
{
//=========================== Descartes Default Move Profile ===========================
template <>
struct convert<tesseract_planning::DescartesDefaultMoveProfile<double>>
{
  static Node encode(const tesseract_planning::DescartesDefaultMoveProfile<double>& p)
  {
    Node node;
    node["key"] = p.getKey();
    node["target_pose_fixed"] = p.target_pose_fixed;
    node["target_pose_sample_axis"] = p.target_pose_sample_axis;
    node["target_pose_sample_resolution"] = p.target_pose_sample_resolution;
    node["target_pose_sample_min"] = p.target_pose_sample_min;
    node["target_pose_sample_max"] = p.target_pose_sample_max;
    node["manipulator_ik_solver"] = p.manipulator_ik_solver;
    node["allow_collision"] = p.allow_collision;
    node["enable_collision"] = p.enable_collision;
    node["vertex_contact_manager_config"] = p.vertex_contact_manager_config;
    node["vertex_collision_check_config"] = p.vertex_collision_check_config;
    node["enable_edge_collision"] = p.enable_edge_collision;
    node["edge_contact_manager_config"] = p.edge_contact_manager_config;
    node["edge_collision_check_config"] = p.edge_collision_check_config;
    node["use_redundant_joint_solutions"] = p.use_redundant_joint_solutions;
    node["debug"] = p.debug;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::DescartesDefaultMoveProfile<double>& p)
  {
    if (!node.IsMap())
      return false;
    if (n = node["target_pose_fixed"])
      p.target_pose_fixed = n.as<bool>();
    if (n = node["target_pose_sample_axis"])
      p.target_pose_sample_axis = n.as<Eigen::Vector3d>();
    if (n = node["target_pose_sample_resolution"])
      p.target_pose_sample_resolution = n.as<double>();
    if (n = node["target_pose_sample_min"])
      p.target_pose_sample_min = n.as<double>();
    if (n = node["target_pose_sample_max"])
      p.target_pose_sample_max = n.as<double>();
    if (n = node["manipulator_ik_solver"])
      p.manipulator_ik_solver = n.as<std::string>();
    if (n = node["allow_collision"])
      p.allow_collision = n.as<bool>();
    if (n = node["enable_collision"])
      p.enable_collision = n.as<bool>();
    if (n = node["vertex_contact_manager_config"])
      p.vertex_contact_manager_config = n.as<tesseract_collision::ContactManagerConfig>();
    if (n = node["vertex_collision_check_config"])
      p.vertex_collision_check_config = n.as<tesseract_collision::CollisionCheckConfig>();
    if (n = node["enable_edge_collision"])
      p.enable_edge_collision = n.as<bool>();
    if (n = node["edge_contact_manager_config"])
      p.edge_contact_manager_config = n.as<tesseract_collision::ContactManagerConfig>();
    if (n = node["edge_collision_check_config"])
      p.edge_collision_check_config = n.as<tesseract_collision::CollisionCheckConfig>();
    if (n = node["use_redundant_joint_solutions"])
      p.use_redundant_joint_solutions = n.as<bool>();
    if (n = node["debug"])
      p.debug = n.as<bool>();
    return true;
  }
};

//=========================== Descartes Ladder Graph Solver Profile ===========================
template <>
struct convert<tesseract_planning::DescartesLadderGraphSolverProfile<double>>
{
  static Node encode(const tesseract_planning::DescartesLadderGraphSolverProfile<double>& p)
  {
    Node node;
    node["num_threads"] = p.num_threads;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::DescartesLadderGraphSolverProfile<double>& p)
  {
    if (!node.IsMap())
      return false;
    if (n = node["num_threads"])
      p.num_threads = n.as<int>();
    return true;
  }
};
}  // namespace YAML

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_YAML_EXTENSIONS_H
