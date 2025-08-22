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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>
#include <tesseract_common/yaml_extensions.h>
#include <trajopt_common/yaml_extensions.h>
#include <trajopt_sco/optimizer.hpp>
#include <osqp/types.h>

namespace YAML
{
//=========================== linsys_solver_type Enum ===========================
template <>
struct convert<linsys_solver_type>
{
  static Node encode(const linsys_solver_type& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<linsys_solver_type, std::string> m = {
      { linsys_solver_type::QDLDL_SOLVER, "QDLDL_SOLVER" },
      { linsys_solver_type::MKL_PARDISO_SOLVER, "MKL_PARDISO_SOLVER" },
      { linsys_solver_type::UNKNOWN_SOLVER, "UNKNOWN_SOLVER" }
    };
    // LCOV_EXCL_STOP
    return Node(m.at(rhs));
  }

  static bool decode(const Node& node, linsys_solver_type& rhs)
  {
    // LCOV_EXCL_START
    static const std::map<std::string, linsys_solver_type> inv = {
      { "QDLDL_SOLVER", linsys_solver_type::QDLDL_SOLVER },
      { "MKL_PARDISO_SOLVER", linsys_solver_type::MKL_PARDISO_SOLVER },
      { "UNKNOWN_SOLVER", linsys_solver_type::UNKNOWN_SOLVER }
    };
    // LCOV_EXCL_STOP

    if (!node.IsScalar())
      return false;

    auto it = inv.find(node.Scalar());
    if (it == inv.end())
      return false;

    rhs = it->second;
    return true;
  }
};

//=========================== OSQPSettings ===========================
template <>
struct convert<OSQPSettings>
{
  static Node encode(const OSQPSettings& rhs)
  {
    Node node;
    node["rho"] = rhs.rho;
    node["sigma"] = rhs.sigma;
    node["scaling"] = rhs.scaling;
    node["adaptive_rho"] = rhs.adaptive_rho;
    node["adaptive_rho_interval"] = rhs.adaptive_rho_interval;
    node["adaptive_rho_tolerance"] = rhs.adaptive_rho_tolerance;
    node["adaptive_rho_fraction"] = rhs.adaptive_rho_fraction;
    node["max_iter"] = rhs.max_iter;
    node["eps_abs"] = rhs.eps_abs;
    node["eps_rel"] = rhs.eps_rel;
    node["eps_prim_inf"] = rhs.eps_prim_inf;
    node["eps_dual_inf"] = rhs.eps_dual_inf;
    node["alpha"] = rhs.alpha;
    node["linsys_solver_type"] = rhs.linsys_solver_type;
    node["delta"] = rhs.delta;
    node["polish"] = rhs.polish;
    node["polish_refine_iter"] = rhs.polish_refine_iter;
    node["verbose"] = rhs.verbose;
    node["scaled_termination"] = rhs.scaled_termination;
    node["check_termination"] = rhs.check_termination;
    node["warm_start"] = rhs.warm_start;
    node["time_limit"] = rhs.time_limit;

    return node;
  }

  static bool decode(const Node& node, OSQPSettings& rhs)
  {
    sco::OSQPModelConfig::setDefaultOSQPSettings(rhs);
    // Check for required entries
    if (const YAML::Node& n = node["rho"])
      rhs.rho = n.as<c_float>();
    if (const YAML::Node& n = node["sigma"])
      rhs.sigma = n.as<c_float>();
    if (const YAML::Node& n = node["scaling"])
      rhs.scaling = n.as<c_int>();
    if (const YAML::Node& n = node["adaptive_rho"])
      rhs.adaptive_rho = n.as<c_int>();
    if (const YAML::Node& n = node["adaptive_rho_interval"])
      rhs.adaptive_rho_interval = n.as<c_int>();
    if (const YAML::Node& n = node["adaptive_rho_tolerance"])
      rhs.adaptive_rho_tolerance = n.as<c_float>();
    if (const YAML::Node& n = node["adaptive_rho_fraction"])
      rhs.adaptive_rho_fraction = n.as<c_float>();
    if (const YAML::Node& n = node["max_iter"])
      rhs.max_iter = n.as<c_int>();
    if (const YAML::Node& n = node["eps_abs"])
      rhs.eps_abs = n.as<c_float>();
    if (const YAML::Node& n = node["eps_rel"])
      rhs.eps_rel = n.as<c_float>();
    if (const YAML::Node& n = node["eps_prim_inf"])
      rhs.eps_prim_inf = n.as<c_float>();
    if (const YAML::Node& n = node["eps_dual_inf"])
      rhs.eps_dual_inf = n.as<c_float>();
    if (const YAML::Node& n = node["alpha"])
      rhs.alpha = n.as<c_float>();
    if (const YAML::Node& n = node["linsys_solver"])
      rhs.linsys_solver = n.as<linsys_solver_type>();
    if (const YAML::Node& n = node["delta"])
      rhs.delta = n.as<c_float>();
    if (const YAML::Node& n = node["polish"])
      rhs.polish = n.as<c_int>();
    if (const YAML::Node& n = node["polish_refine_iter"])
      rhs.polish_refine_iter = n.as<c_int>();
    if (const YAML::Node& n = node["verbose"])
      rhs.verbose = n.as<c_int>();
    if (const YAML::Node& n = node["scaled_termination"])
      rhs.scaled_termination = n.as<c_int>();
    if (const YAML::Node& n = node["check_termination"])
      rhs.check_termination = n.as<c_int>();
    if (const YAML::Node& n = node["warm_start"])
      rhs.warm_start = n.as<c_int>();
    if (const YAML::Node& n = node["time_limit"])
      rhs.time_limit = n.as<c_float>();

    return true;
  }
};

//==================== trajopt_sco::BasicTrustRegionSQPParameters ======================
template <>
struct convert<trajopt_sco::BasicTrustRegionSQPParameters>
{
  static Node encode(const trajopt_sco::BasicTrustRegionSQPParameters& rhs)
  {
    Node node;
    node["improve_ratio_threshold"] = rhs.improve_ratio_threshold;
    node["min_trust_box_size"] = rhs.min_trust_box_size;
    node["min_approx_improve"] = rhs.min_approx_improve;
    node["min_approx_improve_frac"] = rhs.min_approx_improve_frac;
    node["max_iter"] = rhs.max_iter;
    node["trust_shrink_ratio"] = rhs.trust_shrink_ratio;
    node["trust_expand_ratio"] = rhs.trust_expand_ratio;
    node["cnt_tolerance"] = rhs.cnt_tolerance;
    node["max_merit_coeff_increases"] = rhs.max_merit_coeff_increases;
    node["max_qp_solver_failures"] = rhs.max_qp_solver_failures;
    node["merit_coeff_increase_ratio"] = rhs.merit_coeff_increase_ratio;
    node["max_time"] = rhs.max_time;
    node["initial_merit_error_coeff"] = rhs.initial_merit_error_coeff;
    node["inflate_constraints_individually"] = rhs.inflate_constraints_individually;
    node["trust_box_size"] = rhs.trust_box_size;
    node["log_results"] = rhs.log_results;
    node["log_dir"] = rhs.log_dir;
    node["num_threads"] = rhs.num_threads;
    return node;
  }

  static bool decode(const Node& node, trajopt_sco::BasicTrustRegionSQPParameters& rhs)
  {
    // Check for required entries
    if (const YAML::Node& n = node["improve_ratio_threshold"])
      rhs.improve_ratio_threshold = n.as<double>();
    if (const YAML::Node& n = node["min_trust_box_size"])
      rhs.min_trust_box_size = n.as<double>();
    if (const YAML::Node& n = node["min_approx_improve"])
      rhs.min_approx_improve = n.as<double>();
    if (const YAML::Node& n = node["min_approx_improve_frac"])
      rhs.min_approx_improve_frac = n.as<double>();
    if (const YAML::Node& n = node["max_iter"])
      rhs.max_iter = n.as<int>();
    if (const YAML::Node& n = node["trust_shrink_ratio"])
      rhs.trust_shrink_ratio = n.as<double>();
    if (const YAML::Node& n = node["trust_expand_ratio"])
      rhs.trust_expand_ratio = n.as<double>();
    if (const YAML::Node& n = node["cnt_tolerance"])
      rhs.cnt_tolerance = n.as<double>();
    if (const YAML::Node& n = node["max_merit_coeff_increases"])
      rhs.max_merit_coeff_increases = n.as<double>();
    if (const YAML::Node& n = node["max_qp_solver_failures"])
      rhs.max_qp_solver_failures = n.as<int>();
    if (const YAML::Node& n = node["merit_coeff_increase_ratio"])
      rhs.merit_coeff_increase_ratio = n.as<double>();
    if (const YAML::Node& n = node["max_time"])
      rhs.max_time = n.as<double>();
    if (const YAML::Node& n = node["initial_merit_error_coeff"])
      rhs.initial_merit_error_coeff = n.as<double>();
    if (const YAML::Node& n = node["inflate_constraints_individually"])
      rhs.inflate_constraints_individually = n.as<bool>();
    if (const YAML::Node& n = node["trust_box_size"])
      rhs.trust_box_size = n.as<double>();
    if (const YAML::Node& n = node["log_results"])
      rhs.log_results = n.as<bool>();
    if (const YAML::Node& n = node["log_dir"])
      rhs.log_dir = n.as<std::string>();
    if (const YAML::Node& n = node["num_threads"])
      rhs.num_threads = n.as<int>();
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
