/**
 * @file yaml_extensions.h
 * @brief YAML Type conversions for trajopt_ifopt types
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
#include <OsqpEigen/OsqpEigen.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_waypoint_config.h>
#include <tesseract_common/yaml_extensions.h>
#include <trajopt_common/yaml_extensions.h>
#include <trajopt_sqp/types.h>

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
struct convert<OsqpEigen::Settings>
{
  static Node encode(const OsqpEigen::Settings& rhs)
  {
    const OSQPSettings& settings = *rhs.getSettings();

    Node node;
    node["rho"] = settings.rho;
    node["sigma"] = settings.sigma;
    node["scaling"] = settings.scaling;
    node["adaptive_rho"] = settings.adaptive_rho;
    node["adaptive_rho_interval"] = settings.adaptive_rho_interval;
    node["adaptive_rho_tolerance"] = settings.adaptive_rho_tolerance;
    node["adaptive_rho_fraction"] = settings.adaptive_rho_fraction;
    node["max_iter"] = settings.max_iter;
    node["eps_abs"] = settings.eps_abs;
    node["eps_rel"] = settings.eps_rel;
    node["eps_prim_inf"] = settings.eps_prim_inf;
    node["eps_dual_inf"] = settings.eps_dual_inf;
    node["alpha"] = settings.alpha;
    node["linsys_solver"] = settings.linsys_solver;
    node["delta"] = settings.delta;
    node["polish"] = settings.polish;
    node["polish_refine_iter"] = settings.polish_refine_iter;
    node["verbose"] = settings.verbose;
    node["scaled_termination"] = settings.scaled_termination;
    node["check_termination"] = settings.check_termination;
    node["warm_start"] = settings.warm_start;
    node["time_limit"] = settings.time_limit;

    return node;
  }

  static bool decode(const Node& node, OsqpEigen::Settings& rhs)
  {
    OSQPSettings& settings = *rhs.getSettings();

    // Check for required entries
    if (const YAML::Node& n = node["rho"])
      settings.rho = n.as<c_float>();
    if (const YAML::Node& n = node["sigma"])
      settings.sigma = n.as<c_float>();
    if (const YAML::Node& n = node["scaling"])
      settings.scaling = n.as<c_int>();
    if (const YAML::Node& n = node["adaptive_rho"])
      settings.adaptive_rho = n.as<c_int>();
    if (const YAML::Node& n = node["adaptive_rho_interval"])
      settings.adaptive_rho_interval = n.as<c_int>();
    if (const YAML::Node& n = node["adaptive_rho_tolerance"])
      settings.adaptive_rho_tolerance = n.as<c_float>();
    if (const YAML::Node& n = node["adaptive_rho_fraction"])
      settings.adaptive_rho_fraction = n.as<c_float>();
    if (const YAML::Node& n = node["max_iter"])
      settings.max_iter = n.as<c_int>();
    if (const YAML::Node& n = node["eps_abs"])
      settings.eps_abs = n.as<c_float>();
    if (const YAML::Node& n = node["eps_rel"])
      settings.eps_rel = n.as<c_float>();
    if (const YAML::Node& n = node["eps_prim_inf"])
      settings.eps_prim_inf = n.as<c_float>();
    if (const YAML::Node& n = node["eps_dual_inf"])
      settings.eps_dual_inf = n.as<c_float>();
    if (const YAML::Node& n = node["alpha"])
      settings.alpha = n.as<c_float>();
    if (const YAML::Node& n = node["linsys_solver"])
      settings.linsys_solver = n.as<linsys_solver_type>();
    if (const YAML::Node& n = node["delta"])
      settings.delta = n.as<c_float>();
    if (const YAML::Node& n = node["polish"])
      settings.polish = n.as<c_int>();
    if (const YAML::Node& n = node["polish_refine_iter"])
      settings.polish_refine_iter = n.as<c_int>();
    if (const YAML::Node& n = node["verbose"])
      settings.verbose = n.as<c_int>();
    if (const YAML::Node& n = node["scaled_termination"])
      settings.scaled_termination = n.as<c_int>();
    if (const YAML::Node& n = node["check_termination"])
      settings.check_termination = n.as<c_int>();
    if (const YAML::Node& n = node["warm_start"])
      settings.warm_start = n.as<c_int>();
    if (const YAML::Node& n = node["time_limit"])
      settings.time_limit = n.as<c_float>();

    return true;
  }
};

//==================== trajopt_sqp::SQPParameters ======================
template <>
struct convert<trajopt_sqp::SQPParameters>
{
  static Node encode(const trajopt_sqp::SQPParameters& rhs)
  {
    Node node;
    node["improve_ratio_threshold"] = rhs.improve_ratio_threshold;
    node["min_trust_box_size"] = rhs.min_trust_box_size;
    node["min_approx_improve"] = rhs.min_approx_improve;
    node["min_approx_improve_frac"] = rhs.min_approx_improve_frac;
    node["max_iterations"] = rhs.max_iterations;
    node["trust_shrink_ratio"] = rhs.trust_shrink_ratio;
    node["trust_expand_ratio"] = rhs.trust_expand_ratio;
    node["cnt_tolerance"] = rhs.cnt_tolerance;
    node["max_merit_coeff_increases"] = rhs.max_merit_coeff_increases;
    node["max_qp_solver_failures"] = rhs.max_qp_solver_failures;
    node["merit_coeff_increase_ratio"] = rhs.merit_coeff_increase_ratio;
    node["max_time"] = rhs.max_time;
    node["initial_merit_error_coeff"] = rhs.initial_merit_error_coeff;
    node["inflate_constraints_individually"] = rhs.inflate_constraints_individually;
    node["initial_trust_box_size"] = rhs.initial_trust_box_size;
    node["log_results"] = rhs.log_results;
    node["log_dir"] = rhs.log_dir;
    return node;
  }

  static bool decode(const Node& node, trajopt_sqp::SQPParameters& rhs)
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
    if (const YAML::Node& n = node["max_iterations"])
      rhs.max_iterations = n.as<int>();
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
    if (const YAML::Node& n = node["initial_trust_box_size"])
      rhs.initial_trust_box_size = n.as<double>();
    if (const YAML::Node& n = node["log_results"])
      rhs.log_results = n.as<bool>();
    if (const YAML::Node& n = node["log_dir"])
      rhs.log_dir = n.as<std::string>();

    return true;
  }
};

//=========================== TrajOptCartesianWaypointConfig ===========================
template <>
struct convert<tesseract_planning::TrajOptIfoptCartesianWaypointConfig>
{
  static Node encode(const tesseract_planning::TrajOptIfoptCartesianWaypointConfig& rhs)
  {
    Node node;
    node["enabled"] = rhs.enabled;
    node["use_tolerance_override"] = rhs.use_tolerance_override;
    node["lower_tolerance"] = rhs.lower_tolerance;
    node["upper_tolerance"] = rhs.upper_tolerance;
    node["coeff"] = rhs.coeff;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::TrajOptIfoptCartesianWaypointConfig& rhs)
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

//=========================== TrajOptIfoptJointWaypointConfig ===========================
template <>
struct convert<tesseract_planning::TrajOptIfoptJointWaypointConfig>
{
  static Node encode(const tesseract_planning::TrajOptIfoptJointWaypointConfig& rhs)
  {
    Node node;
    node["enabled"] = rhs.enabled;
    node["use_tolerance_override"] = rhs.use_tolerance_override;
    node["lower_tolerance"] = rhs.lower_tolerance;
    node["upper_tolerance"] = rhs.upper_tolerance;
    node["coeff"] = rhs.coeff;
    return node;
  }

  static bool decode(const Node& node, tesseract_planning::TrajOptIfoptJointWaypointConfig& rhs)
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
