/**
 * @file yaml_extensions.h
 * @brief YAML conversions for PIQP settings
 *
 * @author Roelof Oomen
 * @date September 24, 2026
 *
 * @copyright Copyright (c) 2026, Roelof Oomen
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
 *
 * The motion_planners component does not depend on PIQP. Include this header only from a planner component whose
 * solver library was built with PIQP; that library supplies the PIQP include path.
 */
#ifndef TESSERACT_MOTION_PLANNERS_PIQP_YAML_EXTENSIONS_H
#define TESSERACT_MOTION_PLANNERS_PIQP_YAML_EXTENSIONS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <array>
#include <string>
#include <yaml-cpp/yaml.h>
#include <piqp/settings.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace YAML
{
//=========================== piqp::KKTSolver Enum ===========================
template <>
struct convert<piqp::KKTSolver>
{
  static Node encode(const piqp::KKTSolver& rhs) { return Node(std::string(piqp::kkt_solver_to_string(rhs))); }

  static bool decode(const Node& node, piqp::KKTSolver& rhs)
  {
    static constexpr std::array<piqp::KKTSolver, 6> values = {
      piqp::KKTSolver::dense_cholesky,        piqp::KKTSolver::sparse_ldlt,      piqp::KKTSolver::sparse_ldlt_eq_cond,
      piqp::KKTSolver::sparse_ldlt_ineq_cond, piqp::KKTSolver::sparse_ldlt_cond, piqp::KKTSolver::sparse_multistage
    };

    if (!node.IsScalar())
      return false;

    for (const piqp::KKTSolver value : values)
    {
      if (node.Scalar() == piqp::kkt_solver_to_string(value))
      {
        rhs = value;
        return true;
      }
    }
    return false;
  }
};

//=========================== piqp::Settings ===========================
template <>
struct convert<piqp::Settings<double>>
{
  static Node encode(const piqp::Settings<double>& rhs)
  {
    Node node;
    node["rho_init"] = rhs.rho_init;
    node["delta_init"] = rhs.delta_init;
    node["eps_abs"] = rhs.eps_abs;
    node["eps_rel"] = rhs.eps_rel;
    node["check_duality_gap"] = rhs.check_duality_gap;
    node["eps_duality_gap_abs"] = rhs.eps_duality_gap_abs;
    node["eps_duality_gap_rel"] = rhs.eps_duality_gap_rel;
    node["infeasibility_threshold"] = rhs.infeasibility_threshold;
    node["reg_lower_limit"] = rhs.reg_lower_limit;
    node["reg_finetune_lower_limit"] = rhs.reg_finetune_lower_limit;
    node["reg_finetune_primal_update_threshold"] = rhs.reg_finetune_primal_update_threshold;
    node["reg_finetune_dual_update_threshold"] = rhs.reg_finetune_dual_update_threshold;
    node["max_iter"] = rhs.max_iter;
    node["max_factor_retires"] = rhs.max_factor_retires;
    node["preconditioner_scale_cost"] = rhs.preconditioner_scale_cost;
    node["preconditioner_reuse_on_update"] = rhs.preconditioner_reuse_on_update;
    node["preconditioner_iter"] = rhs.preconditioner_iter;
    node["tau"] = rhs.tau;
    node["kkt_solver"] = rhs.kkt_solver;
    node["iterative_refinement_always_enabled"] = rhs.iterative_refinement_always_enabled;
    node["iterative_refinement_eps_abs"] = rhs.iterative_refinement_eps_abs;
    node["iterative_refinement_eps_rel"] = rhs.iterative_refinement_eps_rel;
    node["iterative_refinement_max_iter"] = rhs.iterative_refinement_max_iter;
    node["iterative_refinement_min_improvement_rate"] = rhs.iterative_refinement_min_improvement_rate;
    node["iterative_refinement_static_regularization_eps"] = rhs.iterative_refinement_static_regularization_eps;
    node["iterative_refinement_static_regularization_rel"] = rhs.iterative_refinement_static_regularization_rel;
    node["verbose"] = rhs.verbose;
    node["compute_timings"] = rhs.compute_timings;
    return node;
  }

  static bool decode(const Node& node, piqp::Settings<double>& rhs)
  {
    if (const YAML::Node& n = node["rho_init"])
      rhs.rho_init = n.as<double>();
    if (const YAML::Node& n = node["delta_init"])
      rhs.delta_init = n.as<double>();
    if (const YAML::Node& n = node["eps_abs"])
      rhs.eps_abs = n.as<double>();
    if (const YAML::Node& n = node["eps_rel"])
      rhs.eps_rel = n.as<double>();
    if (const YAML::Node& n = node["check_duality_gap"])
      rhs.check_duality_gap = n.as<bool>();
    if (const YAML::Node& n = node["eps_duality_gap_abs"])
      rhs.eps_duality_gap_abs = n.as<double>();
    if (const YAML::Node& n = node["eps_duality_gap_rel"])
      rhs.eps_duality_gap_rel = n.as<double>();
    if (const YAML::Node& n = node["infeasibility_threshold"])
      rhs.infeasibility_threshold = n.as<double>();
    if (const YAML::Node& n = node["reg_lower_limit"])
      rhs.reg_lower_limit = n.as<double>();
    if (const YAML::Node& n = node["reg_finetune_lower_limit"])
      rhs.reg_finetune_lower_limit = n.as<double>();
    if (const YAML::Node& n = node["reg_finetune_primal_update_threshold"])
      rhs.reg_finetune_primal_update_threshold = n.as<piqp::isize>();
    if (const YAML::Node& n = node["reg_finetune_dual_update_threshold"])
      rhs.reg_finetune_dual_update_threshold = n.as<piqp::isize>();
    if (const YAML::Node& n = node["max_iter"])
      rhs.max_iter = n.as<piqp::isize>();
    if (const YAML::Node& n = node["max_factor_retires"])
      rhs.max_factor_retires = n.as<piqp::isize>();
    if (const YAML::Node& n = node["preconditioner_scale_cost"])
      rhs.preconditioner_scale_cost = n.as<bool>();
    if (const YAML::Node& n = node["preconditioner_reuse_on_update"])
      rhs.preconditioner_reuse_on_update = n.as<bool>();
    if (const YAML::Node& n = node["preconditioner_iter"])
      rhs.preconditioner_iter = n.as<piqp::isize>();
    if (const YAML::Node& n = node["tau"])
      rhs.tau = n.as<double>();
    if (const YAML::Node& n = node["kkt_solver"])
      rhs.kkt_solver = n.as<piqp::KKTSolver>();
    if (const YAML::Node& n = node["iterative_refinement_always_enabled"])
      rhs.iterative_refinement_always_enabled = n.as<bool>();
    if (const YAML::Node& n = node["iterative_refinement_eps_abs"])
      rhs.iterative_refinement_eps_abs = n.as<double>();
    if (const YAML::Node& n = node["iterative_refinement_eps_rel"])
      rhs.iterative_refinement_eps_rel = n.as<double>();
    if (const YAML::Node& n = node["iterative_refinement_max_iter"])
      rhs.iterative_refinement_max_iter = n.as<piqp::isize>();
    if (const YAML::Node& n = node["iterative_refinement_min_improvement_rate"])
      rhs.iterative_refinement_min_improvement_rate = n.as<double>();
    if (const YAML::Node& n = node["iterative_refinement_static_regularization_eps"])
      rhs.iterative_refinement_static_regularization_eps = n.as<double>();
    if (const YAML::Node& n = node["iterative_refinement_static_regularization_rel"])
      rhs.iterative_refinement_static_regularization_rel = n.as<double>();
    if (const YAML::Node& n = node["verbose"])
      rhs.verbose = n.as<bool>();
    if (const YAML::Node& n = node["compute_timings"])
      rhs.compute_timings = n.as<bool>();
    return true;
  }
};
}  // namespace YAML

#endif  // TESSERACT_MOTION_PLANNERS_PIQP_YAML_EXTENSIONS_H
