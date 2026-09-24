/**
 * @file settings_utils.h
 * @brief Comparison and validation of PIQP settings
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
#ifndef TESSERACT_MOTION_PLANNERS_PIQP_SETTINGS_UTILS_H
#define TESSERACT_MOTION_PLANNERS_PIQP_SETTINGS_UTILS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <limits>
#include <stdexcept>
#include <string>
#include <piqp/settings.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/utils.h>

namespace tesseract::motion_planners
{
/** @brief Compare every field, with a relative tolerance on floating-point fields */
inline bool operator==(const piqp::Settings<double>& lhs, const piqp::Settings<double>& rhs)
{
  // Purely relative: most fields are tolerances far below any fixed absolute threshold
  const auto near = [](double a, double b) {
    return tesseract::common::almostEqualRelativeAndAbs(
        a, b, 0.0, static_cast<double>(std::numeric_limits<float>::epsilon()));
  };

  bool equal = true;
  equal &= near(lhs.rho_init, rhs.rho_init);
  equal &= near(lhs.delta_init, rhs.delta_init);
  equal &= near(lhs.eps_abs, rhs.eps_abs);
  equal &= near(lhs.eps_rel, rhs.eps_rel);
  equal &= (lhs.check_duality_gap == rhs.check_duality_gap);
  equal &= near(lhs.eps_duality_gap_abs, rhs.eps_duality_gap_abs);
  equal &= near(lhs.eps_duality_gap_rel, rhs.eps_duality_gap_rel);
  equal &= near(lhs.infeasibility_threshold, rhs.infeasibility_threshold);
  equal &= near(lhs.reg_lower_limit, rhs.reg_lower_limit);
  equal &= near(lhs.reg_finetune_lower_limit, rhs.reg_finetune_lower_limit);
  equal &= (lhs.reg_finetune_primal_update_threshold == rhs.reg_finetune_primal_update_threshold);
  equal &= (lhs.reg_finetune_dual_update_threshold == rhs.reg_finetune_dual_update_threshold);
  equal &= (lhs.max_iter == rhs.max_iter);
  equal &= (lhs.max_factor_retires == rhs.max_factor_retires);
  equal &= (lhs.preconditioner_scale_cost == rhs.preconditioner_scale_cost);
  equal &= (lhs.preconditioner_reuse_on_update == rhs.preconditioner_reuse_on_update);
  equal &= (lhs.preconditioner_iter == rhs.preconditioner_iter);
  equal &= near(lhs.tau, rhs.tau);
  equal &= (lhs.kkt_solver == rhs.kkt_solver);
  equal &= (lhs.iterative_refinement_always_enabled == rhs.iterative_refinement_always_enabled);
  equal &= near(lhs.iterative_refinement_eps_abs, rhs.iterative_refinement_eps_abs);
  equal &= near(lhs.iterative_refinement_eps_rel, rhs.iterative_refinement_eps_rel);
  equal &= (lhs.iterative_refinement_max_iter == rhs.iterative_refinement_max_iter);
  equal &= near(lhs.iterative_refinement_min_improvement_rate, rhs.iterative_refinement_min_improvement_rate);
  equal &= near(lhs.iterative_refinement_static_regularization_eps, rhs.iterative_refinement_static_regularization_eps);
  equal &= near(lhs.iterative_refinement_static_regularization_rel, rhs.iterative_refinement_static_regularization_rel);
  equal &= (lhs.verbose == rhs.verbose);
  equal &= (lhs.compute_timings == rhs.compute_timings);
  return equal;
}

/**
 * @brief Check that the settings are valid for PIQP's sparse backend
 * @details kkt_solver must be a sparse_ldlt variant, or sparse_multistage when PIQP was built with BLASFEO, and every
 * other field must pass piqp::Settings::verify_settings().
 * @param settings The settings to check
 * @param profile_name The name that prefixes the error message
 * @throws std::runtime_error if the sparse backend would reject the settings at setup
 */
inline void checkPIQPSettings(const piqp::Settings<double>& settings, const std::string& profile_name)
{
  switch (settings.kkt_solver)
  {
    case piqp::KKTSolver::sparse_ldlt:
    case piqp::KKTSolver::sparse_ldlt_eq_cond:
    case piqp::KKTSolver::sparse_ldlt_ineq_cond:
    case piqp::KKTSolver::sparse_ldlt_cond:
#ifdef PIQP_HAS_BLASFEO
    case piqp::KKTSolver::sparse_multistage:
#endif
      break;
    default:
      throw std::runtime_error(profile_name + ": kkt_solver '" + piqp::kkt_solver_to_string(settings.kkt_solver) +
                               "' is not supported by PIQP's sparse backend");
  }

  if (!settings.verify_settings())
    throw std::runtime_error(profile_name + ": settings rejected by piqp::Settings::verify_settings()");
}
}  // namespace tesseract::motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_PIQP_SETTINGS_UTILS_H
