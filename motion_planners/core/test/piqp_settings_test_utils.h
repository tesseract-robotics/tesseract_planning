/**
 * @file piqp_settings_test_utils.h
 * @brief A PIQP settings fixture shared by the planner YAML tests
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
 */
#ifndef TESSERACT_MOTION_PLANNERS_PIQP_SETTINGS_TEST_UTILS_H
#define TESSERACT_MOTION_PLANNERS_PIQP_SETTINGS_TEST_UTILS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <piqp/settings.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract::motion_planners::test_suite
{
/** @brief A YAML mapping that sets every PIQP setting to a valid value other than its default */
inline constexpr const char* PIQP_ALL_SETTINGS_YAML = R"(
rho_init: 2e-6
delta_init: 3e-4
eps_abs: 5e-5
eps_rel: 7e-7
check_duality_gap: false
eps_duality_gap_abs: 2e-7
eps_duality_gap_rel: 3e-8
infeasibility_threshold: 0.8
reg_lower_limit: 2e-10
reg_finetune_lower_limit: 3e-13
reg_finetune_primal_update_threshold: 5
reg_finetune_dual_update_threshold: 6
max_iter: 123
max_factor_retires: 4
preconditioner_scale_cost: true
preconditioner_reuse_on_update: true
preconditioner_iter: 3
tau: 0.95
kkt_solver: sparse_ldlt_ineq_cond
iterative_refinement_always_enabled: true
iterative_refinement_eps_abs: 2e-12
iterative_refinement_eps_rel: 3e-12
iterative_refinement_max_iter: 8
iterative_refinement_min_improvement_rate: 4.5
iterative_refinement_static_regularization_eps: 2e-8
iterative_refinement_static_regularization_rel: 1e-30
verbose: true
compute_timings: true
)";

/** @brief The settings PIQP_ALL_SETTINGS_YAML decodes to */
inline piqp::Settings<double> piqpAllSettings()
{
  piqp::Settings<double> settings;
  settings.rho_init = 2e-6;
  settings.delta_init = 3e-4;
  settings.eps_abs = 5e-5;
  settings.eps_rel = 7e-7;
  settings.check_duality_gap = false;
  settings.eps_duality_gap_abs = 2e-7;
  settings.eps_duality_gap_rel = 3e-8;
  settings.infeasibility_threshold = 0.8;
  settings.reg_lower_limit = 2e-10;
  settings.reg_finetune_lower_limit = 3e-13;
  settings.reg_finetune_primal_update_threshold = 5;
  settings.reg_finetune_dual_update_threshold = 6;
  settings.max_iter = 123;
  settings.max_factor_retires = 4;
  settings.preconditioner_scale_cost = true;
  settings.preconditioner_reuse_on_update = true;
  settings.preconditioner_iter = 3;
  settings.tau = 0.95;
  settings.kkt_solver = piqp::KKTSolver::sparse_ldlt_ineq_cond;
  settings.iterative_refinement_always_enabled = true;
  settings.iterative_refinement_eps_abs = 2e-12;
  settings.iterative_refinement_eps_rel = 3e-12;
  settings.iterative_refinement_max_iter = 8;
  settings.iterative_refinement_min_improvement_rate = 4.5;
  settings.iterative_refinement_static_regularization_eps = 2e-8;
  settings.iterative_refinement_static_regularization_rel = 1e-30;
  settings.verbose = true;
  settings.compute_timings = true;
  return settings;
}
}  // namespace tesseract::motion_planners::test_suite

#endif  // TESSERACT_MOTION_PLANNERS_PIQP_SETTINGS_TEST_UTILS_H
