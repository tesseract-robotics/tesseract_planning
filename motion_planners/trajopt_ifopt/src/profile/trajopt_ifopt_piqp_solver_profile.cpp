/**
 * @file trajopt_ifopt_piqp_solver_profile.cpp
 * @brief A TrajOpt Ifopt solver profile that solves each QP with PIQP
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <limits>
#include <stdexcept>
#include <string>
#include <trajopt_sqp/piqp_solver.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_piqp_solver_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/yaml_extensions.h>
#include <tesseract/common/profile_plugin_factory.h>
#include <tesseract/common/utils.h>

namespace tesseract::motion_planners
{
namespace
{
void checkKKTSolver(const piqp::Settings<double>& settings)
{
  switch (settings.kkt_solver)
  {
    case piqp::KKTSolver::sparse_ldlt:
    case piqp::KKTSolver::sparse_ldlt_eq_cond:
    case piqp::KKTSolver::sparse_ldlt_ineq_cond:
    case piqp::KKTSolver::sparse_ldlt_cond:
      return;
    default:
      throw std::runtime_error(std::string("TrajOptIfoptPIQPSolverProfile: kkt_solver '") +
                               piqp::kkt_solver_to_string(settings.kkt_solver) +
                               "' is not supported, use a sparse_ldlt variant");
  }
}
}  // namespace

bool operator==(const piqp::Settings<double>& lhs, const piqp::Settings<double>& rhs)
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

TrajOptIfoptPIQPSolverProfile::TrajOptIfoptPIQPSolverProfile()
{
  trajopt_sqp::PIQPSolver::setDefaultPIQPSettings(qp_settings);
}

TrajOptIfoptPIQPSolverProfile::TrajOptIfoptPIQPSolverProfile(
    const YAML::Node& config,
    const tesseract::common::ProfilePluginFactory& /*plugin_factory*/)
  : TrajOptIfoptPIQPSolverProfile()
{
  try
  {
    if (YAML::Node n = config["opt_params"])
    {
      if (!YAML::convert<trajopt_sqp::SQPParameters>::decode(n, opt_params))
        throw std::runtime_error("Failed to decode 'opt_params'");
    }

    if (YAML::Node n = config["settings"])
    {
      if (!YAML::convert<piqp::Settings<double>>::decode(n, qp_settings))
        throw std::runtime_error("Failed to decode 'settings'");
    }

    checkKKTSolver(qp_settings);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("TrajOptIfoptPIQPSolverProfile: Failed to parse yaml config! Details: " +
                             std::string(e.what()));
  }
}

std::unique_ptr<trajopt_sqp::TrustRegionSQPSolver> TrajOptIfoptPIQPSolverProfile::create(bool verbose) const
{
  checkKKTSolver(qp_settings);

  auto qp_solver = std::make_shared<trajopt_sqp::PIQPSolver>();
  qp_solver->settings = qp_settings;
  qp_solver->settings.verbose |= verbose;

  return createSolver(std::move(qp_solver), verbose);
}

bool TrajOptIfoptPIQPSolverProfile::operator==(const TrajOptIfoptPIQPSolverProfile& rhs) const
{
  return (opt_params == rhs.opt_params) && (qp_settings == rhs.qp_settings);
}

bool TrajOptIfoptPIQPSolverProfile::operator!=(const TrajOptIfoptPIQPSolverProfile& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract::motion_planners
