/**
 * @file trajopt_ifopt_osqp_solver_profile.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date December 13, 2020
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <OsqpEigen/OsqpEigen.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/sqp_callback.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>
#include <tesseract_motion_planners/trajopt_ifopt/yaml_extensions.h>
#include <tesseract_common/profile_plugin_factory.h>

namespace tesseract_planning
{
bool operator==(const OsqpEigen::Settings& lhs, const OsqpEigen::Settings& rhs)
{
  OSQPSettings& lhs_settings = *lhs.getSettings();
  OSQPSettings& rhs_settings = *rhs.getSettings();

  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (lhs_settings.device == rhs_settings.device);
  equal &= (lhs_settings.linsys_solver == rhs_settings.linsys_solver);
  equal &= (lhs_settings.allocate_solution == rhs_settings.allocate_solution);
  equal &= (lhs_settings.verbose == rhs_settings.verbose);
  equal &= (lhs_settings.profiler_level == rhs_settings.profiler_level);
  equal &= (lhs_settings.warm_starting == rhs_settings.warm_starting);
  equal &= (lhs_settings.scaling == rhs_settings.scaling);
  equal &= (lhs_settings.polishing == rhs_settings.polishing);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs_settings.rho, rhs_settings.rho, max_diff);
  equal &= (lhs_settings.rho_is_vec == rhs_settings.rho_is_vec);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs_settings.sigma, rhs_settings.sigma, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs_settings.alpha, rhs_settings.alpha, max_diff);
  equal &= (lhs_settings.cg_max_iter == rhs_settings.cg_max_iter);
  equal &= (lhs_settings.cg_tol_reduction == rhs_settings.cg_tol_reduction);
  equal &=
      tesseract_common::almostEqualRelativeAndAbs(lhs_settings.cg_tol_fraction, rhs_settings.cg_tol_fraction, max_diff);
  equal &= (lhs_settings.cg_precond == rhs_settings.cg_precond);
  equal &= (lhs_settings.adaptive_rho == rhs_settings.adaptive_rho);
  equal &= (lhs_settings.adaptive_rho_interval == rhs_settings.adaptive_rho_interval);
  equal &= tesseract_common::almostEqualRelativeAndAbs(
      lhs_settings.adaptive_rho_tolerance, rhs_settings.adaptive_rho_tolerance, max_diff);
  equal &= (lhs_settings.max_iter == rhs_settings.max_iter);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs_settings.eps_abs, rhs_settings.eps_abs, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs_settings.eps_rel, rhs_settings.eps_rel, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs_settings.eps_prim_inf, rhs_settings.eps_prim_inf, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs_settings.eps_dual_inf, rhs_settings.eps_dual_inf, max_diff);
  equal &= (lhs_settings.scaled_termination == rhs_settings.scaled_termination);
  equal &= (lhs_settings.check_termination == rhs_settings.check_termination);
  equal &= (lhs_settings.check_dualgap == rhs_settings.check_dualgap);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs_settings.time_limit, rhs_settings.time_limit, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs_settings.delta, rhs_settings.delta, max_diff);
  equal &= (lhs_settings.polish_refine_iter == rhs_settings.polish_refine_iter);
  return equal;
}

TrajOptIfoptOSQPSolverProfile::TrajOptIfoptOSQPSolverProfile()
{
  qp_settings = std::make_unique<OsqpEigen::Settings>();
  trajopt_sqp::OSQPEigenSolver::setDefaultOSQPSettings(*qp_settings);
}

TrajOptIfoptOSQPSolverProfile::TrajOptIfoptOSQPSolverProfile(
    const YAML::Node& config,
    const tesseract_common::ProfilePluginFactory& /*plugin_factory*/)
  : TrajOptIfoptOSQPSolverProfile()
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
      if (!YAML::convert<OsqpEigen::Settings>::decode(n, *qp_settings))
        throw std::runtime_error("Failed to decode 'qp_settings'");
    }
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("TrajOptOSQPSolverProfile: Failed to parse yaml config! Details: " +
                             std::string(e.what()));
  }
}

std::unique_ptr<trajopt_sqp::TrustRegionSQPSolver> TrajOptIfoptOSQPSolverProfile::create(bool verbose) const
{
  // Create QP Solver
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();

  // There seems to be no way to set objects solver_->settings() (OsqpEigen::Settings)
  // or solver_->settings()->getSettings() (OSQPSettings) at once
  copyOSQPEigenSettings(*qp_solver->solver_->settings(), *qp_settings);
  qp_solver->solver_->settings()->setVerbosity((qp_settings->getSettings()->verbose != 0) || verbose);

  auto solver = std::make_unique<trajopt_sqp::TrustRegionSQPSolver>(qp_solver);
  solver->params = opt_params;
  solver->verbose = verbose;

  // Add all callbacks
  std::vector<std::shared_ptr<trajopt_sqp::SQPCallback>> callbacks = createOptimizationCallbacks();
  for (const trajopt_sqp::SQPCallback::Ptr& callback : callbacks)
    solver->registerCallback(callback);

  return solver;
}

std::vector<std::shared_ptr<trajopt_sqp::SQPCallback>>
TrajOptIfoptOSQPSolverProfile::createOptimizationCallbacks() const
{
  return {};
}

bool TrajOptIfoptOSQPSolverProfile::operator==(const TrajOptIfoptOSQPSolverProfile& rhs) const
{
  bool equal = true;
  equal &= (opt_params == rhs.opt_params);
  if (qp_settings == nullptr && rhs.qp_settings == nullptr)
  {
    equal &= true;
  }
  else if (qp_settings != nullptr && rhs.qp_settings != nullptr)
  {
    equal &= (*qp_settings == *rhs.qp_settings);
  }
  else
  {
    equal &= false;
  }
  return equal;
}

bool TrajOptIfoptOSQPSolverProfile::operator!=(const TrajOptIfoptOSQPSolverProfile& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract_planning
