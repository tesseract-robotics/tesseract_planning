/**
 * @file trajopt_osqp_solver_profile.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date December 13, 2020
 * @version TODO
 * @bug No known bugs
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
#include <trajopt/problem_description.hpp>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/profile_plugin_factory.h>
#include <tesseract_common/utils.h>
#include <tesseract_motion_planners/trajopt/yaml_extensions.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>

namespace tesseract_planning
{
bool operator==(const OSQPSettings& lhs, const OSQPSettings& rhs)
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (lhs.device == rhs.device);
  equal &= (lhs.linsys_solver == rhs.linsys_solver);
  equal &= (lhs.allocate_solution == rhs.allocate_solution);
  equal &= (lhs.verbose == rhs.verbose);
  equal &= (lhs.profiler_level == rhs.profiler_level);
  equal &= (lhs.warm_starting == rhs.warm_starting);
  equal &= (lhs.scaling == rhs.scaling);
  equal &= (lhs.polishing == rhs.polishing);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs.rho, rhs.rho, max_diff);
  equal &= (lhs.rho_is_vec == rhs.rho_is_vec);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs.sigma, rhs.sigma, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs.alpha, rhs.alpha, max_diff);
  equal &= (lhs.cg_max_iter == rhs.cg_max_iter);
  equal &= (lhs.cg_tol_reduction == rhs.cg_tol_reduction);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs.cg_tol_fraction, rhs.cg_tol_fraction, max_diff);
  equal &= (lhs.cg_precond == rhs.cg_precond);
  equal &= (lhs.adaptive_rho == rhs.adaptive_rho);
  equal &= (lhs.adaptive_rho_interval == rhs.adaptive_rho_interval);
  equal &=
      tesseract_common::almostEqualRelativeAndAbs(lhs.adaptive_rho_tolerance, rhs.adaptive_rho_tolerance, max_diff);
  equal &= (lhs.max_iter == rhs.max_iter);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs.eps_abs, rhs.eps_abs, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs.eps_rel, rhs.eps_rel, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs.eps_prim_inf, rhs.eps_prim_inf, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs.eps_dual_inf, rhs.eps_dual_inf, max_diff);
  equal &= (lhs.scaled_termination == rhs.scaled_termination);
  equal &= (lhs.check_termination == rhs.check_termination);
  equal &= (lhs.check_dualgap == rhs.check_dualgap);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs.time_limit, rhs.time_limit, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(lhs.delta, rhs.delta, max_diff);
  equal &= (lhs.polish_refine_iter == rhs.polish_refine_iter);
  return equal;
}

TrajOptOSQPSolverProfile::TrajOptOSQPSolverProfile() { sco::OSQPModelConfig::setDefaultOSQPSettings(settings); }

TrajOptOSQPSolverProfile::TrajOptOSQPSolverProfile(const YAML::Node& config,
                                                   const tesseract_common::ProfilePluginFactory& /*plugin_factory*/)
  : TrajOptOSQPSolverProfile()
{
  try
  {
    if (YAML::Node n = config["opt_params"])
    {
      if (!YAML::convert<sco::BasicTrustRegionSQPParameters>::decode(n, opt_params))
        throw std::runtime_error("Failed to decode 'opt_params'");
    }

    if (YAML::Node n = config["settings"])
    {
      if (!YAML::convert<OSQPSettings>::decode(n, settings))
        throw std::runtime_error("Failed to decode 'settings'");
    }

    if (YAML::Node n = config["update_workspace"])
      update_workspace = n.as<bool>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("TrajOptOSQPSolverProfile: Failed to parse yaml config! Details: " +
                             std::string(e.what()));
  }
}

sco::ModelType TrajOptOSQPSolverProfile::getSolverType() const { return sco::ModelType::OSQP; }

std::unique_ptr<sco::ModelConfig> TrajOptOSQPSolverProfile::createSolverConfig() const
{
  auto config = std::make_unique<sco::OSQPModelConfig>();
  config->settings = settings;
  config->update_workspace = update_workspace;
  return config;
}

bool TrajOptOSQPSolverProfile::operator==(const TrajOptOSQPSolverProfile& rhs) const
{
  bool equal = true;
  equal &= (opt_params == rhs.opt_params);
  equal &= (settings == rhs.settings);
  equal &= (update_workspace == rhs.update_workspace);
  return equal;
}

bool TrajOptOSQPSolverProfile::operator!=(const TrajOptOSQPSolverProfile& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_planning
