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
#include <stdexcept>
#include <string>
#include <trajopt_sqp/piqp_solver.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_piqp_solver_profile.h>
#include <tesseract/motion_planners/piqp/settings_utils.h>
#include <tesseract/motion_planners/trajopt_ifopt/yaml_extensions.h>
#include <tesseract/common/profile_plugin_factory.h>

namespace tesseract::motion_planners
{
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
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("TrajOptIfoptPIQPSolverProfile: Failed to parse yaml config! Details: " +
                             std::string(e.what()));
  }

  checkPIQPSettings(qp_settings, "TrajOptIfoptPIQPSolverProfile");
}

std::unique_ptr<trajopt_sqp::TrustRegionSQPSolver> TrajOptIfoptPIQPSolverProfile::create(bool verbose) const
{
  checkPIQPSettings(qp_settings, "TrajOptIfoptPIQPSolverProfile");

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
