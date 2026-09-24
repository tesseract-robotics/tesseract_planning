/**
 * @file trajopt_piqp_solver_profile.cpp
 * @brief A TrajOpt solver profile that solves each QP with PIQP
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
#include <trajopt_sco/piqp_interface.hpp>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/motion_planners/trajopt/profile/trajopt_piqp_solver_profile.h>
#include <tesseract/motion_planners/piqp/settings_utils.h>
#include <tesseract/motion_planners/trajopt/yaml_extensions.h>
#include <tesseract/common/profile_plugin_factory.h>

namespace tesseract::motion_planners
{
TrajOptPIQPSolverProfile::TrajOptPIQPSolverProfile() { sco::PIQPModelConfig::setDefaultPIQPSettings(settings); }

TrajOptPIQPSolverProfile::TrajOptPIQPSolverProfile(const YAML::Node& config,
                                                   const tesseract::common::ProfilePluginFactory& /*plugin_factory*/)
  : TrajOptPIQPSolverProfile()
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
      if (!YAML::convert<piqp::Settings<double>>::decode(n, settings))
        throw std::runtime_error("Failed to decode 'settings'");
    }
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("TrajOptPIQPSolverProfile: Failed to parse yaml config! Details: " +
                             std::string(e.what()));
  }

  checkPIQPSettings(settings, "TrajOptPIQPSolverProfile");
}

sco::ModelType TrajOptPIQPSolverProfile::getSolverType() const { return sco::ModelType::PIQP; }

std::unique_ptr<sco::ModelConfig> TrajOptPIQPSolverProfile::createSolverConfig() const
{
  checkPIQPSettings(settings, "TrajOptPIQPSolverProfile");

  auto config = std::make_unique<sco::PIQPModelConfig>();
  config->settings = settings;
  return config;
}

bool TrajOptPIQPSolverProfile::operator==(const TrajOptPIQPSolverProfile& rhs) const
{
  return (opt_params == rhs.opt_params) && (settings == rhs.settings);
}

bool TrajOptPIQPSolverProfile::operator!=(const TrajOptPIQPSolverProfile& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::motion_planners
