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
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/profile_plugin_factory.h>
#include <tesseract_motion_planners/trajopt/yaml_extensions.h>

#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>

namespace boost::serialization
{
template <class Archive>
void serialize(Archive& ar, OSQPSettings& settings, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("rho", settings.rho);
  ar& boost::serialization::make_nvp("sigma", settings.sigma);
  ar& boost::serialization::make_nvp("scaling", settings.scaling);
  ar& boost::serialization::make_nvp("adaptive_rho", settings.adaptive_rho);
  ar& boost::serialization::make_nvp("adaptive_rho_interval", settings.adaptive_rho_interval);
  ar& boost::serialization::make_nvp("adaptive_rho_tolerance", settings.adaptive_rho_tolerance);
  ar& boost::serialization::make_nvp("adaptive_rho_fraction", settings.adaptive_rho_fraction);
  ar& boost::serialization::make_nvp("max_iter", settings.max_iter);
  ar& boost::serialization::make_nvp("eps_abs", settings.eps_abs);
  ar& boost::serialization::make_nvp("eps_rel", settings.eps_rel);
  ar& boost::serialization::make_nvp("eps_prim_inf", settings.eps_prim_inf);
  ar& boost::serialization::make_nvp("eps_dual_inf", settings.eps_dual_inf);
  ar& boost::serialization::make_nvp("alpha", settings.alpha);
  ar& boost::serialization::make_nvp("linsys_solver", settings.linsys_solver);
  ar& boost::serialization::make_nvp("delta", settings.delta);
  ar& boost::serialization::make_nvp("polishing", settings.polishing);
  ar& boost::serialization::make_nvp("polish_refine_iter", settings.polish_refine_iter);
  ar& boost::serialization::make_nvp("verbose", settings.verbose);
  ar& boost::serialization::make_nvp("scaled_termination", settings.scaled_termination);
  ar& boost::serialization::make_nvp("check_termination", settings.check_termination);
  ar& boost::serialization::make_nvp("warm_starting", settings.warm_starting);
  ar& boost::serialization::make_nvp("time_limit", settings.time_limit);
  ar& boost::serialization::make_nvp("allocate_solution", settings.allocate_solution);
  ar& boost::serialization::make_nvp("cg_max_iter", settings.cg_max_iter);
  ar& boost::serialization::make_nvp("cg_precond", settings.cg_precond);
  ar& boost::serialization::make_nvp("cg_tol_fraction", settings.cg_tol_fraction);
  ar& boost::serialization::make_nvp("cg_tol_reduction", settings.cg_tol_reduction);
  ar& boost::serialization::make_nvp("check_dualgap", settings.check_dualgap);
  ar& boost::serialization::make_nvp("device", settings.device);
  ar& boost::serialization::make_nvp("profiler_level", settings.profiler_level);
  ar& boost::serialization::make_nvp("rho_is_vec", settings.rho_is_vec);
}
}  // namespace boost::serialization

namespace tesseract_planning
{
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

template <class Archive>
void TrajOptOSQPSolverProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TrajOptSolverProfile);
  ar& BOOST_SERIALIZATION_NVP(settings);
  ar& BOOST_SERIALIZATION_NVP(update_workspace);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_FREE_ARCHIVES_INSTANTIATE(OSQPSettings)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptOSQPSolverProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptOSQPSolverProfile)
