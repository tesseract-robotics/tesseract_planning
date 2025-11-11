/**
 * @file trajopt_ifopt_osqp_solver_profile.cpp
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

#ifdef _WIN32
// Macros to avoid Windows.h conflicts
#define NOGDI
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#endif

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <OsqpEigen/OsqpEigen.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/sqp_callback.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <yaml-cpp/yaml.h>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>
#include <tesseract_motion_planners/trajopt_ifopt/yaml_extensions.h>
#include <tesseract_common/profile_plugin_factory.h>

namespace boost::serialization
{
template <class Archive>
void serialize(Archive& ar, OsqpEigen::Settings& osqp_eigen_settings, const unsigned int /*version*/)
{
  OSQPSettings& settings = *osqp_eigen_settings.getSettings();
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

template <class Archive>
void TrajOptIfoptOSQPSolverProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TrajOptIfoptSolverProfile);
  ar& BOOST_SERIALIZATION_NVP(qp_settings);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_FREE_ARCHIVES_INSTANTIATE(OsqpEigen::Settings)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptOSQPSolverProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptOSQPSolverProfile)
