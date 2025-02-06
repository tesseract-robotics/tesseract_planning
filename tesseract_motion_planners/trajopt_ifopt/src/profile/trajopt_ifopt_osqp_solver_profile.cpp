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
#include <OsqpEigen/Settings.hpp>
#include <OsqpEigen/OsqpEigen.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/sqp_callback.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unique_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

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
  ar& boost::serialization::make_nvp("polish", settings.polish);
  ar& boost::serialization::make_nvp("polish_refine_iter", settings.polish_refine_iter);
  ar& boost::serialization::make_nvp("verbose", settings.verbose);
  ar& boost::serialization::make_nvp("scaled_termination", settings.scaled_termination);
  ar& boost::serialization::make_nvp("check_termination", settings.check_termination);
  ar& boost::serialization::make_nvp("warm_start", settings.warm_start);
  ar& boost::serialization::make_nvp("time_limit", settings.time_limit);
}

template <class Archive>
void serialize(Archive& ar, trajopt_sqp::SQPParameters& params, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("improve_ratio_threshold", params.improve_ratio_threshold);
  ar& boost::serialization::make_nvp("min_trust_box_size", params.min_trust_box_size);
  ar& boost::serialization::make_nvp("min_approx_improve", params.min_approx_improve);
  ar& boost::serialization::make_nvp("min_approx_improve_frac", params.min_approx_improve_frac);
  ar& boost::serialization::make_nvp("max_iter", params.max_iterations);
  ar& boost::serialization::make_nvp("trust_shrink_ratio", params.trust_shrink_ratio);
  ar& boost::serialization::make_nvp("trust_expand_ratio", params.trust_expand_ratio);
  ar& boost::serialization::make_nvp("cnt_tolerance", params.cnt_tolerance);
  ar& boost::serialization::make_nvp("max_merit_coeff_increases", params.max_merit_coeff_increases);
  ar& boost::serialization::make_nvp("max_qp_solver_failures", params.max_qp_solver_failures);
  ar& boost::serialization::make_nvp("merit_coeff_increase_ratio", params.merit_coeff_increase_ratio);
  ar& boost::serialization::make_nvp("max_time", params.max_time);
  ar& boost::serialization::make_nvp("initial_merit_error_coeff", params.initial_merit_error_coeff);
  ar& boost::serialization::make_nvp("inflate_constraints_individually", params.inflate_constraints_individually);
  ar& boost::serialization::make_nvp("trust_box_size", params.initial_trust_box_size);
  ar& boost::serialization::make_nvp("log_results", params.log_results);
  ar& boost::serialization::make_nvp("log_dir", params.log_dir);
  // ar& boost::serialization::make_nvp("num_threads", params.num_threads);
}
}  // namespace boost::serialization

namespace tesseract_planning
{
TrajOptIfoptOSQPSolverProfile::TrajOptIfoptOSQPSolverProfile()
{
  qp_settings = std::make_unique<OsqpEigen::Settings>();
  qp_settings->setVerbosity(false);
  qp_settings->setWarmStart(true);
  qp_settings->setPolish(true);
  qp_settings->setAdaptiveRho(true);
  qp_settings->setMaxIteration(8192);
  qp_settings->setAbsoluteTolerance(1e-4);
  qp_settings->setRelativeTolerance(1e-6);
}

TrajOptIfoptOSQPSolverProfile::~TrajOptIfoptOSQPSolverProfile() = default;

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
  ar& BOOST_SERIALIZATION_NVP(opt_params);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_FREE_ARCHIVES_INSTANTIATE(OsqpEigen::Settings)
TESSERACT_SERIALIZE_FREE_ARCHIVES_INSTANTIATE(trajopt_sqp::SQPParameters)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptOSQPSolverProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptOSQPSolverProfile)
