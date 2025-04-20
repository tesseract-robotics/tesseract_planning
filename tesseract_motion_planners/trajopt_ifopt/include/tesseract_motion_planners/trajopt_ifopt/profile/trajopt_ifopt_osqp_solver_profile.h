/**
 * @file trajopt_osqp_solver_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_OSQP_SOLVER_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_OSQP_SOLVER_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <OsqpEigen/Settings.hpp>
#include <trajopt_sqp/fwd.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>

namespace boost::serialization
{
template <class Archive>
void serialize(Archive& ar, OsqpEigen::Settings& osqp_eigen_settings, const unsigned int version);  // NOLINT

template <class Archive>
void serialize(Archive& ar, trajopt_sqp::SQPParameters& params, const unsigned int version);  // NOLINT

}  // namespace boost::serialization

namespace tesseract_planning
{
/** @brief The contains the default solver parameters available for setting up TrajOpt */
class TrajOptIfoptOSQPSolverProfile : public TrajOptIfoptSolverProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptOSQPSolverProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptOSQPSolverProfile>;

  TrajOptIfoptOSQPSolverProfile();
  TrajOptIfoptOSQPSolverProfile(TrajOptIfoptOSQPSolverProfile&&) = default;
  TrajOptIfoptOSQPSolverProfile& operator=(TrajOptIfoptOSQPSolverProfile&&) = default;

  // Delete because OsqpEigen::Settings stores raw pointer
  TrajOptIfoptOSQPSolverProfile(const TrajOptIfoptOSQPSolverProfile&) = delete;
  TrajOptIfoptOSQPSolverProfile& operator=(const TrajOptIfoptOSQPSolverProfile&) = delete;

  ~TrajOptIfoptOSQPSolverProfile() override = default;

  /** @brief The OSQP convex solver settings to use */
  std::unique_ptr<OsqpEigen::Settings> qp_settings;

  std::unique_ptr<trajopt_sqp::TrustRegionSQPSolver> create(bool verbose = false) const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT

  /** @brief Optimization callbacks */
  virtual std::vector<std::shared_ptr<trajopt_sqp::SQPCallback>> createOptimizationCallbacks() const;
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptIfoptOSQPSolverProfile)

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_OSQP_SOLVER_PROFILE_H
