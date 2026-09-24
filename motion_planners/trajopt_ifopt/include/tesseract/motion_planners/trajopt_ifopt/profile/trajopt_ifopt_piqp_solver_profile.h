/**
 * @file trajopt_ifopt_piqp_solver_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PIQP_SOLVER_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PIQP_SOLVER_PROFILE_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <trajopt_sqp/fwd.h>
#include <piqp/settings.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>

namespace YAML
{
class Node;
}

namespace tesseract::motion_planners
{
/** @brief Compare every field, with a relative tolerance on floating-point fields */
bool operator==(const piqp::Settings<double>& lhs, const piqp::Settings<double>& rhs);

/** @brief Solver parameters for TrajOpt Ifopt with the PIQP interior-point QP solver */
class TrajOptIfoptPIQPSolverProfile : public TrajOptIfoptSolverProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptPIQPSolverProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptPIQPSolverProfile>;

  TrajOptIfoptPIQPSolverProfile();

  /** @throws std::runtime_error if the config fails to decode or names a KKT solver the sparse backend lacks */
  TrajOptIfoptPIQPSolverProfile(const YAML::Node& config,
                                const tesseract::common::ProfilePluginFactory& plugin_factory);

  /**
   * @brief The PIQP settings to use
   * @details kkt_solver must be one of the sparse_ldlt variants: PIQP's sparse backend rejects the others at setup.
   */
  piqp::Settings<double> qp_settings;

  /** @throws std::runtime_error if qp_settings.kkt_solver is not a sparse_ldlt variant */
  std::unique_ptr<trajopt_sqp::TrustRegionSQPSolver> create(bool verbose = false) const override;

  bool operator==(const TrajOptIfoptPIQPSolverProfile& rhs) const;
  bool operator!=(const TrajOptIfoptPIQPSolverProfile& rhs) const;
};
}  // namespace tesseract::motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PIQP_SOLVER_PROFILE_H
