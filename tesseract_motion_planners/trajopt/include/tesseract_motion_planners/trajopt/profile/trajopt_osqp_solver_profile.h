/**
 * @file trajopt_osqp_solver_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_OSQP_SOLVER_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_OSQP_SOLVER_PROFILE_H

#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <trajopt_sco/osqp_interface.hpp>

namespace YAML
{
class Node;
}

namespace tesseract::motion_planners
{
bool operator==(const OSQPSettings& lhs, const OSQPSettings& rhs);

/** @brief The contains the OSQP Solver and Trust Region Parameters available for setting up TrajOpt */
class TrajOptOSQPSolverProfile : public TrajOptSolverProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptOSQPSolverProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptOSQPSolverProfile>;

  TrajOptOSQPSolverProfile();
  TrajOptOSQPSolverProfile(const YAML::Node& config, const tesseract::common::ProfilePluginFactory& plugin_factory);

  OSQPSettings settings{};

  bool update_workspace{ false };

  sco::ModelType getSolverType() const override;

  std::unique_ptr<sco::ModelConfig> createSolverConfig() const override;

  bool operator==(const TrajOptOSQPSolverProfile& rhs) const;
  bool operator!=(const TrajOptOSQPSolverProfile& rhs) const;
};
}  // namespace tesseract::motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_OSQP_SOLVER_PROFILE_H
