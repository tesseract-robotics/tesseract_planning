/**
 * @file trajopt_profile.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <trajopt_ifopt/fwd.h>
#include <trajopt_sqp/fwd.h>
#include <trajopt_sqp/types.h>
#include <trajopt_ifopt/variable_sets/node.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <tesseract_environment/fwd.h>
#include <tesseract_command_language/fwd.h>
#include <tesseract_common/profile.h>

namespace ifopt
{
class ConstraintSet;
}

namespace tesseract_planning
{
/** @brief Structure to store TrajOpt IFOPT constrant and cost term infos */
struct TrajOptIfoptTermInfos
{
  std::vector<std::shared_ptr<ifopt::ConstraintSet>> constraints;
  std::vector<std::shared_ptr<ifopt::ConstraintSet>> squared_costs;
  std::vector<std::shared_ptr<ifopt::ConstraintSet>> absolute_costs;
  std::vector<std::shared_ptr<ifopt::ConstraintSet>> hinge_costs;
};

/** @brief Structure to store TrajOpt waypoint cost and constrant term infos */
struct TrajOptIfoptWaypointInfo
{
  TrajOptIfoptTermInfos term_infos;
  std::unique_ptr<trajopt_ifopt::Node> node;
  bool fixed{ false };
};

class TrajOptIfoptMoveProfile : public tesseract_common::Profile
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptMoveProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptMoveProfile>;

  TrajOptIfoptMoveProfile();

  virtual TrajOptIfoptWaypointInfo create(const MoveInstructionPoly& move_instruction,
                                          const tesseract_common::ManipulatorInfo& composite_manip_info,
                                          const std::shared_ptr<const tesseract_environment::Environment>& env,
                                          int index) const = 0;
};

class TrajOptIfoptCompositeProfile : public tesseract_common::Profile
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptCompositeProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptCompositeProfile>;

  TrajOptIfoptCompositeProfile();

  virtual TrajOptIfoptTermInfos create(const tesseract_common::ManipulatorInfo& composite_manip_info,
                                       const std::shared_ptr<const tesseract_environment::Environment>& env,
                                       const std::vector<std::shared_ptr<const trajopt_ifopt::Var>>& vars,
                                       const std::vector<int>& fixed_indices) const = 0;
};

class TrajOptIfoptSolverProfile : public tesseract_common::Profile
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptSolverProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptSolverProfile>;

  TrajOptIfoptSolverProfile();

  /** @brief Optimization parameters */
  trajopt_sqp::SQPParameters opt_params{};

  /**
   * @brief Optimization callbacks
   * @note This is not serialized
   */
  std::vector<std::shared_ptr<trajopt_sqp::SQPCallback>> callbacks;

  virtual std::unique_ptr<trajopt_sqp::TrustRegionSQPSolver> create(bool verbose = false) const = 0;

  /** @brief Optimization callbacks */
  virtual std::vector<std::shared_ptr<trajopt_sqp::SQPCallback>> createOptimizationCallbacks() const;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PROFILE_H
