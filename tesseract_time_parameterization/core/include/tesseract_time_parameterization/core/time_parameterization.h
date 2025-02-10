/**
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_TIME_PARAMETERIZATION_TIME_PARAMETERIZATION_H
#define TESSERACT_TIME_PARAMETERIZATION_TIME_PARAMETERIZATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
class TrajectoryContainer;

/** @brief A generic container that the time parameterization classes use */
class TimeParameterization
{
public:
  virtual ~TimeParameterization() = default;

  /**
   * @brief Compute the time stamps for a flattened vector of move instruction
   * @param trajectory Flattended vector of move instruction
   * @param velocity_limits The min/max velocities for each joint
   * @param acceleration_limits The min/max acceleration for each joint
   * @param jerk_limits The min/max jerk for each joint
   * @param velocity_scaling_factor The velocity scaling factor. Size should be trajectory.size()
   * @param acceleration_scaling_factor The acceleration scaling factor. Size should be trajectory.size()
   * @param jerk_scaling_factor The jerk scaling factor. Size should be trajectory.size()
   * @param minimum_time_delta The smallest-allowable difference in seconds between timestamps of consecutive trajectory
   * points.
   * @return True if successful, otherwise false
   */
  virtual bool compute(TrajectoryContainer& trajectory,
                       const Eigen::Ref<const Eigen::MatrixX2d>& velocity_limits,
                       const Eigen::Ref<const Eigen::MatrixX2d>& acceleration_limits,
                       const Eigen::Ref<const Eigen::MatrixX2d>& jerk_limits,
                       const Eigen::Ref<const Eigen::VectorXd>& velocity_scaling_factors,
                       const Eigen::Ref<const Eigen::VectorXd>& acceleration_scaling_factors,
                       const Eigen::Ref<const Eigen::VectorXd>& jerk_scaling_factors,
                       const double& minimum_time_delta) const = 0;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_TIME_PARAMETERIZATION_TIME_PARAMETERIZATION_H
