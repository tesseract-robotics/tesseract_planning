/**
 * @file ruckig_trajectory_smoothing.h
 * @brief Leveraging Ruckig to smooth trajectory
 *
 * @author Levi Armstrong
 * @date July 27, 2022
 * @version TODO
 * @bug No known bugs
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

#ifndef TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_H
#define TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_time_parameterization/core/trajectory_container.h>

namespace tesseract_planning
{
class RuckigTrajectorySmoothing
{
public:
  RuckigTrajectorySmoothing(double duration_extension_fraction = 1.1, double max_duration_extension_factor = 10);
  virtual ~RuckigTrajectorySmoothing() = default;
  RuckigTrajectorySmoothing(const RuckigTrajectorySmoothing&) = default;
  RuckigTrajectorySmoothing& operator=(const RuckigTrajectorySmoothing&) = default;
  RuckigTrajectorySmoothing(RuckigTrajectorySmoothing&&) = default;
  RuckigTrajectorySmoothing& operator=(RuckigTrajectorySmoothing&&) = default;

  /** @brief Set the duration extension fraction */
  void setDurationExtensionFraction(double duration_extension_fraction);

  /** @brief Set the max duration extension factor */
  void setMaxDurationExtensionFactor(double max_duration_extension_factor);

  /**
   * @brief Compute the time stamps for a flattened vector of move instruction
   * @param trajectory Flattended vector of move instruction
   * @param max_velocities The max velocities for each joint
   * @param max_accelerations The max acceleration for each joint
   * @param max_velocity_scaling_factor The max velocity scaling factor
   * @param max_acceleration_scaling_factor The max acceleration scaling factor
   * @return True if successful, otherwise false
   */
  bool compute(TrajectoryContainer& trajectory,
               const double& max_velocity,
               const double& max_acceleration,
               const double& max_jerk,
               double max_velocity_scaling_factor = 1.0,
               double max_acceleration_scaling_factor = 1.0,
               double max_jerk_scaling_factor = 1.0) const;

  /**
   * @brief Compute the time stamps for a flattened vector of move instruction
   * @param trajectory Flattended vector of move instruction
   * @param max_velocities The max velocities for each joint
   * @param max_accelerations The max acceleration for each joint
   * @param max_velocity_scaling_factor The max velocity scaling factor
   * @param max_acceleration_scaling_factor The max acceleration scaling factor
   * @return True if successful, otherwise false
   */
  bool compute(TrajectoryContainer& trajectory,
               const std::vector<double>& max_velocity,
               const std::vector<double>& max_acceleration,
               const std::vector<double>& max_jerk,
               double max_velocity_scaling_factor = 1.0,
               double max_acceleration_scaling_factor = 1.0,
               double max_jerk_scaling_factor = 1.0) const;

  /**
   * @brief Compute the time stamps for a flattened vector of move instruction
   * @param trajectory Flattended vector of move instruction
   * @param max_velocities The max velocities for each joint
   * @param max_accelerations The max acceleration for each joint
   * @param max_velocity_scaling_factor The max velocity scaling factor
   * @param max_acceleration_scaling_factor The max acceleration scaling factor
   * @return True if successful, otherwise false
   */
  bool compute(TrajectoryContainer& trajectory,
               const Eigen::Ref<const Eigen::VectorXd>& max_velocity,
               const Eigen::Ref<const Eigen::VectorXd>& max_acceleration,
               const Eigen::Ref<const Eigen::VectorXd>& max_jerk,
               double max_velocity_scaling_factor = 1.0,
               double max_acceleration_scaling_factor = 1.0,
               double max_jerk_scaling_factor = 1.0) const;

  /**
   * @brief Compute the time stamps for a flattened vector of move instruction
   * @param trajectory Flattended vector of move instruction
   * @param max_velocities The max velocities for each joint
   * @param max_accelerations The max acceleration for each joint
   * @param max_velocity_scaling_factor The max velocity scaling factor. Size should be trajectory.size()
   * @param max_acceleration_scaling_factor The max acceleration scaling factor. Size should be trajectory.size()
   * @return True if successful, otherwise false
   */
  bool compute(TrajectoryContainer& trajectory,
               const Eigen::Ref<const Eigen::VectorXd>& max_velocity,
               const Eigen::Ref<const Eigen::VectorXd>& max_acceleration,
               const Eigen::Ref<const Eigen::VectorXd>& max_jerk,
               const Eigen::Ref<const Eigen::VectorXd>& max_velocity_scaling_factors,
               const Eigen::Ref<const Eigen::VectorXd>& max_acceleration_scaling_factors,
               const Eigen::Ref<const Eigen::VectorXd>& max_jerk_scaling_factors) const;

protected:
  double duration_extension_fraction_;
  double max_duration_extension_factor_;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_H
