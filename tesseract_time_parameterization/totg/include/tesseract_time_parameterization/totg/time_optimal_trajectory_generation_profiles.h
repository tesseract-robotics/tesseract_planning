/**
 * @file time_optimal_trajectory_generation_profile.h
 * @brief Time Optimal Trajectory Generation Profile
 *
 * @author Levi Armstrong
 * @date May 15, 2025
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
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
#ifndef TESSERACT_TIME_PARAMETERIZATION_TIME_OPTIMAL_TRAJECTORY_GENERATION_PROFILES_H
#define TESSERACT_TIME_PARAMETERIZATION_TIME_OPTIMAL_TRAJECTORY_GENERATION_PROFILES_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/profile.h>

namespace tesseract::time_parameterization
{
struct TimeOptimalTrajectoryGenerationCompositeProfile : public tesseract::common::Profile
{
  using Ptr = std::shared_ptr<TimeOptimalTrajectoryGenerationCompositeProfile>;
  using ConstPtr = std::shared_ptr<const TimeOptimalTrajectoryGenerationCompositeProfile>;

  TimeOptimalTrajectoryGenerationCompositeProfile();
  TimeOptimalTrajectoryGenerationCompositeProfile(double max_velocity_scaling_factor,
                                                  double max_acceleration_scaling_factor,
                                                  double path_tolerance,
                                                  double min_angle_change);

  /** @brief Indicate if overriding limits, otherwise manipulator limits will be used. */
  bool override_limits{ false };
  /** @brief The min/max velocities for each joint */
  Eigen::MatrixX2d velocity_limits;
  /** @brief The min/max acceleration for each joint */
  Eigen::MatrixX2d acceleration_limits;

  /** @brief The max velocity scaling factor passed to the solver. Default: 1.0*/
  double max_velocity_scaling_factor{ 1.0 };

  /** @brief The max acceleration scaling factor passed to the solver. Default: 1.0 */
  double max_acceleration_scaling_factor{ 1.0 };

  /** @brief path_tolerance. Default: 0.1*/
  double path_tolerance{ 0.1 };

  /** @brief At least one joint must change by greater than this amount for the point to be added. Default: 0.001*/
  double min_angle_change{ 0.001 };

  bool operator==(const TimeOptimalTrajectoryGenerationCompositeProfile& rhs) const;
  bool operator!=(const TimeOptimalTrajectoryGenerationCompositeProfile& rhs) const;
};
}  // namespace tesseract::time_parameterization

#endif  // TESSERACT_TIME_PARAMETERIZATION_TIME_OPTIMAL_TRAJECTORY_GENERATION_PROFILES_H
