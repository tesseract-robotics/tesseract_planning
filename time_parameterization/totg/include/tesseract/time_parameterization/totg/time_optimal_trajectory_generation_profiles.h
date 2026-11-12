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
  /**
   * @brief The min/max velocities for each joint, finite, with min <= 0 <= max
   * @details TOTG limits each joint by the tighter of its two sides in both directions, so a zero on either side locks
   * the joint, and compute fails for a program that moves it
   */
  Eigen::MatrixX2d velocity_limits;
  /**
   * @brief The min/max acceleration for each joint, finite, with min <= 0 <= max
   * @details TOTG limits each joint by the tighter of its two sides in both directions, so a zero on either side locks
   * the joint, and compute fails for a program that moves it
   */
  Eigen::MatrixX2d acceleration_limits;

  /** @brief The max velocity scaling factor passed to the solver. Default: 1.0*/
  double max_velocity_scaling_factor{ 1.0 };

  /** @brief The max acceleration scaling factor passed to the solver. Default: 1.0 */
  double max_acceleration_scaling_factor{ 1.0 };

  /**
   * @brief The maximum deviation from a corner of the path where the corner is blended; must be greater than zero.
   * @details The distance in joint space, the Euclidean norm over the joints, from the corner waypoint to the blend; a
   * blend also reaches no further than halfway along either adjacent segment. One number for all joints, so in a group
   * that mixes prismatic and revolute joints it applies in metres and in radians alike. Default: 0.1
   */
  double path_tolerance{ 0.1 };

  /**
   * @brief At least one joint must change by greater than this amount for the point to be added; finite and at least
   * 1e-5.
   * @details A waypoint within this of the last one kept, in every joint, is removed from the program. At the end of
   * the program the goal is kept instead of the first waypoint of such a run, unless another instruction lies between
   * them, and it also replaces the kept waypoints before it that are within this of it; a removed waypoint then lies
   * within twice this of a kept one. One number for all joints, so in a group that mixes prismatic and revolute joints
   * it applies in metres and in radians alike. Default: 0.001
   */
  double min_angle_change{ 0.001 };

  /**
   * @brief Start at the first waypoint's velocity instead of at rest
   * @details The velocity must point forward along the path's first segment: its part across it at most 1e-9 plus 1e-3
   * times its part along it. The motion starts at the part along it, which must be within the scaled velocity limits,
   * and is written back to the first waypoint. compute fails otherwise, as it does when the path leaves no room to
   * slow down. A zero or empty velocity starts at rest. Default: false
   */
  bool use_start_velocity{ false };

  /**
   * @brief End at the last waypoint's velocity instead of at rest
   * @details The velocity must point forward along the path's last segment: its part across it at most 1e-9 plus 1e-3
   * times its part along it. The motion ends at the part along it, which must be within the scaled velocity limits,
   * and is written to the last waypoint kept. compute fails otherwise, as it does when the end velocity is out of
   * reach. A zero or empty velocity ends at rest. Default: false
   */
  bool use_end_velocity{ false };

  /**
   * @brief Insert waypoints between the kept ones so that, within rounding, no two are more than this far apart in time
   * @details The interval between two kept waypoints is split into the fewest equal parts that are at most this long,
   * so a shorter interval is left as it is. Each inserted waypoint lies on the time-optimal trajectory, with its
   * position, velocity, acceleration and time. Must be finite and not negative, and cannot be combined with
   * upsample_num_waypoints. Default: 0, which inserts none
   */
  double upsample_dt{ 0.0 };

  /**
   * @brief Insert waypoints between the kept ones until the output has this many, or keep the kept ones if there are
   * more of them
   * @details The waypoints are inserted where the time between waypoints is longest, which makes the longest time
   * between two output waypoints as short as it can be. They lie on the time-optimal trajectory, as for upsample_dt.
   * A motion that does not move keeps its one waypoint. Must not be negative, and cannot be combined with upsample_dt.
   * Default: 0, which inserts none
   */
  int upsample_num_waypoints{ 0 };

  bool operator==(const TimeOptimalTrajectoryGenerationCompositeProfile& rhs) const;
  bool operator!=(const TimeOptimalTrajectoryGenerationCompositeProfile& rhs) const;
};
}  // namespace tesseract::time_parameterization

#endif  // TESSERACT_TIME_PARAMETERIZATION_TIME_OPTIMAL_TRAJECTORY_GENERATION_PROFILES_H
