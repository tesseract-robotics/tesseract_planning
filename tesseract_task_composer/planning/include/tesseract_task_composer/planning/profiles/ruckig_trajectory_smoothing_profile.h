/**
 * @file ruckig_trajectory_smoothing_profile.h
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
#ifndef TESSERACT_TASK_COMPOSER_RUCKIG_TRAJECTORY_SMOOTHING_PROFILE_H
#define TESSERACT_TASK_COMPOSER_RUCKIG_TRAJECTORY_SMOOTHING_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
struct RuckigTrajectorySmoothingCompositeProfile
{
  using Ptr = std::shared_ptr<RuckigTrajectorySmoothingCompositeProfile>;
  using ConstPtr = std::shared_ptr<const RuckigTrajectorySmoothingCompositeProfile>;

  RuckigTrajectorySmoothingCompositeProfile() = default;
  RuckigTrajectorySmoothingCompositeProfile(double duration_extension_fraction,
                                            double max_duration_extension_factor,
                                            double max_velocity_scaling_factor,
                                            double max_acceleration_scaling_factor)
    : duration_extension_fraction(duration_extension_fraction)
    , max_duration_extension_factor(max_duration_extension_factor)
    , max_velocity_scaling_factor(max_velocity_scaling_factor)
    , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
  {
  }

  /** @brief duration_extension_fraction The amount to scale the trajectory each time */
  double duration_extension_fraction{ 1.1 };

  /** @brief The max allow extension factor */
  double max_duration_extension_factor{ 10.0 };

  /** @brief max_velocity_scaling_factor The max velocity scaling factor passed to the solver */
  double max_velocity_scaling_factor{ 1.0 };

  /** @brief max_velocity_scaling_factor The max acceleration scaling factor passed to the solver */
  double max_acceleration_scaling_factor{ 1.0 };

  /** @brief max_jerk_scaling_factor The max jerk scaling factor passed to the solver */
  double max_jerk_scaling_factor{ 1.0 };
};

struct RuckigTrajectorySmoothingMoveProfile
{
  using Ptr = std::shared_ptr<RuckigTrajectorySmoothingMoveProfile>;
  using ConstPtr = std::shared_ptr<const RuckigTrajectorySmoothingMoveProfile>;

  RuckigTrajectorySmoothingMoveProfile() = default;
  RuckigTrajectorySmoothingMoveProfile(double max_velocity_scaling_factor, double max_acceleration_scaling_factor)
    : max_velocity_scaling_factor(max_velocity_scaling_factor)
    , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
  {
  }

  /** @brief max_velocity_scaling_factor The max velocity scaling factor passed to the solver */
  double max_velocity_scaling_factor{ 1.0 };

  /** @brief max_velocity_scaling_factor The max acceleration scaling factor passed to the solver */
  double max_acceleration_scaling_factor{ 1.0 };

  /** @brief max_jerk_scaling_factor The max jerk scaling factor passed to the solver */
  double max_jerk_scaling_factor{ 1.0 };
};
}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_RUCKIG_TRAJECTORY_SMOOTHING_PROFILE_H
