/**
 * @file ruckig_trajectory_smoothing_profile.h
 * @brief Leveraging Ruckig to smooth trajectory profile
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
#ifndef TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_PROFILES_H
#define TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_PROFILES_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/profile.h>

namespace tesseract::time_parameterization
{
struct RuckigTrajectorySmoothingCompositeProfile : public tesseract::common::Profile
{
  using Ptr = std::shared_ptr<RuckigTrajectorySmoothingCompositeProfile>;
  using ConstPtr = std::shared_ptr<const RuckigTrajectorySmoothingCompositeProfile>;

  RuckigTrajectorySmoothingCompositeProfile();
  RuckigTrajectorySmoothingCompositeProfile(double duration_extension_fraction, double max_duration_extension_factor);

  /** @brief The duration extension fraction */
  double duration_extension_fraction{ 1.1 };
  /** @brief The max duration extension factor */
  double max_duration_extension_factor{ 10 };

  /** @brief Indicate if overriding limits, otherwise manipulator limits will be used. */
  bool override_limits{ false };
  /** @brief The min/max velocities for each joint */
  Eigen::MatrixX2d velocity_limits;
  /** @brief The min/max acceleration for each joint */
  Eigen::MatrixX2d acceleration_limits;
  /** @brief The min/max jerk for each joint */
  Eigen::MatrixX2d jerk_limits;

  bool operator==(const RuckigTrajectorySmoothingCompositeProfile& rhs) const;
  bool operator!=(const RuckigTrajectorySmoothingCompositeProfile& rhs) const;
};

}  // namespace tesseract::time_parameterization

#endif  // TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_PROFILES_H
