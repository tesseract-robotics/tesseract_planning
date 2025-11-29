/**
 * @file iterative_spline_parameterization_profile.h
 * @brief Iterative Spline Parameterization Profile
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
#ifndef TESSERACT_TIME_PARAMETERIZATION_ITERATIVE_SPLINE_PARAMETERIZATION_PROFILES_H
#define TESSERACT_TIME_PARAMETERIZATION_ITERATIVE_SPLINE_PARAMETERIZATION_PROFILES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/profile.h>

namespace tesseract_planning
{
struct IterativeSplineParameterizationCompositeProfile : public tesseract_common::Profile
{
  using Ptr = std::shared_ptr<IterativeSplineParameterizationCompositeProfile>;
  using ConstPtr = std::shared_ptr<const IterativeSplineParameterizationCompositeProfile>;

  IterativeSplineParameterizationCompositeProfile();
  IterativeSplineParameterizationCompositeProfile(double max_velocity_scaling_factor,
                                                  double max_acceleration_scaling_factor);

  /**
   * @brief If true, add two points to trajectory (first and last segments).
   *
   * If false, move the 2nd and 2nd-last points.
   */
  bool add_points{ true };

  /** @brief Indicate if overriding limits, otherwise manipulator limits will be used. */
  bool override_limits{ false };
  /** @brief The min/max velocities for each joint */
  Eigen::MatrixX2d velocity_limits;
  /** @brief The min/max acceleration for each joint */
  Eigen::MatrixX2d acceleration_limits;

  /** @brief max_velocity_scaling_factor The max velocity scaling factor passed to the solver */
  double max_velocity_scaling_factor{ 1.0 };

  /** @brief max_velocity_scaling_factor The max acceleration scaling factor passed to the solver */
  double max_acceleration_scaling_factor{ 1.0 };

  bool operator==(const IterativeSplineParameterizationCompositeProfile& rhs) const;
  bool operator!=(const IterativeSplineParameterizationCompositeProfile& rhs) const;
};

struct IterativeSplineParameterizationMoveProfile : public tesseract_common::Profile
{
  using Ptr = std::shared_ptr<IterativeSplineParameterizationMoveProfile>;
  using ConstPtr = std::shared_ptr<const IterativeSplineParameterizationMoveProfile>;

  IterativeSplineParameterizationMoveProfile();
  IterativeSplineParameterizationMoveProfile(double max_velocity_scaling_factor,
                                             double max_acceleration_scaling_factor);

  /** @brief max_velocity_scaling_factor The max velocity scaling factor passed to the solver */
  double max_velocity_scaling_factor{ 1.0 };

  /** @brief max_velocity_scaling_factor The max acceleration scaling factor passed to the solver */
  double max_acceleration_scaling_factor{ 1.0 };

  bool operator==(const IterativeSplineParameterizationMoveProfile& rhs) const;
  bool operator!=(const IterativeSplineParameterizationMoveProfile& rhs) const;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_TIME_PARAMETERIZATION_ITERATIVE_SPLINE_PARAMETERIZATION_PROFILES_H
