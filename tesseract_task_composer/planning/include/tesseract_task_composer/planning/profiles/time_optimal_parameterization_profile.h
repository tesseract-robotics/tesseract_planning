/**
 * @file time_optimal_parameterization_profile.h
 * @brief Profile for TOTG process
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Jan 22, 2021
 * @version TODO
 * @bug No known bugs
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
#ifndef TIME_OPTIMAL_PARAMETERIZATION_PROFILE_H
#define TIME_OPTIMAL_PARAMETERIZATION_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
struct TimeOptimalParameterizationProfile
{
  using Ptr = std::shared_ptr<TimeOptimalParameterizationProfile>;
  using ConstPtr = std::shared_ptr<const TimeOptimalParameterizationProfile>;

  TimeOptimalParameterizationProfile() = default;
  TimeOptimalParameterizationProfile(double max_velocity_scaling_factor,
                                     double max_acceleration_scaling_factor,
                                     double path_tolerance,
                                     double min_angle_change)
    : max_velocity_scaling_factor(max_velocity_scaling_factor)
    , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
    , path_tolerance(path_tolerance)
    , min_angle_change(min_angle_change)
  {
  }

  /** @brief The max velocity scaling factor passed to the solver. Default: 1.0*/
  double max_velocity_scaling_factor{ 1.0 };

  /** @brief The max acceleration scaling factor passed to the solver. Default: 1.0 */
  double max_acceleration_scaling_factor{ 1.0 };

  /** @brief path_tolerance. Default: 0.1*/
  double path_tolerance{ 0.1 };

  /** @brief At least one joint must change by greater than this amount for the point to be added. Default: 0.001*/
  double min_angle_change{ 0.001 };
};

}  // namespace tesseract_planning
#endif  // TIME_OPTIMAL_PARAMETERIZATION_PROFILE_H
