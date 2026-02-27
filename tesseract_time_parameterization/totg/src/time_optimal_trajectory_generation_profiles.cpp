/**
 * @file time_optimal_trajectory_generation_profile.cpp
 * @brief Time Optimal Trajectory Generation Profile
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

#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation_profiles.h>
#include <tesseract/common/utils.h>

namespace tesseract::time_parameterization
{
TimeOptimalTrajectoryGenerationCompositeProfile::TimeOptimalTrajectoryGenerationCompositeProfile()
  : Profile(createKey<TimeOptimalTrajectoryGenerationCompositeProfile>())
{
}

TimeOptimalTrajectoryGenerationCompositeProfile::TimeOptimalTrajectoryGenerationCompositeProfile(
    double max_velocity_scaling_factor,
    double max_acceleration_scaling_factor,
    double path_tolerance,
    double min_angle_change)
  : Profile(createKey<TimeOptimalTrajectoryGenerationCompositeProfile>())
  , max_velocity_scaling_factor(max_velocity_scaling_factor)
  , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
  , path_tolerance(path_tolerance)
  , min_angle_change(min_angle_change)
{
}

bool TimeOptimalTrajectoryGenerationCompositeProfile::operator==(
    const TimeOptimalTrajectoryGenerationCompositeProfile& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (override_limits == rhs.override_limits);
  equal &= tesseract::common::almostEqualRelativeAndAbs(velocity_limits.col(0), rhs.velocity_limits.col(0), max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(velocity_limits.col(1), rhs.velocity_limits.col(1), max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      acceleration_limits.col(0), rhs.acceleration_limits.col(0), max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      acceleration_limits.col(1), rhs.acceleration_limits.col(1), max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      max_velocity_scaling_factor, rhs.max_velocity_scaling_factor, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      max_acceleration_scaling_factor, rhs.max_acceleration_scaling_factor, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(path_tolerance, rhs.path_tolerance, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(min_angle_change, rhs.min_angle_change, max_diff);
  return equal;
}

bool TimeOptimalTrajectoryGenerationCompositeProfile::operator!=(
    const TimeOptimalTrajectoryGenerationCompositeProfile& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract::time_parameterization
