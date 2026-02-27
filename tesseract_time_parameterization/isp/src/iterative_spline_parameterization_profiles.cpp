/**
 * @file iterative_spline_parameterization_profile.cpp
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

#include <tesseract_time_parameterization/isp/iterative_spline_parameterization_profiles.h>
#include <tesseract/common/utils.h>

namespace tesseract::time_parameterization
{
IterativeSplineParameterizationCompositeProfile::IterativeSplineParameterizationCompositeProfile()
  : Profile(createKey<IterativeSplineParameterizationCompositeProfile>())
{
}
IterativeSplineParameterizationCompositeProfile::IterativeSplineParameterizationCompositeProfile(
    double max_velocity_scaling_factor,
    double max_acceleration_scaling_factor)
  : Profile(createKey<IterativeSplineParameterizationCompositeProfile>())
  , max_velocity_scaling_factor(max_velocity_scaling_factor)
  , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
{
}

bool IterativeSplineParameterizationCompositeProfile::operator==(
    const IterativeSplineParameterizationCompositeProfile& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (add_points == rhs.add_points);
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
  equal &= tesseract::common::almostEqualRelativeAndAbs(minimum_time_delta, rhs.minimum_time_delta, max_diff);
  return equal;
}

bool IterativeSplineParameterizationCompositeProfile::operator!=(
    const IterativeSplineParameterizationCompositeProfile& rhs) const
{
  return !operator==(rhs);
}

IterativeSplineParameterizationMoveProfile::IterativeSplineParameterizationMoveProfile()
  : Profile(createKey<IterativeSplineParameterizationMoveProfile>())
{
}
IterativeSplineParameterizationMoveProfile::IterativeSplineParameterizationMoveProfile(
    double max_velocity_scaling_factor,
    double max_acceleration_scaling_factor)
  : Profile(createKey<IterativeSplineParameterizationMoveProfile>())
  , max_velocity_scaling_factor(max_velocity_scaling_factor)
  , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
{
}

bool IterativeSplineParameterizationMoveProfile::operator==(const IterativeSplineParameterizationMoveProfile& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      max_velocity_scaling_factor, rhs.max_velocity_scaling_factor, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      max_acceleration_scaling_factor, rhs.max_acceleration_scaling_factor, max_diff);
  return equal;
}

bool IterativeSplineParameterizationMoveProfile::operator!=(const IterativeSplineParameterizationMoveProfile& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract::time_parameterization
