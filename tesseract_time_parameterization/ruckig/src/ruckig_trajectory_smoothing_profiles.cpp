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

#include <tesseract_time_parameterization/ruckig/ruckig_trajectory_smoothing_profiles.h>
#include <tesseract_common/utils.h>
#include <typeindex>

namespace tesseract_planning
{
RuckigTrajectorySmoothingCompositeProfile::RuckigTrajectorySmoothingCompositeProfile()
  : Profile(RuckigTrajectorySmoothingCompositeProfile::getStaticKey())
{
}
RuckigTrajectorySmoothingCompositeProfile::RuckigTrajectorySmoothingCompositeProfile(
    double duration_extension_fraction,
    double max_duration_extension_factor)
  : Profile(RuckigTrajectorySmoothingCompositeProfile::getStaticKey())
  , duration_extension_fraction(duration_extension_fraction)
  , max_duration_extension_factor(max_duration_extension_factor)
{
}

std::size_t RuckigTrajectorySmoothingCompositeProfile::getStaticKey()
{
  return std::type_index(typeid(RuckigTrajectorySmoothingCompositeProfile)).hash_code();
}

bool RuckigTrajectorySmoothingCompositeProfile::operator==(const RuckigTrajectorySmoothingCompositeProfile& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= tesseract_common::almostEqualRelativeAndAbs(
      duration_extension_fraction, rhs.duration_extension_fraction, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(
      max_duration_extension_factor, rhs.max_duration_extension_factor, max_diff);
  equal &= (override_limits == rhs.override_limits);
  equal &= tesseract_common::almostEqualRelativeAndAbs(velocity_limits.col(0), rhs.velocity_limits.col(0), max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(velocity_limits.col(1), rhs.velocity_limits.col(1), max_diff);
  equal &=
      tesseract_common::almostEqualRelativeAndAbs(acceleration_limits.col(0), rhs.acceleration_limits.col(0), max_diff);
  equal &=
      tesseract_common::almostEqualRelativeAndAbs(acceleration_limits.col(1), rhs.acceleration_limits.col(1), max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(jerk_limits.col(0), rhs.jerk_limits.col(0), max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(jerk_limits.col(1), rhs.jerk_limits.col(1), max_diff);
  return equal;
}

bool RuckigTrajectorySmoothingCompositeProfile::operator!=(const RuckigTrajectorySmoothingCompositeProfile& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract_planning
