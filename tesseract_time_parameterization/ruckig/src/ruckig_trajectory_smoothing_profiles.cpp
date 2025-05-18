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
#include <tesseract_common/serialization.h>
#include <tesseract_common/eigen_serialization.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
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

template <class Archive>
void RuckigTrajectorySmoothingCompositeProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(duration_extension_fraction);
  ar& BOOST_SERIALIZATION_NVP(max_duration_extension_factor);
  ar& BOOST_SERIALIZATION_NVP(override_limits);
  ar& BOOST_SERIALIZATION_NVP(velocity_limits);
  ar& BOOST_SERIALIZATION_NVP(acceleration_limits);
  ar& BOOST_SERIALIZATION_NVP(jerk_limits);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RuckigTrajectorySmoothingCompositeProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RuckigTrajectorySmoothingCompositeProfile)
