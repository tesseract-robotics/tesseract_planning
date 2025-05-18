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
#include <tesseract_common/serialization.h>
#include <tesseract_common/eigen_serialization.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>

namespace tesseract_planning
{
IterativeSplineParameterizationCompositeProfile::IterativeSplineParameterizationCompositeProfile()
  : Profile(IterativeSplineParameterizationCompositeProfile::getStaticKey())
{
}
IterativeSplineParameterizationCompositeProfile::IterativeSplineParameterizationCompositeProfile(
    double max_velocity_scaling_factor,
    double max_acceleration_scaling_factor)
  : Profile(IterativeSplineParameterizationCompositeProfile::getStaticKey())
  , max_velocity_scaling_factor(max_velocity_scaling_factor)
  , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
{
}

std::size_t IterativeSplineParameterizationCompositeProfile::getStaticKey()
{
  return std::type_index(typeid(IterativeSplineParameterizationCompositeProfile)).hash_code();
}

template <class Archive>
void IterativeSplineParameterizationCompositeProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(add_points);
  ar& BOOST_SERIALIZATION_NVP(override_limits);
  ar& BOOST_SERIALIZATION_NVP(velocity_limits);
  ar& BOOST_SERIALIZATION_NVP(max_velocity_scaling_factor);
  ar& BOOST_SERIALIZATION_NVP(max_acceleration_scaling_factor);
}

IterativeSplineParameterizationMoveProfile::IterativeSplineParameterizationMoveProfile()
  : Profile(IterativeSplineParameterizationMoveProfile::getStaticKey())
{
}
IterativeSplineParameterizationMoveProfile::IterativeSplineParameterizationMoveProfile(
    double max_velocity_scaling_factor,
    double max_acceleration_scaling_factor)
  : Profile(IterativeSplineParameterizationMoveProfile::getStaticKey())
  , max_velocity_scaling_factor(max_velocity_scaling_factor)
  , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
{
}

std::size_t IterativeSplineParameterizationMoveProfile::getStaticKey()
{
  return std::type_index(typeid(IterativeSplineParameterizationMoveProfile)).hash_code();
}

template <class Archive>
void IterativeSplineParameterizationMoveProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(max_velocity_scaling_factor);
  ar& BOOST_SERIALIZATION_NVP(max_acceleration_scaling_factor);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::IterativeSplineParameterizationCompositeProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::IterativeSplineParameterizationCompositeProfile)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::IterativeSplineParameterizationMoveProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::IterativeSplineParameterizationMoveProfile)
