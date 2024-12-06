/**
 * @file time_optimal_parameterization_profile.cpp
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

#include <tesseract_task_composer/planning/profiles/time_optimal_parameterization_profile.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>

namespace tesseract_planning
{
TimeOptimalParameterizationProfile::TimeOptimalParameterizationProfile()
  : Profile(TimeOptimalParameterizationProfile::getStaticKey())
{
}

TimeOptimalParameterizationProfile::TimeOptimalParameterizationProfile(double max_velocity_scaling_factor,
                                                                       double max_acceleration_scaling_factor,
                                                                       double max_jerk_scaling_factor,
                                                                       double path_tolerance,
                                                                       double min_angle_change)
  : Profile(TimeOptimalParameterizationProfile::getStaticKey())
  , max_velocity_scaling_factor(max_velocity_scaling_factor)
  , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
  , max_jerk_scaling_factor(max_jerk_scaling_factor)
  , path_tolerance(path_tolerance)
  , min_angle_change(min_angle_change)
{
}

std::size_t TimeOptimalParameterizationProfile::getStaticKey()
{
  return std::type_index(typeid(TimeOptimalParameterizationProfile)).hash_code();
}

template <class Archive>
void TimeOptimalParameterizationProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(max_velocity_scaling_factor);
  ar& BOOST_SERIALIZATION_NVP(max_acceleration_scaling_factor);
  ar& BOOST_SERIALIZATION_NVP(max_jerk_scaling_factor);
  ar& BOOST_SERIALIZATION_NVP(path_tolerance);
  ar& BOOST_SERIALIZATION_NVP(min_angle_change);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TimeOptimalParameterizationProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TimeOptimalParameterizationProfile)
