/**
 * @file kinematic_limits_check_profile.cpp
 * @brief Profile for kinematic limits check task
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
#include <tesseract_task_composer/planning/profiles/kinematic_limits_check_profile.h>
#include <tesseract_common/serialization.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>

namespace tesseract_planning
{
KinematicLimitsCheckProfile::KinematicLimitsCheckProfile(bool check_position,
                                                         bool check_velocity,
                                                         bool check_acceleration)
  : Profile(KinematicLimitsCheckProfile::getStaticKey())
  , check_position(check_position)
  , check_velocity(check_velocity)
  , check_acceleration(check_acceleration)
{
}

std::size_t KinematicLimitsCheckProfile::getStaticKey()
{
  return std::type_index(typeid(KinematicLimitsCheckProfile)).hash_code();
}

template <class Archive>
void KinematicLimitsCheckProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(check_position);
  ar& BOOST_SERIALIZATION_NVP(check_velocity);
  ar& BOOST_SERIALIZATION_NVP(check_acceleration);
}
}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::KinematicLimitsCheckProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::KinematicLimitsCheckProfile)
