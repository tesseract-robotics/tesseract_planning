/**
 * @file ruckig_trajectory_smoothing_profile.cpp
 * @brief Leveraging Ruckig to smooth trajectory
 *
 * @author Levi Armstrong
 * @date July 27, 2022
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_task_composer/planning/profiles/ruckig_trajectory_smoothing_profile.h>
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
    double max_duration_extension_factor,
    double max_velocity_scaling_factor,
    double max_acceleration_scaling_factor)
  : Profile(RuckigTrajectorySmoothingCompositeProfile::getStaticKey())
  , duration_extension_fraction(duration_extension_fraction)
  , max_duration_extension_factor(max_duration_extension_factor)
  , max_velocity_scaling_factor(max_velocity_scaling_factor)
  , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
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
  ar& BOOST_SERIALIZATION_NVP(max_velocity_scaling_factor);
  ar& BOOST_SERIALIZATION_NVP(max_acceleration_scaling_factor);
  ar& BOOST_SERIALIZATION_NVP(max_jerk_scaling_factor);
}

RuckigTrajectorySmoothingMoveProfile::RuckigTrajectorySmoothingMoveProfile()
  : Profile(RuckigTrajectorySmoothingMoveProfile::getStaticKey())
{
}
RuckigTrajectorySmoothingMoveProfile::RuckigTrajectorySmoothingMoveProfile(double max_velocity_scaling_factor,
                                                                           double max_acceleration_scaling_factor)
  : Profile(RuckigTrajectorySmoothingMoveProfile::getStaticKey())
  , max_velocity_scaling_factor(max_velocity_scaling_factor)
  , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
{
}

std::size_t RuckigTrajectorySmoothingMoveProfile::getStaticKey()
{
  return std::type_index(typeid(RuckigTrajectorySmoothingMoveProfile)).hash_code();
}

template <class Archive>
void RuckigTrajectorySmoothingMoveProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(max_velocity_scaling_factor);
  ar& BOOST_SERIALIZATION_NVP(max_acceleration_scaling_factor);
  ar& BOOST_SERIALIZATION_NVP(max_jerk_scaling_factor);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RuckigTrajectorySmoothingCompositeProfile)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RuckigTrajectorySmoothingMoveProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RuckigTrajectorySmoothingCompositeProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RuckigTrajectorySmoothingMoveProfile)
