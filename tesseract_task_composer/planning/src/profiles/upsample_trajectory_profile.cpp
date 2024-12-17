/**
 * @file upsample_trajectory_profile.cpp
 *
 * @author Levi Armstrong
 * @date December 15, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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

#include <tesseract_task_composer/planning/profiles/upsample_trajectory_profile.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>

namespace tesseract_planning
{
UpsampleTrajectoryProfile::UpsampleTrajectoryProfile() : Profile(UpsampleTrajectoryProfile::getStaticKey()) {}

UpsampleTrajectoryProfile::UpsampleTrajectoryProfile(double longest_valid_segment_length)
  : Profile(UpsampleTrajectoryProfile::getStaticKey()), longest_valid_segment_length(longest_valid_segment_length)
{
}

std::size_t UpsampleTrajectoryProfile::getStaticKey()
{
  return std::type_index(typeid(UpsampleTrajectoryProfile)).hash_code();
}

template <class Archive>
void UpsampleTrajectoryProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(longest_valid_segment_length);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::UpsampleTrajectoryProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::UpsampleTrajectoryProfile)
