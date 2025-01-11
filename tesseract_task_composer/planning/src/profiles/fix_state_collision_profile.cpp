/**
 * @file fix_state_collision_profile.cpp
 * @brief Profile for process that pushes plan instructions to be out of collision
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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

#include <tesseract_task_composer/planning/profiles/fix_state_collision_profile.h>
#include <tesseract_collision/core/serialization.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <typeindex>

namespace tesseract_planning
{
FixStateCollisionProfile::FixStateCollisionProfile(Settings mode)
  : Profile(FixStateCollisionProfile::getStaticKey()), mode(mode)
{
  collision_check_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  collision_check_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
}

std::size_t FixStateCollisionProfile::getStaticKey()
{
  return std::type_index(typeid(FixStateCollisionProfile)).hash_code();
}

template <class Archive>
void FixStateCollisionProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(mode);
  ar& BOOST_SERIALIZATION_NVP(correction_workflow);
  ar& BOOST_SERIALIZATION_NVP(jiggle_factor);
  ar& BOOST_SERIALIZATION_NVP(collision_check_config);
  ar& BOOST_SERIALIZATION_NVP(sampling_attempts);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::FixStateCollisionProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::FixStateCollisionProfile)
