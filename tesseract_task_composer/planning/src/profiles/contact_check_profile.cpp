/**
 * @file contact_check_profile.cpp
 * @brief Contact check trajectory profile
 *
 * @author Levi Armstrong
 * @date August 10. 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <typeindex>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract_collision/core/serialization.h>

namespace tesseract_planning
{
ContactCheckProfile::ContactCheckProfile() : ContactCheckProfile(0.05, 0) {}

ContactCheckProfile::ContactCheckProfile(double longest_valid_segment_length, double contact_distance)
  : Profile(ContactCheckProfile::getStaticKey())
{
  contact_manager_config.default_margin = contact_distance;

  collision_check_config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
  collision_check_config.longest_valid_segment_length = longest_valid_segment_length;

  if (collision_check_config.longest_valid_segment_length <= 0)
  {
    CONSOLE_BRIDGE_logWarn("ContactCheckProfile: Invalid longest valid segment. Defaulting to 0.05");
    collision_check_config.longest_valid_segment_length = 0.05;
  }
}

std::size_t ContactCheckProfile::getStaticKey() { return std::type_index(typeid(ContactCheckProfile)).hash_code(); }

template <class Archive>
void ContactCheckProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(contact_manager_config);
  ar& BOOST_SERIALIZATION_NVP(collision_check_config);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ContactCheckProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ContactCheckProfile)
