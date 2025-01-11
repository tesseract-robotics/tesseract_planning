/**
 * @file min_length_profile.cpp
 * @brief Profile for task that processing the program so it meets a minimum length. Planners like trajopt
 * need at least the user defined number of states in the trajectory to perform velocity, acceleration and jerk
 * smoothing.
 *
 * @author Levi Armstrong
 * @date November 2. 2020
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

#include <tesseract_task_composer/planning/profiles/min_length_profile.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>

namespace tesseract_planning
{
MinLengthProfile::MinLengthProfile() : Profile(MinLengthProfile::getStaticKey()) {}
MinLengthProfile::MinLengthProfile(long min_length) : Profile(MinLengthProfile::getStaticKey()), min_length(min_length)
{
}

std::size_t MinLengthProfile::getStaticKey() { return std::type_index(typeid(MinLengthProfile)).hash_code(); }

template <class Archive>
void MinLengthProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(min_length);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MinLengthProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::MinLengthProfile)
