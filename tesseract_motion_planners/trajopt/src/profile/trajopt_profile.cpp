/**
 * @file trajopt_profile.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>

namespace tesseract_planning
{
TrajOptPlanProfile::TrajOptPlanProfile() : Profile(TrajOptPlanProfile::getStaticKey()) {}

std::size_t TrajOptPlanProfile::getStaticKey() { return std::type_index(typeid(TrajOptPlanProfile)).hash_code(); }

template <class Archive>
void TrajOptPlanProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
}

TrajOptCompositeProfile::TrajOptCompositeProfile() : Profile(TrajOptCompositeProfile::getStaticKey()) {}

std::size_t TrajOptCompositeProfile::getStaticKey()
{
  return std::type_index(typeid(TrajOptCompositeProfile)).hash_code();
}

template <class Archive>
void TrajOptCompositeProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
}

TrajOptSolverProfile::TrajOptSolverProfile() : Profile(TrajOptSolverProfile::getStaticKey()) {}

std::size_t TrajOptSolverProfile::getStaticKey() { return std::type_index(typeid(TrajOptSolverProfile)).hash_code(); }

template <class Archive>
void TrajOptSolverProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptPlanProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptPlanProfile)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptCompositeProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptCompositeProfile)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptSolverProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptSolverProfile)
