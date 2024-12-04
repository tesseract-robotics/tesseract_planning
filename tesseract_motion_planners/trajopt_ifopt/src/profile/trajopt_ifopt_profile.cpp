/**
 * @file trajopt_ifopt_profile.cpp
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
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>

namespace tesseract_planning
{
TrajOptIfoptPlanProfile::TrajOptIfoptPlanProfile() : Profile(TrajOptIfoptPlanProfile::getStaticKey()) {}

std::size_t TrajOptIfoptPlanProfile::getStaticKey()
{
  return std::type_index(typeid(TrajOptIfoptPlanProfile)).hash_code();
}

template <class Archive>
void TrajOptIfoptPlanProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
}

TrajOptIfoptCompositeProfile::TrajOptIfoptCompositeProfile() : Profile(TrajOptIfoptCompositeProfile::getStaticKey()) {}

std::size_t TrajOptIfoptCompositeProfile::getStaticKey()
{
  return std::type_index(typeid(TrajOptIfoptCompositeProfile)).hash_code();
}

template <class Archive>
void TrajOptIfoptCompositeProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
}

TrajOptIfoptSolverProfile::TrajOptIfoptSolverProfile() : Profile(TrajOptIfoptSolverProfile::getStaticKey()) {}

std::size_t TrajOptIfoptSolverProfile::getStaticKey()
{
  return std::type_index(typeid(TrajOptIfoptSolverProfile)).hash_code();
}

template <class Archive>
void TrajOptIfoptSolverProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptPlanProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptPlanProfile)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptCompositeProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptCompositeProfile)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptSolverProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptSolverProfile)
