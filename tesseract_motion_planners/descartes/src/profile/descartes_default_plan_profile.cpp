/**
 * @file descartes_default_plan_profile.cpp
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
#include <tesseract_motion_planners/descartes/impl/profile/descartes_default_plan_profile.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>

namespace tesseract_planning
{
// Explicit template instantiation
template class DescartesDefaultPlanProfile<float>;
template class DescartesDefaultPlanProfile<double>;

template <typename FloatType>
template <class Archive>
void DescartesDefaultPlanProfile<FloatType>::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(DescartesPlanProfile<FloatType>);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::DescartesDefaultPlanProfile<float>)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::DescartesDefaultPlanProfile<float>)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::DescartesDefaultPlanProfile<double>)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::DescartesDefaultPlanProfile<double>)
