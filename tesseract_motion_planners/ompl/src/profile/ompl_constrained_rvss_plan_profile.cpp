/**
 * @file ompl_constrained_rvss_plan_profile.cpp
 * @brief
 *
 * @author Michael Ripperger
 * @date December 26, 2024
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
#ifndef OMPL_LESS_1_4_0

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/profile/ompl_constrained_rvss_plan_profile.h>

#include <tesseract_environment/environment.h>

namespace tesseract_planning
{
OMPLConstrainedRvssPlanProfile::OMPLConstrainedRvssPlanProfile()
  : OMPLRealVectorPlanProfile()
{
  // Update the state converter function
  state_converter_ = &fromConstrainedRealVectorStateSpace;
}

std::vector<ompl::base::ConstraintPtr> OMPLConstrainedRvssPlanProfile::createConstraints(const tesseract_common::ManipulatorInfo& /*mi*/,
                                                                                         const std::shared_ptr<const tesseract_environment::Environment>& /*env*/) const
{
  return {};
}

ompl::base::StateSpacePtr OMPLConstrainedRvssPlanProfile::createStateSpace(const tesseract_common::ManipulatorInfo& mi,
                                                                           const std::shared_ptr<const tesseract_environment::Environment>& env) const
{
  // Create the ambient real vector state space
  ompl::base::StateSpacePtr rvss = OMPLRealVectorPlanProfile::createStateSpace(mi, env);

  // Create the constraints
  auto constraints = std::make_shared<ompl::base::ConstraintIntersection>(rvss->getDimension(), createConstraints(mi, env));

  // Create the projected state space
  return std::make_shared<ompl::base::ProjectedStateSpace>(rvss, constraints);
}

template <class Archive>
void OMPLConstrainedRvssPlanProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(OMPLRealVectorPlanProfile);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::OMPLConstrainedRvssPlanProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::OMPLConstrainedRvssPlanProfile)

#endif  // OMPL_LESS_1_4_0
