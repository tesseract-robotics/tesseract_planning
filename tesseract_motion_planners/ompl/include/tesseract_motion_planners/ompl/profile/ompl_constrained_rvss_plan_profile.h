/**
 * @file ompl_constrained_rvss_plan_profile.h
 * @brief Tesseract OMPL constrained real vector state space plan profile
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_PROFILE_CONSTRAINED_RVSS_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_OMPL_PROFILE_CONSTRAINED_RVSS_PLAN_PROFILE_H

#ifndef OMPL_LESS_1_4_0

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_plan_profile.h>

namespace ompl::base
{
class Constraint;
using ConstraintPtr = std::shared_ptr<Constraint>;
}

namespace tesseract_planning
{
class OMPLConstrainedRvssPlanProfile : public OMPLRealVectorPlanProfile
{
public:
  OMPLConstrainedRvssPlanProfile();

protected:
  virtual std::vector<ompl::base::ConstraintPtr> createConstraints(const tesseract_common::ManipulatorInfo& mi,
                                                                   const std::shared_ptr<const tesseract_environment::Environment>& env) const;

  ompl::base::StateSpacePtr createStateSpace(const tesseract_common::ManipulatorInfo& mi,
                                             const std::shared_ptr<const tesseract_environment::Environment>& env) const override;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::OMPLConstrainedRvssPlanProfile)

#endif // OMPL_LESS_1_4_0

#endif // TESSERACT_MOTION_PLANNERS_OMPL_PROFILE_CONSTRAINED_RVSS_PLAN_PROFILE_H
