/**
 * @file utils.cpp
 * @brief Tesseract OMPL planner utility functions
 *
 * @author Levi Armstrong
 * @date February 17, 2020
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>
#include <memory>

#ifndef OMPL_LESS_1_4_0
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#endif

TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/utils.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>

#include <tesseract/collision/types.h>
#include <tesseract/collision/discrete_contact_manager.h>

#include <tesseract/kinematics/joint_group.h>

namespace tesseract::motion_planners
{
Eigen::Map<Eigen::VectorXd> RealVectorStateSpaceExtractor(const ompl::base::State* s1, unsigned dimension)
{
  assert(dynamic_cast<const ompl::base::RealVectorStateSpace::StateType*>(s1) != nullptr);
  const auto* s = s1->template as<ompl::base::RealVectorStateSpace::StateType>();
  return Eigen::Map<Eigen::VectorXd>{ s->values, dimension };
}

#ifndef OMPL_LESS_1_4_0
Eigen::Map<Eigen::VectorXd> ConstrainedStateSpaceExtractor(const ompl::base::State* s1)
{
  assert(dynamic_cast<const ompl::base::ProjectedStateSpace::StateType*>(s1) != nullptr);
  const Eigen::Map<Eigen::VectorXd>& s = *(s1->template as<ompl::base::ProjectedStateSpace::StateType>());
  return s;
}
#endif

tesseract::common::TrajArray toTrajArray(const ompl::geometric::PathGeometric& path,
                                         const OMPLStateExtractor& extractor)
{
  const auto n_points = static_cast<long>(path.getStateCount());
  const auto dof = static_cast<long>(path.getSpaceInformation()->getStateDimension());

  tesseract::common::TrajArray result(n_points, dof);
  for (long i = 0; i < n_points; ++i)
    result.row(i) = extractor(path.getState(static_cast<unsigned>(i)));

  return result;
}

void processLongestValidSegment(const ompl::base::StateSpacePtr& state_space_ptr,
                                double longest_valid_segment_fraction,
                                double longest_valid_segment_length)
{
  if (longest_valid_segment_fraction > 0 && longest_valid_segment_length > 0)
  {
    double val =
        std::min(longest_valid_segment_fraction, longest_valid_segment_length / state_space_ptr->getMaximumExtent());
    longest_valid_segment_fraction = val;
  }
  else if (longest_valid_segment_length > 0)
  {
    longest_valid_segment_fraction = longest_valid_segment_length / state_space_ptr->getMaximumExtent();
  }
  else
  {
    longest_valid_segment_fraction = 0.01;
  }
  state_space_ptr->setLongestValidSegmentFraction(longest_valid_segment_fraction);
}

void processLongestValidSegment(const ompl::base::StateSpacePtr& state_space_ptr,
                                const tesseract::collision::CollisionCheckConfig& collision_check_config)
{
  double longest_valid_segment_fraction = 0.01;
  if (collision_check_config.longest_valid_segment_length > 0)
    longest_valid_segment_fraction =
        collision_check_config.longest_valid_segment_length / state_space_ptr->getMaximumExtent();

  state_space_ptr->setLongestValidSegmentFraction(longest_valid_segment_fraction);
}

bool checkStateInCollision(tesseract::collision::ContactResultMap& contact_map,
                           tesseract::collision::DiscreteContactManager& contact_checker,
                           const tesseract::kinematics::JointGroup& manip,
                           const Eigen::VectorXd& state)
{
  /** @brief Making this thread_local does not help because it is only used by applyStartState and applyGoalState */
  tesseract::common::TransformMap link_transforms;
  manip.calcFwdKin(link_transforms, state);

  for (const auto& link_name : contact_checker.getActiveCollisionObjects())
    contact_checker.setCollisionObjectsTransform(link_name, link_transforms[link_name]);

  contact_checker.contactTest(contact_map, tesseract::collision::ContactTestType::FIRST);

  return (!contact_map.empty());
}

ompl::base::StateSamplerPtr allocWeightedRealVectorStateSampler(const ompl::base::StateSpace* space,
                                                                const Eigen::VectorXd& weights,
                                                                const Eigen::MatrixX2d& limits)
{
  return std::make_shared<WeightedRealVectorStateSampler>(space, weights, limits);
}

}  // namespace tesseract::motion_planners
