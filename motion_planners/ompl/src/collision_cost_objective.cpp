/**
 * @file collision_cost_objective.cpp
 * @brief Tesseract OMPL planner collision cost objective
 *
 * @author Levi Armstrong
 * @date July 5, 2025
 *
 * @copyright Copyright (c) 2025, Levi Armstrong
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
#include <ompl/base/SpaceInformation.h>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/motion_planners/ompl/utils.h>
#include <tesseract/motion_planners/ompl/collision_cost_objective.h>

#include <tesseract/kinematics/joint_group.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/types.h>
#include <tesseract/environment/environment.h>

namespace tesseract::motion_planners
{
CollisionCostObjective::CollisionCostObjective(const ompl::base::SpaceInformationPtr& space_info,
                                               const tesseract::environment::Environment& env,
                                               std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                                               const tesseract::collision::ContactManagerConfig& contact_manager_config,
                                               OMPLStateExtractor extractor,
                                               bool optimize_by_motion)
  : StateCostIntegralObjective(space_info, optimize_by_motion)
  , manip_(std::move(manip))
  , link_ids_(manip_->getActiveLinkIds())
  , extractor_(std::move(extractor))
{
  std::unique_ptr<tesseract::collision::DiscreteContactManager> contact_manager = env.getDiscreteContactManager();
  contact_manager->setActiveCollisionObjects(link_ids_);
  contact_manager->applyContactManagerConfig(contact_manager_config);

  // Each checking thread gets its own clone of the configured manager
  contact_managers_ =
      std::make_unique<tesseract::collision::PerThreadDiscreteContactManager>(std::move(contact_manager));
}

ompl::base::Cost CollisionCostObjective::stateCost(const ompl::base::State* state) const
{
  tesseract::collision::DiscreteContactManager& cm = contact_managers_->get();

  Eigen::Map<Eigen::VectorXd> finish_joints = extractor_(state);
  TESSERACT_THREAD_LOCAL tesseract::common::LinkIdTransformMap state1;
  state1.clear();
  manip_->calcFwdKin(state1, finish_joints);

  cm.setCollisionObjectsTransform(link_ids_, state1);

  tesseract::collision::ContactResultMap contact_map;
  cm.contactTest(contact_map, tesseract::collision::ContactTestType::CLOSEST);

  if (contact_map.empty())
    return ompl::base::Cost(0.0);

  tesseract::collision::ContactResultVector contact_vector;
  contact_map.flattenMoveResults(contact_vector);

  // penalty = how far you're *inside* (0 outside)
  double penetration{ 0.0 };
  for (const auto& cr : contact_vector)
    penetration = std::max(-cr.distance, penetration);

  return ompl::base::Cost(penetration);
}
}  // namespace tesseract::motion_planners
