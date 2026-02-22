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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/SpaceInformation.h>
#include <thread>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/utils.h>
#include <tesseract_motion_planners/ompl/collision_cost_objective.h>

#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_environment/environment.h>

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
  , contact_manager_(env.getDiscreteContactManager())
  , extractor_(std::move(extractor))
{
  links_ = manip_->getActiveLinkNames();

  contact_manager_->setActiveCollisionObjects(links_);
  contact_manager_->applyContactManagerConfig(contact_manager_config);
}

ompl::base::Cost CollisionCostObjective::stateCost(const ompl::base::State* state) const
{
  // It was time using chronos time elapsed and it was faster to cache the contact manager
  unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  tesseract::collision::DiscreteContactManager::Ptr cm;
  mutex_.lock();
  auto it = contact_managers_.find(hash);
  if (it == contact_managers_.end())
  {
    cm = contact_manager_->clone();
    contact_managers_[hash] = cm;
  }
  else
  {
    cm = it->second;
  }
  mutex_.unlock();

  Eigen::Map<Eigen::VectorXd> finish_joints = extractor_(state);
  TESSERACT_THREAD_LOCAL tesseract::common::TransformMap state1;
  state1.clear();
  manip_->calcFwdKin(state1, finish_joints);

  for (const auto& link_name : links_)
    cm->setCollisionObjectsTransform(link_name, state1[link_name]);

  tesseract::collision::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract::collision::ContactTestType::CLOSEST);

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
