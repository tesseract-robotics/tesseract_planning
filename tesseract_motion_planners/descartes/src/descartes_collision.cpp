/**
 * @file descartes_collision.cpp
 * @brief Tesseract Descartes Collision Implementation
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_environment/utils.h>

namespace tesseract_planning
{
DescartesCollision::DescartesCollision(const tesseract_environment::Environment& collision_env,
                                       tesseract_kinematics::JointGroup::ConstPtr manip,
                                       tesseract_collision::CollisionCheckConfig collision_check_config,
                                       bool debug)
  : manip_(std::move(manip))
  , active_link_names_(manip_->getActiveLinkNames())
  , contact_manager_(collision_env.getDiscreteContactManager())
  , collision_check_config_(std::move(collision_check_config))
  , debug_(debug)
{
  contact_manager_->setActiveCollisionObjects(active_link_names_);
  contact_manager_->applyContactManagerConfig(collision_check_config_.contact_manager_config);
}

DescartesCollision::DescartesCollision(const DescartesCollision& collision_interface)
  : manip_(collision_interface.manip_)
  , active_link_names_(collision_interface.active_link_names_)
  , contact_manager_(collision_interface.contact_manager_->clone())
  , collision_check_config_(collision_interface.collision_check_config_)
  , debug_(collision_interface.debug_)
{
  contact_manager_->applyContactManagerConfig(collision_check_config_.contact_manager_config);
}

bool DescartesCollision::validate(const Eigen::Ref<const Eigen::VectorXd>& pos)
{
  // Happens in two phases:
  // 1. Compute the transform of all objects
  tesseract_common::TransformMap state = manip_->calcFwdKin(pos);

  tesseract_collision::CollisionCheckConfig config(collision_check_config_);
  config.contact_request.type = tesseract_collision::ContactTestType::FIRST;

  tesseract_collision::ContactResultMap results;
  tesseract_environment::checkTrajectoryState(results, *contact_manager_, state, config);
  return results.empty();
}

double DescartesCollision::distance(const Eigen::Ref<const Eigen::VectorXd>& pos)
{
  // Happens in two phases:
  // 1. Compute the transform of all objects
  tesseract_common::TransformMap state = manip_->calcFwdKin(pos);

  tesseract_collision::CollisionCheckConfig config(collision_check_config_);
  config.contact_request.type = tesseract_collision::ContactTestType::CLOSEST;

  tesseract_collision::ContactResultMap results;
  tesseract_environment::checkTrajectoryState(results, *contact_manager_, state, config);

  if (results.empty())
    return contact_manager_->getCollisionMarginData().getMaxCollisionMargin();

  return results.begin()->second.front().distance;
}

DescartesCollision::Ptr DescartesCollision::clone() const { return std::make_shared<DescartesCollision>(*this); }

}  // namespace tesseract_planning
