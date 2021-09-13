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
#include <tesseract_environment/core/utils.h>

namespace tesseract_planning
{
bool DescartesCollision::isContactAllowed(const std::string& a, const std::string& b) const
{
  return acm_.isCollisionAllowed(a, b);
}

DescartesCollision::DescartesCollision(const tesseract_environment::Environment::ConstPtr& collision_env,
                                       std::vector<std::string> active_links,
                                       std::vector<std::string> joint_names,
                                       tesseract_collision::CollisionCheckConfig collision_check_config,
                                       bool debug)
  : state_solver_(collision_env->getStateSolver())
  , acm_(*(collision_env->getAllowedCollisionMatrix()))
  , active_link_names_(std::move(active_links))
  , joint_names_(std::move(joint_names))
  , contact_manager_(collision_env->getDiscreteContactManager())
  , collision_check_config_(std::move(collision_check_config))
  , debug_(debug)
{
  contact_manager_->setActiveCollisionObjects(active_link_names_);
  contact_manager_->setCollisionMarginData(collision_check_config_.collision_margin_data,
                                           collision_check_config_.collision_margin_override_type);
  contact_manager_->setIsContactAllowedFn(
      [this](const std::string& a, const std::string& b) { return isContactAllowed(a, b); });
}

DescartesCollision::DescartesCollision(const DescartesCollision& collision_interface)
  : state_solver_(collision_interface.state_solver_->clone())
  , acm_(collision_interface.acm_)
  , active_link_names_(collision_interface.active_link_names_)
  , joint_names_(collision_interface.joint_names_)
  , contact_manager_(collision_interface.contact_manager_->clone())
  , collision_check_config_(collision_interface.collision_check_config_)
  , debug_(collision_interface.debug_)
{
  contact_manager_->setActiveCollisionObjects(active_link_names_);
  contact_manager_->setCollisionMarginData(collision_check_config_.collision_margin_data,
                                           collision_check_config_.collision_margin_override_type);
  contact_manager_->setIsContactAllowedFn(
      [this](const std::string& a, const std::string& b) { return isContactAllowed(a, b); });
}

bool DescartesCollision::validate(const Eigen::Ref<const Eigen::VectorXd>& pos)
{
  // Happens in two phases:
  // 1. Compute the transform of all objects
  tesseract_environment::EnvState::Ptr env_state = state_solver_->getState(joint_names_, pos);

  std::vector<tesseract_collision::ContactResultMap> results;
  tesseract_collision::CollisionCheckConfig config(collision_check_config_);
  config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  bool in_contact = checkTrajectoryState(results, *contact_manager_, env_state, config);
  return (!in_contact);
}

double DescartesCollision::distance(const Eigen::Ref<const Eigen::VectorXd>& pos)
{
  // Happens in two phases:
  // 1. Compute the transform of all objects
  tesseract_environment::EnvState::Ptr env_state = state_solver_->getState(joint_names_, pos);

  std::vector<tesseract_collision::ContactResultMap> results;
  tesseract_collision::CollisionCheckConfig config(collision_check_config_);
  config.contact_request.type = tesseract_collision::ContactTestType::CLOSEST;
  bool in_contact = checkTrajectoryState(results, *contact_manager_, env_state, config);

  if (!in_contact)
    return contact_manager_->getCollisionMarginData().getMaxCollisionMargin();

  return results.begin()->begin()->second[0].distance;
}

DescartesCollision::Ptr DescartesCollision::clone() const { return std::make_shared<DescartesCollision>(*this); }

}  // namespace tesseract_planning
