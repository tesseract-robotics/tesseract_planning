/**
 * @file state_collision_validator.cpp
 * @brief Tesseract OMPL planner OMPL state collision check
 *
 * @author Levi Armstrong
 * @date February 17, 2020
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/SpaceInformation.h>
#include <thread>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/utils.h>
#include <tesseract_motion_planners/ompl/state_collision_validator.h>

#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_environment/environment.h>

namespace tesseract_planning
{
StateCollisionValidator::StateCollisionValidator(
    const ompl::base::SpaceInformationPtr& space_info,
    const tesseract_environment::Environment& env,
    std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
    const tesseract_collision::ContactManagerConfig& contact_manager_config,
    OMPLStateExtractor extractor)
  : StateValidityChecker(space_info)
  , manip_(std::move(manip))
  , contact_manager_(env.getDiscreteContactManager())
  , extractor_(std::move(extractor))
{
  links_ = manip_->getActiveLinkNames();

  contact_manager_->setActiveCollisionObjects(links_);
  contact_manager_->applyContactManagerConfig(contact_manager_config);
}

bool StateCollisionValidator::isValid(const ompl::base::State* state) const
{
  // It was time using chronos time elapsed and it was faster to cache the contact manager
  unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  tesseract_collision::DiscreteContactManager::Ptr cm;
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
  tesseract_common::TransformMap state1 = manip_->calcFwdKin(finish_joints);

  for (const auto& link_name : links_)
    cm->setCollisionObjectsTransform(link_name, state1[link_name]);

  tesseract_collision::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

  return contact_map.empty();
}

}  // namespace tesseract_planning
