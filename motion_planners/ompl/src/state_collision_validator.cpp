/**
 * @file state_collision_validator.cpp
 * @brief Tesseract OMPL planner OMPL state collision check
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
#include <ompl/base/SpaceInformation.h>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/motion_planners/ompl/utils.h>
#include <tesseract/motion_planners/ompl/state_collision_validator.h>

#include <tesseract/kinematics/joint_group.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/types.h>
#include <tesseract/environment/environment.h>

namespace tesseract::motion_planners
{
StateCollisionValidator::StateCollisionValidator(
    const ompl::base::SpaceInformationPtr& space_info,
    const tesseract::environment::Environment& env,
    std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
    const tesseract::collision::ContactManagerConfig& contact_manager_config,
    OMPLStateExtractor extractor)
  : StateValidityChecker(space_info)
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

bool StateCollisionValidator::isValid(const ompl::base::State* state) const
{
  tesseract::collision::DiscreteContactManager& cm = contact_managers_->get();

  Eigen::Map<Eigen::VectorXd> finish_joints = extractor_(state);
  TESSERACT_THREAD_LOCAL tesseract::common::LinkIdTransformMap state1;
  state1.clear();
  manip_->calcFwdKin(state1, finish_joints);

  cm.setCollisionObjectsTransform(link_ids_, state1);

  tesseract::collision::ContactResultMap contact_map;
  cm.contactTest(contact_map, tesseract::collision::ContactTestType::FIRST);

  return contact_map.empty();
}

}  // namespace tesseract::motion_planners
