/**
 * @file continuous_motion_validator.cpp
 * @brief Tesseract OMPL planner continuous collision check between two states
 *
 * @author Jonathan Meyer
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/SpaceInformation.h>
#include <thread>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>

namespace tesseract_planning
{
ContinuousMotionValidator::ContinuousMotionValidator(
    const ompl::base::SpaceInformationPtr& space_info,
    ompl::base::StateValidityCheckerPtr state_validator,
    const tesseract_environment::Environment& env,
    tesseract_kinematics::JointGroup::ConstPtr manip,
    const tesseract_collision::CollisionCheckConfig& collision_check_config,
    OMPLStateExtractor extractor)
  : MotionValidator(space_info)
  , state_validator_(std::move(state_validator))
  , manip_(std::move(manip))
  , continuous_contact_manager_(env.getContinuousContactManager())
  , extractor_(std::move(extractor))
{
  links_ = manip_->getActiveLinkNames();

  continuous_contact_manager_->setActiveCollisionObjects(links_);
  continuous_contact_manager_->setCollisionMarginData(collision_check_config.collision_margin_data,
                                                      collision_check_config.collision_margin_override_type);
}

bool ContinuousMotionValidator::checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const
{
  std::pair<ompl::base::State*, double> dummy = { nullptr, 0.0 };
  return checkMotion(s1, s2, dummy);
}

bool ContinuousMotionValidator::checkMotion(const ompl::base::State* s1,
                                            const ompl::base::State* s2,
                                            std::pair<ompl::base::State*, double>& lastValid) const
{
  const ompl::base::StateSpace& state_space = *si_->getStateSpace();

  unsigned n_steps = state_space.validSegmentCount(s1, s2);
  bool is_valid = true;

  ompl::base::State* start_interp = si_->allocState();
  if (n_steps > 1)
  {
    ompl::base::State* end_interp = si_->allocState();

    for (unsigned i = 1; i < n_steps; ++i)
    {
      state_space.interpolate(s1, s2, (i - 1) / static_cast<double>(n_steps), start_interp);
      state_space.interpolate(s1, s2, (i) / static_cast<double>(n_steps), end_interp);
      if (state_validator_ && !state_validator_->isValid(end_interp))
      {
        lastValid.second = static_cast<double>(i - 1) / static_cast<double>(n_steps);
        if (lastValid.first != nullptr)
          state_space.interpolate(s1, s2, lastValid.second, lastValid.first);

        is_valid = false;
      }
      else if (!continuousCollisionCheck(start_interp, end_interp))
      {
        lastValid.second = (i - 1) / static_cast<double>(n_steps);
        if (lastValid.first != nullptr)
          state_space.interpolate(s1, s2, lastValid.second, lastValid.first);

        is_valid = false;
      }
    }

    si_->freeState(end_interp);
  }

  if (is_valid)
  {
    state_space.interpolate(s1, s2, (n_steps - 1) / static_cast<double>(n_steps), start_interp);
    if ((state_validator_ != nullptr && !state_validator_->isValid(s2)) || !continuousCollisionCheck(start_interp, s2))
    {
      lastValid.second = static_cast<double>(n_steps - 1) / static_cast<double>(n_steps);
      if (lastValid.first != nullptr)
        state_space.interpolate(s1, s2, lastValid.second, lastValid.first);

      is_valid = false;
    }
  }

  si_->freeState(start_interp);

  return is_valid;
}

bool ContinuousMotionValidator::continuousCollisionCheck(const ompl::base::State* s1, const ompl::base::State* s2) const
{
  // It was time using chronos time elapsed and it was faster to cache the contact manager
  unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  tesseract_collision::ContinuousContactManager::Ptr cm;
  mutex_.lock();
  auto it = continuous_contact_managers_.find(hash);
  if (it == continuous_contact_managers_.end())
  {
    cm = continuous_contact_manager_->clone();
    continuous_contact_managers_[hash] = cm;
  }
  else
  {
    cm = it->second;
  }
  mutex_.unlock();

  Eigen::Map<Eigen::VectorXd> start_joints = extractor_(s1);
  Eigen::Map<Eigen::VectorXd> finish_joints = extractor_(s2);

  tesseract_common::TransformMap state0 = manip_->calcFwdKin(start_joints);
  tesseract_common::TransformMap state1 = manip_->calcFwdKin(finish_joints);

  for (const auto& link_name : links_)
    cm->setCollisionObjectsTransform(link_name, state0[link_name], state1[link_name]);

  tesseract_collision::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

  return contact_map.empty();
}

}  // namespace tesseract_planning
