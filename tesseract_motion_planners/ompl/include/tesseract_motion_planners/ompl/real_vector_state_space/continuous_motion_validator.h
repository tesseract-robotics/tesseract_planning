/**
 * @file continuous_motion_validator.h
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_REAL_VECTOR_STATE_SPACE_CONTINUOUS_MOTION_VALIDATOR_H
#define TESSERACT_MOTION_PLANNERS_OMPL_REAL_VECTOR_STATE_SPACE_CONTINUOUS_MOTION_VALIDATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/MotionValidator.h>
#include <vector>
#include <map>
#include <mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/fwd.h>
#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_environment/fwd.h>

#include <tesseract_motion_planners/ompl/real_vector_state_space/utils.h>

namespace ompl::base
{
class SpaceInformation;
using SpaceInformationPtr = std::shared_ptr<SpaceInformation>;
class StateValidityChecker;
using StateValidityCheckerPtr = std::shared_ptr<StateValidityChecker>;
}  // namespace ompl::base

namespace tesseract_planning
{
/** @brief Continuous collision check between two states */
class ContinuousMotionValidator : public ompl::base::MotionValidator
{
public:
  ContinuousMotionValidator(const ompl::base::SpaceInformationPtr& space_info,
                            ompl::base::StateValidityCheckerPtr state_validator,
                            const tesseract_environment::Environment& env,
                            std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                            const tesseract_collision::CollisionCheckConfig& collision_check_config,
                            StateConverterFn state_converter);

  bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const override;

  bool checkMotion(const ompl::base::State* s1,
                   const ompl::base::State* s2,
                   std::pair<ompl::base::State*, double>& lastValid) const override;

private:
  /**
   * @brief Perform a continuous collision check between two ompl states
   * @param s1 First OMPL State
   * @param s2 Second OMPL State
   * @return True if not in collision, otherwise false.
   */
  bool continuousCollisionCheck(const ompl::base::State* s1, const ompl::base::State* s2) const;

  /**
   * @brief The state validator without collision checking
   *
   * Since this performs collision checking we only want to use state validator without collision checking.
   */
  ompl::base::StateValidityCheckerPtr state_validator_;

  /** @brief The Tesseract Forward Kinematics */
  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;

  /** @brief The continuous contact manager used for creating cached continuous contact managers. */
  std::shared_ptr<tesseract_collision::ContinuousContactManager> continuous_contact_manager_;

  /** @brief A list of active links */
  std::vector<std::string> links_;

  /** @brief Function to convert an OMPL state (typically of type RealVectorStateSpace::StateType or ConstrainedStateSpace::StateType) into a vector of doubles representing a joint state */
  StateConverterFn state_converter_;

  // The items below are to cache the contact manager based on thread ID. Currently ompl is multi
  // threaded but the methods used to implement collision checking are not thread safe. To prevent
  // reconstructing the collision environment for every check this will cache a contact manager
  // based on its thread ID.

  /** @brief Contact manager caching mutex */
  mutable std::mutex mutex_;

  /** @brief The continuous contact manager cache */
  mutable std::map<unsigned long int, std::shared_ptr<tesseract_collision::ContinuousContactManager>>
      continuous_contact_managers_;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_REAL_VECTOR_STATE_SPACE_CONTINUOUS_MOTION_VALIDATOR_H
