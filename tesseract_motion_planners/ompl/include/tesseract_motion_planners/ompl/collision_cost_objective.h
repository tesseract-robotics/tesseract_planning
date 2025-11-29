/**
 * @file collision_cost_objective.h
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
#ifndef TESSERACT_MOTION_PLANNERS_COLLISION_COST_OBJECTIVE_H
#define TESSERACT_MOTION_PLANNERS_COLLISION_COST_OBJECTIVE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <mutex>
#include <memory>
#include <map>
#include <vector>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/types.h>

#include <tesseract_environment/fwd.h>
#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_collision/core/fwd.h>

namespace tesseract_planning
{
/**
 * @class PenetrationDepthObjective
 * @brief Optimization objective for minimizing the obstacles within the contact threshold.
 *
 * This objective assigns zero cost to states outside contact threshold and a cost equal to the distance within the
 * contact threshold (positive) for states inside obstacles. When used with StateCostIntegralObjective
 * (optimize_by_motion=true), the planner minimizes the integral of penetration along the path.
 */
class CollisionCostObjective : public ompl::base::StateCostIntegralObjective
{
public:
  CollisionCostObjective(const ompl::base::SpaceInformationPtr& space_info,
                         const tesseract_environment::Environment& env,
                         std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                         const tesseract_collision::ContactManagerConfig& contact_manager_config,
                         OMPLStateExtractor extractor,
                         bool optimize_by_motion = false);

  /**
   * @brief State cost function.
   * @param state The state to evaluate.
   * @return Cost equal to the distance within the contact threshold at state s (>=0).
   */
  ompl::base::Cost stateCost(const ompl::base::State* state) const override;

private:
  /** @brief The Tesseract Joint Group */
  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;

  /** @brief The continuous contact manager used for creating cached continuous contact managers. */
  std::shared_ptr<tesseract_collision::DiscreteContactManager> contact_manager_;

  /** @brief A list of active links */
  std::vector<std::string> links_;

  /** @brief This will extract an Eigen::VectorXd from the OMPL State */
  OMPLStateExtractor extractor_;

  // The items below are to cache the contact manager based on thread ID. Currently ompl is multi
  // threaded but the methods used to implement collision checking are not thread safe. To prevent
  // reconstructing the collision environment for every check this will cache a contact manager
  // based on its thread ID.

  /** @brief Contact manager caching mutex */
  mutable std::mutex mutex_;

  /** @brief The continuous contact manager cache */
  mutable std::map<unsigned long int, std::shared_ptr<tesseract_collision::DiscreteContactManager>> contact_managers_;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_COLLISION_COST_OBJECTIVE_H
