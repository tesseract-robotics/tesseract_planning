/**
 * @file descartes_collision_edge_evaluator.h
 * @brief Tesseract Descartes Collision Edge Evaluator Implementation
 *
 * @author Levi Armstrong
 * @date December 18, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_EDGE_EVALUATOR_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_EDGE_EVALUATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <mutex>
#include <descartes_light/core/edge_evaluator.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/eigen_types.h>
#include <tesseract_collision/core/fwd.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_environment/fwd.h>

namespace tesseract_planning
{
template <typename FloatType>
class DescartesCollisionEdgeEvaluator : public descartes_light::EdgeEvaluator<FloatType>
{
public:
  DescartesCollisionEdgeEvaluator(const tesseract_environment::Environment& collision_env,
                                  std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                                  const tesseract_collision::ContactManagerConfig& contact_manager_config,
                                  tesseract_collision::CollisionCheckConfig collision_check_config,
                                  bool allow_collision = false,
                                  bool debug = false);

  std::pair<bool, FloatType> evaluate(const descartes_light::State<FloatType>& start,
                                      const descartes_light::State<FloatType>& end) const override;

protected:
  /** @brief The tesseract state solver */
  std::shared_ptr<const tesseract_kinematics::JointGroup> manip_;
  /** @brief A vector of active link names */
  std::vector<std::string> active_link_names_;
  /** @brief The discrete contact manager */
  std::shared_ptr<tesseract_collision::DiscreteContactManager> discrete_contact_manager_;
  /** @brief The discrete contact manager */
  std::shared_ptr<tesseract_collision::ContinuousContactManager> continuous_contact_manager_;
  /** @brief The collision margin data */
  tesseract_common::CollisionMarginData contact_margin_data_;
  /** @brief The contact request used during collision checking */
  tesseract_collision::CollisionCheckConfig collision_check_config_;
  /** @brief If true and no valid edges are found it will return the one with the lowest cost */
  bool allow_collision_;
  /** @brief Enable debug information to be printed to the terminal */
  bool debug_;

  // The member variables below are to cache the contact manager based on thread ID. Currently descartes is multi
  // threaded but the methods used to implement collision checking are not thread safe. To prevent
  // reconstructing the collision environment for every check this will cache a contact manager
  // based on its thread ID.

  /** @brief Contact manager caching mutex */
  mutable std::mutex mutex_;

  /** @brief The continuous contact manager cache */
  mutable std::map<unsigned long int, std::shared_ptr<tesseract_collision::ContinuousContactManager>>
      continuous_contact_managers_;

  /** @brief The discrete contact manager cache */
  mutable std::map<unsigned long int, std::shared_ptr<tesseract_collision::DiscreteContactManager>>
      discrete_contact_managers_;

  /**
   * @brief Perform a continuous collision check between two states
   * @param results Store results from collision check.
   * @param segment Trajectory containing two states
   * @return True if in collision otherwise false
   */
  bool continuousCollisionCheck(std::vector<tesseract_collision::ContactResultMap>& results,
                                const tesseract_common::TrajArray& segment) const;

  /**
   * @brief Perform a continuous discrete check between two states
   * @param results Store results from collision check.
   * @param segment Trajectory containing two states
   * @return True if in collision otherwise false
   */
  bool discreteCollisionCheck(std::vector<tesseract_collision::ContactResultMap>& results,
                              const tesseract_common::TrajArray& segment) const;
};

using DescartesCollisionEdgeEvaluatorF = DescartesCollisionEdgeEvaluator<float>;
using DescartesCollisionEdgeEvaluatorD = DescartesCollisionEdgeEvaluator<double>;

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_EDGE_EVALUATOR_H
