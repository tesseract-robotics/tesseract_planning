/**
 * @file descartes_collision_edge_evaluator.h
 * @brief Tesseract Descartes Collision Edge Evaluator Implementation
 *
 * @author Levi Armstrong
 * @date December 18, 2019
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <descartes_light/core/edge_evaluator.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/eigen_types.h>
#include <tesseract/collision/fwd.h>
#include <tesseract/collision/per_thread_contact_manager.h>
#include <tesseract/collision/types.h>
#include <tesseract/kinematics/fwd.h>
#include <tesseract/environment/fwd.h>

namespace tesseract::motion_planners
{
template <typename FloatType>
class DescartesCollisionEdgeEvaluator : public descartes_light::EdgeEvaluator<FloatType>
{
public:
  DescartesCollisionEdgeEvaluator(const tesseract::environment::Environment& collision_env,
                                  std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                                  const tesseract::collision::ContactManagerConfig& contact_manager_config,
                                  tesseract::collision::CollisionCheckConfig collision_check_config,
                                  bool allow_collision = false,
                                  bool debug = false);

  std::pair<bool, FloatType> evaluate(const descartes_light::State<FloatType>& start,
                                      const descartes_light::State<FloatType>& end) const override;

protected:
  /** @brief The tesseract state solver */
  std::shared_ptr<const tesseract::kinematics::JointGroup> manip_;
  /** @brief A vector of active link ids */
  std::vector<tesseract::common::LinkId> active_link_ids_;
  /** @brief The collision margin data */
  tesseract::common::CollisionMarginData contact_margin_data_;
  /** @brief The contact request used during collision checking */
  tesseract::collision::CollisionCheckConfig collision_check_config_;
  /** @brief If true and no valid edges are found it will return the one with the lowest cost */
  bool allow_collision_;
  /** @brief Enable debug information to be printed to the terminal */
  bool debug_;

  /**
   * @brief Each thread's own clone of the configured discrete contact manager, null if the environment has none
   * @details Descartes may evaluate edges from several threads at once, and a contact manager is not thread safe.
   */
  std::unique_ptr<tesseract::collision::PerThreadDiscreteContactManager> discrete_contact_managers_;

  /** @brief Each thread's own clone of the configured continuous contact manager, null if the environment has none */
  std::unique_ptr<tesseract::collision::PerThreadContinuousContactManager> continuous_contact_managers_;

  /**
   * @brief Perform a continuous collision check between two states
   * @param results Store results from collision check.
   * @param segment Trajectory containing two states
   * @return True if in collision otherwise false
   */
  bool continuousCollisionCheck(std::vector<tesseract::collision::ContactResultMap>& results,
                                const tesseract::common::TrajArray& segment) const;

  /**
   * @brief Perform a continuous discrete check between two states
   * @param results Store results from collision check.
   * @param segment Trajectory containing two states
   * @return True if in collision otherwise false
   */
  bool discreteCollisionCheck(std::vector<tesseract::collision::ContactResultMap>& results,
                              const tesseract::common::TrajArray& segment) const;
};

using DescartesCollisionEdgeEvaluatorF = DescartesCollisionEdgeEvaluator<float>;
using DescartesCollisionEdgeEvaluatorD = DescartesCollisionEdgeEvaluator<double>;

}  // namespace tesseract::motion_planners
#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_EDGE_EVALUATOR_H
