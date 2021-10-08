/**
 * @file descartes_collision.h
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <tesseract_environment/environment.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
class DescartesCollision
{
public:
  using Ptr = std::shared_ptr<DescartesCollision>;
  using ConstPtr = std::shared_ptr<const DescartesCollision>;

  /**
   * @brief TesseractCollision
   * @param collision_env The collision Environment
   * @param manip The manipulator joint group
   * @param edge_collision_check_config Config used to set up collision checking
   * @param longest_valid_segment_length Used to check collisions between two state if norm(state0-state1) >
   * longest_valid_segment_length.
   * @param debug If true, this print debug information to the terminal
   */
  DescartesCollision(const tesseract_environment::Environment& collision_env,
                     tesseract_kinematics::JointGroup::ConstPtr manip,
                     tesseract_collision::CollisionCheckConfig collision_check_config =
                         tesseract_collision::CollisionCheckConfig{ 0.025 },
                     bool debug = false);
  virtual ~DescartesCollision() = default;

  /**
   * @brief Copy constructor that clones the object
   * @param collision_interface Object to copy/clone
   */
  DescartesCollision(const DescartesCollision& collision_interface);
  DescartesCollision& operator=(const DescartesCollision&) = delete;
  DescartesCollision(DescartesCollision&&) = delete;
  DescartesCollision& operator=(DescartesCollision&&) = delete;

  /**
   * @brief This check is the provided solution passes the collision test defined by this class
   * @param pos The joint values array to validate
   * @return True if passes collision test, otherwise false
   */
  bool validate(const Eigen::Ref<const Eigen::VectorXd>& pos);

  /**
   * @brief This gets the distance to the closest object
   * @param pos The joint values array to calculate the distance to the closest object
   * @return The distance to the closest object
   */
  double distance(const Eigen::Ref<const Eigen::VectorXd>& pos);

  /**
   * @brief This should clone the object and make new instance of objects that are not safe to share across threads
   * @return Descartes collision interface
   */
  DescartesCollision::Ptr clone() const;

private:
  /**
   * @brief Check if two links are allowed to be in collision
   * @param a The name of the first link
   * @param b The name of the second link
   * @return True if allowed to be in collision, otherwise false
   */
  bool isContactAllowed(const std::string& a, const std::string& b) const;

  tesseract_kinematics::JointGroup::ConstPtr manip_;                 /**< @brief The tesseract state solver */
  tesseract_scene_graph::AllowedCollisionMatrix acm_;                /**< @brief The allowed collision matrix */
  std::vector<std::string> active_link_names_;                       /**< @brief A vector of active link names */
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_; /**< @brief The discrete contact manager */
  tesseract_collision::CollisionCheckConfig collision_check_config_;
  bool debug_; /**< @brief Enable debug information to be printed to the terminal */
};

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_COLLISION_H
