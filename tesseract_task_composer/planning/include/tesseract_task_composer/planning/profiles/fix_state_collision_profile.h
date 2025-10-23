/**
 * @file fix_state_collision_profile.h
 * @brief Profile for process that pushes plan instructions to be out of collision
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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
#ifndef TESSERACT_TASK_COMPOSER_FIX_STATE_COLLISION_PROFILE_H
#define TESSERACT_TASK_COMPOSER_FIX_STATE_COLLISION_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
#include <trajopt/fwd.hpp>
#include <trajopt_sco/optimizers.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>
#include <tesseract_common/profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>
#include <tesseract_common/fwd.h>
#include <trajopt_common/collision_types.h>
#include <trajopt_sco/osqp_interface.hpp>

namespace YAML
{
class Node;
}

namespace tesseract_planning
{
struct FixStateCollisionProfile : public tesseract_common::Profile
{
  using Ptr = std::shared_ptr<FixStateCollisionProfile>;
  using ConstPtr = std::shared_ptr<const FixStateCollisionProfile>;

  enum class Settings
  {
    START_ONLY,
    END_ONLY,
    INTERMEDIATE_ONLY,
    ALL,
    ALL_EXCEPT_START,
    ALL_EXCEPT_END,
    DISABLED
  };

  /** @brief Used to specify method used to correct states in collision */
  enum class CorrectionMethod
  {
    NONE,
    TRAJOPT,
    RANDOM_SAMPLER
  };

  FixStateCollisionProfile(Settings mode = Settings::ALL);
  FixStateCollisionProfile(const YAML::Node& config, const tesseract_common::ProfilePluginFactory& plugin_factory);

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  /** @brief Sets which terms will be corrected  */
  Settings mode;

  /** @brief Order that correction methods will be applied. These will be attempted in order until one succeeds or all
   * have been tried */
  std::vector<CorrectionMethod> correction_workflow{ CorrectionMethod::TRAJOPT, CorrectionMethod::RANDOM_SAMPLER };

  /** @brief Percent of the total joint range that a joint will be allowed to be adjusted */
  double jiggle_factor{ 0.02 };

  /** @brief The contact manager config */
  tesseract_collision::ContactManagerConfig contact_manager_config;

  /** @brief The collision check config */
  tesseract_collision::CollisionCheckConfig collision_check_config;

  /** @brief Number of sampling attempts if TrajOpt correction fails*/
  int sampling_attempts{ 100 };

  /** @brief The TrajOpt joint waypoint constraint config */
  TrajOptJointWaypointConfig trajopt_joint_constraint_config;

  /** @brief The TrajOpt joint waypoint cost config */
  TrajOptJointWaypointConfig trajopt_joint_cost_config;

  /** @brief Coefficient for collision constraint in TrajOpt optimization */
  trajopt_common::CollisionCoeffData collision_constraint_coeff;

  /** @brief Coefficient for collision cost in TrajOpt optimization */
  trajopt_common::CollisionCoeffData collision_cost_coeff;

  /** @brief Optimization parameters */
  sco::BasicTrustRegionSQPParameters opt_params;

  /** @brief OSQP settings */
  OSQPSettings osqp_settings{};

  /** @brief Update the OSQP workspace for subsequent optimizations, instead of recreating it each time */
  bool update_workspace{ false };

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::FixStateCollisionProfile)

#endif  // TESSERACT_TASK_COMPOSER_FIX_STATE_COLLISION_PROFILE_H
