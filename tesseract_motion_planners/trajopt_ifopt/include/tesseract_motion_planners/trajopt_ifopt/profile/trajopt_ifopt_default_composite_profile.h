/**
 * @file trajopt_default_composite_profile.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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

#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_COMPOSITE_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_COMPOSITE_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <trajopt_common/collision_types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>

namespace YAML
{
class Node;
}

namespace tesseract_planning
{
class TrajOptIfoptDefaultCompositeProfile : public TrajOptIfoptCompositeProfile
{
public:
  TrajOptIfoptDefaultCompositeProfile() = default;
  TrajOptIfoptDefaultCompositeProfile(const YAML::Node& config,
                                      const tesseract_common::ProfilePluginFactory& plugin_factory);

  /** @brief Configuration info for collisions that are modeled as costs */
  trajopt_common::TrajOptCollisionConfig collision_cost_config;
  /** @brief Configuration info for collisions that are modeled as constraints */
  trajopt_common::TrajOptCollisionConfig collision_constraint_config;
  /** @brief If true, a joint velocity cost with a target of 0 will be applied for all timesteps Default: true*/
  bool smooth_velocities = true;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd velocity_coeff;
  /** @brief If true, a joint acceleration cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_accelerations = false;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd acceleration_coeff;
  /** @brief If true, a joint jerk cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_jerks = false;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd jerk_coeff;

  TrajOptIfoptTermInfos create(const tesseract_common::ManipulatorInfo& composite_manip_info,
                               const std::shared_ptr<const tesseract_environment::Environment>& env,
                               const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition> >& vars,
                               const std::vector<int>& fixed_indices) const override;

  bool operator==(const TrajOptIfoptDefaultCompositeProfile& rhs) const;
  bool operator!=(const TrajOptIfoptDefaultCompositeProfile& rhs) const;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_COMPOSITE_PROFILE_H
