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

#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_COMPOSITE_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_COMPOSITE_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <Eigen/Core>
#include <trajopt/fwd.hpp>
#include <trajopt_common/collision_types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>

#include <tesseract_collision/core/fwd.h>
#include <tesseract_collision/core/types.h>

namespace YAML
{
class Node;
}

namespace tesseract_planning
{
class TrajOptDefaultCompositeProfile : public TrajOptCompositeProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptDefaultCompositeProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptDefaultCompositeProfile>;

  TrajOptDefaultCompositeProfile() = default;

  TrajOptDefaultCompositeProfile(const YAML::Node& config, const tesseract_common::ProfilePluginFactory& plugin_factory);

  /** @brief Configuration info for collisions that are modeled as costs */
  trajopt_common::TrajOptCollisionConfig collision_cost_config;
  /** @brief Configuration info for collisions that are modeled as constraints */
  trajopt_common::TrajOptCollisionConfig collision_constraint_config;
  /** @brief If true, a joint velocity cost with a target of 0 will be applied for all timesteps Default: true*/
  bool smooth_velocities = true;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd velocity_coeff{};
  /** @brief If true, a joint acceleration cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_accelerations = true;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd acceleration_coeff{};
  /** @brief If true, a joint jerk cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_jerks = true;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd jerk_coeff{};
  /** @brief If true, applies a cost to avoid kinematic singularities */
  bool avoid_singularity = false;
  /** @brief Optimization weight associated with kinematic singularity avoidance */
  double avoid_singularity_coeff = 5.0;

  TrajOptTermInfos create(const tesseract_common::ManipulatorInfo& composite_manip_info,
                          const std::shared_ptr<const tesseract_environment::Environment>& env,
                          const std::vector<int>& fixed_indices,
                          int start_index,
                          int end_index) const override;

  /**
   * @brief Compute the longest valid segment length
   * @param joint_limits
   * @param longest_valid_segment_fraction Set the resolution at which state validity needs to be verified in order for
   * a motion between two states to be considered valid in post checking of trajectory returned by trajopt. The
   * resolution is equal to longest_valid_segment_fraction * state_space.getMaximumExtent() Note: The planner takes the
   * conservative of either longest_valid_segment_fraction or longest_valid_segment_length.
   * @param longest_valid_segment_length Set the resolution at which state validity needs to be verified in order for a
   * motion between two states to be considered valid. If norm(state1 - state0) > longest_valid_segment_length. Note:
   * This gets converted to longest_valid_segment_fraction. longest_valid_segment_fraction =
   * longest_valid_segment_length / state_space.getMaximumExtent()
   * @return The computed longest valid segment length given length and fraction
   */
  static double computeLongestValidSegmentLength(const Eigen::MatrixX2d& joint_limits,
                                                 double longest_valid_segment_fraction,
                                                 double longest_valid_segment_length);

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptDefaultCompositeProfile)

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_COMPOSITE_PROFILE_H
