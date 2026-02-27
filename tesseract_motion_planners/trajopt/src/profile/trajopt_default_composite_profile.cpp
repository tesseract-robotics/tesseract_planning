/**
 * @file trajopt_default_composite_profile.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#include <trajopt/problem_description.hpp>
#include <boost/algorithm/string.hpp>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/trajopt/yaml_extensions.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>

#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/profile_plugin_factory.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/environment/environment.h>
#include <trajopt_common/utils.hpp>

static const double LONGEST_VALID_SEGMENT_FRACTION_DEFAULT = 0.01;

namespace tesseract::motion_planners
{
TrajOptDefaultCompositeProfile::TrajOptDefaultCompositeProfile(
    const YAML::Node& config,
    const tesseract::common::ProfilePluginFactory& /*plugin_factory*/)
  : TrajOptDefaultCompositeProfile()
{
  try
  {
    if (YAML::Node n = config["collision_cost_config"])
      collision_cost_config = n.as<trajopt_common::TrajOptCollisionConfig>();

    if (YAML::Node n = config["collision_constraint_config"])
      collision_constraint_config = n.as<trajopt_common::TrajOptCollisionConfig>();

    if (YAML::Node n = config["smooth_velocities"])
      smooth_velocities = n.as<bool>();

    if (YAML::Node n = config["velocity_coeff"])
      velocity_coeff = n.as<Eigen::VectorXd>();

    if (YAML::Node n = config["smooth_accelerations"])
      smooth_accelerations = n.as<bool>();

    if (YAML::Node n = config["acceleration_coeff"])
      acceleration_coeff = n.as<Eigen::VectorXd>();

    if (YAML::Node n = config["smooth_jerks"])
      smooth_jerks = n.as<bool>();

    if (YAML::Node n = config["jerk_coeff"])
      jerk_coeff = n.as<Eigen::VectorXd>();

    if (YAML::Node n = config["avoid_singularity"])
      avoid_singularity = n.as<bool>();

    if (YAML::Node n = config["avoid_singularity_coeff"])
      avoid_singularity_coeff = n.as<double>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("TrajOptDefaultCompositeProfile: Failed to parse yaml config! Details: " +
                             std::string(e.what()));
  }
}

TrajOptTermInfos
TrajOptDefaultCompositeProfile::create(const tesseract::common::ManipulatorInfo& composite_manip_info,
                                       const std::shared_ptr<const tesseract::environment::Environment>& env,
                                       const std::vector<int>& fixed_indices,
                                       int start_index,
                                       int end_index) const
{
  // -------- Construct the problem ------------
  // -------------------------------------------
  TrajOptTermInfos term_infos;
  tesseract::kinematics::JointGroup::ConstPtr joint_group = env->getJointGroup(composite_manip_info.manipulator);
  const Eigen::Index dof = joint_group->numJoints();

  if (collision_constraint_config.enabled)
  {
    trajopt::TermInfo::Ptr ti = createCollisionTermInfo(
        start_index, end_index, fixed_indices, collision_constraint_config, trajopt::TermType::TT_CNT);

    term_infos.constraints.push_back(ti);
  }

  if (collision_cost_config.enabled)
  {
    // Create a default collision term info
    trajopt::TermInfo::Ptr ti = createCollisionTermInfo(
        start_index, end_index, fixed_indices, collision_cost_config, trajopt::TermType::TT_COST);

    term_infos.costs.push_back(ti);
  }

  if (smooth_velocities)
  {
    if (velocity_coeff.size() == 0)
      term_infos.costs.push_back(createSmoothVelocityTermInfo(start_index, end_index, static_cast<int>(dof)));
    else
      term_infos.costs.push_back(createSmoothVelocityTermInfo(start_index, end_index, velocity_coeff));
  }

  if (smooth_accelerations)
  {
    if (acceleration_coeff.size() == 0)
      term_infos.costs.push_back(createSmoothAccelerationTermInfo(start_index, end_index, static_cast<int>(dof)));
    else
      term_infos.costs.push_back(createSmoothAccelerationTermInfo(start_index, end_index, acceleration_coeff));
  }

  if (smooth_jerks)
  {
    if (jerk_coeff.size() == 0)
      term_infos.costs.push_back(createSmoothJerkTermInfo(start_index, end_index, static_cast<int>(dof)));
    else
      term_infos.costs.push_back(createSmoothJerkTermInfo(start_index, end_index, jerk_coeff));
  }

  if (avoid_singularity)
    term_infos.costs.push_back(createAvoidSingularityTermInfo(
        start_index, end_index, composite_manip_info.tcp_frame, avoid_singularity_coeff));

  return term_infos;
}

double TrajOptDefaultCompositeProfile::computeLongestValidSegmentLength(const Eigen::MatrixX2d& joint_limits,
                                                                        double longest_valid_segment_fraction,
                                                                        double longest_valid_segment_length)
{
  // Calculate longest valid segment length
  double extent = (joint_limits.col(1) - joint_limits.col(0)).norm();
  if (longest_valid_segment_fraction > 0 && longest_valid_segment_length > 0)
    return std::min(longest_valid_segment_fraction * extent, longest_valid_segment_length);

  if (longest_valid_segment_fraction > 0)
    return longest_valid_segment_fraction * extent;

  if (longest_valid_segment_length > 0)
    return longest_valid_segment_length;

  return LONGEST_VALID_SEGMENT_FRACTION_DEFAULT * extent;
}

bool TrajOptDefaultCompositeProfile::operator==(const TrajOptDefaultCompositeProfile& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (collision_cost_config == rhs.collision_cost_config);
  equal &= (collision_constraint_config == rhs.collision_constraint_config);
  equal &= (smooth_velocities == rhs.smooth_velocities);
  equal &= tesseract::common::almostEqualRelativeAndAbs(velocity_coeff, rhs.velocity_coeff, max_diff);
  equal &= (smooth_accelerations == rhs.smooth_accelerations);
  equal &= tesseract::common::almostEqualRelativeAndAbs(acceleration_coeff, rhs.acceleration_coeff, max_diff);
  equal &= (smooth_jerks == rhs.smooth_jerks);
  equal &= tesseract::common::almostEqualRelativeAndAbs(jerk_coeff, rhs.jerk_coeff, max_diff);
  equal &= (avoid_singularity == rhs.avoid_singularity);
  equal &= tesseract::common::almostEqualRelativeAndAbs(avoid_singularity_coeff, rhs.avoid_singularity_coeff, max_diff);
  return equal;
}

bool TrajOptDefaultCompositeProfile::operator!=(const TrajOptDefaultCompositeProfile& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract::motion_planners
