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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt_common/collision_types.h>
#include <trajopt_common/utils.hpp>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/profile_plugin_factory.h>
#include <tesseract_motion_planners/trajopt_ifopt/yaml_extensions.h>

namespace tesseract::motion_planners
{
TrajOptIfoptDefaultCompositeProfile::TrajOptIfoptDefaultCompositeProfile(
    const YAML::Node& config,
    const tesseract::common::ProfilePluginFactory& /*plugin_factory*/)
  : TrajOptIfoptDefaultCompositeProfile()
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
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("TrajOptDefaultCompositeProfile: Failed to parse yaml config! Details: " +
                             std::string(e.what()));
  }
}

TrajOptIfoptTermInfos
TrajOptIfoptDefaultCompositeProfile::create(const tesseract::common::ManipulatorInfo& composite_manip_info,
                                            const std::shared_ptr<const tesseract::environment::Environment>& env,
                                            const std::vector<std::shared_ptr<const trajopt_ifopt::Node>>& nodes,
                                            const std::vector<int>& fixed_indices) const
{
  if (nodes.empty())
    throw std::runtime_error("TrajOptIfoptDefaultCompositeProfile: vars is empty.");

  std::vector<std::shared_ptr<const trajopt_ifopt::Var>> vars;
  vars.reserve(nodes.size());
  for (const auto& node : nodes)
    vars.push_back(node->getVar("position"));

  TrajOptIfoptTermInfos term_infos;

  if (collision_constraint_config.enabled)
  {
    auto constraints =
        createCollisionConstraints(vars, env, composite_manip_info, collision_constraint_config, fixed_indices, false);
    term_infos.constraints.insert(term_infos.constraints.end(), constraints.begin(), constraints.end());
  }

  if (collision_cost_config.enabled)
  {
    // Coefficients are applied within the constraint
    auto constraints =
        createCollisionConstraints(vars, env, composite_manip_info, collision_cost_config, fixed_indices, false);
    term_infos.hinge_costs.insert(term_infos.hinge_costs.end(), constraints.begin(), constraints.end());
  }

  if (smooth_velocities)
  {
    Eigen::VectorXd target = Eigen::VectorXd::Zero(vars.front()->size());
    auto constraint = createJointVelocityConstraint(target, vars, velocity_coeff);
    term_infos.squared_costs.push_back(constraint);
  }

  if (smooth_accelerations)
  {
    Eigen::VectorXd target = Eigen::VectorXd::Zero(vars.front()->size());
    auto constraint = createJointAccelerationConstraint(target, vars, acceleration_coeff);
    term_infos.squared_costs.push_back(constraint);
  }

  if (smooth_jerks)
  {
    Eigen::VectorXd target = Eigen::VectorXd::Zero(vars.front()->size());
    auto constraint = createJointJerkConstraint(target, vars, jerk_coeff);
    term_infos.squared_costs.push_back(constraint);
  }

  return term_infos;
}

bool TrajOptIfoptDefaultCompositeProfile::operator==(const TrajOptIfoptDefaultCompositeProfile& rhs) const
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
  return equal;
}

bool TrajOptIfoptDefaultCompositeProfile::operator!=(const TrajOptIfoptDefaultCompositeProfile& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract::motion_planners
