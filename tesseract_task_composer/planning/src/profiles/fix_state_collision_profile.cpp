/**
 * @file fix_state_collision_profile.cpp
 * @brief Profile for process that pushes plan instructions to be out of collision
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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

#include <tesseract_task_composer/planning/profiles/fix_state_collision_profile.h>
#include <tesseract_task_composer/planning/yaml_extensions.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>
#include <tesseract_collision/core/yaml_extensions.h>
#include <tesseract_motion_planners/trajopt/yaml_extensions.h>
#include <trajopt_common/yaml_extensions.h>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/profile_plugin_factory.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
FixStateCollisionProfile::FixStateCollisionProfile(Settings mode)
  : Profile(createKey<FixStateCollisionProfile>()), mode(mode)
{
  collision_check_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  collision_check_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  trajopt_joint_constraint_config.coeff = Eigen::VectorXd::Constant(1, 1, 1);
  trajopt_joint_cost_config.coeff = Eigen::VectorXd::Constant(1, 1, 5);
  collision_constraint_coeff = trajopt_common::CollisionCoeffData(1.0);
  collision_cost_coeff = trajopt_common::CollisionCoeffData(20.0);
  sco::OSQPModelConfig::setDefaultOSQPSettings(osqp_settings);
}

FixStateCollisionProfile::FixStateCollisionProfile(const YAML::Node& config,
                                                   const tesseract_common::ProfilePluginFactory& /*plugin_factory*/)
  : FixStateCollisionProfile()
{
  try
  {
    if (YAML::Node n = config["mode"])
      mode = n.as<Settings>();
    if (YAML::Node n = config["correction_workflow"])
      correction_workflow = n.as<std::vector<CorrectionMethod>>();
    if (YAML::Node n = config["jiggle_factor"])
      jiggle_factor = n.as<double>();
    if (YAML::Node n = config["contact_manager_config"])
      contact_manager_config = n.as<tesseract_collision::ContactManagerConfig>();
    if (YAML::Node n = config["collision_check_config"])
      collision_check_config = n.as<tesseract_collision::CollisionCheckConfig>();
    if (YAML::Node n = config["sampling_attempts"])
      sampling_attempts = n.as<int>();
    if (YAML::Node n = config["trajopt_joint_constraint_config"])
      trajopt_joint_constraint_config = n.as<tesseract_planning::TrajOptJointWaypointConfig>();
    if (YAML::Node n = config["trajopt_joint_cost_config"])
      trajopt_joint_cost_config = n.as<tesseract_planning::TrajOptJointWaypointConfig>();
    if (YAML::Node n = config["collision_constraint_coeff"])
      collision_constraint_coeff = n.as<trajopt_common::CollisionCoeffData>();
    if (YAML::Node n = config["collision_cost_coeff"])
      collision_cost_coeff = n.as<trajopt_common::CollisionCoeffData>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("FixStateCollisionProfile: Failed to parse yaml config! Details: " +
                             std::string(e.what()));
  }
}

bool FixStateCollisionProfile::operator==(const FixStateCollisionProfile& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (mode == rhs.mode);
  equal &= (correction_workflow == rhs.correction_workflow);
  equal &= tesseract_common::almostEqualRelativeAndAbs(jiggle_factor, rhs.jiggle_factor, max_diff);
  equal &= (contact_manager_config == rhs.contact_manager_config);
  equal &= (collision_check_config == rhs.collision_check_config);
  equal &= (sampling_attempts == rhs.sampling_attempts);
  equal &= (trajopt_joint_constraint_config == rhs.trajopt_joint_constraint_config);
  equal &= (trajopt_joint_cost_config == rhs.trajopt_joint_cost_config);
  equal &= (collision_constraint_coeff == rhs.collision_constraint_coeff);
  equal &= (collision_cost_coeff == rhs.collision_cost_coeff);
  equal &= (opt_params == rhs.opt_params);
  equal &= (osqp_settings == rhs.osqp_settings);
  equal &= (update_workspace == rhs.update_workspace);
  return equal;
}

bool FixStateCollisionProfile::operator!=(const FixStateCollisionProfile& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_planning
