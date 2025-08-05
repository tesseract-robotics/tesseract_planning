/**
 * @file trajopt_default_composite_profile.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_common/collision_types.h>
#include <trajopt_common/utils.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/eigen_serialization.h>
#include <tesseract_collision/core/serialization.h>
#include <tesseract_common/profile_plugin_factory.h>
#include <tesseract_motion_planners/trajopt_ifopt/yaml_extensions.h>


namespace tesseract_planning
{
TrajOptIfoptDefaultCompositeProfile::TrajOptIfoptDefaultCompositeProfile(const YAML::Node& config, const tesseract_common::ProfilePluginFactory& /*plugin_factory*/) 
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
    throw std::runtime_error("TrajOptDefaultCompositeProfile: Failed to parse yaml config! Details: " + std::string(e.what()));
  }
}

  
TrajOptIfoptTermInfos TrajOptIfoptDefaultCompositeProfile::create(
    const tesseract_common::ManipulatorInfo& composite_manip_info,
    const std::shared_ptr<const tesseract_environment::Environment>& env,
    const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition> >& vars,
    const std::vector<int>& fixed_indices) const
{
  if (vars.empty())
    throw std::runtime_error("TrajOptIfoptDefaultCompositeProfile: vars is empty.");

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
    Eigen::VectorXd target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
    auto constraint = createJointVelocityConstraint(target, vars, velocity_coeff);
    term_infos.squared_costs.push_back(constraint);
  }

  if (smooth_accelerations)
  {
    Eigen::VectorXd target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
    auto constraint = createJointAccelerationConstraint(target, vars, acceleration_coeff);
    term_infos.squared_costs.push_back(constraint);
  }

  if (smooth_jerks)
  {
    Eigen::VectorXd target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
    auto constraint = createJointJerkConstraint(target, vars, jerk_coeff);
    term_infos.squared_costs.push_back(constraint);
  }

  return term_infos;
}

template <class Archive>
void TrajOptIfoptDefaultCompositeProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TrajOptIfoptCompositeProfile);
  ar& BOOST_SERIALIZATION_NVP(collision_cost_config);
  ar& BOOST_SERIALIZATION_NVP(collision_constraint_config);
  ar& BOOST_SERIALIZATION_NVP(smooth_velocities);
  ar& BOOST_SERIALIZATION_NVP(velocity_coeff);
  ar& BOOST_SERIALIZATION_NVP(smooth_accelerations);
  ar& BOOST_SERIALIZATION_NVP(acceleration_coeff);
  ar& BOOST_SERIALIZATION_NVP(smooth_jerks);
  ar& BOOST_SERIALIZATION_NVP(jerk_coeff);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptDefaultCompositeProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptDefaultCompositeProfile)
