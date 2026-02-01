/**
 * @file trajopt_default_move_profile.cpp
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
#include <trajopt_sqp/qp_problem.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

#include <tesseract_common/joint_state.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/profile_plugin_factory.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_motion_planners/trajopt_ifopt/yaml_extensions.h>

namespace tesseract_planning
{
TrajOptIfoptDefaultMoveProfile::TrajOptIfoptDefaultMoveProfile()
{
  cartesian_cost_config.enabled = false;
  joint_cost_config.enabled = false;
}

TrajOptIfoptDefaultMoveProfile::TrajOptIfoptDefaultMoveProfile(
    const YAML::Node& config,
    const tesseract_common::ProfilePluginFactory& /*plugin_factory*/)
  : TrajOptIfoptDefaultMoveProfile()
{
  try
  {
    if (YAML::Node n = config["cartesian_cost_config"])
      cartesian_cost_config = n.as<tesseract_planning::TrajOptIfoptCartesianWaypointConfig>();

    if (YAML::Node n = config["cartesian_constraint_config"])
      cartesian_constraint_config = n.as<tesseract_planning::TrajOptIfoptCartesianWaypointConfig>();

    if (YAML::Node n = config["joint_cost_config"])
      joint_cost_config = n.as<tesseract_planning::TrajOptIfoptJointWaypointConfig>();

    if (YAML::Node n = config["joint_constraint_config"])
      joint_constraint_config = n.as<tesseract_planning::TrajOptIfoptJointWaypointConfig>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("TrajOptIfoptDefaultMoveProfile: Failed to parse yaml config! Details: " +
                             std::string(e.what()));
  }
}

TrajOptIfoptWaypointInfo
TrajOptIfoptDefaultMoveProfile::create(const MoveInstructionPoly& move_instruction,
                                       const tesseract_common::ManipulatorInfo& composite_manip_info,
                                       const std::shared_ptr<const tesseract_environment::Environment>& env,
                                       int index) const
{
  assert(!(composite_manip_info.empty() && move_instruction.getManipulatorInfo().empty()));
  tesseract_common::ManipulatorInfo mi = composite_manip_info.getCombined(move_instruction.getManipulatorInfo());
  std::vector<std::string> joint_names = env->getGroupJointNames(mi.manipulator);
  assert(checkJointPositionFormat(joint_names, move_instruction.getWaypoint()));

  tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup(mi.manipulator);
  const std::vector<trajopt_ifopt::Bounds> bounds = trajopt_ifopt::toBounds(manip->getLimits().joint_limits);

  TrajOptIfoptWaypointInfo info;
  if (move_instruction.getWaypoint().isCartesianWaypoint())
  {
    if (mi.empty())
      throw std::runtime_error("TrajOptMoveProfile, manipulator info is empty!");

    const auto& cwp = move_instruction.getWaypoint().as<CartesianWaypointPoly>();

    Eigen::VectorXd seed;
    if (cwp.hasSeed())
      seed = cwp.getSeed().position;
    else
      seed = env->getCurrentJointValues(joint_names);

    info.node = std::make_unique<trajopt_ifopt::Node>("Node_" + std::to_string(index));
    std::shared_ptr<const trajopt_ifopt::Var> var = info.node->addVar("position", joint_names, seed, bounds);

    Eigen::Isometry3d tcp_offset = env->findTCPOffset(mi);

    // Override cost tolerances if the profile specifies that they should be overrided.
    Eigen::VectorXd lower_tolerance_cost = cwp.getLowerTolerance();
    Eigen::VectorXd upper_tolerance_cost = cwp.getUpperTolerance();
    if (cartesian_cost_config.use_tolerance_override)
    {
      lower_tolerance_cost = cartesian_cost_config.lower_tolerance;
      upper_tolerance_cost = cartesian_cost_config.upper_tolerance;
    }
    Eigen::VectorXd lower_tolerance_cnt = cwp.getLowerTolerance();
    Eigen::VectorXd upper_tolerance_cnt = cwp.getUpperTolerance();
    if (cartesian_constraint_config.use_tolerance_override)
    {
      lower_tolerance_cnt = cartesian_constraint_config.lower_tolerance;
      upper_tolerance_cnt = cartesian_constraint_config.upper_tolerance;
    }

    /** @todo Levi, update to support toleranced cartesian */

    if (cartesian_cost_config.enabled)
    {
      std::vector<trajopt_ifopt::Bounds> bounds;
      if (cwp.isToleranced() || cartesian_cost_config.use_tolerance_override)
        bounds = trajopt_ifopt::toBounds(lower_tolerance_cost, upper_tolerance_cost);
      else
        bounds = std::vector<trajopt_ifopt::Bounds>(static_cast<std::size_t>(cartesian_cost_config.coeff.rows()),
                                                    trajopt_ifopt::BoundZero);

      auto constraint = createCartesianPositionConstraint(var,
                                                          manip,
                                                          mi.tcp_frame,
                                                          mi.working_frame,
                                                          tcp_offset,
                                                          cwp.getTransform(),
                                                          cartesian_cost_config.coeff,
                                                          bounds);

      info.term_infos.squared_costs.push_back(constraint);
    }

    if (cartesian_constraint_config.enabled)
    {
      std::vector<trajopt_ifopt::Bounds> bounds;
      if (cwp.isToleranced() || cartesian_constraint_config.use_tolerance_override)
        bounds = trajopt_ifopt::toBounds(lower_tolerance_cost, upper_tolerance_cost);
      else
        bounds = std::vector<trajopt_ifopt::Bounds>(static_cast<std::size_t>(cartesian_constraint_config.coeff.rows()),
                                                    trajopt_ifopt::BoundZero);

      auto constraint = createCartesianPositionConstraint(var,
                                                          manip,
                                                          mi.tcp_frame,
                                                          mi.working_frame,
                                                          tcp_offset,
                                                          cwp.getTransform(),
                                                          cartesian_constraint_config.coeff,
                                                          bounds);
      info.term_infos.constraints.push_back(constraint);
    }

    /** @todo If fixed cartesian and not term_type cost add as fixed */
    info.fixed = false;
  }
  else if (move_instruction.getWaypoint().isJointWaypoint() || move_instruction.getWaypoint().isStateWaypoint())
  {
    JointWaypointPoly jwp;
    if (move_instruction.getWaypoint().isStateWaypoint())
    {
      const auto& swp = move_instruction.getWaypoint().as<StateWaypointPoly>();
      jwp = move_instruction.createJointWaypoint();
      jwp.setNames(swp.getNames());
      jwp.setPosition(swp.getPosition());
      jwp.setIsConstrained(true);
      info.fixed = true;
    }
    else
    {
      jwp = move_instruction.getWaypoint().as<JointWaypointPoly>();
      if (jwp.isConstrained())
      {
        // Add to fixed indices
        if (!jwp.isToleranced()) /** @todo Should not make fixed if term_type is cost */
          info.fixed = true;
      }
    }

    // Create var set
    info.node = std::make_unique<trajopt_ifopt::Node>("Node_" + std::to_string(index));
    std::shared_ptr<const trajopt_ifopt::Var> var =
        info.node->addVar("position", joint_names, jwp.getPosition(), bounds);

    if (jwp.isConstrained())
    {
      // Override cost tolerances if the profile specifies that they should be overrided.
      Eigen::VectorXd lower_tolerance_cost = jwp.getLowerTolerance();
      Eigen::VectorXd upper_tolerance_cost = jwp.getUpperTolerance();
      if (joint_cost_config.use_tolerance_override)
      {
        lower_tolerance_cost = joint_cost_config.lower_tolerance;
        upper_tolerance_cost = joint_cost_config.upper_tolerance;
      }
      Eigen::VectorXd lower_tolerance_cnt = jwp.getLowerTolerance();
      Eigen::VectorXd upper_tolerance_cnt = jwp.getUpperTolerance();
      if (joint_constraint_config.use_tolerance_override)
      {
        lower_tolerance_cnt = joint_constraint_config.lower_tolerance;
        upper_tolerance_cnt = joint_constraint_config.upper_tolerance;
      }

      if (joint_cost_config.enabled)
      {
        // createJointPositionConstraint handles the tolerance when creating the constraint
        if (jwp.isToleranced())
        {
          jwp.setLowerTolerance(lower_tolerance_cost);
          jwp.setUpperTolerance(upper_tolerance_cost);
        }
        auto constraint = createJointPositionConstraint(jwp, var, joint_cost_config.coeff);
        info.term_infos.squared_costs.push_back(constraint);
      }
      if (joint_constraint_config.enabled)
      {
        // createJointPositionConstraint handles the tolerance when creating the constraint
        if (jwp.isToleranced())
        {
          jwp.setLowerTolerance(lower_tolerance_cnt);
          jwp.setUpperTolerance(upper_tolerance_cnt);
        }
        auto constraint = createJointPositionConstraint(jwp, var, joint_constraint_config.coeff);
        info.term_infos.constraints.push_back(constraint);
      }
    }
  }

  return info;
}

bool TrajOptIfoptDefaultMoveProfile::operator==(const TrajOptIfoptDefaultMoveProfile& rhs) const
{
  bool equal = true;
  equal &= (cartesian_cost_config == rhs.cartesian_cost_config);
  equal &= (cartesian_constraint_config == rhs.cartesian_constraint_config);
  equal &= (joint_cost_config == rhs.joint_cost_config);
  equal &= (joint_constraint_config == rhs.joint_constraint_config);
  return equal;
}

bool TrajOptIfoptDefaultMoveProfile::operator!=(const TrajOptIfoptDefaultMoveProfile& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract_planning
