/**
 * @file trajopt_default_plan_profile.cpp
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
#include <trajopt_sqp/qp_problem.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

#include <tesseract_common/joint_state.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>

#include <tesseract_common/eigen_serialization.h>

namespace tesseract_planning
{
TrajOptIfoptDefaultPlanProfile::TrajOptIfoptDefaultPlanProfile()
{
  cartesian_cost_config.enabled = false;
  joint_cost_config.enabled = false;
}

TrajOptIfoptWaypointInfo
TrajOptIfoptDefaultPlanProfile::create(const MoveInstructionPoly& move_instruction,
                                       const tesseract_common::ManipulatorInfo& composite_manip_info,
                                       const std::shared_ptr<const tesseract_environment::Environment>& env,
                                       int index) const
{
  assert(!(composite_manip_info.empty() && move_instruction.getManipulatorInfo().empty()));
  tesseract_common::ManipulatorInfo mi = composite_manip_info.getCombined(move_instruction.getManipulatorInfo());
  std::vector<std::string> joint_names = env->getGroupJointNames(mi.manipulator);
  assert(checkJointPositionFormat(joint_names, move_instruction.getWaypoint()));

  tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup(mi.manipulator);
  Eigen::MatrixX2d joint_limits = manip->getLimits().joint_limits;

  TrajOptIfoptWaypointInfo info;
  if (move_instruction.getWaypoint().isCartesianWaypoint())
  {
    if (mi.empty())
      throw std::runtime_error("TrajOptPlanProfile, manipulator info is empty!");

    const auto& cwp = move_instruction.getWaypoint().as<CartesianWaypointPoly>();

    Eigen::VectorXd seed;
    if (cwp.hasSeed())
      seed = cwp.getSeed().position;
    else
      seed = env->getCurrentJointValues(joint_names);

    info.var =
        std::make_shared<trajopt_ifopt::JointPosition>(seed, joint_names, "Joint_Position_" + std::to_string(index));
    info.var->SetBounds(joint_limits);

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
      std::vector<double> coeffs;
      for (Eigen::Index i = 0; i < cartesian_cost_config.coeff.rows(); ++i)
      {
        if (tesseract_common::almostEqualRelativeAndAbs(cartesian_cost_config.coeff(i), 0.0))
          coeffs.push_back(0);
        else
          coeffs.push_back(1);
      }

      auto constraint = createCartesianPositionConstraint(
          info.var,
          manip,
          mi.tcp_frame,
          mi.working_frame,
          tcp_offset,
          cwp.getTransform(),
          Eigen::Map<Eigen::VectorXd>(coeffs.data(), static_cast<Eigen::Index>(coeffs.size())));

      info.term_infos.squared_costs.push_back(constraint);
    }

    if (cartesian_constraint_config.enabled)
    {
      auto constraint = createCartesianPositionConstraint(info.var,
                                                          manip,
                                                          mi.tcp_frame,
                                                          mi.working_frame,
                                                          tcp_offset,
                                                          cwp.getTransform(),
                                                          cartesian_constraint_config.coeff);
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
    info.var = std::make_shared<trajopt_ifopt::JointPosition>(
        jwp.getPosition(), joint_names, "Joint_Position_" + std::to_string(index));
    info.var->SetBounds(joint_limits);

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
        auto constraint = createJointPositionConstraint(jwp, info.var, joint_cost_config.coeff);
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
        auto constraint = createJointPositionConstraint(jwp, info.var, joint_constraint_config.coeff);
        info.term_infos.constraints.push_back(constraint);
      }
    }
  }

  return info;
}

template <class Archive>
void TrajOptIfoptDefaultPlanProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TrajOptIfoptPlanProfile);
  ar& BOOST_SERIALIZATION_NVP(cartesian_cost_config);
  ar& BOOST_SERIALIZATION_NVP(cartesian_constraint_config);
  ar& BOOST_SERIALIZATION_NVP(joint_cost_config);
  ar& BOOST_SERIALIZATION_NVP(joint_constraint_config);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptDefaultPlanProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptDefaultPlanProfile)
