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
#include <boost/algorithm/string.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_common/joint_state.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_environment/environment.h>

namespace tesseract_planning
{
TrajOptDefaultPlanProfile::TrajOptDefaultPlanProfile()
{
  cartesian_cost_config.enabled = false;
  joint_cost_config.enabled = false;
}

TrajOptWaypointInfo
TrajOptDefaultPlanProfile::create(const MoveInstructionPoly& move_instruction,
                                  const tesseract_common::ManipulatorInfo& composite_manip_info,
                                  const std::shared_ptr<const tesseract_environment::Environment>& env,
                                  const std::vector<std::string>& active_links,
                                  int index) const
{
  assert(!(composite_manip_info.empty() && move_instruction.getManipulatorInfo().empty()));
  tesseract_common::ManipulatorInfo mi = composite_manip_info.getCombined(move_instruction.getManipulatorInfo());
  std::vector<std::string> joint_names = env->getGroupJointNames(mi.manipulator);
  assert(checkJointPositionFormat(joint_names, move_instruction.getWaypoint()));

  TrajOptWaypointInfo info;
  if (move_instruction.getWaypoint().isCartesianWaypoint())
  {
    if (mi.empty())
      throw std::runtime_error("TrajOptPlanProfile, manipulator info is empty!");

    const auto& cwp = move_instruction.getWaypoint().as<CartesianWaypointPoly>();

    Eigen::Isometry3d tcp_offset = env->findTCPOffset(mi);

    /* Check if this cartesian waypoint is dynamic
     * (i.e. defined relative to a frame that will move with the kinematic chain)
     */
    bool is_active_tcp_frame =
        (std::find(active_links.begin(), active_links.end(), mi.tcp_frame) != active_links.end());
    bool is_static_working_frame =
        (std::find(active_links.begin(), active_links.end(), mi.working_frame) == active_links.end());

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

    if ((is_static_working_frame && is_active_tcp_frame) || (!is_active_tcp_frame && !is_static_working_frame))
    {
      if (cartesian_cost_config.enabled)
      {
        trajopt::TermInfo::Ptr ti = createCartesianWaypointTermInfo(index,
                                                                    mi.working_frame,
                                                                    cwp.getTransform(),
                                                                    mi.tcp_frame,
                                                                    tcp_offset,
                                                                    cartesian_cost_config.coeff,
                                                                    trajopt::TermType::TT_COST,
                                                                    lower_tolerance_cost,
                                                                    upper_tolerance_cost);
        info.term_infos.costs.push_back(ti);
      }
      if (cartesian_constraint_config.enabled)
      {
        trajopt::TermInfo::Ptr ti = createCartesianWaypointTermInfo(index,
                                                                    mi.working_frame,
                                                                    cwp.getTransform(),
                                                                    mi.tcp_frame,
                                                                    tcp_offset,
                                                                    cartesian_constraint_config.coeff,
                                                                    trajopt::TermType::TT_CNT,
                                                                    lower_tolerance_cnt,
                                                                    upper_tolerance_cnt);
        info.term_infos.constraints.push_back(ti);
      }
    }
    else if (!is_static_working_frame && is_active_tcp_frame)
    {
      if (cartesian_cost_config.enabled)
      {
        trajopt::TermInfo::Ptr ti = createDynamicCartesianWaypointTermInfo(index,
                                                                           mi.working_frame,
                                                                           cwp.getTransform(),
                                                                           mi.tcp_frame,
                                                                           tcp_offset,
                                                                           cartesian_cost_config.coeff,
                                                                           trajopt::TermType::TT_COST,
                                                                           lower_tolerance_cost,
                                                                           upper_tolerance_cost);
        info.term_infos.costs.push_back(ti);
      }
      if (cartesian_constraint_config.enabled)
      {
        trajopt::TermInfo::Ptr ti = createDynamicCartesianWaypointTermInfo(index,
                                                                           mi.working_frame,
                                                                           cwp.getTransform(),
                                                                           mi.tcp_frame,
                                                                           tcp_offset,
                                                                           cartesian_constraint_config.coeff,
                                                                           trajopt::TermType::TT_CNT,
                                                                           lower_tolerance_cnt,
                                                                           upper_tolerance_cnt);
        info.term_infos.constraints.push_back(ti);
      }
    }
    else
    {
      throw std::runtime_error("TrajOpt, both tcp_frame and working_frame are both static!");
    }

    if (cwp.hasSeed())
      info.seed = cwp.getSeed().position;
    else
      info.seed = env->getCurrentJointValues(joint_names);

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

    // Set seed state
    info.seed = jwp.getPosition();

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
      if (jwp.isToleranced())
      {
        if (joint_cost_config.enabled)
        {
          trajopt::TermInfo::Ptr ti = createTolerancedJointWaypointTermInfo(jwp.getPosition(),
                                                                            lower_tolerance_cost,
                                                                            upper_tolerance_cost,
                                                                            index,
                                                                            joint_cost_config.coeff,
                                                                            trajopt::TermType::TT_COST);
          info.term_infos.costs.push_back(ti);
        }
        if (joint_constraint_config.enabled)
        {
          trajopt::TermInfo::Ptr ti = createTolerancedJointWaypointTermInfo(jwp.getPosition(),
                                                                            lower_tolerance_cnt,
                                                                            upper_tolerance_cnt,
                                                                            index,
                                                                            joint_constraint_config.coeff,
                                                                            trajopt::TermType::TT_CNT);
          info.term_infos.constraints.push_back(ti);
        }
      }
      else
      {
        if (joint_cost_config.enabled)
        {
          trajopt::TermInfo::Ptr ti = createJointWaypointTermInfo(
              jwp.getPosition(), index, joint_cost_config.coeff, trajopt::TermType::TT_COST);
          info.term_infos.costs.push_back(ti);
        }
        if (joint_constraint_config.enabled)
        {
          trajopt::TermInfo::Ptr ti = createJointWaypointTermInfo(
              jwp.getPosition(), index, joint_constraint_config.coeff, trajopt::TermType::TT_CNT);
          info.term_infos.constraints.push_back(ti);
        }
      }
    }
  }

  return info;
}

std::shared_ptr<trajopt::TermInfo>
TrajOptDefaultPlanProfile::createConstraintFromErrorFunction(sco::VectorOfVector::func error_function,
                                                             sco::MatrixOfVector::func jacobian_function,
                                                             sco::ConstraintType type,
                                                             const Eigen::VectorXd& coeff,
                                                             int index)
{
  trajopt::TermInfo::Ptr ti = createUserDefinedTermInfo(
      index, index, std::move(error_function), std::move(jacobian_function), trajopt::TermType::TT_CNT);

  // Update the term info with the (possibly) new start and end state indices for which to apply this cost
  std::shared_ptr<trajopt::UserDefinedTermInfo> ef = std::static_pointer_cast<trajopt::UserDefinedTermInfo>(ti);
  ef->constraint_type = type;
  ef->coeff = coeff;
  return ef;
}

template <class Archive>
void TrajOptDefaultPlanProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TrajOptPlanProfile);
  ar& BOOST_SERIALIZATION_NVP(cartesian_cost_config);
  ar& BOOST_SERIALIZATION_NVP(cartesian_constraint_config);
  ar& BOOST_SERIALIZATION_NVP(joint_cost_config);
  ar& BOOST_SERIALIZATION_NVP(joint_constraint_config);
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptDefaultPlanProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptDefaultPlanProfile)
