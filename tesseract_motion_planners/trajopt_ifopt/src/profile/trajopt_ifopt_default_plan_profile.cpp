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

#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/instruction_type.h>
#include <trajopt_ifopt/trajopt_ifopt.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

namespace tesseract_planning
{
void TrajOptIfoptDefaultPlanProfile::apply(TrajOptIfoptProblem& problem,
                                           const CartesianWaypoint& cartesian_waypoint,
                                           const Instruction& parent_instruction,
                                           const ManipulatorInfo& manip_info,
                                           const std::vector<std::string>& active_links,
                                           int index) const
{
  assert(isPlanInstruction(parent_instruction));
  const auto& base_instruction = parent_instruction.as<PlanInstruction>();
  assert(!(manip_info.empty() && base_instruction.getManipulatorInfo().empty()));
  ManipulatorInfo mi = manip_info.getCombined(base_instruction.getManipulatorInfo());

  const auto& env = problem.environment;
  auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), problem.manip_fwd_kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  Eigen::Isometry3d world_to_base =
      env->getCurrentState()->link_transforms.at(problem.manip_fwd_kin->getBaseLinkName());
  Eigen::Isometry3d tcp = env->findTCP(mi);
  auto kin_info = std::make_shared<trajopt_ifopt::KinematicsInfo>(problem.manip_fwd_kin, adjacency_map, world_to_base);

  bool is_dyanmic{ false };
  std::string source_link;
  std::string target_link;  // Only Dynamic cartesian cartesian will have target_link
  Eigen::Isometry3d source_tcp{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d target_tcp{ Eigen::Isometry3d::Identity() };

  /* Check if this cartesian waypoint is dynamic
   * (i.e. defined relative to a frame that will move with the kinematic chain)
   */
  auto it = std::find(active_links.begin(), active_links.end(), mi.working_frame);
  if (it != active_links.end())
  {
    if (cartesian_waypoint.isToleranced())
      CONSOLE_BRIDGE_logWarn("Toleranced cartesian waypoints are not supported in this version of TrajOpt.");

    if (mi.tcp.isExternal() && mi.tcp.isString())
    {
      // If external, the part is attached to the robot so working frame is passed as link instead of target frame
      // Since it is a link name then it has the possibility to also be dynamic so need to check
      auto tcp_it = std::find(active_links.begin(), active_links.end(), mi.tcp.getString());
      if (tcp_it != active_links.end())
      {
        source_link = "";
        target_link = "";
        source_tcp = Eigen::Isometry3d::Identity();
        target_tcp = Eigen::Isometry3d::Identity();
        // target_tcp, index, target, tcp relative to robot tip link, coeffs, robot tip link, term_type
        //        ti = createDynamicCartesianWaypointTermInfo(Eigen::Isometry3d::Identity(),
        //                                                    index,
        //                                                    mi.tcp.getString(),
        //                                                    cartesian_waypoint,
        //                                                    cartesian_coeff,
        //                                                    mi.working_frame,
        //                                                    term_type);
      }
      else
      {
        source_link = mi.working_frame;
        source_tcp = cartesian_waypoint.waypoint;
        target_tcp = tcp;
      }
    }
    else if (mi.tcp.isExternal())
    {
      // If external, the part is attached to the robot so working frame is passed as link instead of target frame
      Eigen::Isometry3d local_tcp = tcp;
      if (!mi.tcp.getExternalFrame().empty())
        local_tcp = env->getCurrentState()->link_transforms.at(mi.tcp.getExternalFrame()) * tcp;

      source_link = mi.working_frame;
      source_tcp = cartesian_waypoint.waypoint;
      target_tcp = local_tcp;
      //      ti = createCartesianWaypointTermInfo(
      //          tcp, index, mi.tcp.getExternalFrame(), cartesian_waypoint, cartesian_coeff, mi.working_frame,
      //          term_type);
    }
    else
    {
      source_link = problem.manip_fwd_kin->getTipLinkName();
      source_tcp = tcp;
      target_tcp = cartesian_waypoint.waypoint;
      // target_tcp, index, target, tcp relative to robot tip link, coeffs, robot tip link, term_type
      //      ti = createDynamicCartesianWaypointTermInfo(
      //          cartesian_waypoint, index, mi.working_frame, tcp, cartesian_coeff, pci.kin->getTipLinkName(),
      //          term_type);
    }
  }
  else
  {
    source_link = problem.manip_fwd_kin->getTipLinkName();
    source_tcp = tcp;
    target_tcp = cartesian_waypoint.waypoint;

    //    ti = createCartesianWaypointTermInfo(
    //        cartesian_waypoint, index, mi.working_frame, tcp, cartesian_coeff, pci.kin->getTipLinkName(), term_type);
  }

  if (cartesian_coeff.rows() != 6)
    throw std::runtime_error("TrajOptIfoptDefaultPlanProfile: cartesian_coeff size must be 6.");

  trajopt_ifopt::JointPosition::ConstPtr var = problem.vars[static_cast<std::size_t>(index)];
  if (is_dyanmic)
  {
  }
  else
  {
    switch (term_type)
    {
      case TrajOptIfoptTermType::CONSTRAINT:
        addCartesianPositionConstraint(
            problem.nlp, target_tcp, var, kin_info, source_link, source_tcp, cartesian_coeff);
        break;
      case TrajOptIfoptTermType::SQUARED_COST:
        addCartesianPositionSquaredCost(
            problem.nlp, target_tcp, var, kin_info, source_link, source_tcp, cartesian_coeff);
        break;
      case TrajOptIfoptTermType::ABSOLUTE_COST:
        addCartesianPositionAbsoluteCost(
            problem.nlp, target_tcp, var, kin_info, source_link, source_tcp, cartesian_coeff);
        break;
    }
  }

  //  /* Check if this cartesian waypoint is dynamic
  //   * (i.e. defined relative to a frame that will move with the kinematic chain)
  //   */
  //  auto it = std::find(active_links.begin(), active_links.end(), mi.working_frame);
  //  if (it != active_links.end())
  //  {
  //    CONSOLE_BRIDGE_logWarn("Dynamic cartesian terms are not supported by trajopt_ifopt. PRs welcome");
  //  }
  //  else
  //  {
  //    auto idx = static_cast<std::size_t>(index);
  //    switch (term_type)
  //    {
  //      case TrajOptIfoptTermType::CONSTRAINT:
  //        addCartesianPositionConstraint(problem.nlp, cartesian_waypoint, problem.vars[idx], kin_info,
  //        cartesian_coeff);

  //        break;
  //      case TrajOptIfoptTermType::SQUARED_COST:
  //        addCartesianPositionConstraint(problem.nlp, cartesian_waypoint, problem.vars[idx], kin_info,
  //        cartesian_coeff); break;
  //    }
  //  }
}

void TrajOptIfoptDefaultPlanProfile::apply(TrajOptIfoptProblem& problem,
                                           const JointWaypoint& joint_waypoint,
                                           const Instruction& /*parent_instruction*/,
                                           const ManipulatorInfo& /*manip_info*/,
                                           const std::vector<std::string>& /*active_links*/,
                                           int index) const
{
  auto idx = static_cast<std::size_t>(index);
  auto vel_constraint = createJointPositionConstraint(joint_waypoint, problem.vars[idx], joint_coeff);
  switch (term_type)
  {
    case TrajOptIfoptTermType::CONSTRAINT:
      problem.nlp->addConstraintSet(vel_constraint);
      break;
    case TrajOptIfoptTermType::SQUARED_COST:
      problem.nlp->addCostSet(vel_constraint, trajopt_sqp::CostPenaltyType::SQUARED);
      break;
    case TrajOptIfoptTermType::ABSOLUTE_COST:
      problem.nlp->addCostSet(vel_constraint, trajopt_sqp::CostPenaltyType::ABSOLUTE);
      break;
  }
}

tinyxml2::XMLElement* TrajOptIfoptDefaultPlanProfile::toXML(tinyxml2::XMLDocument& /*doc*/) const
{
  throw std::runtime_error("TrajOptIfoptDefaultPlanProfile::toXML is not implemented!");
}

}  // namespace tesseract_planning
