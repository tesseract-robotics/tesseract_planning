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

  if (manip_info.manipulator.empty())
    throw std::runtime_error("TrajOptIfoptDefaultPlanProfile, manipulator is empty!");

  if (manip_info.tcp_frame.empty())
    throw std::runtime_error("TrajOptIfoptDefaultPlanProfile, tcp_frame is empty!");

  if (manip_info.working_frame.empty())
    throw std::runtime_error("TrajOptIfoptDefaultPlanProfile, working_frame is empty!");

  Eigen::Isometry3d tcp_offset = problem.environment->findTCPOffset(mi);

  if (cartesian_coeff.rows() != 6)
    throw std::runtime_error("TrajOptIfoptDefaultPlanProfile: cartesian_coeff size must be 6.");

  trajopt_ifopt::JointPosition::ConstPtr var = problem.vars[static_cast<std::size_t>(index)];

  /* Check if this cartesian waypoint is dynamic
   * (i.e. defined relative to a frame that will move with the kinematic chain)
   */
  bool is_active_tcp_frame = (std::find(active_links.begin(), active_links.end(), mi.tcp_frame) == active_links.end());
  bool is_static_working_frame =
      (std::find(active_links.begin(), active_links.end(), mi.working_frame) == active_links.end());

  if ((is_static_working_frame && is_active_tcp_frame) || (!is_active_tcp_frame && !is_static_working_frame))
  {
    switch (term_type)
    {
      case TrajOptIfoptTermType::CONSTRAINT:
        addCartesianPositionConstraint(*problem.nlp,
                                       var,
                                       problem.manip,
                                       mi.tcp_frame,
                                       mi.working_frame,
                                       tcp_offset,
                                       cartesian_waypoint,
                                       cartesian_coeff);
        break;
      case TrajOptIfoptTermType::SQUARED_COST:
        addCartesianPositionSquaredCost(*problem.nlp,
                                        var,
                                        problem.manip,
                                        mi.tcp_frame,
                                        mi.working_frame,
                                        tcp_offset,
                                        cartesian_waypoint,
                                        cartesian_coeff);
        break;
      case TrajOptIfoptTermType::ABSOLUTE_COST:
        addCartesianPositionAbsoluteCost(*problem.nlp,
                                         var,
                                         problem.manip,
                                         mi.tcp_frame,
                                         mi.working_frame,
                                         tcp_offset,
                                         cartesian_waypoint,
                                         cartesian_coeff);
        break;
    }
  }
  else if (!is_static_working_frame && is_active_tcp_frame)
  {
    throw std::runtime_error("TrajOpt IFOPT currently does not support dynamic cartesian waypoints!");
  }
  else
  {
    throw std::runtime_error("TrajOpt, both tcp_frame and working_frame are both static!");
  }
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
