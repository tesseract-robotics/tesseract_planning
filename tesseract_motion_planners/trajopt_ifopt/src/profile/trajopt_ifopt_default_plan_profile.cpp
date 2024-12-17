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
#include <tinyxml2.h>
#include <trajopt_sqp/qp_problem.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_problem.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

#include <tesseract_common/manipulator_info.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>

#include <tesseract_common/eigen_serialization.h>

namespace tesseract_planning
{
void TrajOptIfoptDefaultPlanProfile::apply(TrajOptIfoptProblem& problem,
                                           const CartesianWaypointPoly& cartesian_waypoint,
                                           const InstructionPoly& parent_instruction,
                                           const tesseract_common::ManipulatorInfo& manip_info,
                                           const std::vector<std::string>& /*active_links*/,
                                           int index) const
{
  assert(parent_instruction.isMoveInstruction());
  const auto& base_instruction = parent_instruction.as<MoveInstructionPoly>();
  assert(!(manip_info.empty() && base_instruction.getManipulatorInfo().empty()));
  tesseract_common::ManipulatorInfo mi = manip_info.getCombined(base_instruction.getManipulatorInfo());

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
  switch (term_type)
  {
    case TrajOptIfoptTermType::CONSTRAINT:
      addCartesianPositionConstraint(*problem.nlp,
                                     var,
                                     problem.manip,
                                     mi.tcp_frame,
                                     mi.working_frame,
                                     tcp_offset,
                                     cartesian_waypoint.getTransform(),
                                     cartesian_coeff);
      break;
    case TrajOptIfoptTermType::SQUARED_COST:
      addCartesianPositionSquaredCost(*problem.nlp,
                                      var,
                                      problem.manip,
                                      mi.tcp_frame,
                                      mi.working_frame,
                                      tcp_offset,
                                      cartesian_waypoint.getTransform(),
                                      cartesian_coeff);
      break;
    case TrajOptIfoptTermType::ABSOLUTE_COST:
      addCartesianPositionAbsoluteCost(*problem.nlp,
                                       var,
                                       problem.manip,
                                       mi.tcp_frame,
                                       mi.working_frame,
                                       tcp_offset,
                                       cartesian_waypoint.getTransform(),
                                       cartesian_coeff);
      break;
  }
}

void TrajOptIfoptDefaultPlanProfile::apply(TrajOptIfoptProblem& problem,
                                           const JointWaypointPoly& joint_waypoint,
                                           const InstructionPoly& /*parent_instruction*/,
                                           const tesseract_common::ManipulatorInfo& /*manip_info*/,
                                           const std::vector<std::string>& /*active_links*/,
                                           int index) const
{
  auto idx = static_cast<std::size_t>(index);
  switch (term_type)
  {
    case TrajOptIfoptTermType::CONSTRAINT:
      addJointPositionConstraint(*problem.nlp, joint_waypoint, problem.vars[idx], joint_coeff);
      break;
    case TrajOptIfoptTermType::SQUARED_COST:
      addJointPositionSquaredCost(*problem.nlp, joint_waypoint, problem.vars[idx], joint_coeff);
      break;
    case TrajOptIfoptTermType::ABSOLUTE_COST:
      addJointPositionAbsoluteCost(*problem.nlp, joint_waypoint, problem.vars[idx], joint_coeff);
      break;
  }
}

tinyxml2::XMLElement* TrajOptIfoptDefaultPlanProfile::toXML(tinyxml2::XMLDocument& /*doc*/) const
{
  throw std::runtime_error("TrajOptIfoptDefaultPlanProfile::toXML is not implemented!");
}

template <class Archive>
void TrajOptIfoptDefaultPlanProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TrajOptIfoptPlanProfile);
  ar& BOOST_SERIALIZATION_NVP(cartesian_coeff);
  ar& BOOST_SERIALIZATION_NVP(joint_coeff);
  ar& BOOST_SERIALIZATION_NVP(term_type);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptDefaultPlanProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptDefaultPlanProfile)
