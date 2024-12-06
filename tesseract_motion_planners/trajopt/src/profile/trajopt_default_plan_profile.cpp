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
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#include <tesseract_command_language/poly/move_instruction_poly.h>

#include <tesseract_environment/environment.h>

namespace tesseract_planning
{
TrajOptDefaultPlanProfile::TrajOptDefaultPlanProfile(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* cartesian_cost_element = xml_element.FirstChildElement("CartesianCostConfig");
  const tinyxml2::XMLElement* cartesian_constraint_element = xml_element.FirstChildElement("CartesianConstraintConfig");
  const tinyxml2::XMLElement* joint_cost_element = xml_element.FirstChildElement("JointCostConfig");
  const tinyxml2::XMLElement* joint_constraint_element = xml_element.FirstChildElement("JointConstraintConfig");
  const tinyxml2::XMLElement* cnt_error_fn_element = xml_element.FirstChildElement("ConstraintErrorFunctions");

  int status{ tinyxml2::XMLError::XML_SUCCESS };

  if (cartesian_cost_element != nullptr)
  {
    const tinyxml2::XMLElement* cartesian_waypoint_config = cartesian_cost_element->FirstChildElement("CartesianWaypoin"
                                                                                                      "tConfig");
    cartesian_cost_config = CartesianWaypointConfig(*cartesian_waypoint_config);
  }

  if (cartesian_constraint_element != nullptr)
  {
    const tinyxml2::XMLElement* cartesian_waypoint_config = cartesian_constraint_element->FirstChildElement("CartesianW"
                                                                                                            "aypointCon"
                                                                                                            "fig");
    cartesian_constraint_config = CartesianWaypointConfig(*cartesian_waypoint_config);
  }

  if (joint_cost_element != nullptr)
  {
    const tinyxml2::XMLElement* cartesian_waypoint_config = joint_cost_element->FirstChildElement("JointWaypointConfi"
                                                                                                  "g");
    joint_cost_config = JointWaypointConfig(*cartesian_waypoint_config);
  }

  if (joint_constraint_element != nullptr)
  {
    const tinyxml2::XMLElement* cartesian_waypoint_config = joint_constraint_element->FirstChildElement("JointWaypointC"
                                                                                                        "onfig");
    joint_constraint_config = JointWaypointConfig(*cartesian_waypoint_config);
  }

  if (cnt_error_fn_element != nullptr)
  {
    std::string error_fn_name;
    status = tesseract_common::QueryStringAttribute(cnt_error_fn_element, "type", error_fn_name);
    if (status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("TrajOptPlanProfile: Error parsing ConstraintErrorFunctions plugin attribute.");

    // TODO: Implement plugin capabilities
  }
}
void TrajOptDefaultPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const CartesianWaypointPoly& cartesian_waypoint,
                                      const MoveInstructionPoly& parent_instruction,
                                      const tesseract_common::ManipulatorInfo& manip_info,
                                      const std::vector<std::string>& active_links,
                                      int index) const
{
  assert(!(manip_info.empty() && parent_instruction.getManipulatorInfo().empty()));
  tesseract_common::ManipulatorInfo mi = manip_info.getCombined(parent_instruction.getManipulatorInfo());

  if (mi.manipulator.empty())
    throw std::runtime_error("TrajOptPlanProfile, manipulator is empty!");

  if (mi.tcp_frame.empty())
    throw std::runtime_error("TrajOptPlanProfile, tcp_frame is empty!");

  if (mi.working_frame.empty())
    throw std::runtime_error("TrajOptPlanProfile, working_frame is empty!");

  Eigen::Isometry3d tcp_offset = pci.env->findTCPOffset(mi);

  /* Check if this cartesian waypoint is dynamic
   * (i.e. defined relative to a frame that will move with the kinematic chain)
   */
  bool is_active_tcp_frame = (std::find(active_links.begin(), active_links.end(), mi.tcp_frame) != active_links.end());
  bool is_static_working_frame =
      (std::find(active_links.begin(), active_links.end(), mi.working_frame) == active_links.end());

  // Override cost tolerances if the profile specifies that they should be overrided.
  Eigen::VectorXd lower_tolerance_cost = cartesian_waypoint.getLowerTolerance();
  Eigen::VectorXd upper_tolerance_cost = cartesian_waypoint.getUpperTolerance();
  if (cartesian_cost_config.use_tolerance_override)
  {
    lower_tolerance_cost = cartesian_cost_config.lower_tolerance;
    upper_tolerance_cost = cartesian_cost_config.upper_tolerance;
  }
  Eigen::VectorXd lower_tolerance_cnt = cartesian_waypoint.getLowerTolerance();
  Eigen::VectorXd upper_tolerance_cnt = cartesian_waypoint.getUpperTolerance();
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
                                                                  cartesian_waypoint.getTransform(),
                                                                  mi.tcp_frame,
                                                                  tcp_offset,
                                                                  cartesian_cost_config.coeff,
                                                                  trajopt::TermType::TT_COST,
                                                                  lower_tolerance_cost,
                                                                  upper_tolerance_cost);
      pci.cost_infos.push_back(ti);
    }
    if (cartesian_constraint_config.enabled)
    {
      trajopt::TermInfo::Ptr ti = createCartesianWaypointTermInfo(index,
                                                                  mi.working_frame,
                                                                  cartesian_waypoint.getTransform(),
                                                                  mi.tcp_frame,
                                                                  tcp_offset,
                                                                  cartesian_constraint_config.coeff,
                                                                  trajopt::TermType::TT_CNT,
                                                                  lower_tolerance_cnt,
                                                                  upper_tolerance_cnt);
      pci.cnt_infos.push_back(ti);
    }
  }
  else if (!is_static_working_frame && is_active_tcp_frame)
  {
    if (cartesian_cost_config.enabled)
    {
      trajopt::TermInfo::Ptr ti = createDynamicCartesianWaypointTermInfo(index,
                                                                         mi.working_frame,
                                                                         cartesian_waypoint.getTransform(),
                                                                         mi.tcp_frame,
                                                                         tcp_offset,
                                                                         cartesian_cost_config.coeff,
                                                                         trajopt::TermType::TT_COST,
                                                                         lower_tolerance_cost,
                                                                         upper_tolerance_cost);
      pci.cost_infos.push_back(ti);
    }
    if (cartesian_constraint_config.enabled)
    {
      trajopt::TermInfo::Ptr ti = createDynamicCartesianWaypointTermInfo(index,
                                                                         mi.working_frame,
                                                                         cartesian_waypoint.getTransform(),
                                                                         mi.tcp_frame,
                                                                         tcp_offset,
                                                                         cartesian_constraint_config.coeff,
                                                                         trajopt::TermType::TT_CNT,
                                                                         lower_tolerance_cnt,
                                                                         upper_tolerance_cnt);
      pci.cnt_infos.push_back(ti);
    }
  }
  else
  {
    throw std::runtime_error("TrajOpt, both tcp_frame and working_frame are both static!");
  }

  // Add constraints from error functions if available.
  addConstraintErrorFunctions(pci, index);
}

void TrajOptDefaultPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const JointWaypointPoly& joint_waypoint,
                                      const MoveInstructionPoly& /*parent_instruction*/,
                                      const tesseract_common::ManipulatorInfo& /*manip_info*/,
                                      const std::vector<std::string>& /*active_links*/,
                                      int index) const
{
  // Override cost tolerances if the profile specifies that they should be overrided.
  Eigen::VectorXd lower_tolerance_cost = joint_waypoint.getLowerTolerance();
  Eigen::VectorXd upper_tolerance_cost = joint_waypoint.getUpperTolerance();
  if (joint_cost_config.use_tolerance_override)
  {
    lower_tolerance_cost = joint_cost_config.lower_tolerance;
    upper_tolerance_cost = joint_cost_config.upper_tolerance;
  }
  Eigen::VectorXd lower_tolerance_cnt = joint_waypoint.getLowerTolerance();
  Eigen::VectorXd upper_tolerance_cnt = joint_waypoint.getUpperTolerance();
  if (joint_constraint_config.use_tolerance_override)
  {
    lower_tolerance_cnt = joint_constraint_config.lower_tolerance;
    upper_tolerance_cnt = joint_constraint_config.upper_tolerance;
  }
  if (joint_waypoint.isToleranced())
  {
    if (joint_cost_config.enabled)
    {
      trajopt::TermInfo::Ptr ti = createTolerancedJointWaypointTermInfo(joint_waypoint.getPosition(),
                                                                        lower_tolerance_cost,
                                                                        upper_tolerance_cost,
                                                                        index,
                                                                        joint_cost_config.coeff,
                                                                        trajopt::TermType::TT_COST);
      pci.cost_infos.push_back(ti);
    }
    if (joint_constraint_config.enabled)
    {
      trajopt::TermInfo::Ptr ti = createTolerancedJointWaypointTermInfo(joint_waypoint.getPosition(),
                                                                        lower_tolerance_cnt,
                                                                        upper_tolerance_cnt,
                                                                        index,
                                                                        joint_constraint_config.coeff,
                                                                        trajopt::TermType::TT_CNT);
      pci.cnt_infos.push_back(ti);
    }
  }
  else
  {
    if (joint_cost_config.enabled)
    {
      trajopt::TermInfo::Ptr ti = createJointWaypointTermInfo(
          joint_waypoint.getPosition(), index, joint_cost_config.coeff, trajopt::TermType::TT_COST);
      pci.cost_infos.push_back(ti);
    }
    if (joint_constraint_config.enabled)
    {
      trajopt::TermInfo::Ptr ti = createJointWaypointTermInfo(
          joint_waypoint.getPosition(), index, joint_constraint_config.coeff, trajopt::TermType::TT_CNT);
      pci.cnt_infos.push_back(ti);
    }
  }

  // Add constraints from error functions if available.
  addConstraintErrorFunctions(pci, index);
}

void TrajOptDefaultPlanProfile::addConstraintErrorFunctions(trajopt::ProblemConstructionInfo& pci, int index) const
{
  for (const auto& c : constraint_error_functions)
  {
    trajopt::TermInfo::Ptr ti =
        createUserDefinedTermInfo(index, index, std::get<0>(c), std::get<1>(c), trajopt::TermType::TT_CNT);

    // Update the term info with the (possibly) new start and end state indices for which to apply this cost
    std::shared_ptr<trajopt::UserDefinedTermInfo> ef = std::static_pointer_cast<trajopt::UserDefinedTermInfo>(ti);
    ef->constraint_type = std::get<2>(c);
    ef->coeff = std::get<3>(c);

    pci.cnt_infos.push_back(ef);
  }
}

tinyxml2::XMLElement* TrajOptDefaultPlanProfile::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* xml_planner = doc.NewElement("Planner");
  xml_planner->SetAttribute("type", std::to_string(1).c_str());

  tinyxml2::XMLElement* xml_trajopt = doc.NewElement("TrajOptDefaultPlanProfile");

  tinyxml2::XMLElement* xml_cart_cost_parent = doc.NewElement("CartesianCostConfig");
  tinyxml2::XMLElement* xml_cart_cost = cartesian_cost_config.toXML(doc);
  xml_cart_cost_parent->InsertEndChild(xml_cart_cost);
  xml_trajopt->InsertEndChild(xml_cart_cost_parent);

  tinyxml2::XMLElement* xml_cart_cnt_parent = doc.NewElement("CartesianConstraintConfig");
  tinyxml2::XMLElement* xml_cart_cnt = cartesian_constraint_config.toXML(doc);
  xml_cart_cnt_parent->InsertEndChild(xml_cart_cnt);
  xml_trajopt->InsertEndChild(xml_cart_cnt_parent);

  tinyxml2::XMLElement* xml_joint_cost_parent = doc.NewElement("JointCostConfig");
  tinyxml2::XMLElement* xml_joint_cost = joint_cost_config.toXML(doc);
  xml_joint_cost_parent->InsertEndChild(xml_joint_cost);
  xml_trajopt->InsertEndChild(xml_joint_cost_parent);

  tinyxml2::XMLElement* xml_joint_cnt_parent = doc.NewElement("JointConstraintConfig");
  tinyxml2::XMLElement* xml_joint_cnt = joint_constraint_config.toXML(doc);
  xml_joint_cnt_parent->InsertEndChild(xml_joint_cnt);
  xml_trajopt->InsertEndChild(xml_joint_cnt_parent);

  xml_planner->InsertEndChild(xml_trajopt);

  return xml_planner;
}

template <class Archive>
void TrajOptDefaultPlanProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TrajOptPlanProfile);
  ar& BOOST_SERIALIZATION_NVP(cartesian_cost_config);
  ar& BOOST_SERIALIZATION_NVP(cartesian_constraint_config);
  ar& BOOST_SERIALIZATION_NVP(joint_cost_config);
  ar& BOOST_SERIALIZATION_NVP(joint_constraint_config);
  // ar& BOOST_SERIALIZATION_NVP(constraint_error_functions); /** @todo FIX */
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptDefaultPlanProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptDefaultPlanProfile)
