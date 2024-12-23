/**
 * @file trajopt_default_plan_profile.h
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

#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_PLAN_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt_sco/modeling_utils.hpp>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>

namespace tesseract_planning
{
class TrajOptDefaultPlanProfile : public TrajOptPlanProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptDefaultPlanProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptDefaultPlanProfile>;

  TrajOptDefaultPlanProfile() = default;
  TrajOptDefaultPlanProfile(const tinyxml2::XMLElement& xml_element);

  CartesianWaypointConfig cartesian_cost_config;
  CartesianWaypointConfig cartesian_constraint_config;
  JointWaypointConfig joint_cost_config;
  JointWaypointConfig joint_constraint_config;

  /** @brief Error function that is set as a constraint for each timestep.
   *
   * This is a vector of std::tuple<Error Function, Error Function Jacobian, Constraint Type, Coeff>, the error
   * function, constraint type, and coeff is required, but the jacobian is optional (nullptr).
   *
   * Error Function:
   *   arg: VectorXd will be all of the joint values for one timestep.
   *   return: VectorXd of violations for each joint. Anything != 0 will be a violation
   *
   * Error Function Jacobian:
   *   arg: VectorXd will be all of the joint values for one timestep.
   *   return: Eigen::MatrixXd that represents the change in the error function with respect to joint values
   *
   * Error Constraint Type
   *
   * Coefficients/Weights
   *
   */
  std::vector<std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd>>
      constraint_error_functions;

  void apply(trajopt::ProblemConstructionInfo& pci,
             const CartesianWaypointPoly& cartesian_waypoint,
             const MoveInstructionPoly& parent_instruction,
             const tesseract_common::ManipulatorInfo& manip_info,
             const std::vector<std::string>& active_links,
             int index) const override;

  void apply(trajopt::ProblemConstructionInfo& pci,
             const JointWaypointPoly& joint_waypoint,
             const MoveInstructionPoly& parent_instruction,
             const tesseract_common::ManipulatorInfo& manip_info,
             const std::vector<std::string>& active_links,
             int index) const override;

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const override;

protected:
  void addConstraintErrorFunctions(trajopt::ProblemConstructionInfo& pci, int index) const;

  void addAvoidSingularity(trajopt::ProblemConstructionInfo& pci, const std::vector<int>& fixed_steps) const;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptDefaultPlanProfile)

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_PLAN_PROFILE_H
