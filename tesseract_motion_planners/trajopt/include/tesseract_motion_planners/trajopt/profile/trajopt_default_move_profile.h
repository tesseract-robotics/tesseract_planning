/**
 * @file trajopt_default_move_profile.h
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

#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_MOVE_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_MOVE_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt_sco/modeling_utils.hpp>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>

namespace YAML
{
class Node;
}

namespace tesseract_planning
{
class TrajOptDefaultMoveProfile : public TrajOptMoveProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptDefaultMoveProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptDefaultMoveProfile>;

  TrajOptDefaultMoveProfile();
  TrajOptDefaultMoveProfile(const YAML::Node& config, const tesseract_common::ProfilePluginFactory& plugin_factory);

  TrajOptCartesianWaypointConfig cartesian_cost_config;
  TrajOptCartesianWaypointConfig cartesian_constraint_config;
  TrajOptJointWaypointConfig joint_cost_config;
  TrajOptJointWaypointConfig joint_constraint_config;

  TrajOptWaypointInfo create(const MoveInstructionPoly& move_instruction,
                             const tesseract_common::ManipulatorInfo& composite_manip_info,
                             const std::shared_ptr<const tesseract_environment::Environment>& env,
                             const std::vector<std::string>& active_links,
                             int index) const override;

  bool operator==(const TrajOptDefaultMoveProfile& rhs) const;
  bool operator!=(const TrajOptDefaultMoveProfile& rhs) const;

protected:
  /**
   * @brief Error function that is set as a constraint for each timestep.
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
  static std::shared_ptr<trajopt::TermInfo>
  createConstraintFromErrorFunction(sco::VectorOfVector::func error_function,
                                    sco::MatrixOfVector::func jacobian_function,
                                    sco::ConstraintType type,
                                    const Eigen::VectorXd& coeff,
                                    int index);
};
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_MOVE_PROFILE_H
