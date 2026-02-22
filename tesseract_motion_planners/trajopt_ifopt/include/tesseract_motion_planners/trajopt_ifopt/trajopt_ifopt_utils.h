/**
 * @file trajopt_utils.h
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_UTILS_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <memory>
#include <Eigen/Geometry>
#include <trajopt_common/fwd.h>
#include <trajopt_ifopt/fwd.h>
#include <trajopt_sqp/fwd.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_collision/core/fwd.h>
#include <tesseract_environment/fwd.h>
#include <tesseract_command_language/fwd.h>

namespace OsqpEigen
{
class Settings;
}

namespace tesseract::motion_planners
{
void copyOSQPEigenSettings(OsqpEigen::Settings& lhs, const OsqpEigen::Settings& rhs);

std::shared_ptr<trajopt_ifopt::ConstraintSet>
createJointPositionConstraint(const tesseract::command_language::JointWaypointPoly& joint_waypoint,
                              const std::shared_ptr<const trajopt_ifopt::Var>& var,
                              const Eigen::VectorXd& coeffs);

std::vector<std::shared_ptr<trajopt_ifopt::ConstraintSet>>
createCollisionConstraints(const std::vector<std::shared_ptr<const trajopt_ifopt::Var>>& vars,
                           const std::shared_ptr<const tesseract::environment::Environment>& env,
                           const tesseract::common::ManipulatorInfo& manip_info,
                           const trajopt_common::TrajOptCollisionConfig& config,
                           const std::vector<int>& fixed_indices,
                           bool fixed_sparsity);

std::shared_ptr<trajopt_ifopt::ConstraintSet>
createJointVelocityConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                              const std::vector<std::shared_ptr<const trajopt_ifopt::Var>>& vars,
                              const Eigen::VectorXd& coeffs);

std::shared_ptr<trajopt_ifopt::ConstraintSet>
createJointAccelerationConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                                  const std::vector<std::shared_ptr<const trajopt_ifopt::Var>>& vars,
                                  const Eigen::VectorXd& coeffs);

std::shared_ptr<trajopt_ifopt::ConstraintSet>
createJointJerkConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                          const std::vector<std::shared_ptr<const trajopt_ifopt::Var>>& vars,
                          const Eigen::VectorXd& coeffs);

}  // namespace tesseract::motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_UTILS_H
