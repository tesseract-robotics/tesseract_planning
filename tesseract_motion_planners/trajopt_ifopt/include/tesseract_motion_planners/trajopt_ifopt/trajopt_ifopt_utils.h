/**
 * @file trajopt_utils.h
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

namespace tesseract_planning
{
void copyOSQPEigenSettings(OsqpEigen::Settings& lhs, const OsqpEigen::Settings& rhs);

bool addCartesianPositionConstraint(trajopt_sqp::QPProblem& nlp,
                                    const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                    const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip,
                                    const std::string& source_frame,
                                    const std::string& target_frame,
                                    const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
                                    const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
                                    const Eigen::Ref<const Eigen::VectorXd>& coeffs = Eigen::VectorXd::Ones(6));

bool addCartesianPositionSquaredCost(trajopt_sqp::QPProblem& nlp,
                                     const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                     const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip,
                                     const std::string& source_frame,
                                     const std::string& target_frame,
                                     const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
                                     const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
                                     const Eigen::Ref<const Eigen::VectorXd>& coeffs = Eigen::VectorXd::Ones(6));

bool addCartesianPositionAbsoluteCost(trajopt_sqp::QPProblem& nlp,
                                      const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                      const std::shared_ptr<const tesseract_kinematics::JointGroup>& manip,
                                      const std::string& source_frame,
                                      const std::string& target_frame,
                                      const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
                                      const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
                                      const Eigen::Ref<const Eigen::VectorXd>& coeffs = Eigen::VectorXd::Ones(6));

bool addJointPositionConstraint(trajopt_sqp::QPProblem& nlp,
                                const JointWaypointPoly& joint_waypoint,
                                const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                const Eigen::VectorXd& coeffs);

bool addJointPositionSquaredCost(trajopt_sqp::QPProblem& nlp,
                                 const JointWaypointPoly& joint_waypoint,
                                 const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                 const Eigen::VectorXd& coeffs);

bool addJointPositionAbsoluteCost(trajopt_sqp::QPProblem& nlp,
                                  const JointWaypointPoly& joint_waypoint,
                                  const std::shared_ptr<const trajopt_ifopt::JointPosition>& var,
                                  const Eigen::VectorXd& coeffs);

bool addCollisionConstraint(trajopt_sqp::QPProblem& nlp,
                            const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                            const std::shared_ptr<const tesseract_environment::Environment>& env,
                            const tesseract_common::ManipulatorInfo& manip_info,
                            const std::shared_ptr<const trajopt_common::TrajOptCollisionConfig>& config,
                            const std::vector<int>& fixed_indices);

bool addCollisionCost(trajopt_sqp::QPProblem& nlp,
                      const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                      const std::shared_ptr<const tesseract_environment::Environment>& env,
                      const tesseract_common::ManipulatorInfo& manip_info,
                      const std::shared_ptr<const trajopt_common::TrajOptCollisionConfig>& config,
                      const std::vector<int>& fixed_indices);

bool addJointVelocityConstraint(trajopt_sqp::QPProblem& nlp,
                                const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                                const Eigen::Ref<const Eigen::VectorXd>& coeff);

bool addJointVelocitySquaredCost(trajopt_sqp::QPProblem& nlp,
                                 const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                                 const Eigen::Ref<const Eigen::VectorXd>& coeff);

bool addJointAccelerationConstraint(trajopt_sqp::QPProblem& nlp,
                                    const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                                    const Eigen::Ref<const Eigen::VectorXd>& coeff);

bool addJointAccelerationSquaredCost(trajopt_sqp::QPProblem& nlp,
                                     const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                                     const Eigen::Ref<const Eigen::VectorXd>& coeff);

bool addJointJerkConstraint(trajopt_sqp::QPProblem& nlp,
                            const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                            const Eigen::Ref<const Eigen::VectorXd>& coeff);

bool addJointJerkSquaredCost(trajopt_sqp::QPProblem& nlp,
                             const std::vector<std::shared_ptr<const trajopt_ifopt::JointPosition>>& vars,
                             const Eigen::Ref<const Eigen::VectorXd>& coeff);

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_UTILS_H
