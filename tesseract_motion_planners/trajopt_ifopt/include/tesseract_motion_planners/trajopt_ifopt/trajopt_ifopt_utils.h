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
#include <ifopt/constraint_set.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_ifopt/trajopt_ifopt.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/types.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>

namespace tesseract_planning
{
ifopt::ConstraintSet::Ptr
createCartesianPositionConstraint(const trajopt_ifopt::JointPosition::ConstPtr& var,
                                  const tesseract_kinematics::JointGroup::ConstPtr& manip,
                                  const std::string& source_frame,
                                  const std::string& target_frame,
                                  const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
                                  const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
                                  const Eigen::Ref<const Eigen::VectorXd>& coeffs = Eigen::VectorXd::Ones(6));

bool addCartesianPositionConstraint(trajopt_sqp::QPProblem& nlp,
                                    const trajopt_ifopt::JointPosition::ConstPtr& var,
                                    const tesseract_kinematics::JointGroup::ConstPtr& manip,
                                    const std::string& source_frame,
                                    const std::string& target_frame,
                                    const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
                                    const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
                                    const Eigen::Ref<const Eigen::VectorXd>& coeffs = Eigen::VectorXd::Ones(6));

bool addCartesianPositionSquaredCost(trajopt_sqp::QPProblem& nlp,
                                     const trajopt_ifopt::JointPosition::ConstPtr& var,
                                     const tesseract_kinematics::JointGroup::ConstPtr& manip,
                                     const std::string& source_frame,
                                     const std::string& target_frame,
                                     const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
                                     const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
                                     const Eigen::Ref<const Eigen::VectorXd>& coeffs = Eigen::VectorXd::Ones(6));

bool addCartesianPositionAbsoluteCost(trajopt_sqp::QPProblem& nlp,
                                      const trajopt_ifopt::JointPosition::ConstPtr& var,
                                      const tesseract_kinematics::JointGroup::ConstPtr& manip,
                                      const std::string& source_frame,
                                      const std::string& target_frame,
                                      const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
                                      const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
                                      const Eigen::Ref<const Eigen::VectorXd>& coeffs = Eigen::VectorXd::Ones(6));

ifopt::ConstraintSet::Ptr createJointPositionConstraint(const JointWaypoint& joint_waypoint,
                                                        const trajopt_ifopt::JointPosition::ConstPtr& var,
                                                        const Eigen::VectorXd& /*coeffs*/);

std::vector<ifopt::ConstraintSet::Ptr>
createCollisionConstraints(const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& vars,
                           const tesseract_environment::Environment::ConstPtr& env,
                           const ManipulatorInfo& manip_info,
                           const trajopt_ifopt::TrajOptCollisionConfig::ConstPtr& config,
                           const std::vector<int>& fixed_indices);

bool addCollisionConstraint(trajopt_sqp::QPProblem& nlp,
                            const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& vars,
                            const tesseract_environment::Environment::ConstPtr& env,
                            const ManipulatorInfo& manip_info,
                            const trajopt_ifopt::TrajOptCollisionConfig::ConstPtr& config,
                            const std::vector<int>& fixed_indices);

bool addCollisionCost(trajopt_sqp::QPProblem& nlp,
                      const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& vars,
                      const tesseract_environment::Environment::ConstPtr& env,
                      const ManipulatorInfo& manip_info,
                      const trajopt_ifopt::TrajOptCollisionConfig::ConstPtr& config,
                      const std::vector<int>& fixed_indices);

ifopt::ConstraintSet::Ptr createJointVelocityConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                                                        const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& vars,
                                                        const Eigen::VectorXd& coeffs);

bool addJointVelocityConstraint(trajopt_sqp::QPProblem& nlp,
                                const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& vars,
                                const Eigen::Ref<const Eigen::VectorXd>& coeff);

bool addJointVelocitySquaredCost(trajopt_sqp::QPProblem& nlp,
                                 const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& vars,
                                 const Eigen::Ref<const Eigen::VectorXd>& coeff);

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_UTILS_H
