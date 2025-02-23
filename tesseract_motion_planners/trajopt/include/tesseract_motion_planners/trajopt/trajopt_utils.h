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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_UTILS_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
#include <Eigen/Geometry>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/fwd.h>
#include <tesseract_collision/core/types.h>

namespace tesseract_planning
{
std::shared_ptr<const tesseract_kinematics::JointGroup>
createKinematicGroup(const tesseract_common::ManipulatorInfo& manip_info,
                     const tesseract_environment::Environment& env);

std::shared_ptr<trajopt::TermInfo>
createCartesianWaypointTermInfo(int index,
                                const std::string& working_frame,
                                const Eigen::Isometry3d& c_wp,
                                const std::string& tcp_frame,
                                const Eigen::Isometry3d& tcp_offset,
                                const Eigen::VectorXd& coeffs,
                                trajopt::TermType type,
                                const Eigen::VectorXd& lower_tolerance = Eigen::VectorXd(),
                                const Eigen::VectorXd& upper_tolerance = Eigen::VectorXd());

std::shared_ptr<trajopt::TermInfo>
createDynamicCartesianWaypointTermInfo(int index,
                                       const std::string& working_frame,
                                       const Eigen::Isometry3d& c_wp,
                                       const std::string& tcp_frame,
                                       const Eigen::Isometry3d& tcp_offset,
                                       const Eigen::VectorXd& coeffs,
                                       trajopt::TermType type,
                                       const Eigen::VectorXd& lower_tolerance = Eigen::VectorXd(),
                                       const Eigen::VectorXd& upper_tolerance = Eigen::VectorXd());

std::shared_ptr<trajopt::TermInfo> createNearJointStateTermInfo(const Eigen::VectorXd& target,
                                                                const std::vector<std::string>& joint_names,
                                                                int index,
                                                                const Eigen::VectorXd& coeffs,
                                                                trajopt::TermType type);

std::shared_ptr<trajopt::TermInfo> createJointWaypointTermInfo(const Eigen::VectorXd& j_wp,
                                                               int index,
                                                               const Eigen::VectorXd& coeffs,
                                                               trajopt::TermType type);

std::shared_ptr<trajopt::TermInfo> createTolerancedJointWaypointTermInfo(const Eigen::VectorXd& j_wp,
                                                                         const Eigen::VectorXd& lower_tol,
                                                                         const Eigen::VectorXd& upper_tol,
                                                                         int index,
                                                                         const Eigen::VectorXd& coeffs,
                                                                         trajopt::TermType type);

std::shared_ptr<trajopt::TermInfo> createCollisionTermInfo(
    int start_index,
    int end_index,
    double collision_safety_margin,
    double collision_safety_margin_buffer,
    tesseract_collision::CollisionEvaluatorType evaluator_type,
    bool use_weighted_sum = false,
    double coeff = 20.0,
    tesseract_collision::ContactTestType contact_test_type = tesseract_collision::ContactTestType::ALL,
    double longest_valid_segment_length = 0.5,
    trajopt::TermType type = trajopt::TermType::TT_COST);

std::shared_ptr<trajopt::TermInfo> createSmoothVelocityTermInfo(int start_index,
                                                                int end_index,
                                                                int n_joints,
                                                                double coeff = 5.0,
                                                                trajopt::TermType type = trajopt::TermType::TT_COST);

std::shared_ptr<trajopt::TermInfo> createSmoothVelocityTermInfo(int start_index,
                                                                int end_index,
                                                                const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                                trajopt::TermType type = trajopt::TermType::TT_COST);

std::shared_ptr<trajopt::TermInfo>
createSmoothAccelerationTermInfo(int start_index,
                                 int end_index,
                                 int n_joints,
                                 double coeff = 1.0,
                                 trajopt::TermType type = trajopt::TermType::TT_COST);

std::shared_ptr<trajopt::TermInfo>
createSmoothAccelerationTermInfo(int start_index,
                                 int end_index,
                                 const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                 trajopt::TermType type = trajopt::TermType::TT_COST);

std::shared_ptr<trajopt::TermInfo> createSmoothJerkTermInfo(int start_index,
                                                            int end_index,
                                                            int n_joints,
                                                            double coeff = 1.0,
                                                            trajopt::TermType type = trajopt::TermType::TT_COST);

std::shared_ptr<trajopt::TermInfo> createSmoothJerkTermInfo(int start_index,
                                                            int end_index,
                                                            const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                            trajopt::TermType type = trajopt::TermType::TT_COST);

std::shared_ptr<trajopt::TermInfo> createUserDefinedTermInfo(int start_index,
                                                             int end_index,
                                                             sco::VectorOfVector::func error_function,
                                                             sco::MatrixOfVector::func jacobian_function,
                                                             trajopt::TermType type);

std::shared_ptr<trajopt::TermInfo> createAvoidSingularityTermInfo(int start_index,
                                                                  int end_index,
                                                                  const std::string& link,
                                                                  double coeff = 5.0,
                                                                  trajopt::TermType type = trajopt::TermType::TT_COST);
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_UTILS_H
