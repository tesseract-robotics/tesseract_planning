/**
 * @file descartes_robot_sampler.h
 * @brief Tesseract Descartes Kinematics Sampler
 *
 * @author Levi Armstrong
 * @date June 25, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/core/waypoint_sampler.h>
#include <Eigen/Dense>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/types.h>

namespace tesseract_planning
{
template <typename FloatType>
class DescartesRobotSampler : public descartes_light::WaypointSampler<FloatType>
{
public:
  /**
   * @brief This is a descartes sampler for a robot.
   * @param target_pose The target pose in robot base link coordinates
   * @param target_pose_sampler The target pose sampler function to be used
   * @param manip The manipulator kinematic group object
   * @param collision The collision interface
   * @param robot_tcp The robot tcp to be used.
   * @param allow_collision If true and no valid solution was found it will return the best of the worst
   * @param is_valid This is a user defined function to filter out solution
   */
  DescartesRobotSampler(std::string target_working_frame,
                        const Eigen::Isometry3d& target_pose,
                        PoseSamplerFn target_pose_sampler,
                        tesseract_kinematics::KinematicGroup::ConstPtr manip,
                        DescartesCollision::Ptr collision,
                        std::string tcp_frame,
                        const Eigen::Isometry3d& tcp_offset,
                        bool allow_collision,
                        DescartesVertexEvaluator::Ptr is_valid,
                        bool use_redundant_joint_solutions);

  std::vector<descartes_light::StateSample<FloatType>> sample() const override;

private:
  /** @brief The target pose working frame */
  std::string target_working_frame_;

  /** @brief The target pose to sample */
  Eigen::Isometry3d target_pose_;

  /** @brief The target pose to sample */
  PoseSamplerFn target_pose_sampler_;

  /** @brief The manipulator kinematic group */
  tesseract_kinematics::KinematicGroup::ConstPtr manip_;

  /** @brief The collision interface */
  DescartesCollision::Ptr collision_;

  /** @brief The robot tool center point frame */
  std::string tcp_frame_;

  /** @brief The robot tool center point */
  Eigen::Isometry3d tcp_offset_;

  /** @brief Flag indicating whether states found to be in collision should be returned */
  bool allow_collision_;

  /** @brief The number of joints in the robot */
  int dof_;

  /** @brief The seed for inverse kinematics which is zeros */
  Eigen::VectorXd ik_seed_;

  /** @brief This is the vertex evaluator to filter out solution */
  DescartesVertexEvaluator::Ptr is_valid_;

  /** @brief Should redundant solutions be used */
  bool use_redundant_joint_solutions_{ false };
};

using DescartesRobotSamplerF = DescartesRobotSampler<float>;
using DescartesRobotSamplerD = DescartesRobotSampler<double>;

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_H
