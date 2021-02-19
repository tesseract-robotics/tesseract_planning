/**
 * @file descartes_robot_sampler.hpp
 * @brief Tesseract Descartes Kinematics Sampler Implementation
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_HPP
#define TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/utils.h>
#include <console_bridge/console.h>
#include <Eigen/Geometry>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_robot_sampler.h>

namespace tesseract_planning
{
template <typename FloatType>
DescartesRobotSampler<FloatType>::DescartesRobotSampler(
    const Eigen::Isometry3d& target_pose,
    PoseSamplerFn target_pose_sampler,
    tesseract_kinematics::InverseKinematics::ConstPtr robot_kinematics,
    DescartesCollision::Ptr collision,
    const Eigen::Isometry3d& tcp,
    bool allow_collision,
    DescartesVertexEvaluator::Ptr is_valid)
  : target_pose_(target_pose)
  , target_pose_sampler_(std::move(target_pose_sampler))
  , robot_kinematics_(std::move(robot_kinematics))
  , collision_(std::move(collision))
  , tcp_(tcp)
  , allow_collision_(allow_collision)
  , dof_(static_cast<int>(robot_kinematics_->numJoints()))
  , ik_seed_(Eigen::VectorXd::Zero(dof_))
  , is_valid_(std::move(is_valid))
{
}

template <typename FloatType>
std::vector<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> DescartesRobotSampler<FloatType>::sample() const
{
  double distance = std::numeric_limits<double>::min();
  tesseract_common::VectorIsometry3d target_poses = target_pose_sampler_(target_pose_);
  std::vector<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> solution_set;
  for (const auto& sp : target_poses)
  {
    // Tool pose in rail coordinate system
    Eigen::Isometry3d target_pose = sp * tcp_.inverse();
    ikAt(solution_set, target_pose, false, distance);
  }

  if (solution_set.empty() && allow_collision_)
    getBestSolution(solution_set, target_poses);

  return solution_set;
}

template <typename FloatType>
bool DescartesRobotSampler<FloatType>::isCollisionFree(const Eigen::VectorXd& vertex) const
{
  if (collision_ == nullptr)
    return true;

  return collision_->validate(vertex);
}

template <typename FloatType>
bool DescartesRobotSampler<FloatType>::ikAt(std::vector<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& solution_set,
                                            const Eigen::Isometry3d& target_pose,
                                            bool get_best_solution,
                                            double& distance) const
{
  tesseract_kinematics::IKSolutions robot_solution_set = robot_kinematics_->calcInvKin(target_pose, ik_seed_);
  if (robot_solution_set.empty())
    return false;

  for (const auto& sol : robot_solution_set)
  {
    if ((is_valid_ != nullptr) && !(*is_valid_)(sol))
      continue;

    if (!get_best_solution)
    {
      if (isCollisionFree(sol))
        solution_set.push_back(sol.cast<FloatType>());
    }
    else
    {
      double cur_distance = collision_->distance(sol);
      if (cur_distance > distance)
      {
        distance = cur_distance;
        solution_set.push_back(sol.cast<FloatType>());
      }
    }
  }

  return !solution_set.empty();
}

template <typename FloatType>
bool DescartesRobotSampler<FloatType>::getBestSolution(
    std::vector<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& solution_set,
    const tesseract_common::VectorIsometry3d& target_poses) const
{
  double distance = std::numeric_limits<double>::min();
  for (const auto& sp : target_poses)
  {
    // Tool pose in rail coordinate system
    Eigen::Isometry3d target_pose = sp * tcp_.inverse();
    ikAt(solution_set, target_pose, false, distance);
  }

  return !solution_set.empty();
}

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_HPP
