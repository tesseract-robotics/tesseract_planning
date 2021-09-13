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
#include <console_bridge/console.h>
#include <Eigen/Geometry>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_robot_sampler.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_planning
{
template <typename FloatType>
DescartesRobotSampler<FloatType>::DescartesRobotSampler(const Eigen::Isometry3d& target_pose,
                                                        PoseSamplerFn target_pose_sampler,
                                                        tesseract_kinematics::InverseKinematics::ConstPtr ik,
                                                        DescartesCollision::Ptr collision,
                                                        const Eigen::Isometry3d& tcp,
                                                        bool allow_collision,
                                                        DescartesVertexEvaluator::Ptr is_valid,
                                                        bool use_redundant_joint_solutions)
  : target_pose_(target_pose)
  , target_pose_sampler_(std::move(target_pose_sampler))
  , ik_(std::move(ik))
  , collision_(std::move(collision))
  , tcp_(tcp)
  , allow_collision_(allow_collision)
  , dof_(static_cast<int>(ik_->numJoints()))
  , ik_seed_(Eigen::VectorXd::Zero(dof_))
  , is_valid_(std::move(is_valid))
  , use_redundant_joint_solutions_(use_redundant_joint_solutions)
{
  if (!allow_collision_ && !collision_)
    throw std::runtime_error("Collision checker must not be a nullptr if collisions are not allowed during planning");
}

template <typename FloatType>
std::vector<descartes_light::StateSample<FloatType>> DescartesRobotSampler<FloatType>::sample() const
{
  // Generate all possible Cartesian poses
  tesseract_common::VectorIsometry3d target_poses = target_pose_sampler_(target_pose_);

  // Generate the IK solutions for those poses
  std::vector<descartes_light::StateSample<FloatType>> samples;
  for (const auto& pose : target_poses)
  {
    // Get the transformation to the kinematic tip link
    Eigen::Isometry3d target_pose = pose * tcp_.inverse();

    // Solve IK
    tesseract_kinematics::IKSolutions ik_solutions = ik_->calcInvKin(target_pose, ik_seed_);

    if (ik_solutions.empty())
      continue;

    // Check each individual joint solution
    for (const auto& sol : ik_solutions)
    {
      if ((is_valid_ != nullptr) && !(*is_valid_)(sol))
        continue;

      auto state = std::make_shared<descartes_light::State<FloatType>>(sol.cast<FloatType>());
      if (allow_collision_ && collision_ == nullptr)
      {
        samples.push_back(descartes_light::StateSample<FloatType>{ state, static_cast<FloatType>(0.0) });
      }
      else
      {
        const FloatType cost = static_cast<FloatType>(collision_->distance(sol));
        samples.push_back(descartes_light::StateSample<FloatType>{ state, cost });
      }
    }
  }

  if (samples.empty())
    return samples;

  // Sort state samples in descending order by distance from nearest collision (i.e. state cost)
  std::sort(samples.begin(), samples.end(), [](const auto& a, const auto& b) { return a.cost > b.cost; });

  // Prune collision solutions
  if (!allow_collision_)
  {
    auto it = std::find_if(samples.begin(), samples.end(), [](const auto& sample) { return sample.cost < 0.0; });
    samples.erase(it, samples.end());
  }

  // Convert the distance into a cost and normalize
  if (samples.size() > 1)
  {
    const FloatType max_dist = samples.front().cost;
    const FloatType min_dist = samples.back().cost;
    const FloatType range = max_dist - min_dist;
    if (range > std::numeric_limits<FloatType>::epsilon())
    {
      std::for_each(samples.begin(), samples.end(), [&min_dist, &range](auto& sample) {
        sample.cost = static_cast<FloatType>(1.0) - (sample.cost - min_dist) / range;
      });
    }
    else
    {
      std::for_each(samples.begin(), samples.end(), [](auto& sample) { sample.cost = 0.0; });
    }
  }

  // Generate the redundant solutions
  if (use_redundant_joint_solutions_)
  {
    const Eigen::MatrixX2d& limits = ik_->getLimits().joint_limits;
    std::vector<Eigen::Index> redundancy_capable_joints = ik_->getRedundancyCapableJointIndices();
    std::vector<descartes_light::StateSample<FloatType>> redundant_samples;
    for (const descartes_light::StateSample<FloatType>& sample : samples)
    {
      const auto redundant_solutions =
          tesseract_kinematics::getRedundantSolutions<FloatType>(*(sample.state), limits, redundancy_capable_joints);

      // Add the redundant samples with the same cost as the nominal sample
      std::transform(redundant_solutions.begin(),
                     redundant_solutions.end(),
                     std::back_inserter(redundant_samples),
                     [&sample](const auto& sol) {
                       auto state = std::make_shared<descartes_light::State<FloatType>>(sol.template cast<FloatType>());
                       return descartes_light::StateSample<FloatType>{ state, sample.cost };
                     });
    }

    // Combine the nominal samples with the redundant samples
    samples.insert(samples.end(), redundant_samples.begin(), redundant_samples.end());
  }

  return samples;
}

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_HPP
