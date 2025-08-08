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
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/descartes_vertex_evaluator.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_planning
{
template <typename FloatType>
DescartesRobotSampler<FloatType>::DescartesRobotSampler(
    std::string target_working_frame,
    const Eigen::Isometry3d& target_pose,  // NOLINT(modernize-pass-by-value)
    PoseSamplerFn target_pose_sampler,
    std::shared_ptr<const tesseract_kinematics::KinematicGroup> manip,
    std::shared_ptr<DescartesCollision> collision,
    std::string tcp_frame,
    const Eigen::Isometry3d& tcp_offset,  // NOLINT(modernize-pass-by-value)
    bool allow_collision,
    std::shared_ptr<DescartesVertexEvaluator> is_valid,
    bool use_redundant_joint_solutions)
  : target_working_frame_(std::move(target_working_frame))
  , target_pose_(target_pose)
  , target_pose_sampler_(std::move(target_pose_sampler))
  , manip_(std::move(manip))
  , collision_(std::move(collision))
  , tcp_frame_(std::move(tcp_frame))
  , tcp_offset_(tcp_offset)
  , allow_collision_(allow_collision)
  , dof_(static_cast<int>(manip_->numJoints()))
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

  bool found_ik_sol = false;
  std::stringstream error_string_stream;

  // Solve IK (TODO Should tcp_offset be stored in KinGroupIKInput?)
  tesseract_kinematics::KinGroupIKInputs ik_inputs;
  ik_inputs.emplace_back(Eigen::Isometry3d::Identity(), target_working_frame_, tcp_frame_);

  // Generate the IK solutions for those poses
  std::vector<descartes_light::StateSample<FloatType>> samples;
  for (std::size_t i = 0; i < target_poses.size(); i++)
  {
    const auto& pose = target_poses[i];

    // Get the transformation to the kinematic tip link
    ik_inputs.front().pose = pose * tcp_offset_.inverse();

    thread_local tesseract_kinematics::IKSolutions ik_solutions;
    ik_solutions.clear();

    manip_->calcInvKin(ik_solutions, ik_inputs, ik_seed_);
    if (ik_solutions.empty())
      continue;

    tesseract_collision::ContactTrajectoryResults traj_contacts(manip_->getJointNames(),
                                                                static_cast<int>(ik_solutions.size()));

    found_ik_sol = true;

    // These get cleared in the validate and distance calls
    thread_local tesseract_collision::ContactResultMap coll_results;
    thread_local tesseract_common::TransformMap transforms;

    // Check each individual joint solution
    for (std::size_t j = 0; j < ik_solutions.size(); j++)
    {
      const auto& sol = ik_solutions[j];

      if ((is_valid_ != nullptr) && !(*is_valid_)(sol))
        continue;

      auto state = std::make_shared<descartes_light::State<FloatType>>(sol.cast<FloatType>());
      if (allow_collision_ && collision_ == nullptr)
      {
        samples.push_back(descartes_light::StateSample<FloatType>{ state, static_cast<FloatType>(0.0) });
      }
      else if (!allow_collision_)
      {
        if (collision_->validate(coll_results, transforms, sol))
        {
          samples.push_back(descartes_light::StateSample<FloatType>{ state, 0.0 });
        }
        else if (console_bridge::getLogLevel() == console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG)
        {
          tesseract_collision::ContactTrajectoryStepResults step_contacts(static_cast<int>(j), sol, sol, 1);
          tesseract_collision::ContactTrajectorySubstepResults substep_contacts(1, sol);
          substep_contacts.contacts = coll_results;
          step_contacts.substeps[0] = substep_contacts;
          traj_contacts.steps[j] = step_contacts;
        }
      }
      else
      {
        const FloatType cost = static_cast<FloatType>(collision_->distance(coll_results, transforms, sol));
        samples.push_back(descartes_light::StateSample<FloatType>{ state, cost });
      }
    }

    if (console_bridge::getLogLevel() == console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG)
    {
      error_string_stream << "For sample " << i << " " << ik_solutions.size()
                          << " IK solutions were found, with a collision summary of:\n";
      error_string_stream << traj_contacts.collisionFrequencyPerLink().str();
    }
  }

  if (samples.empty())
  {
    std::stringstream ss;
    ss << "Descartes vertex failure: ";
    if (!found_ik_sol)
      ss << "No IK solutions were found. ";
    else
      ss << "All IK solutions found were in collision or invalid. ";
    ss << target_poses.size() << " samples tried.\n";
    if (found_ik_sol)
      ss << error_string_stream.str();
    error_string_ = ss.str();
    return samples;
  }

  if (allow_collision_)
  {
    // Sort state samples in descending order by distance from nearest collision (i.e. state cost)
    std::sort(samples.begin(), samples.end(), [](const auto& a, const auto& b) { return a.cost > b.cost; });

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
  }

  // Generate the redundant solutions
  if (use_redundant_joint_solutions_)
  {
    const Eigen::MatrixX2d& limits = manip_->getLimits().joint_limits;
    std::vector<Eigen::Index> redundancy_capable_joints = manip_->getRedundancyCapableJointIndices();
    const std::size_t ns = samples.size();
    for (std::size_t i = 0; i < ns; ++i)
    {
      thread_local std::vector<tesseract_kinematics::VectorX<FloatType>> redundant_solutions;
      redundant_solutions.clear();

      const auto& sample = samples[i];
      const FloatType cost = sample.cost;

      tesseract_kinematics::getRedundantSolutions<FloatType>(
          redundant_solutions, sample.state->values, limits, redundancy_capable_joints);

      // Add the redundant samples with the same cost as the nominal sample
      std::transform(
          redundant_solutions.begin(), redundant_solutions.end(), std::back_inserter(samples), [&](const auto& sol) {
            auto state = std::make_shared<descartes_light::State<FloatType>>(sol.template cast<FloatType>());
            return descartes_light::StateSample<FloatType>{ state, cost };
          });
    }
  }

  error_string_ = "Found at least 1 valid solution";

  return samples;
}

template <typename FloatType>
void DescartesRobotSampler<FloatType>::print(std::ostream& os) const
{
  os << "Working Frame: " << target_working_frame_ << ", TCP Frame: " << tcp_frame_ << "\n";
  os << "Target Pose:\n" << target_pose_.matrix() << "\n";
  os << "TCP Offset:\n" << tcp_offset_.matrix() << "\n";
  os << "Error string:\n" << error_string_;
}

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_HPP
