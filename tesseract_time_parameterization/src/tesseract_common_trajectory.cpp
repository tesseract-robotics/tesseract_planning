/**
 * @file tesseract_common_trajectory.cpp
 * @brief Trajectory Container implementation for tesseract_common::JointState trajectories
 *
 * @author Connor Wolfe
 * @date June 18, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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

#include <tesseract_time_parameterization/tesseract_common_trajectory.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/waypoint_type.h>

namespace tesseract_planning
{
TesseractCommonTrajectory::TesseractCommonTrajectory(tesseract_common::JointTrajectory& trajectory)
  : trajectory_(trajectory)
{
  if (trajectory_.empty())
    throw std::runtime_error("Tried to construct TesseractCommonTrajectory with empty trajectory!");

  dof_ = static_cast<Eigen::Index>(trajectory_.front().joint_names.size());
}

const Eigen::VectorXd& TesseractCommonTrajectory::getPosition(Eigen::Index i) const
{
  // TODO add assert that i<dof_
  return trajectory_.at(static_cast<std::size_t>(i)).position;
}
const Eigen::VectorXd& TesseractCommonTrajectory::getVelocity(Eigen::Index i) const
{
  return trajectory_.at(static_cast<std::size_t>(i)).velocity;
}

const Eigen::VectorXd& TesseractCommonTrajectory::getAcceleration(Eigen::Index i) const
{
  return trajectory_.at(static_cast<std::size_t>(i)).acceleration;
}

const double& TesseractCommonTrajectory::getTime(Eigen::Index i) const
{
  return trajectory_.at(static_cast<std::size_t>(i)).time;
}

void TesseractCommonTrajectory::setData(Eigen::Index i,
                                        const Eigen::VectorXd& velocity,
                                        const Eigen::VectorXd& acceleration,
                                        double time)
{
  tesseract_common::JointState& swp = trajectory_.at(static_cast<std::size_t>(i));
  swp.velocity = velocity;
  swp.acceleration = acceleration;
  swp.time = time;
}

Eigen::Index TesseractCommonTrajectory::size() const { return static_cast<Eigen::Index>(trajectory_.size()); }

Eigen::Index TesseractCommonTrajectory::dof() const { return dof_; }

bool TesseractCommonTrajectory::empty() const { return trajectory_.empty(); }

}  // namespace tesseract_planning
