/**
 * @file tesseract_cmmon_trajectory.h
 * @brief Trajectory Container implementation for tesseract_cmmon trajectories
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
#ifndef TESSERACT_COMMON_TRAJECTORY_H
#define TESSERACT_COMMON_TRAJECTORY_H

#include <tesseract_common/macros.h>
#include <tesseract_common/joint_state.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_time_parameterization/trajectory_container.h>

namespace tesseract_planning
{
class TesseractCommonTrajectory : public TrajectoryContainer
{
public:
  TesseractCommonTrajectory(tesseract_common::JointTrajectory& trajectory);

  const Eigen::VectorXd& getPosition(Eigen::Index i) const override final;
  const Eigen::VectorXd& getVelocity(Eigen::Index i) const override final;
  const Eigen::VectorXd& getAcceleration(Eigen::Index i) const override final;
  const double& getTime(Eigen::Index i) const;

  void setData(Eigen::Index i,
               const Eigen::VectorXd& velocity,
               const Eigen::VectorXd& acceleration,
               double time) override final;

  Eigen::Index size() const override final;
  Eigen::Index dof() const override final;
  bool empty() const override final;

private:
  tesseract_common::JointTrajectory& trajectory_;
  Eigen::Index dof_;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_COMMON_TRAJECTORY_H
