/**
 * @file instructions_trajectory.h
 * @brief Trajectory Container implementation for command language instructions
 *
 * @author Levi Armstrong
 * @date March 3, 2021
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
#ifndef TESSERACT_TIME_PARAMETERIZATION_INSTRUCTIONS_TRAJECTORY_H
#define TESSERACT_TIME_PARAMETERIZATION_INSTRUCTIONS_TRAJECTORY_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/fwd.h>

namespace tesseract::time_parameterization
{
class InstructionsTrajectory
{
public:
  InstructionsTrajectory(std::vector<std::reference_wrapper<tesseract::command_language::InstructionPoly>> trajectory);
  InstructionsTrajectory(tesseract::command_language::CompositeInstruction& program);

  const Eigen::VectorXd& getPosition(Eigen::Index i) const;
  Eigen::VectorXd& getPosition(Eigen::Index i);
  const Eigen::VectorXd& getVelocity(Eigen::Index i) const;
  Eigen::VectorXd& getVelocity(Eigen::Index i);
  const Eigen::VectorXd& getAcceleration(Eigen::Index i) const;
  Eigen::VectorXd& getAcceleration(Eigen::Index i);
  double getTimeFromStart(Eigen::Index i) const;

  void setData(Eigen::Index i, const Eigen::VectorXd& velocity, const Eigen::VectorXd& acceleration, double time);

  Eigen::Index size() const;
  Eigen::Index dof() const;
  bool empty() const;

  /** @brief Check if time is strictly increasing */
  bool isTimeStrictlyIncreasing() const;

private:
  std::vector<std::reference_wrapper<tesseract::command_language::InstructionPoly>> trajectory_;
  Eigen::Index dof_;
};
}  // namespace tesseract::time_parameterization
#endif  // TESSERACT_TIME_PARAMETERIZATION_INSTRUCTIONS_TRAJECTORY_H
