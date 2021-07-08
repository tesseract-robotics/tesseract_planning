/**
 * @file instructions_trajectory.cpp
 * @brief Trajectory Container implementation for command language instructions
 *
 * @author Levi Armstrong
 * @date March 3, 2021
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

#include <tesseract_time_parameterization/instructions_trajectory.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/waypoint_type.h>

namespace tesseract_planning
{
static flattenFilterFn programFlattenMoveInstructionFilter =
    [](const Instruction& i, const CompositeInstruction& /*composite*/, bool parent_is_first_composite) {
      if (isMoveInstruction(i))
      {
        if (i.as<MoveInstruction>().isStart())
          return (parent_is_first_composite);

        return true;
      }

      return false;
    };

InstructionsTrajectory::InstructionsTrajectory(std::vector<std::reference_wrapper<Instruction>> trajectory)
  : trajectory_(std::move(trajectory))
{
  if (trajectory_.empty())
    throw std::runtime_error("Tried to construct InstructionsTrajectory with empty trajectory!");

  dof_ = trajectory_.front().get().as<MoveInstruction>().getWaypoint().as<StateWaypoint>().position.rows();
}

InstructionsTrajectory::InstructionsTrajectory(CompositeInstruction& program)
{
  trajectory_ = flatten(program, programFlattenMoveInstructionFilter);

  if (trajectory_.empty())
    throw std::runtime_error("Tried to construct InstructionsTrajectory with empty trajectory!");

  dof_ = trajectory_.front().get().as<MoveInstruction>().getWaypoint().as<StateWaypoint>().position.rows();
}

const Eigen::VectorXd& InstructionsTrajectory::getPosition(Eigen::Index i) const
{
  assert(isMoveInstruction(trajectory_[static_cast<std::size_t>(i)].get()));
  assert(isStateWaypoint(trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstruction>().getWaypoint()));
  return trajectory_[static_cast<std::size_t>(i)]
      .get()
      .as<MoveInstruction>()
      .getWaypoint()
      .as<StateWaypoint>()
      .position;
}
const Eigen::VectorXd& InstructionsTrajectory::getVelocity(Eigen::Index i) const
{
  assert(isMoveInstruction(trajectory_[static_cast<std::size_t>(i)].get()));
  assert(isStateWaypoint(trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstruction>().getWaypoint()));
  return trajectory_[static_cast<std::size_t>(i)]
      .get()
      .as<MoveInstruction>()
      .getWaypoint()
      .as<StateWaypoint>()
      .velocity;
}

const Eigen::VectorXd& InstructionsTrajectory::getAcceleration(Eigen::Index i) const
{
  assert(isMoveInstruction(trajectory_[static_cast<std::size_t>(i)].get()));
  assert(isStateWaypoint(trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstruction>().getWaypoint()));
  return trajectory_[static_cast<std::size_t>(i)]
      .get()
      .as<MoveInstruction>()
      .getWaypoint()
      .as<StateWaypoint>()
      .acceleration;
}

void InstructionsTrajectory::setData(Eigen::Index i,
                                     const Eigen::VectorXd& velocity,
                                     const Eigen::VectorXd& acceleration,
                                     double time)
{
  assert(isMoveInstruction(trajectory_[static_cast<std::size_t>(i)].get()));
  assert(isStateWaypoint(trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstruction>().getWaypoint()));
  StateWaypoint& swp =
      trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstruction>().getWaypoint().as<StateWaypoint>();
  swp.velocity = velocity;
  swp.acceleration = acceleration;
  swp.time = time;
}

Eigen::Index InstructionsTrajectory::size() const { return static_cast<Eigen::Index>(trajectory_.size()); }

Eigen::Index InstructionsTrajectory::dof() const { return dof_; }

bool InstructionsTrajectory::empty() const { return trajectory_.empty(); }

}  // namespace tesseract_planning
