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

#include <tesseract_time_parameterization/core/instructions_trajectory.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>

namespace tesseract_planning
{
static const flattenFilterFn programFlattenMoveInstructionFilter =
    [](const InstructionPoly& i, const CompositeInstruction& /*composite*/) { return i.isMoveInstruction(); };

InstructionsTrajectory::InstructionsTrajectory(std::vector<std::reference_wrapper<InstructionPoly>> trajectory)
  : trajectory_(std::move(trajectory))
{
  if (trajectory_.empty())
    throw std::runtime_error("Tried to construct InstructionsTrajectory with empty trajectory!");

  dof_ = trajectory_.front().get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getPosition().rows();
}

InstructionsTrajectory::InstructionsTrajectory(CompositeInstruction& program)
{
  trajectory_ = program.flatten(programFlattenMoveInstructionFilter);

  if (trajectory_.empty())
    throw std::runtime_error("Tried to construct InstructionsTrajectory with empty trajectory!");

  dof_ = trajectory_.front().get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>().getPosition().rows();
}

const Eigen::VectorXd& InstructionsTrajectory::getPosition(Eigen::Index i) const
{
  assert(trajectory_[static_cast<std::size_t>(i)].get().isMoveInstruction());
  assert(trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstructionPoly>().getWaypoint().isStateWaypoint());
  return trajectory_[static_cast<std::size_t>(i)]
      .get()
      .as<MoveInstructionPoly>()
      .getWaypoint()
      .as<StateWaypointPoly>()
      .getPosition();
}

Eigen::VectorXd& InstructionsTrajectory::getPosition(Eigen::Index i)
{
  assert(trajectory_[static_cast<std::size_t>(i)].get().isMoveInstruction());
  assert(trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstructionPoly>().getWaypoint().isStateWaypoint());
  return trajectory_[static_cast<std::size_t>(i)]
      .get()
      .as<MoveInstructionPoly>()
      .getWaypoint()
      .as<StateWaypointPoly>()
      .getPosition();
}

const Eigen::VectorXd& InstructionsTrajectory::getVelocity(Eigen::Index i) const
{
  assert(trajectory_[static_cast<std::size_t>(i)].get().isMoveInstruction());
  assert(trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstructionPoly>().getWaypoint().isStateWaypoint());
  return trajectory_[static_cast<std::size_t>(i)]
      .get()
      .as<MoveInstructionPoly>()
      .getWaypoint()
      .as<StateWaypointPoly>()
      .getVelocity();
}

Eigen::VectorXd& InstructionsTrajectory::getVelocity(Eigen::Index i)
{
  assert(trajectory_[static_cast<std::size_t>(i)].get().isMoveInstruction());
  assert(trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstructionPoly>().getWaypoint().isStateWaypoint());
  return trajectory_[static_cast<std::size_t>(i)]
      .get()
      .as<MoveInstructionPoly>()
      .getWaypoint()
      .as<StateWaypointPoly>()
      .getVelocity();
}

const Eigen::VectorXd& InstructionsTrajectory::getAcceleration(Eigen::Index i) const
{
  assert(trajectory_[static_cast<std::size_t>(i)].get().isMoveInstruction());
  assert(trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstructionPoly>().getWaypoint().isStateWaypoint());
  return trajectory_[static_cast<std::size_t>(i)]
      .get()
      .as<MoveInstructionPoly>()
      .getWaypoint()
      .as<StateWaypointPoly>()
      .getAcceleration();
}

Eigen::VectorXd& InstructionsTrajectory::getAcceleration(Eigen::Index i)
{
  assert(trajectory_[static_cast<std::size_t>(i)].get().isMoveInstruction());
  assert(trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstructionPoly>().getWaypoint().isStateWaypoint());
  return trajectory_[static_cast<std::size_t>(i)]
      .get()
      .as<MoveInstructionPoly>()
      .getWaypoint()
      .as<StateWaypointPoly>()
      .getAcceleration();
}

double InstructionsTrajectory::getTimeFromStart(Eigen::Index i) const
{
  assert(trajectory_[static_cast<std::size_t>(i)].get().isMoveInstruction());
  assert(trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstructionPoly>().getWaypoint().isStateWaypoint());
  return trajectory_[static_cast<std::size_t>(i)]
      .get()
      .as<MoveInstructionPoly>()
      .getWaypoint()
      .as<StateWaypointPoly>()
      .getTime();
}

void InstructionsTrajectory::setData(Eigen::Index i,
                                     const Eigen::VectorXd& velocity,
                                     const Eigen::VectorXd& acceleration,
                                     double time)
{
  assert(trajectory_[static_cast<std::size_t>(i)].get().isMoveInstruction());
  assert(trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstructionPoly>().getWaypoint().isStateWaypoint());
  auto& swp =
      trajectory_[static_cast<std::size_t>(i)].get().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
  swp.setVelocity(velocity);
  swp.setAcceleration(acceleration);
  swp.setTime(time);
}

Eigen::Index InstructionsTrajectory::size() const { return static_cast<Eigen::Index>(trajectory_.size()); }

Eigen::Index InstructionsTrajectory::dof() const { return dof_; }

bool InstructionsTrajectory::empty() const { return trajectory_.empty(); }

}  // namespace tesseract_planning
