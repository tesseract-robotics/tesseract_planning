/**
 * @file utils.cpp
 * @brief Provides interpolation utils structs
 *
 * @author Levi Armstrong
 * @date January 12, 2021
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

#include <tesseract_motion_planners/simple/step_generators/utils.h>
#include <tesseract_command_language/utils/utils.h>

namespace tesseract_planning
{
InstructionInfo::InstructionInfo(const PlanInstruction& plan_instruction,
                                 const PlannerRequest& request,
                                 const ManipulatorInfo& manip_info)
  : instruction(plan_instruction)
{
  assert(!(manip_info.empty() && plan_instruction.getManipulatorInfo().empty()));
  ManipulatorInfo mi = manip_info.getCombined(plan_instruction.getManipulatorInfo());

  // Get the link_transforms
  const auto& tf = request.env_state->link_transforms;

  // Get Previous Instruction Kinematics
  fwd_kin = request.env->getManipulatorManager()->getFwdKinematicSolver(mi.manipulator);
  inv_kin = request.env->getManipulatorManager()->getInvKinematicSolver(mi.manipulator);
  if (inv_kin->getJointNames() != fwd_kin->getJointNames())
    throw std::runtime_error("Forward and Inverse Kinematic objects joints are not ordered the same!");

  // Get Previous Instruction TCP and Working Frame
  working_frame = (mi.working_frame.empty()) ? Eigen::Isometry3d::Identity() : tf.at(mi.working_frame);
  world_to_base = tf.at(fwd_kin->getBaseLinkName());
  tcp = request.env->findTCP(mi);

  // Get Previous Instruction Waypoint Info
  if (isStateWaypoint(plan_instruction.getWaypoint()) || isJointWaypoint(plan_instruction.getWaypoint()))
    has_cartesian_waypoint = false;
  else if (isCartesianWaypoint(plan_instruction.getWaypoint()))
    has_cartesian_waypoint = true;
  else
    throw std::runtime_error("Simple planner currently only supports State, Joint and Cartesian Waypoint types!");
}

Eigen::Isometry3d InstructionInfo::calcCartesianWorldPose(const Eigen::VectorXd& jp) const
{
  Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
  if (!fwd_kin->calcFwdKin(p1, jp))
    throw std::runtime_error("Interpolation: failed to find forward kinematics solution!");
  return world_to_base * p1 * tcp;
}

Eigen::Isometry3d InstructionInfo::calcCartesianLocalPose(const Eigen::Isometry3d& world) const
{
  return world_to_base.inverse() * world * tcp.inverse();
}

Eigen::Isometry3d InstructionInfo::extractCartesianWorldPose() const
{
  if (!isCartesianWaypoint(instruction.getWaypoint()))
    throw std::runtime_error("Instruction waypoint type is not a CartesianWaypoint, unable to extract cartesian pose!");

  return working_frame * (*instruction.getWaypoint().cast_const<CartesianWaypoint>());
}

MoveInstructionType getMoveInstructionType(const PlanInstruction& base_instruction)
{
  // Get move type base on base instruction type
  MoveInstructionType move_type;
  if (base_instruction.isLinear())
    move_type = MoveInstructionType::LINEAR;
  else if (base_instruction.isFreespace())
    move_type = MoveInstructionType::FREESPACE;
  else
    throw std::runtime_error("Interpolation: Unsupported Move Instruction Type!");

  return move_type;
}

CompositeInstruction getInterpolatedComposite(const Eigen::MatrixXd& states,
                                              const tesseract_kinematics::ForwardKinematics::Ptr& fwd_kin,
                                              const PlanInstruction& base_instruction)
{
  CompositeInstruction composite;

  // Get move type base on base instruction type
  MoveInstructionType move_type = getMoveInstructionType(base_instruction);

  // Convert to MoveInstructions
  for (long i = 1; i < states.cols(); ++i)
  {
    MoveInstruction move_instruction(StateWaypoint(fwd_kin->getJointNames(), states.col(i)), move_type);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    move_instruction.setProfile(base_instruction.getProfile());
    composite.push_back(move_instruction);
  }

  return composite;
}

Eigen::VectorXd getClosestJointSolution(const Eigen::Isometry3d& p,
                                        const tesseract_kinematics::InverseKinematics::Ptr& inv_kin,
                                        const Eigen::VectorXd& seed)
{
  Eigen::VectorXd jp, jp_final;
  bool jp_found = inv_kin->calcInvKin(jp, p, seed);
  if (jp_found)
  {
    // Find closest solution to the start state
    double dist = std::numeric_limits<double>::max();
    const auto dof = inv_kin->numJoints();
    long num_solutions = jp.size() / dof;
    jp_final = jp.middleRows(0, dof);
    for (long i = 0; i < num_solutions; ++i)
    {
      /// @todo: May be nice to add contact checking to find best solution, but may not be neccessary because this is
      /// used to generate the seed.
      auto solution = jp.middleRows(i * dof, dof);
      double d = (solution - seed).norm();
      if (d < dist)
      {
        jp_final = solution;
        dist = d;
      }
    }
  }
  return jp_final;
}

std::array<Eigen::VectorXd, 2> getClosestJointSolution(const Eigen::Isometry3d& p1,
                                                       const Eigen::Isometry3d& p2,
                                                       const tesseract_kinematics::InverseKinematics::Ptr& inv_kin1,
                                                       const tesseract_kinematics::InverseKinematics::Ptr& inv_kin2,
                                                       const Eigen::VectorXd& seed)
{
  std::array<Eigen::VectorXd, 2> results;

  // Calculate IK for start and end
  Eigen::VectorXd j1, j1_final;
  bool found_j1 = inv_kin1->calcInvKin(j1, p1, seed);

  Eigen::VectorXd j2, j2_final;
  bool found_j2 = inv_kin2->calcInvKin(j2, p2, seed);

  if (found_j1 && found_j2)
  {
    // Find closest solution to the end state
    double dist = std::numeric_limits<double>::max();
    const auto dof = inv_kin2->numJoints();
    long j1_num_solutions = j1.size() / dof;
    long j2_num_solutions = j2.size() / dof;
    j1_final = j1.middleRows(0, dof);
    j2_final = j2.middleRows(0, dof);
    for (long i = 0; i < j1_num_solutions; ++i)
    {
      auto j1_solution = j1.middleRows(i * dof, dof);
      for (long j = 0; j < j2_num_solutions; ++j)
      {
        /// @todo: May be nice to add contact checking to find best solution, but may not be neccessary because this is
        /// used to generate the seed.
        auto j2_solution = j2.middleRows(j * dof, dof);
        double d = (j2_solution - j1_solution).norm();
        if (d < dist)
        {
          j1_final = j1_solution;
          j2_final = j2_solution;
          dist = d;
        }
      }
    }
    results[0] = j1_final;
    results[1] = j2_final;
  }
  else if (found_j1)
  {
    double dist = std::numeric_limits<double>::max();
    const auto dof = inv_kin1->numJoints();
    long j1_num_solutions = j1.size() / dof;
    j1_final = j1.middleRows(0, dof);
    for (long i = 0; i < j1_num_solutions; ++i)
    {
      auto j1_solution = j1.middleRows(i * dof, dof);
      double d = (seed - j1_solution).norm();
      if (d < dist)
      {
        j1_final = j1_solution;
        dist = d;
      }
    }

    results[0] = j1_final;
  }
  else if (found_j2)
  {
    double dist = std::numeric_limits<double>::max();
    const auto dof = inv_kin2->numJoints();
    long j2_num_solutions = j2.size() / dof;
    j2_final = j2.middleRows(0, dof);
    for (long i = 0; i < j2_num_solutions; ++i)
    {
      auto j2_solution = j2.middleRows(i * dof, dof);
      double d = (seed - j2_solution).norm();
      if (d < dist)
      {
        j2_final = j2_solution;
        dist = d;
      }
    }

    results[1] = j1_final;
  }

  return results;
}

}  // namespace tesseract_planning
