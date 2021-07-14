/**
 * @file simple_planner_utils.cpp
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

#include <tesseract_motion_planners/simple/profile/simple_planner_utils.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_kinematics/core/utils.h>

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

  // Synchronize the inverse kinematics with the forward kinematics
  inv_kin->synchronize(fwd_kin);

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
  Eigen::Isometry3d p1 = fwd_kin->calcFwdKin(jp);
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

  return working_frame * instruction.getWaypoint().as<CartesianWaypoint>();
}

const Eigen::VectorXd& InstructionInfo::extractJointPosition() const
{
  return getJointPosition(instruction.getWaypoint());
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

CompositeInstruction getInterpolatedComposite(const std::vector<std::string>& joint_names,
                                              const Eigen::MatrixXd& states,
                                              const PlanInstruction& base_instruction)
{
  CompositeInstruction composite;
  composite.setManipulatorInfo(base_instruction.getManipulatorInfo());
  composite.setDescription(base_instruction.getDescription());
  composite.setProfile(base_instruction.getProfile());
  composite.profile_overrides = base_instruction.profile_overrides;

  // Get move type base on base instruction type
  MoveInstructionType move_type = getMoveInstructionType(base_instruction);

  // Convert to MoveInstructions
  for (long i = 1; i < states.cols(); ++i)
  {
    MoveInstruction move_instruction(StateWaypoint(joint_names, states.col(i)), move_type);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    move_instruction.setProfile(base_instruction.getProfile());
    move_instruction.profile_overrides = base_instruction.profile_overrides;
    composite.push_back(move_instruction);
  }

  return composite;
}

Eigen::VectorXd getClosestJointSolution(const Eigen::Isometry3d& p,
                                        const tesseract_kinematics::InverseKinematics::Ptr& inv_kin,
                                        const Eigen::VectorXd& seed)
{
  Eigen::VectorXd jp_final;
  tesseract_kinematics::IKSolutions jp;
  tesseract_kinematics::IKSolutions solutions = inv_kin->calcInvKin(p, seed);
  for (const auto& sol : solutions)
  {
    jp.push_back(sol);
    auto redundant_solutions = tesseract_kinematics::getRedundantSolutions<double>(
        sol, inv_kin->getLimits().joint_limits, inv_kin->getRedundancyCapableJointIndices());
    jp.insert(jp.end(), redundant_solutions.begin(), redundant_solutions.end());
  }

  if (!jp.empty())
  {
    // Find closest solution to the start state
    double dist = std::numeric_limits<double>::max();
    for (const auto& solution : jp)
    {
      if (tesseract_common::satisfiesPositionLimits(solution, inv_kin->getLimits().joint_limits))
      {
        if (jp_final.rows() == 0)
        {
          jp_final = solution;
          continue;
        }

        /// @todo: May be nice to add contact checking to find best solution, but may not be neccessary because this is
        /// used to generate the seed
        double d = (solution - seed).norm();
        if (d < dist)
        {
          jp_final = solution;
          dist = d;
        }
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
  Eigen::VectorXd j1_final;
  tesseract_kinematics::IKSolutions j1;
  tesseract_kinematics::IKSolutions j1_solutions = inv_kin1->calcInvKin(p1, seed);
  j1_solutions.erase(std::remove_if(j1_solutions.begin(),
                                    j1_solutions.end(),
                                    [inv_kin1](const Eigen::VectorXd& solution) {
                                      return !tesseract_common::satisfiesPositionLimits(
                                          solution, inv_kin1->getLimits().joint_limits);
                                    }),
                     j1_solutions.end());

  // Get redundant solutions
  for (const auto& sol : j1_solutions)
  {
    j1.push_back(sol);
    auto redundant_solutions = tesseract_kinematics::getRedundantSolutions<double>(
        sol, inv_kin1->getLimits().joint_limits, inv_kin1->getRedundancyCapableJointIndices());
    j1.insert(j1.end(), redundant_solutions.begin(), redundant_solutions.end());
  }

  Eigen::VectorXd j2_final;
  tesseract_kinematics::IKSolutions j2;
  tesseract_kinematics::IKSolutions j2_solutions = inv_kin2->calcInvKin(p2, seed);
  j2_solutions.erase(std::remove_if(j2_solutions.begin(),
                                    j2_solutions.end(),
                                    [inv_kin2](const Eigen::VectorXd& solution) {
                                      return !tesseract_common::satisfiesPositionLimits(
                                          solution, inv_kin2->getLimits().joint_limits);
                                    }),
                     j2_solutions.end());

  // Get redundant solutions
  for (const auto& sol : j2_solutions)
  {
    j2.push_back(sol);
    auto redundant_solutions = tesseract_kinematics::getRedundantSolutions<double>(
        sol, inv_kin2->getLimits().joint_limits, inv_kin2->getRedundancyCapableJointIndices());
    j2.insert(j2.end(), redundant_solutions.begin(), redundant_solutions.end());
  }

  if (!j1.empty() && !j2.empty())
  {
    // Find closest solution to the end state
    double dist = std::numeric_limits<double>::max();
    j1_final = j1[0];
    j2_final = j2[0];
    for (const auto& j1_solution : j1)
    {
      for (const auto& j2_solution : j2)
      {
        /// @todo: May be nice to add contact checking to find best solution, but may not be neccessary because this is
        /// used to generate the seed.
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
  else if (!j1.empty())
  {
    double dist = std::numeric_limits<double>::max();
    j1_final = j1[0];
    for (const auto& j1_solution : j1)
    {
      double d = (seed - j1_solution).norm();
      if (d < dist)
      {
        j1_final = j1_solution;
        dist = d;
      }
    }

    results[0] = j1_final;
  }
  else if (!j2.empty())
  {
    double dist = std::numeric_limits<double>::max();
    j2_final = j2[0];
    for (const auto& j2_solution : j2)
    {
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
