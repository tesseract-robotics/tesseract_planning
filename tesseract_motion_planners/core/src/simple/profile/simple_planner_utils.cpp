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

  // Check required manipulator information
  if (mi.manipulator.empty())
    throw std::runtime_error("InstructionInfo, manipulator is empty!");

  if (mi.tcp_frame.empty())
    throw std::runtime_error("InstructionInfo, TCP frame is empty!");

  if (mi.working_frame.empty())
    throw std::runtime_error("InstructionInfo, working frame is empty!");

  // Get Previous Instruction Kinematics
  manip = request.env->getKinematicGroup(mi.manipulator);

  // Get Previous Instruction TCP and Working Frame
  working_frame = mi.working_frame;
  tcp_frame = mi.tcp_frame;
  tcp_offset = request.env->findTCPOffset(mi);

  // Get Previous Instruction Waypoint Info
  if (isStateWaypoint(plan_instruction.getWaypoint()) || isJointWaypoint(plan_instruction.getWaypoint()))
    has_cartesian_waypoint = false;
  else if (isCartesianWaypoint(plan_instruction.getWaypoint()))
    has_cartesian_waypoint = true;
  else
    throw std::runtime_error("Simple planner currently only supports State, Joint and Cartesian Waypoint types!");
}

Eigen::Isometry3d InstructionInfo::calcCartesianPose(const Eigen::VectorXd& jp) const
{
  return manip->calcFwdKin(jp)[tcp_frame] * tcp_offset;
}

Eigen::Isometry3d InstructionInfo::extractCartesianPose() const
{
  if (!isCartesianWaypoint(instruction.getWaypoint()))
    throw std::runtime_error("Instruction waypoint type is not a CartesianWaypoint, unable to extract cartesian pose!");

  return instruction.getWaypoint().as<CartesianWaypoint>();
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

Eigen::VectorXd getClosestJointSolution(const InstructionInfo& info, const Eigen::VectorXd& seed)
{
  auto limits = info.manip->getLimits();
  auto redundancy_indices = info.manip->getRedundancyCapableJointIndices();

  if (!info.has_cartesian_waypoint)
    throw std::runtime_error("Instruction waypoint type is not a CartesianWaypoint, unable to extract cartesian pose!");

  Eigen::Isometry3d cwp = info.instruction.getWaypoint().as<CartesianWaypoint>() * info.tcp_offset.inverse();

  Eigen::VectorXd jp_final;
  tesseract_kinematics::IKSolutions jp;
  tesseract_kinematics::KinGroupIKInput ik_input(cwp, info.working_frame, info.tcp_frame);
  tesseract_kinematics::IKSolutions solutions = info.manip->calcInvKin({ ik_input }, seed);
  for (const auto& sol : solutions)
  {
    jp.push_back(sol);
    auto redundant_solutions =
        tesseract_kinematics::getRedundantSolutions<double>(sol, limits.joint_limits, redundancy_indices);
    jp.insert(jp.end(), redundant_solutions.begin(), redundant_solutions.end());
  }

  if (!jp.empty())
  {
    // Find closest solution to the start state
    double dist = std::numeric_limits<double>::max();
    for (const auto& solution : jp)
    {
      if (tesseract_common::satisfiesPositionLimits(solution, limits.joint_limits))
      {
        if (jp_final.rows() == 0)
        {
          jp_final = solution;
          continue;
        }

        /// @todo: May be nice to add contact checking to find best solution, but may not be necessary because this is
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

std::array<Eigen::VectorXd, 2> getClosestJointSolution(const InstructionInfo& info1,
                                                       const InstructionInfo& info2,
                                                       const Eigen::VectorXd& seed)
{
  auto manip1_limits = info1.manip->getLimits();
  auto manip1_redundancy_indices = info1.manip->getRedundancyCapableJointIndices();

  auto manip2_limits = info2.manip->getLimits();
  auto manip2_redundancy_indices = info2.manip->getRedundancyCapableJointIndices();

  if (!info1.has_cartesian_waypoint || !info2.has_cartesian_waypoint)
    throw std::runtime_error("Instruction waypoint type is not a CartesianWaypoint, unable to extract cartesian pose!");

  Eigen::Isometry3d cwp1 = info1.instruction.getWaypoint().as<CartesianWaypoint>() * info1.tcp_offset.inverse();
  Eigen::Isometry3d cwp2 = info2.instruction.getWaypoint().as<CartesianWaypoint>() * info2.tcp_offset.inverse();

  std::array<Eigen::VectorXd, 2> results;

  // Calculate IK for start and end
  Eigen::VectorXd j1_final;
  tesseract_kinematics::IKSolutions j1;
  tesseract_kinematics::KinGroupIKInput ik_input1(cwp1, info1.working_frame, info1.tcp_frame);
  tesseract_kinematics::IKSolutions j1_solutions = info1.manip->calcInvKin({ ik_input1 }, seed);
  j1_solutions.erase(std::remove_if(j1_solutions.begin(),
                                    j1_solutions.end(),
                                    [&manip1_limits](const Eigen::VectorXd& solution) {
                                      return !tesseract_common::satisfiesPositionLimits(solution,
                                                                                        manip1_limits.joint_limits);
                                    }),
                     j1_solutions.end());

  // Get redundant solutions
  for (const auto& sol : j1_solutions)
  {
    j1.push_back(sol);
    auto redundant_solutions =
        tesseract_kinematics::getRedundantSolutions<double>(sol, manip1_limits.joint_limits, manip1_redundancy_indices);
    j1.insert(j1.end(), redundant_solutions.begin(), redundant_solutions.end());
  }

  Eigen::VectorXd j2_final;
  tesseract_kinematics::IKSolutions j2;
  tesseract_kinematics::KinGroupIKInput ik_input2(cwp2, info2.working_frame, info2.tcp_frame);
  tesseract_kinematics::IKSolutions j2_solutions = info2.manip->calcInvKin({ ik_input2 }, seed);
  j2_solutions.erase(std::remove_if(j2_solutions.begin(),
                                    j2_solutions.end(),
                                    [&manip2_limits](const Eigen::VectorXd& solution) {
                                      return !tesseract_common::satisfiesPositionLimits(solution,
                                                                                        manip2_limits.joint_limits);
                                    }),
                     j2_solutions.end());

  // Get redundant solutions
  for (const auto& sol : j2_solutions)
  {
    j2.push_back(sol);
    auto redundant_solutions =
        tesseract_kinematics::getRedundantSolutions<double>(sol, manip2_limits.joint_limits, manip2_redundancy_indices);
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
        /// @todo: May be nice to add contact checking to find best solution, but may not be necessary because this is
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
