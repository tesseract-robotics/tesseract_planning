/**
 * @file simple_planner_utils.h
 * @brief Provides interpolation utils for simple planner profiles
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
#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_PROFILE_SIMPLE_PLANNER_UTILS_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_PROFILE_SIMPLE_PLANNER_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <array>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/command_language.h>
#include <tesseract_motion_planners/core/types.h>

namespace tesseract_planning
{
/** @brief The Instruction Information struct */
struct InstructionInfo
{
  InstructionInfo(const PlanInstruction& plan_instruction,
                  const PlannerRequest& request,
                  const ManipulatorInfo& manip_info);

  const PlanInstruction& instruction;
  tesseract_kinematics::KinematicGroup::UPtr manip;
  std::string working_frame;
  std::string tcp_frame;
  Eigen::Isometry3d tcp_offset{ Eigen::Isometry3d::Identity() };
  bool has_cartesian_waypoint{ false };

  /**
   * @brief Calculate the cartesian pose given the joint solution
   * @param jp The joint solution to calculate the pose
   * @return The pose give the joint solution
   */
  Eigen::Isometry3d calcCartesianPose(const Eigen::VectorXd& jp) const;

  /**
   * @brief Extract the cartesian pose from the instruction
   * @details If the instruction does not have a cartesian waypoint this throws an exception
   * @return Cartesian pose
   */
  Eigen::Isometry3d extractCartesianPose() const;

  /**
   * @brief Extract the joint position from the instruction waypoint
   * @details If the instruction does not have a joint/state waypoint this throws an exception
   * @return Joint Position
   */
  const Eigen::VectorXd& extractJointPosition() const;
};

/**
 * @brief Get the associated move instruction type for the give plan instruction type
 * @param base_instruction The plan instruction
 * @returnThe associated move instruction type for the give plan instruction type
 */
MoveInstructionType getMoveInstructionType(const PlanInstruction& base_instruction);

/**
 * @brief This takes the provided seed state for the base_instruction and create a corresponding composite instruction
 * @param joint_names The joint names associated with the states
 * @param states The joint states to populate the composite instruction with
 * @param base_instruction The base instruction used to extract profile and manipulator information from
 * @return The composite instruction
 */
CompositeInstruction getInterpolatedComposite(const std::vector<std::string>& joint_names,
                                              const Eigen::MatrixXd& states,
                                              const PlanInstruction& base_instruction);

/**
 * @brief Find the closest joint solution for p to the provided seed
 * @param info The instruction info to find closest joint solution
 * @param seed The seed to find the closest solution
 * @return The closest solution to the seed. This will be empty if a solution was not found during inverse kinematics
 */
Eigen::VectorXd getClosestJointSolution(const InstructionInfo& info, const Eigen::VectorXd& seed);

/**
 * @brief Find the closest joint solution for the two provided cartesian poses.
 * @param info1 The instruction info to find closest joint solution
 * @param info2 The instruction info to find closest joint solution
 * @param seed The seed to use during inverse kinematics
 * @return The closest joint solution for the provided cartesian positions. If either are empty then it failed to solve
 * inverse kinematics.
 */
std::array<Eigen::VectorXd, 2> getClosestJointSolution(const InstructionInfo& info1,
                                                       const InstructionInfo& info2,
                                                       const Eigen::VectorXd& seed);

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_PROFILE_SIMPLE_PLANNER_UTILS_H
