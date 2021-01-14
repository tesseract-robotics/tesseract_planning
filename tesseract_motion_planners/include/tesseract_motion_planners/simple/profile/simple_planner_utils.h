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
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin;
  tesseract_kinematics::InverseKinematics::Ptr inv_kin;
  Eigen::Isometry3d world_to_base;
  Eigen::Isometry3d working_frame;
  Eigen::Isometry3d tcp;
  bool has_cartesian_waypoint{ false };

  /**
   * @brief Calculate the cartesian pose in world coordinate given the joint solution
   * @param jp The joint solution to calculate the world pose
   * @return The world pose give the joint solution
   */
  Eigen::Isometry3d calcCartesianWorldPose(const Eigen::VectorXd& jp) const;

  /**
   * @brief Covert cartesian pose in world to the local robot base frame
   * @return Cartesian pose in the local robot base frame
   */
  Eigen::Isometry3d calcCartesianLocalPose(const Eigen::Isometry3d& world) const;

  /**
   * @brief Extract the cartesian pose in the world
   * @details If the instruction does not have a cartesian waypoint this throws an exception
   * @return Cartesian pose in the world
   */
  Eigen::Isometry3d extractCartesianWorldPose() const;

  /**
   * @brief Extract the joint positiion from the instruction waypoint
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
 * @param p The cartesian position to solve inverse kinematics
 * @param inv_kin The inverse kinematics
 * @param seed The seed to find the closest solution
 * @return The closest solution to the seed. This will be empty if a solution was not found during inverse kinematics
 */
Eigen::VectorXd getClosestJointSolution(const Eigen::Isometry3d& p,
                                        const tesseract_kinematics::InverseKinematics::Ptr& inv_kin,
                                        const Eigen::VectorXd& seed);

/**
 * @brief Find the closest joint solution for the two provided cartesian poses.
 * @param p1 The first cartesian position to solve inverse kinematics
 * @param p2 The second cartesian position to solve inverse kinematics
 * @param inv_kin1 The inverse kinematics associated to the first cartesian position
 * @param inv_kin2 The inverse kinematics associated to the first cartesian position
 * @param seed The seed to use during inverse kinematics
 * @return The closest joint solution for the provided cartesian positions. If either are empty then it failed to solve
 * inverse kinematics.
 */
std::array<Eigen::VectorXd, 2> getClosestJointSolution(const Eigen::Isometry3d& p1,
                                                       const Eigen::Isometry3d& p2,
                                                       const tesseract_kinematics::InverseKinematics::Ptr& inv_kin1,
                                                       const tesseract_kinematics::InverseKinematics::Ptr& inv_kin2,
                                                       const Eigen::VectorXd& seed);

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_PROFILE_SIMPLE_PLANNER_UTILS_H
