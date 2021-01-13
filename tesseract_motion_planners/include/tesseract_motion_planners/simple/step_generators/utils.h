/**
 * @file utils.h
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
#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_STEP_GENERATOR_UTILS_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_STEP_GENERATOR_UTILS_H

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
/** @brief The Instruction Infomation struct */
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
};

MoveInstructionType getMoveInstructionType(const PlanInstruction& base_instruction);

CompositeInstruction getInterpolatedComposite(const Eigen::MatrixXd& states,
                                              const tesseract_kinematics::ForwardKinematics::Ptr& fwd_kin,
                                              const PlanInstruction& base_instruction);

Eigen::VectorXd getClosestJointSolution(const Eigen::Isometry3d& p,
                                        const tesseract_kinematics::InverseKinematics::Ptr& inv_kin,
                                        const Eigen::VectorXd& seed);

std::array<Eigen::VectorXd, 2> getClosestJointSolution(const Eigen::Isometry3d& p1,
                                                       const Eigen::Isometry3d& p2,
                                                       const tesseract_kinematics::InverseKinematics::Ptr& inv_kin1,
                                                       const tesseract_kinematics::InverseKinematics::Ptr& inv_kin2,
                                                       const Eigen::VectorXd& seed);

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_STEP_GENERATOR_UTILS_H
