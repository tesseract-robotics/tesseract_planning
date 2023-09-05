/**
 * @file interpolation.h
 * @brief Provides interpolation utils
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
#ifndef TESSERACT_MOTION_PLANNERS_INTERPOLATION_H
#define TESSERACT_MOTION_PLANNERS_INTERPOLATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_motion_planners/core/types.h>

namespace tesseract_planning
{
/** @brief The Joint Group Instruction Information struct */
struct JointGroupInstructionInfo
{
  JointGroupInstructionInfo(const MoveInstructionPoly& plan_instruction,
                            const PlannerRequest& request,
                            const tesseract_common::ManipulatorInfo& manip_info);

  const MoveInstructionPoly& instruction;
  tesseract_kinematics::JointGroup::UPtr manip;
  std::string working_frame;
  Eigen::Isometry3d working_frame_transform{ Eigen::Isometry3d::Identity() };
  std::string tcp_frame;
  Eigen::Isometry3d tcp_offset{ Eigen::Isometry3d::Identity() };
  bool has_cartesian_waypoint{ false };

  /**
   * @brief Get the working frame
   * @return The working frame transfrom relative world
   */
  Eigen::Isometry3d getWorkingFrame() const;

  /**
   * @brief Calculate the cartesian pose given the joint solution
   * @param jp The joint solution to calculate the pose
   * @param in_world Indicate if the results should be in world or relative to working frame
   * @return The pose give the joint solution
   */
  Eigen::Isometry3d calcCartesianPose(const Eigen::VectorXd& jp, bool in_world = true) const;

  /**
   * @brief Extract the cartesian pose from the instruction
   * @details If the instruction does not have a cartesian waypoint this throws an exception
   * @param in_world Indicate if the results should be in world or relative to working frame
   * @return Cartesian pose
   */
  Eigen::Isometry3d extractCartesianPose(bool in_world = true) const;

  /**
   * @brief Extract the joint position from the instruction waypoint
   * @details If the instruction does not have a joint/state waypoint this throws an exception
   * @return Joint Position
   */
  const Eigen::VectorXd& extractJointPosition() const;
};

/** @brief The Kinematic Group Instruction Information struct */
struct KinematicGroupInstructionInfo
{
  KinematicGroupInstructionInfo(const MoveInstructionPoly& plan_instruction,
                                const PlannerRequest& request,
                                const tesseract_common::ManipulatorInfo& manip_info);

  const MoveInstructionPoly& instruction;
  tesseract_kinematics::KinematicGroup::UPtr manip;
  std::string working_frame;
  Eigen::Isometry3d working_frame_transform{ Eigen::Isometry3d::Identity() };
  std::string tcp_frame;
  Eigen::Isometry3d tcp_offset{ Eigen::Isometry3d::Identity() };
  bool has_cartesian_waypoint{ false };

  /**
   * @brief Calculate the cartesian pose given the joint solution
   * @param jp The joint solution to calculate the pose
   * @param in_world Indicate if the results should be in world or relative to working frame
   * @return The pose give the joint solution
   */
  Eigen::Isometry3d calcCartesianPose(const Eigen::VectorXd& jp, bool in_world = true) const;

  /**
   * @brief Extract the cartesian pose from the instruction
   * @details If the instruction does not have a cartesian waypoint this throws an exception
   * @param in_world Indicate if the results should be in world or relative to working frame
   * @return Cartesian pose
   */
  Eigen::Isometry3d extractCartesianPose(bool in_world = true) const;

  /**
   * @brief Extract the joint position from the instruction waypoint
   * @details If the instruction does not have a joint/state waypoint this throws an exception
   * @return Joint Position
   */
  const Eigen::VectorXd& extractJointPosition() const;
};

std::vector<MoveInstructionPoly> interpolateJointJointWaypoint(const KinematicGroupInstructionInfo& prev,
                                                               const KinematicGroupInstructionInfo& base,
                                                               int linear_steps,
                                                               int freespace_steps);

std::vector<MoveInstructionPoly> interpolateJointCartWaypoint(const KinematicGroupInstructionInfo& prev,
                                                              const KinematicGroupInstructionInfo& base,
                                                              int linear_steps,
                                                              int freespace_steps);

std::vector<MoveInstructionPoly> interpolateCartJointWaypoint(const KinematicGroupInstructionInfo& prev,
                                                              const KinematicGroupInstructionInfo& base,
                                                              int linear_steps,
                                                              int freespace_steps);

std::vector<MoveInstructionPoly> interpolateCartCartWaypoint(const KinematicGroupInstructionInfo& prev,
                                                             const KinematicGroupInstructionInfo& base,
                                                             int linear_steps,
                                                             int freespace_steps,
                                                             const tesseract_scene_graph::SceneState& scene_state);

/**
 * @brief JointWaypoint to JointWaypoint
 * @details This does not include the prev instruction but does include the base instruction
 *
 * This function interpolates the motion from start state to end state.
 *
 * - the number of steps for the plan will be calculated such that:
 *   - the translation distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *   - the number of steps for the plan will be calculated such that the norm of all joint distances between
 *successive steps is no longer than state_longest_valid_segment_length
 *   - the max steps from the above calculations will be be compared to the min_steps and the largest will be chosen
 * - the interpolation will be done in joint space
 *
 * @param state_lvs_length The maximum joint distance, the norm of changes to all joint positions between successive
 *steps.
 * @param translation_lvs_length The maximum translation distance between successive steps
 * @param rotation_lvs_length The maximum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @param max_steps The maximum number of steps for the plan
 * @return A vector of move instruction
 **/
std::vector<MoveInstructionPoly> interpolateJointJointWaypoint(const KinematicGroupInstructionInfo& prev,
                                                               const KinematicGroupInstructionInfo& base,
                                                               double state_lvs_length,
                                                               double translation_lvs_length,
                                                               double rotation_lvs_length,
                                                               int min_steps,
                                                               int max_steps);

/**
 * @brief JointWaypoint to CartesianWaypoint
 * @details This does not include the prev instruction but does include the base instruction
 *
 * - the number of steps for the plan will be calculated such that:
 *   - the translation distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *   - the number of steps for the plan will be calculated such that the norm of all joint distances between
 *successive steps is no longer than state_longest_valid_segment_length
 *   - the max steps from the above calculations will be be compared to the min_steps and the largest will be chosen
 * - the interpolation will be done based on the condition below
 *   - Case 1: Joint solution found for end cartesian waypoint
 *     - It interpolates the joint position from the start to the end state
 *   - Case 2: Unable to find joint solution for end cartesian waypoint
 *     - It creates number states based on the steps and sets the value to start joint waypoint
 *
 * @param state_lvs_length The maximum joint distance, the norm of changes to all joint positions between successive
 *steps.
 * @param translation_lvs_length The maximum translation distance between successive steps
 * @param rotation_lvs_length The maximum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @param max_steps The maximum number of steps for the plan
 * @return A vector of move instruction
 **/
std::vector<MoveInstructionPoly> interpolateJointCartWaypoint(const KinematicGroupInstructionInfo& prev,
                                                              const KinematicGroupInstructionInfo& base,
                                                              double state_lvs_length,
                                                              double translation_lvs_length,
                                                              double rotation_lvs_length,
                                                              int min_steps,
                                                              int max_steps);

/**
 * @brief CartesianWaypoint to JointWaypoint
 * @details This does not include the prev instruction but does include the base instruction
 *
 * This function interpolates the motion from start state to end state.
 *
 * - the number of steps for the plan will be calculated such that:
 *   - the translation distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *   - the number of steps for the plan will be calculated such that the norm of all joint distances between
 *successive steps is no longer than state_longest_valid_segment_length
 *   - the max steps from the above calculations will be be compared to the min_steps and the largest will be chosen
 * - the interpolation will be done based on the condition below
 *   - Case 1: Joint solution found for start cartesian waypoint
 *     - It interpolates the joint position from the start to the end state
 *   - Case 2: Unable to find joint solution for start cartesian waypoint
 *     - It creates number states based on the steps and sets the value to end joint waypoint
 *
 * @param state_lvs_length The maximum joint distance, the norm of changes to all joint positions between successive
 *steps.
 * @param translation_lvs_length The maximum translation distance between successive steps
 * @param rotation_lvs_length The maximum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @param max_steps The maximum number of steps for the plan
 * @return A vector of move instruction
 **/
std::vector<MoveInstructionPoly> interpolateCartJointWaypoint(const KinematicGroupInstructionInfo& prev,
                                                              const KinematicGroupInstructionInfo& base,
                                                              double state_lvs_length,
                                                              double translation_lvs_length,
                                                              double rotation_lvs_length,
                                                              int min_steps,
                                                              int max_steps);

/**
 * @brief CartesianWaypoint to CartesianWaypoint
 * @details This does not include the prev instruction but does include the base instruction
 *
 * This function interpolates the motion from start state to end state.
 *
 * - the number of steps for the plan will be calculated such that:
 *   - the translation distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *   - the number of steps for the plan will be calculated such that the norm of all joint distances between
 *successive steps is no longer than state_longest_valid_segment_length
 *   - the max steps from the above calculations will be be compared to the min_steps and the largest will be chosen
 * - the interpolation will be done based on the condition below
 *   - Case 1: Joint solution found for start and end cartesian waypoint
 *     - It interpolates the joint position from the start to the end state
 *   - Case 2: Joint solution only found for start cartesian waypoint
 *     - It creates number states based on the steps and sets the value to found start solution
 *   - Case 3: Joint solution only found for end cartesian waypoint
 *     - It creates number states based on the steps and sets the value to found end solution
 *   - Case 4: No joint solution found for end and start cartesian waypoint
 *     - It creates number states based on the steps and sets the value to the current state of the environment
 *
 * @param state_lvs_length The maximum joint distance, the norm of changes to all joint positions between successive
 *steps.
 * @param translation_lvs_length The maximum translation distance between successive steps
 * @param rotation_lvs_length The maximum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @param max_steps The maximum number of steps for the plan
 * @return A vector of move instruction
 **/
std::vector<MoveInstructionPoly> interpolateCartCartWaypoint(const KinematicGroupInstructionInfo& prev,
                                                             const KinematicGroupInstructionInfo& base,
                                                             double state_lvs_length,
                                                             double translation_lvs_length,
                                                             double rotation_lvs_length,
                                                             int min_steps,
                                                             int max_steps,
                                                             const tesseract_scene_graph::SceneState& scene_state);

/**
 * @brief JointWaypoint to JointWaypoint
 * @details This does not include the prev instruction but does include the base instruction
 *
 * This function interpolates the motion from start state to end state.
 *
 * - the number of steps for the plan will be calculated such that:
 *   - the translation distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *   - the number of steps for the plan will be calculated such that the norm of all joint distances between
 *successive steps is no longer than state_longest_valid_segment_length
 *   - the max steps from the above calculations will be be compared to the min_steps and the largest will be chosen
 * - the interpolation will be done in joint space
 *
 * @param state_lvs_length The maximum joint distance, the norm of changes to all joint positions between successive
 *steps.
 * @param translation_lvs_length The maximum translation distance between successive steps
 * @param rotation_lvs_length The maximum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @param max_steps The maximum number of steps for the plan
 * @return A vector of move instruction
 **/
std::vector<MoveInstructionPoly> interpolateJointJointWaypoint(const JointGroupInstructionInfo& prev,
                                                               const JointGroupInstructionInfo& base,
                                                               double state_lvs_length,
                                                               double translation_lvs_length,
                                                               double rotation_lvs_length,
                                                               int min_steps,
                                                               int max_steps);

/**
 * @brief JointWaypoint to CartesianWaypoint
 * @details This does not include the prev instruction but does include the base instruction
 *
 * - the number of steps for the plan will be calculated such that:
 *   - the translation distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *   - the number of steps for the plan will be calculated such that the norm of all joint distances between
 *successive steps is no longer than state_longest_valid_segment_length
 *   - the max steps from the above calculations will be be compared to the min_steps and the largest will be chosen
 * - the interpolation will be done based on the condition below
 *   - Case 1: Joint solution found for end cartesian waypoint
 *     - It interpolates the joint position from the start to the end state
 *   - Case 2: Unable to find joint solution for end cartesian waypoint
 *     - It creates number states based on the steps and sets the value to start joint waypoint
 *
 * @param state_lvs_length The maximum joint distance, the norm of changes to all joint positions between successive
 *steps.
 * @param translation_lvs_length The maximum translation distance between successive steps
 * @param rotation_lvs_length The maximum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @param max_steps The maximum number of steps for the plan
 * @return A vector of move instruction
 **/
std::vector<MoveInstructionPoly> interpolateJointCartWaypoint(const JointGroupInstructionInfo& prev,
                                                              const JointGroupInstructionInfo& base,
                                                              double state_lvs_length,
                                                              double translation_lvs_length,
                                                              double rotation_lvs_length,
                                                              int min_steps,
                                                              int max_steps);

/**
 * @brief CartesianWaypoint to JointWaypoint
 * @details This does not include the prev instruction but does include the base instruction
 *
 * This function interpolates the motion from start state to end state.
 *
 * - the number of steps for the plan will be calculated such that:
 *   - the translation distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *   - the number of steps for the plan will be calculated such that the norm of all joint distances between
 *successive steps is no longer than state_longest_valid_segment_length
 *   - the max steps from the above calculations will be be compared to the min_steps and the largest will be chosen
 * - the interpolation will be done based on the condition below
 *   - Case 1: Joint solution found for start cartesian waypoint
 *     - It interpolates the joint position from the start to the end state
 *   - Case 2: Unable to find joint solution for start cartesian waypoint
 *     - It creates number states based on the steps and sets the value to end joint waypoint
 *
 * @param state_lvs_length The maximum joint distance, the norm of changes to all joint positions between successive
 *steps.
 * @param translation_lvs_length The maximum translation distance between successive steps
 * @param rotation_lvs_length The maximum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @param max_steps The maximum number of steps for the plan
 * @return A vector of move instruction
 **/
std::vector<MoveInstructionPoly> interpolateCartJointWaypoint(const JointGroupInstructionInfo& prev,
                                                              const JointGroupInstructionInfo& base,
                                                              double state_lvs_length,
                                                              double translation_lvs_length,
                                                              double rotation_lvs_length,
                                                              int min_steps,
                                                              int max_steps);

/**
 * @brief CartesianWaypoint to CartesianWaypoint
 * @details This does not include the prev instruction but does include the base instruction
 *
 * This function interpolates the motion from start state to end state.
 *
 * - the number of steps for the plan will be calculated such that:
 *   - the translation distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *   - the number of steps for the plan will be calculated such that the norm of all joint distances between
 *successive steps is no longer than state_longest_valid_segment_length
 *   - the max steps from the above calculations will be be compared to the min_steps and the largest will be chosen
 * - the interpolation will be done based on the condition below
 *   - Case 1: Joint solution found for start and end cartesian waypoint
 *     - It interpolates the joint position from the start to the end state
 *   - Case 2: Joint solution only found for start cartesian waypoint
 *     - It creates number states based on the steps and sets the value to found start solution
 *   - Case 3: Joint solution only found for end cartesian waypoint
 *     - It creates number states based on the steps and sets the value to found end solution
 *   - Case 4: No joint solution found for end and start cartesian waypoint
 *     - It creates number states based on the steps and sets the value to the current state of the environment
 *
 * @param state_lvs_length The maximum joint distance, the norm of changes to all joint positions between successive
 *steps.
 * @param translation_lvs_length The maximum translation distance between successive steps
 * @param rotation_lvs_length The maximum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @param max_steps The maximum number of steps for the plan
 * @return A vector of move instruction
 **/
std::vector<MoveInstructionPoly> interpolateCartCartWaypoint(const JointGroupInstructionInfo& prev,
                                                             const JointGroupInstructionInfo& base,
                                                             double state_lvs_length,
                                                             double translation_lvs_length,
                                                             double rotation_lvs_length,
                                                             int min_steps,
                                                             int max_steps,
                                                             const tesseract_scene_graph::SceneState& scene_state);
/**
 * @brief Interpolate between two transforms return a vector of Eigen::Isometry transforms.
 * @param start The Start Transform
 * @param stop The Stop/End Transform
 * @param steps The number of step
 * @return A vector of Eigen::Isometry with a length = steps + 1
 */
tesseract_common::VectorIsometry3d interpolate(const Eigen::Isometry3d& start,
                                               const Eigen::Isometry3d& stop,
                                               long steps);

/**
 * @brief Interpolate between two Eigen::VectorXd and return a Matrix
 * @param start The Start State
 * @param stop The Stop/End State
 * @param steps The number of step
 * @return A matrix where columns = steps + 1
 */
Eigen::MatrixXd interpolate(const Eigen::Ref<const Eigen::VectorXd>& start,
                            const Eigen::Ref<const Eigen::VectorXd>& stop,
                            long steps);

/**
 * @brief Interpolate between two waypoints return a vector of waypoints.
 * @param start The Start Waypoint
 * @param stop The Stop/End Waypoint
 * @param steps The number of step
 * @return A vector of waypoints with a length = steps + 1
 */
std::vector<WaypointPoly> interpolate_waypoint(const WaypointPoly& start, const WaypointPoly& stop, long steps);

/**
 * @brief This takes the provided seed state for the base_instruction and create a vector of move instruction
 * @details This skips the first state
 * @param joint_names The joint names associated with the states
 * @param states The joint states to populate the composite instruction with
 * @param base_instruction The base instruction used to extract profile and manipulator information from
 * @return A vector of move instruction
 */
std::vector<MoveInstructionPoly> getInterpolatedInstructions(const std::vector<std::string>& joint_names,
                                                             const Eigen::MatrixXd& states,
                                                             const MoveInstructionPoly& base_instruction);

/**
 * @brief This takes the provided seed state for the base_instruction and create a vector of move instruction
 * @details This skips the first state
 * @param joint_names The joint names associated with the states
 * @param states The joint states to populate the composite instruction with
 * @param base_instruction The base instruction used to extract profile and manipulator information from
 * @return A vector of move instruction
 */
std::vector<MoveInstructionPoly> getInterpolatedInstructions(const tesseract_common::VectorIsometry3d& poses,
                                                             const std::vector<std::string>& joint_names,
                                                             const Eigen::MatrixXd& states,
                                                             const MoveInstructionPoly& base_instruction);

/**
 * @brief Find the closest joint solution for p to the provided seed
 * @param info The instruction info to find closest joint solution
 * @param seed The seed to find the closest solution
 * @return The closest solution to the seed. This will be empty if a solution was not found during inverse kinematics
 */
Eigen::VectorXd getClosestJointSolution(const KinematicGroupInstructionInfo& info, const Eigen::VectorXd& seed);

/**
 * @brief Find the closest joint solution for the two provided cartesian poses.
 * @param info1 The instruction info to find closest joint solution
 * @param info2 The instruction info to find closest joint solution
 * @param seed The seed to use during inverse kinematics
 * @return The closest joint solution for the provided cartesian positions. If either are empty then it failed to solve
 * inverse kinematics.
 */
std::array<Eigen::VectorXd, 2> getClosestJointSolution(const KinematicGroupInstructionInfo& info1,
                                                       const KinematicGroupInstructionInfo& info2,
                                                       const Eigen::VectorXd& seed);

/** @brief Provided for backwards compatibility */
CompositeInstruction generateInterpolatedProgram(const CompositeInstruction& instructions,
                                                 const tesseract_scene_graph::SceneState& current_state,
                                                 const tesseract_environment::Environment::ConstPtr& env,
                                                 double state_longest_valid_segment_length = 5 * M_PI / 180,
                                                 double translation_longest_valid_segment_length = 0.15,
                                                 double rotation_longest_valid_segment_length = 5 * M_PI / 180,
                                                 int min_steps = 1);

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_INTERPOLATION_H
