/**
 * @file lvs_interpolation.h
 * @brief
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date July 23, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <algorithm>  // std::max
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/simple/step_generators/lvs_interpolation.h>

namespace tesseract_planning
{
LVSTransitionInfo::LVSTransitionInfo(const InstructionInfo& prev,
                                     const InstructionInfo& base,
                                     const PlannerRequest& request)
  : prev(prev), base(base), request(request)
{
}

CompositeInstruction simplePlannerGeneratorLVS(const PlanInstruction& prev_instruction,
                                               const PlanInstruction& base_instruction,
                                               const PlannerRequest& request,
                                               const ManipulatorInfo& manip_info,
                                               double state_longest_valid_segment_length,
                                               double translation_longest_valid_segment_length,
                                               double rotation_longest_valid_segment_length,
                                               int min_steps)
{
  InstructionInfo info1(prev_instruction, request, manip_info);
  InstructionInfo info2(base_instruction, request, manip_info);

  LVSTransitionInfo trans_info(info1, info2, request);
  trans_info.state_longest_valid_segment_length = state_longest_valid_segment_length;
  trans_info.translation_longest_valid_segment_length = translation_longest_valid_segment_length;
  trans_info.rotation_longest_valid_segment_length = rotation_longest_valid_segment_length;
  trans_info.min_steps = min_steps;

  if (!info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return stateJointJointWaypointLVS(trans_info);

  if (!info1.has_cartesian_waypoint && info2.has_cartesian_waypoint)
    return stateJointCartWaypointLVS(trans_info);

  if (info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return stateCartJointWaypointLVS(trans_info);

  return stateCartCartWaypointLVS(trans_info);
}

CompositeInstruction stateJointJointWaypointLVS(const LVSTransitionInfo& trans_info)
{
  // Calculate FK for start and end
  const Eigen::VectorXd& j1 = getJointPosition(trans_info.prev.instruction.getWaypoint());
  Eigen::Isometry3d p1_world = trans_info.prev.calcCartesianWorldPose(j1);

  const Eigen::VectorXd& j2 = getJointPosition(trans_info.base.instruction.getWaypoint());
  Eigen::Isometry3d p2_world = trans_info.base.calcCartesianWorldPose(j2);

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  double joint_dist = (j2 - j1).norm();

  int trans_steps = int(trans_dist / trans_info.translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / trans_info.rotation_longest_valid_segment_length) + 1;
  int joint_steps = int(joint_dist / trans_info.state_longest_valid_segment_length) + 1;

  int steps = std::max(trans_steps, rot_steps);
  steps = std::max(steps, joint_steps);
  steps = std::max(steps, trans_info.min_steps);

  // Linearly interpolate in joint space
  Eigen::MatrixXd states = interpolate(j1, j2, steps);
  return getInterpolatedComposite(states, trans_info.base.fwd_kin, trans_info.base.instruction);
}

CompositeInstruction stateJointCartWaypointLVS(const LVSTransitionInfo& trans_info)
{
  // Calculate FK for start
  const Eigen::VectorXd& j1 = getJointPosition(trans_info.prev.instruction.getWaypoint());
  Eigen::Isometry3d p1_world = trans_info.prev.calcCartesianWorldPose(j1);

  // Calculate p2 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p2_world = trans_info.base.extractCartesianWorldPose();
  Eigen::Isometry3d p2 = trans_info.base.calcCartesianLocalPose(p2_world);

  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / trans_info.translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / trans_info.rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  Eigen::VectorXd j2_final = getClosestJointSolution(p2, trans_info.base.inv_kin, j1);
  if (j2_final.size() != 0)
  {
    double joint_dist = (j2_final - j1).norm();
    int state_steps = int(joint_dist / trans_info.state_longest_valid_segment_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, trans_info.min_steps);

    // Linearly interpolate in joint space
    Eigen::MatrixXd states = interpolate(j1, j2_final, steps);
    return getInterpolatedComposite(states, trans_info.base.fwd_kin, trans_info.base.instruction);
  }

  // Check min steps requirement
  steps = std::max(steps, trans_info.min_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j1.replicate(1, steps + 1);
  return getInterpolatedComposite(states, trans_info.base.fwd_kin, trans_info.base.instruction);
}

CompositeInstruction stateCartJointWaypointLVS(const LVSTransitionInfo& trans_info)
{
  // Calculate FK for end
  const Eigen::VectorXd& j2 = getJointPosition(trans_info.base.instruction.getWaypoint());
  Eigen::Isometry3d p2_world = trans_info.base.calcCartesianWorldPose(j2);

  // Calculate p1 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p1_world = trans_info.prev.extractCartesianWorldPose();
  Eigen::Isometry3d p1 = trans_info.prev.calcCartesianLocalPose(p1_world);

  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / trans_info.translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / trans_info.rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  Eigen::VectorXd j1_final = getClosestJointSolution(p1, trans_info.prev.inv_kin, j2);
  if (j1_final.size() != 0)
  {
    double joint_dist = (j2 - j1_final).norm();
    int state_steps = int(joint_dist / trans_info.state_longest_valid_segment_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, trans_info.min_steps);

    // Linearly interpolate in joint space
    Eigen::MatrixXd states = interpolate(j1_final, j2, steps);
    return getInterpolatedComposite(states, trans_info.base.fwd_kin, trans_info.base.instruction);
  }

  // Check min steps requirement
  steps = std::max(steps, trans_info.min_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j2.replicate(1, steps + 1);
  return getInterpolatedComposite(states, trans_info.base.fwd_kin, trans_info.base.instruction);
}

CompositeInstruction stateCartCartWaypointLVS(const LVSTransitionInfo& trans_info)
{
  // Get IK seed
  Eigen::VectorXd seed = trans_info.request.env_state->getJointValues(trans_info.base.inv_kin->getJointNames());

  // Calculate IK for start and end
  Eigen::Isometry3d p1_world = trans_info.prev.extractCartesianWorldPose();
  Eigen::Isometry3d p1 = trans_info.prev.calcCartesianLocalPose(p1_world);

  Eigen::Isometry3d p2_world = trans_info.base.extractCartesianWorldPose();
  Eigen::Isometry3d p2 = trans_info.base.calcCartesianLocalPose(p2_world);

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / trans_info.translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / trans_info.rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  std::array<Eigen::VectorXd, 2> sol =
      getClosestJointSolution(p1, p2, trans_info.prev.inv_kin, trans_info.base.inv_kin, seed);

  Eigen::MatrixXd states;
  if (sol[0].size() != 0 && sol[1].size() != 0)
  {
    double joint_dist = (sol[1] - sol[0]).norm();
    int state_steps = int(joint_dist / trans_info.state_longest_valid_segment_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, trans_info.min_steps);

    // Interpolate
    states = interpolate(sol[0], sol[1], steps);
  }
  else if (sol[0].size() != 0)
  {
    // Check min steps requirement
    steps = std::max(steps, trans_info.min_steps);

    // Interpolate
    states = sol[0].replicate(1, steps + 1);
  }
  else if (sol[1].size() != 0)
  {
    // Check min steps requirement
    steps = std::max(steps, trans_info.min_steps);

    // Interpolate
    states = sol[1].replicate(1, steps + 1);
  }
  else
  {
    // Check min steps requirement
    steps = std::max(steps, trans_info.min_steps);

    states = seed.replicate(1, steps + 1);
  }

  // Convert to MoveInstructions
  return getInterpolatedComposite(states, trans_info.base.fwd_kin, trans_info.base.instruction);
}

}  // namespace tesseract_planning
