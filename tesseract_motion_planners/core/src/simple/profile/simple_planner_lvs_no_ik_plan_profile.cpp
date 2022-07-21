/**
 * @file simple_planner_lvs_no_ik_plan_profile.cpp
 * @brief This does not use inverse kinematics
 *
 * @author Levi Armstrong
 * @date November 8, 2021
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

#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
SimplePlannerLVSNoIKPlanProfile::SimplePlannerLVSNoIKPlanProfile(double state_longest_valid_segment_length,
                                                                 double translation_longest_valid_segment_length,
                                                                 double rotation_longest_valid_segment_length,
                                                                 int min_steps,
                                                                 int max_steps)
  : state_longest_valid_segment_length(state_longest_valid_segment_length)
  , translation_longest_valid_segment_length(translation_longest_valid_segment_length)
  , rotation_longest_valid_segment_length(rotation_longest_valid_segment_length)
  , min_steps(min_steps)
  , max_steps(max_steps)
{
}

CompositeInstruction
SimplePlannerLVSNoIKPlanProfile::generate(const MoveInstructionPoly& prev_instruction,
                                          const MoveInstructionPoly& /*prev_seed*/,
                                          const MoveInstructionPoly& base_instruction,
                                          const InstructionPoly& /*next_instruction*/,
                                          const PlannerRequest& request,
                                          const tesseract_common::ManipulatorInfo& global_manip_info) const
{
  JointGroupInstructionInfo info1(prev_instruction, request, global_manip_info);
  JointGroupInstructionInfo info2(base_instruction, request, global_manip_info);

  if (!info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return stateJointJointWaypoint(info1, info2);

  if (!info1.has_cartesian_waypoint && info2.has_cartesian_waypoint)
    return stateJointCartWaypoint(info1, info2);

  if (info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return stateCartJointWaypoint(info1, info2);

  return stateCartCartWaypoint(info1, info2, request);
}

CompositeInstruction
SimplePlannerLVSNoIKPlanProfile::stateJointJointWaypoint(const JointGroupInstructionInfo& prev,
                                                         const JointGroupInstructionInfo& base) const
{
  // Calculate FK for start and end
  const Eigen::VectorXd& j1 = prev.extractJointPosition();
  Eigen::Isometry3d p1_world = prev.calcCartesianPose(j1);

  const Eigen::VectorXd& j2 = base.extractJointPosition();
  Eigen::Isometry3d p2_world = base.calcCartesianPose(j2);

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  double joint_dist = (j2 - j1).norm();

  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int joint_steps = int(joint_dist / state_longest_valid_segment_length) + 1;

  int steps = std::max(trans_steps, rot_steps);
  steps = std::max(steps, joint_steps);
  steps = std::max(steps, min_steps);
  steps = std::min(steps, max_steps);

  // Linearly interpolate in joint space
  Eigen::MatrixXd states = interpolate(j1, j2, steps);
  return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
}

CompositeInstruction
SimplePlannerLVSNoIKPlanProfile::stateJointCartWaypoint(const JointGroupInstructionInfo& prev,
                                                        const JointGroupInstructionInfo& base) const
{
  // Calculate FK for start
  const Eigen::VectorXd& j1 = prev.extractJointPosition();
  Eigen::Isometry3d p1_world = prev.calcCartesianPose(j1);

  // Calculate p2 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p2_world = base.extractCartesianPose();

  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  // Check min steps requirement
  steps = std::max(steps, min_steps);
  steps = std::min(steps, max_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j1.replicate(1, steps + 1);
  return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
}

CompositeInstruction
SimplePlannerLVSNoIKPlanProfile::stateCartJointWaypoint(const JointGroupInstructionInfo& prev,
                                                        const JointGroupInstructionInfo& base) const
{
  // Calculate FK for end
  const Eigen::VectorXd& j2 = base.extractJointPosition();
  Eigen::Isometry3d p2_world = base.calcCartesianPose(j2);

  // Calculate p1 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p1_world = prev.extractCartesianPose();

  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  // Check min steps requirement
  steps = std::max(steps, min_steps);
  steps = std::min(steps, max_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j2.replicate(1, steps + 1);
  return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
}

CompositeInstruction SimplePlannerLVSNoIKPlanProfile::stateCartCartWaypoint(const JointGroupInstructionInfo& prev,
                                                                            const JointGroupInstructionInfo& base,
                                                                            const PlannerRequest& request) const
{
  // Get IK seed
  Eigen::VectorXd seed = request.env_state.getJointValues(base.manip->getJointNames());
  tesseract_common::enforcePositionLimits<double>(seed, base.manip->getLimits().joint_limits);

  // Calculate IK for start and end
  Eigen::Isometry3d p1_world = prev.extractCartesianPose();
  Eigen::Isometry3d p2_world = base.extractCartesianPose();

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  // Check min steps requirement
  steps = std::max(steps, min_steps);
  steps = std::min(steps, max_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = seed.replicate(1, steps + 1);
  return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
}
}  // namespace tesseract_planning
