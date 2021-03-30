/**
 * @file simple_planner_default_lvs_plan_profile.cpp
 * @brief
 *
 * @author Tyler Marr
 * @date Septemeber 16, 2020
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

#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
SimplePlannerLVSPlanProfile::SimplePlannerLVSPlanProfile(double state_longest_valid_segment_length,
                                                         double translation_longest_valid_segment_length,
                                                         double rotation_longest_valid_segment_length,
                                                         int min_steps)
  : state_longest_valid_segment_length(state_longest_valid_segment_length)
  , translation_longest_valid_segment_length(translation_longest_valid_segment_length)
  , rotation_longest_valid_segment_length(rotation_longest_valid_segment_length)
  , min_steps(min_steps)
{
}

CompositeInstruction SimplePlannerLVSPlanProfile::generate(const PlanInstruction& prev_instruction,
                                                           const PlanInstruction& base_instruction,
                                                           const PlannerRequest& request,
                                                           const ManipulatorInfo& global_manip_info) const
{
  InstructionInfo info1(prev_instruction, request, global_manip_info);
  InstructionInfo info2(base_instruction, request, global_manip_info);

  if (!info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return stateJointJointWaypoint(info1, info2);

  if (!info1.has_cartesian_waypoint && info2.has_cartesian_waypoint)
    return stateJointCartWaypoint(info1, info2);

  if (info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return stateCartJointWaypoint(info1, info2);

  return stateCartCartWaypoint(info1, info2, request);
}

CompositeInstruction SimplePlannerLVSPlanProfile::stateJointJointWaypoint(const InstructionInfo& prev,
                                                                          const InstructionInfo& base) const
{
  // Calculate FK for start and end
  const Eigen::VectorXd& j1 = prev.extractJointPosition();
  Eigen::Isometry3d p1_world = prev.calcCartesianWorldPose(j1);

  const Eigen::VectorXd& j2 = base.extractJointPosition();
  Eigen::Isometry3d p2_world = base.calcCartesianWorldPose(j2);

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  double joint_dist = (j2 - j1).norm();

  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int joint_steps = int(joint_dist / state_longest_valid_segment_length) + 1;

  int steps = std::max(trans_steps, rot_steps);
  steps = std::max(steps, joint_steps);
  steps = std::max(steps, min_steps);

  // Linearly interpolate in joint space
  Eigen::MatrixXd states = interpolate(j1, j2, steps);
  return getInterpolatedComposite(base.fwd_kin->getJointNames(), states, base.instruction);
}

CompositeInstruction SimplePlannerLVSPlanProfile::stateJointCartWaypoint(const InstructionInfo& prev,
                                                                         const InstructionInfo& base) const
{
  // Calculate FK for start
  const Eigen::VectorXd& j1 = prev.extractJointPosition();
  Eigen::Isometry3d p1_world = prev.calcCartesianWorldPose(j1);

  // Calculate p2 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p2_world = base.extractCartesianWorldPose();
  Eigen::Isometry3d p2 = base.calcCartesianLocalPose(p2_world);

  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  Eigen::VectorXd j2_final = getClosestJointSolution(p2, base.inv_kin, j1);
  if (j2_final.size() != 0)
  {
    double joint_dist = (j2_final - j1).norm();
    int state_steps = int(joint_dist / state_longest_valid_segment_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Linearly interpolate in joint space
    Eigen::MatrixXd states = interpolate(j1, j2_final, steps);
    return getInterpolatedComposite(base.fwd_kin->getJointNames(), states, base.instruction);
  }

  // Check min steps requirement
  steps = std::max(steps, min_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j1.replicate(1, steps + 1);
  return getInterpolatedComposite(base.fwd_kin->getJointNames(), states, base.instruction);
}

CompositeInstruction SimplePlannerLVSPlanProfile::stateCartJointWaypoint(const InstructionInfo& prev,
                                                                         const InstructionInfo& base) const
{
  // Calculate FK for end
  const Eigen::VectorXd& j2 = base.extractJointPosition();
  Eigen::Isometry3d p2_world = base.calcCartesianWorldPose(j2);

  // Calculate p1 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p1_world = prev.extractCartesianWorldPose();
  Eigen::Isometry3d p1 = prev.calcCartesianLocalPose(p1_world);

  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  Eigen::VectorXd j1_final = getClosestJointSolution(p1, prev.inv_kin, j2);
  if (j1_final.size() != 0)
  {
    double joint_dist = (j2 - j1_final).norm();
    int state_steps = int(joint_dist / state_longest_valid_segment_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Linearly interpolate in joint space
    Eigen::MatrixXd states = interpolate(j1_final, j2, steps);
    return getInterpolatedComposite(base.fwd_kin->getJointNames(), states, base.instruction);
  }

  // Check min steps requirement
  steps = std::max(steps, min_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j2.replicate(1, steps + 1);
  return getInterpolatedComposite(base.fwd_kin->getJointNames(), states, base.instruction);
}

CompositeInstruction SimplePlannerLVSPlanProfile::stateCartCartWaypoint(const InstructionInfo& prev,
                                                                        const InstructionInfo& base,
                                                                        const PlannerRequest& request) const
{
  // Get IK seed
  Eigen::VectorXd seed = request.env_state->getJointValues(base.inv_kin->getJointNames());
  tesseract_common::enforcePositionLimits(seed, base.fwd_kin->getLimits().joint_limits);

  // Calculate IK for start and end
  Eigen::Isometry3d p1_world = prev.extractCartesianWorldPose();
  Eigen::Isometry3d p1 = prev.calcCartesianLocalPose(p1_world);

  Eigen::Isometry3d p2_world = base.extractCartesianWorldPose();
  Eigen::Isometry3d p2 = base.calcCartesianLocalPose(p2_world);

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  std::array<Eigen::VectorXd, 2> sol = getClosestJointSolution(p1, p2, prev.inv_kin, base.inv_kin, seed);

  Eigen::MatrixXd states;
  if (sol[0].size() != 0 && sol[1].size() != 0)
  {
    double joint_dist = (sol[1] - sol[0]).norm();
    int state_steps = int(joint_dist / state_longest_valid_segment_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Interpolate
    states = interpolate(sol[0], sol[1], steps);
  }
  else if (sol[0].size() != 0)
  {
    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Interpolate
    states = sol[0].replicate(1, steps + 1);
  }
  else if (sol[1].size() != 0)
  {
    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Interpolate
    states = sol[1].replicate(1, steps + 1);
  }
  else
  {
    // Check min steps requirement
    steps = std::max(steps, min_steps);

    states = seed.replicate(1, steps + 1);
  }

  // Convert to MoveInstructions
  return getInterpolatedComposite(base.fwd_kin->getJointNames(), states, base.instruction);
}
}  // namespace tesseract_planning
