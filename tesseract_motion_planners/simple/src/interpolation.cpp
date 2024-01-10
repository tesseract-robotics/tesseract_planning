/**
 * @file interpolation.cpp
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

#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_planning
{
JointGroupInstructionInfo::JointGroupInstructionInfo(const MoveInstructionPoly& plan_instruction,
                                                     const PlannerRequest& request,
                                                     const tesseract_common::ManipulatorInfo& manip_info)
  : instruction(plan_instruction)
{
  assert(!(manip_info.empty() && plan_instruction.getManipulatorInfo().empty()));
  tesseract_common::ManipulatorInfo mi = manip_info.getCombined(plan_instruction.getManipulatorInfo());

  // Check required manipulator information
  if (mi.manipulator.empty())
    throw std::runtime_error("InstructionInfo, manipulator is empty!");

  if (mi.tcp_frame.empty())
    throw std::runtime_error("InstructionInfo, TCP frame is empty!");

  if (mi.working_frame.empty())
    throw std::runtime_error("InstructionInfo, working frame is empty!");

  // Get Previous Instruction Kinematics
  manip = request.env->getJointGroup(mi.manipulator);

  // Get Previous Instruction TCP and Working Frame
  working_frame = mi.working_frame;
  working_frame_transform = request.env_state.link_transforms.at(working_frame);
  tcp_frame = mi.tcp_frame;
  tcp_offset = request.env->findTCPOffset(mi);

  // Get Previous Instruction Waypoint Info
  if (plan_instruction.getWaypoint().isStateWaypoint() || plan_instruction.getWaypoint().isJointWaypoint())
    has_cartesian_waypoint = false;
  else if (plan_instruction.getWaypoint().isCartesianWaypoint())
    has_cartesian_waypoint = true;
  else
    throw std::runtime_error("Simple planner currently only supports State, Joint and Cartesian Waypoint types!");
}

Eigen::Isometry3d JointGroupInstructionInfo::calcCartesianPose(const Eigen::VectorXd& jp, bool in_world) const
{
  tesseract_common::TransformMap transforms = manip->calcFwdKin(jp);

  if (in_world)
    return transforms[tcp_frame] * tcp_offset;

  return working_frame_transform.inverse() * (transforms[tcp_frame] * tcp_offset);
}

Eigen::Isometry3d JointGroupInstructionInfo::extractCartesianPose(bool in_world) const
{
  if (!instruction.getWaypoint().isCartesianWaypoint())
    throw std::runtime_error("Instruction waypoint type is not a CartesianWaypoint, unable to extract cartesian pose!");

  if (in_world)
    return working_frame_transform * instruction.getWaypoint().as<CartesianWaypointPoly>().getTransform();

  return instruction.getWaypoint().as<CartesianWaypointPoly>().getTransform();
}

const Eigen::VectorXd& JointGroupInstructionInfo::extractJointPosition() const
{
  return getJointPosition(instruction.getWaypoint());
}

KinematicGroupInstructionInfo::KinematicGroupInstructionInfo(const MoveInstructionPoly& plan_instruction,
                                                             const PlannerRequest& request,
                                                             const tesseract_common::ManipulatorInfo& manip_info)
  : instruction(plan_instruction)
{
  assert(!(manip_info.empty() && plan_instruction.getManipulatorInfo().empty()));
  tesseract_common::ManipulatorInfo mi = manip_info.getCombined(plan_instruction.getManipulatorInfo());

  // Check required manipulator information
  if (mi.manipulator.empty())
    throw std::runtime_error("InstructionInfo, manipulator is empty!");

  if (mi.tcp_frame.empty())
    throw std::runtime_error("InstructionInfo, TCP frame is empty!");

  if (mi.working_frame.empty())
    throw std::runtime_error("InstructionInfo, working frame is empty!");

  // Get Previous Instruction Kinematics
  manip = request.env->getKinematicGroup(mi.manipulator, mi.manipulator_ik_solver);

  // Get Previous Instruction TCP and Working Frame
  working_frame = mi.working_frame;
  working_frame_transform = request.env_state.link_transforms.at(working_frame);
  tcp_frame = mi.tcp_frame;
  tcp_offset = request.env->findTCPOffset(mi);

  // Get Previous Instruction Waypoint Info
  if (plan_instruction.getWaypoint().isStateWaypoint() || plan_instruction.getWaypoint().isJointWaypoint())
    has_cartesian_waypoint = false;
  else if (plan_instruction.getWaypoint().isCartesianWaypoint())
    has_cartesian_waypoint = true;
  else
    throw std::runtime_error("Simple planner currently only supports State, Joint and Cartesian Waypoint types!");
}

Eigen::Isometry3d KinematicGroupInstructionInfo::calcCartesianPose(const Eigen::VectorXd& jp, bool in_world) const
{
  tesseract_common::TransformMap transforms = manip->calcFwdKin(jp);

  if (in_world)
    return transforms[tcp_frame] * tcp_offset;

  return working_frame_transform.inverse() * (transforms[tcp_frame] * tcp_offset);
}

Eigen::Isometry3d KinematicGroupInstructionInfo::extractCartesianPose(bool in_world) const
{
  if (!instruction.getWaypoint().isCartesianWaypoint())
    throw std::runtime_error("Instruction waypoint type is not a CartesianWaypoint, unable to extract cartesian pose!");

  if (in_world)
    return working_frame_transform * instruction.getWaypoint().as<CartesianWaypointPoly>().getTransform();

  return instruction.getWaypoint().as<CartesianWaypointPoly>().getTransform();
}

const Eigen::VectorXd& KinematicGroupInstructionInfo::extractJointPosition() const
{
  return getJointPosition(instruction.getWaypoint());
}

std::vector<MoveInstructionPoly> interpolateJointJointWaypoint(const KinematicGroupInstructionInfo& prev,
                                                               const KinematicGroupInstructionInfo& base,
                                                               int linear_steps,
                                                               int freespace_steps)
{
  // Calculate FK for start and end
  const Eigen::VectorXd& j1 = prev.extractJointPosition();
  const Eigen::VectorXd& j2 = base.extractJointPosition();

  Eigen::MatrixXd states;
  if (base.instruction.isLinear())
  {
    if (linear_steps > 1)
      states = interpolate(j1, j2, linear_steps);
    else
      states = j2.replicate(1, 2);
  }
  else if (base.instruction.isFreespace())
  {
    if (freespace_steps > 1)
      states = interpolate(j1, j2, freespace_steps);
    else
      states = j2.replicate(1, 2);
  }
  else
  {
    throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");
  }

  // Linearly interpolate in cartesian space if linear move
  if (base.instruction.isLinear())
  {
    Eigen::Isometry3d p1_world = prev.calcCartesianPose(j1);
    Eigen::Isometry3d p2_world = base.calcCartesianPose(j2);

    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, linear_steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

std::vector<MoveInstructionPoly> interpolateJointCartWaypoint(const KinematicGroupInstructionInfo& prev,
                                                              const KinematicGroupInstructionInfo& base,
                                                              int linear_steps,
                                                              int freespace_steps)
{
  const Eigen::VectorXd& j1 = prev.extractJointPosition();

  // Check if the cartesian has a seed and if so use it
  const auto& base_cwp = base.instruction.getWaypoint().as<CartesianWaypointPoly>();
  Eigen::VectorXd j2;
  if (base_cwp.hasSeed())
    j2 = base_cwp.getSeed().position;
  else
    j2 = getClosestJointSolution(base, j1);

  Eigen::MatrixXd states;
  if (j2.size() == 0)
  {
    if (base.instruction.isLinear())
      states = j1.replicate(1, linear_steps + 1);
    else if (base.instruction.isFreespace())
      states = j1.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointCartWaypointFixedSize: Unsupported MoveInstructionType!");
  }
  else
  {
    if (base.instruction.isLinear())
    {
      if (linear_steps > 1)
        states = interpolate(j1, j2, linear_steps);
      else
        states = j2.replicate(1, 2);
    }
    else if (base.instruction.isFreespace())
    {
      if (freespace_steps > 1)
        states = interpolate(j1, j2, freespace_steps);
      else
        states = j2.replicate(1, 2);
    }
    else
    {
      throw std::runtime_error("stateJointCartWaypointFixedSize: Unsupported MoveInstructionType!");
    }
  }

  // Linearly interpolate in cartesian space if linear move
  if (base.instruction.isLinear())
  {
    Eigen::Isometry3d p1_world = prev.calcCartesianPose(j1);
    Eigen::Isometry3d p2_world = base.extractCartesianPose();

    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, linear_steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

std::vector<MoveInstructionPoly> interpolateCartJointWaypoint(const KinematicGroupInstructionInfo& prev,
                                                              const KinematicGroupInstructionInfo& base,
                                                              int linear_steps,
                                                              int freespace_steps)
{
  const Eigen::VectorXd& j2 = base.extractJointPosition();

  // Check if the cartesian has a seed and if so use it
  const auto& prev_cwp = prev.instruction.getWaypoint().as<CartesianWaypointPoly>();
  Eigen::VectorXd j1;
  if (prev_cwp.hasSeed())
    j1 = prev_cwp.getSeed().position;
  else
    j1 = getClosestJointSolution(prev, j2);

  Eigen::MatrixXd states;
  if (j1.size() == 0)
  {
    if (base.instruction.isLinear())
      states = j2.replicate(1, linear_steps + 1);
    else if (base.instruction.isFreespace())
      states = j2.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointCartWaypointFixedSize: Unsupported MoveInstructionType!");
  }
  else
  {
    if (base.instruction.isLinear())
    {
      if (linear_steps > 1)
        states = interpolate(j1, j2, linear_steps);
      else
        states = j2.replicate(1, 2);
    }
    else if (base.instruction.isFreespace())
    {
      if (freespace_steps > 1)
        states = interpolate(j1, j2, freespace_steps);
      else
        states = j2.replicate(1, 2);
    }
    else
    {
      throw std::runtime_error("stateJointCartWaypointFixedSize: Unsupported MoveInstructionType!");
    }
  }

  // Linearly interpolate in cartesian space if linear move
  if (base.instruction.isLinear())
  {
    Eigen::Isometry3d p1_world = prev.extractCartesianPose();
    Eigen::Isometry3d p2_world = base.calcCartesianPose(j2);
    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, linear_steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

std::vector<MoveInstructionPoly> interpolateCartCartWaypoint(const KinematicGroupInstructionInfo& prev,
                                                             const KinematicGroupInstructionInfo& base,
                                                             int linear_steps,
                                                             int freespace_steps,
                                                             const tesseract_scene_graph::SceneState& scene_state)
{
  // Get IK seed
  Eigen::VectorXd seed = scene_state.getJointValues(base.manip->getJointNames());
  tesseract_common::enforcePositionLimits<double>(seed, base.manip->getLimits().joint_limits);

  std::array<Eigen::VectorXd, 2> sol;
  const auto& base_cwp = base.instruction.getWaypoint().as<CartesianWaypointPoly>();
  const auto& prev_cwp = prev.instruction.getWaypoint().as<CartesianWaypointPoly>();
  const bool base_has_seed = base_cwp.hasSeed();
  const bool prev_has_seed = prev_cwp.hasSeed();

  if (base_has_seed && prev_has_seed)
  {
    sol[0] = prev_cwp.getSeed().position;
    sol[1] = base_cwp.getSeed().position;
  }
  else if (!base_has_seed && prev_has_seed)
  {
    sol[0] = prev_cwp.getSeed().position;
    sol[1] = getClosestJointSolution(base, sol[0]);
  }
  else if (base_has_seed && !prev_has_seed)
  {
    sol[1] = base_cwp.getSeed().position;
    sol[0] = getClosestJointSolution(prev, sol[1]);
  }
  else
  {
    sol = getClosestJointSolution(prev, base, seed);
  }

  Eigen::MatrixXd states;
  if (sol[0].size() != 0 && sol[1].size() != 0)
  {
    if (base.instruction.isLinear())
    {
      if (linear_steps > 1)
        states = interpolate(sol[0], sol[1], linear_steps);
      else
        states = sol[1].replicate(1, 2);
    }
    else if (base.instruction.isFreespace())
    {
      if (freespace_steps > 1)
        states = interpolate(sol[0], sol[1], freespace_steps);
      else
        states = sol[1].replicate(1, 2);
    }
    else
    {
      throw std::runtime_error("SimplePlannerFixedSizePlanProfile: Unsupported MoveInstructionType!");
    }
  }
  else if (sol[0].size() != 0)
  {
    if (base.instruction.isLinear())
      states = sol[0].replicate(1, linear_steps + 1);
    else if (base.instruction.isFreespace())
      states = sol[0].replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("SimplePlannerFixedSizePlanProfile: Unsupported MoveInstructionType!");
  }
  else if (sol[1].size() != 0)
  {
    if (base.instruction.isLinear())
      states = sol[1].replicate(1, linear_steps + 1);
    else if (base.instruction.isFreespace())
      states = sol[1].replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("SimplePlannerFixedSizePlanProfile: Unsupported MoveInstructionType!");
  }
  else
  {
    if (base.instruction.isLinear())
      states = seed.replicate(1, linear_steps + 1);
    else if (base.instruction.isFreespace())
      states = seed.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("SimplePlannerFixedSizePlanProfile: Unsupported MoveInstructionType!");
  }

  // Linearly interpolate in cartesian space if linear move
  if (base.instruction.isLinear())
  {
    Eigen::Isometry3d p1_world = prev.extractCartesianPose();
    Eigen::Isometry3d p2_world = base.extractCartesianPose();
    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, linear_steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  // Convert to MoveInstructions
  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

std::vector<MoveInstructionPoly> interpolateJointJointWaypoint(const KinematicGroupInstructionInfo& prev,
                                                               const KinematicGroupInstructionInfo& base,
                                                               double state_lvs_length,
                                                               double translation_lvs_length,
                                                               double rotation_lvs_length,
                                                               int min_steps,
                                                               int max_steps)
{
  // Calculate FK for start and end
  const Eigen::VectorXd& j1 = prev.extractJointPosition();
  Eigen::Isometry3d p1_world = prev.calcCartesianPose(j1);

  const Eigen::VectorXd& j2 = base.extractJointPosition();
  Eigen::Isometry3d p2_world = base.calcCartesianPose(j2);

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  double joint_dist = (j2 - j1).norm();

  int trans_steps = int(trans_dist / translation_lvs_length) + 1;
  int rot_steps = int(rot_dist / rotation_lvs_length) + 1;
  int joint_steps = int(joint_dist / state_lvs_length) + 1;

  int steps = std::max(trans_steps, rot_steps);
  steps = std::max(steps, joint_steps);
  steps = std::max(steps, min_steps);
  steps = std::min(steps, max_steps);

  return interpolateJointJointWaypoint(prev, base, steps, steps);
}

std::vector<MoveInstructionPoly> interpolateJointCartWaypoint(const KinematicGroupInstructionInfo& prev,
                                                              const KinematicGroupInstructionInfo& base,
                                                              double state_lvs_length,
                                                              double translation_lvs_length,
                                                              double rotation_lvs_length,
                                                              int min_steps,
                                                              int max_steps)
{
  // Calculate FK for start
  const Eigen::VectorXd& j1 = prev.extractJointPosition();
  Eigen::Isometry3d p1_world = prev.calcCartesianPose(j1);

  // Calculate p2 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p2_world = base.extractCartesianPose();

  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_lvs_length) + 1;
  int rot_steps = int(rot_dist / rotation_lvs_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  // Check if the cartesian has a seed and if so use it
  const auto& base_cwp = base.instruction.getWaypoint().as<CartesianWaypointPoly>();
  Eigen::VectorXd j2_final;
  if (base_cwp.hasSeed())
    j2_final = base_cwp.getSeed().position;
  else
    j2_final = getClosestJointSolution(base, j1);

  if (j2_final.size() != 0)
  {
    double joint_dist = (j2_final - j1).norm();
    int state_steps = int(joint_dist / state_lvs_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, min_steps);
    steps = std::min(steps, max_steps);

    // Linearly interpolate in joint space
    Eigen::MatrixXd states = interpolate(j1, j2_final, steps);

    // Linearly interpolate in cartesian space if linear move
    if (base.instruction.isLinear())
    {
      tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, steps);
      for (auto& pose : poses)
        pose = base.working_frame_transform.inverse() * pose;

      assert(poses.size() == states.cols());
      return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
    }

    return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
  }

  // Check min steps requirement
  steps = std::max(steps, min_steps);
  steps = std::min(steps, max_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j1.replicate(1, steps + 1);

  // Linearly interpolate in cartesian space if linear move
  if (base.instruction.isLinear())
  {
    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

std::vector<MoveInstructionPoly> interpolateCartJointWaypoint(const KinematicGroupInstructionInfo& prev,
                                                              const KinematicGroupInstructionInfo& base,
                                                              double state_lvs_length,
                                                              double translation_lvs_length,
                                                              double rotation_lvs_length,
                                                              int min_steps,
                                                              int max_steps)
{
  // Calculate FK for end
  const Eigen::VectorXd& j2 = base.extractJointPosition();
  Eigen::Isometry3d p2_world = base.calcCartesianPose(j2);

  // Calculate p1 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p1_world = prev.extractCartesianPose();

  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_lvs_length) + 1;
  int rot_steps = int(rot_dist / rotation_lvs_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  // Check if the cartesian has a seed and if so use it
  const auto& prev_cwp = prev.instruction.getWaypoint().as<CartesianWaypointPoly>();
  Eigen::VectorXd j1_final;
  if (prev_cwp.hasSeed())
    j1_final = prev_cwp.getSeed().position;
  else
    j1_final = getClosestJointSolution(prev, j2);

  if (j1_final.size() != 0)
  {
    double joint_dist = (j2 - j1_final).norm();
    int state_steps = int(joint_dist / state_lvs_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, min_steps);
    steps = std::min(steps, max_steps);

    // Linearly interpolate in joint space
    Eigen::MatrixXd states = interpolate(j1_final, j2, steps);

    // Linearly interpolate in cartesian space if linear move
    if (base.instruction.isLinear())
    {
      tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, steps);
      for (auto& pose : poses)
        pose = base.working_frame_transform.inverse() * pose;

      assert(poses.size() == states.cols());
      return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
    }

    return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
  }

  // Check min steps requirement
  steps = std::max(steps, min_steps);
  steps = std::min(steps, max_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j2.replicate(1, steps + 1);

  // Linearly interpolate in cartesian space if linear move
  if (base.instruction.isLinear())
  {
    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

std::vector<MoveInstructionPoly> interpolateCartCartWaypoint(const KinematicGroupInstructionInfo& prev,
                                                             const KinematicGroupInstructionInfo& base,
                                                             double state_lvs_length,
                                                             double translation_lvs_length,
                                                             double rotation_lvs_length,
                                                             int min_steps,
                                                             int max_steps,
                                                             const tesseract_scene_graph::SceneState& scene_state)
{
  // Get IK seed
  Eigen::VectorXd seed = scene_state.getJointValues(base.manip->getJointNames());
  tesseract_common::enforcePositionLimits<double>(seed, base.manip->getLimits().joint_limits);

  // Calculate IK for start and end
  Eigen::Isometry3d p1_world = prev.extractCartesianPose();
  Eigen::Isometry3d p2_world = base.extractCartesianPose();

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_lvs_length) + 1;
  int rot_steps = int(rot_dist / rotation_lvs_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  std::array<Eigen::VectorXd, 2> sol;
  const auto& base_cwp = base.instruction.getWaypoint().as<CartesianWaypointPoly>();
  const auto& prev_cwp = prev.instruction.getWaypoint().as<CartesianWaypointPoly>();
  const bool base_has_seed = base_cwp.hasSeed();
  const bool prev_has_seed = prev_cwp.hasSeed();
  if (base_has_seed && prev_has_seed)
  {
    sol[0] = prev_cwp.getSeed().position;
    sol[1] = base_cwp.getSeed().position;
  }
  else if (!base_has_seed && prev_has_seed)
  {
    sol[0] = prev_cwp.getSeed().position;
    sol[1] = getClosestJointSolution(base, sol[0]);
  }
  else if (base_has_seed && !prev_has_seed)
  {
    sol[1] = base_cwp.getSeed().position;
    sol[0] = getClosestJointSolution(prev, sol[1]);
  }
  else
  {
    sol = getClosestJointSolution(prev, base, seed);
  }

  Eigen::MatrixXd states;
  if (sol[0].size() != 0 && sol[1].size() != 0)
  {
    double joint_dist = (sol[1] - sol[0]).norm();
    int state_steps = int(joint_dist / state_lvs_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, min_steps);
    steps = std::min(steps, max_steps);

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
    steps = std::min(steps, max_steps);

    states = seed.replicate(1, steps + 1);
  }

  // Linearly interpolate in cartesian space if linear move
  if (base.instruction.isLinear())
  {
    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  // Convert to MoveInstructions
  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

std::vector<MoveInstructionPoly> interpolateJointJointWaypoint(const JointGroupInstructionInfo& prev,
                                                               const JointGroupInstructionInfo& base,
                                                               double state_lvs_length,
                                                               double translation_lvs_length,
                                                               double rotation_lvs_length,
                                                               int min_steps,
                                                               int max_steps)
{
  // Calculate FK for start and end
  const Eigen::VectorXd& j1 = prev.extractJointPosition();
  Eigen::Isometry3d p1_world = prev.calcCartesianPose(j1);

  const Eigen::VectorXd& j2 = base.extractJointPosition();
  Eigen::Isometry3d p2_world = base.calcCartesianPose(j2);

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  double joint_dist = (j2 - j1).norm();

  int trans_steps = int(trans_dist / translation_lvs_length) + 1;
  int rot_steps = int(rot_dist / rotation_lvs_length) + 1;
  int joint_steps = int(joint_dist / state_lvs_length) + 1;

  int steps = std::max(trans_steps, rot_steps);
  steps = std::max(steps, joint_steps);
  steps = std::max(steps, min_steps);
  steps = std::min(steps, max_steps);

  // Linearly interpolate in joint space
  Eigen::MatrixXd states = interpolate(j1, j2, steps);

  // Linearly interpolate in cartesian space if linear move
  if (base.instruction.isLinear())
  {
    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

std::vector<MoveInstructionPoly> interpolateJointCartWaypoint(const JointGroupInstructionInfo& prev,
                                                              const JointGroupInstructionInfo& base,
                                                              double state_lvs_length,
                                                              double translation_lvs_length,
                                                              double rotation_lvs_length,
                                                              int min_steps,
                                                              int max_steps)
{
  // Calculate FK for start
  const Eigen::VectorXd& j1 = prev.extractJointPosition();
  Eigen::Isometry3d p1_world = prev.calcCartesianPose(j1);

  // Calculate p2 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p2_world = base.extractCartesianPose();

  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_lvs_length) + 1;
  int rot_steps = int(rot_dist / rotation_lvs_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  // Check if the cartesian has a seed and if so use it
  const auto& base_cwp = base.instruction.getWaypoint().as<CartesianWaypointPoly>();
  if (base_cwp.hasSeed())
  {
    Eigen::VectorXd j2 = base_cwp.getSeed().position;
    double joint_dist = (j2 - j1).norm();
    int joint_steps = int(joint_dist / state_lvs_length) + 1;
    steps = std::max(steps, joint_steps);
  }

  // Check min steps requirement
  steps = std::max(steps, min_steps);
  steps = std::min(steps, max_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j1.replicate(1, steps + 1);

  // Linearly interpolate in cartesian space if linear move
  if (base.instruction.isLinear())
  {
    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

std::vector<MoveInstructionPoly> interpolateCartJointWaypoint(const JointGroupInstructionInfo& prev,
                                                              const JointGroupInstructionInfo& base,
                                                              double state_lvs_length,
                                                              double translation_lvs_length,
                                                              double rotation_lvs_length,
                                                              int min_steps,
                                                              int max_steps)
{
  // Calculate FK for end
  const Eigen::VectorXd& j2 = base.extractJointPosition();
  Eigen::Isometry3d p2_world = base.calcCartesianPose(j2);

  // Calculate p1 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p1_world = prev.extractCartesianPose();

  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_lvs_length) + 1;
  int rot_steps = int(rot_dist / rotation_lvs_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  // Check if the cartesian has a seed and if so use it
  const auto& prev_cwp = prev.instruction.getWaypoint().as<CartesianWaypointPoly>();
  if (prev_cwp.hasSeed())
  {
    Eigen::VectorXd j1 = prev_cwp.getSeed().position;
    double joint_dist = (j2 - j1).norm();
    int joint_steps = int(joint_dist / state_lvs_length) + 1;
    steps = std::max(steps, joint_steps);
  }

  // Check min steps requirement
  steps = std::max(steps, min_steps);
  steps = std::min(steps, max_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j2.replicate(1, steps + 1);

  // Linearly interpolate in cartesian space if linear move
  if (base.instruction.isLinear())
  {
    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

std::vector<MoveInstructionPoly> interpolateCartCartWaypoint(const JointGroupInstructionInfo& prev,
                                                             const JointGroupInstructionInfo& base,
                                                             double state_lvs_length,
                                                             double translation_lvs_length,
                                                             double rotation_lvs_length,
                                                             int min_steps,
                                                             int max_steps,
                                                             const tesseract_scene_graph::SceneState& scene_state)
{
  // Get IK seed
  Eigen::VectorXd seed = scene_state.getJointValues(base.manip->getJointNames());
  tesseract_common::enforcePositionLimits<double>(seed, base.manip->getLimits().joint_limits);

  // Calculate IK for start and end
  Eigen::Isometry3d p1_world = prev.extractCartesianPose();
  Eigen::Isometry3d p2_world = base.extractCartesianPose();

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_lvs_length) + 1;
  int rot_steps = int(rot_dist / rotation_lvs_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  // Check if the cartesian has a seed and if so use it
  const auto& prev_cwp = prev.instruction.getWaypoint().as<CartesianWaypointPoly>();
  const auto& base_cwp = base.instruction.getWaypoint().as<CartesianWaypointPoly>();
  if (prev_cwp.hasSeed() && base_cwp.hasSeed())
  {
    Eigen::VectorXd j1 = prev_cwp.getSeed().position;
    Eigen::VectorXd j2 = base_cwp.getSeed().position;
    double joint_dist = (j2 - j1).norm();
    int joint_steps = int(joint_dist / state_lvs_length) + 1;
    steps = std::max(steps, joint_steps);
  }

  // Check min steps requirement
  steps = std::max(steps, min_steps);
  steps = std::min(steps, max_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = seed.replicate(1, steps + 1);

  // Linearly interpolate in cartesian space if linear move
  if (base.instruction.isLinear())
  {
    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

tesseract_common::VectorIsometry3d interpolate(const Eigen::Isometry3d& start,
                                               const Eigen::Isometry3d& stop,
                                               long steps)
{
  // Required position change
  Eigen::Vector3d delta_translation = (stop.translation() - start.translation());
  Eigen::Vector3d start_pos = start.translation();
  Eigen::Affine3d stop_prime = start.inverse() * stop;
  Eigen::AngleAxisd delta_rotation(stop_prime.rotation());

  // Step size
  Eigen::Vector3d step = delta_translation / steps;

  // Orientation interpolation
  Eigen::Quaterniond start_q(start.rotation());
  Eigen::Quaterniond stop_q(stop.rotation());
  double slerp_ratio = 1.0 / static_cast<double>(steps);

  tesseract_common::VectorIsometry3d result;
  Eigen::Vector3d trans;
  Eigen::Quaterniond q;
  Eigen::Isometry3d pose;
  result.reserve(static_cast<size_t>(steps) + 1);
  for (unsigned i = 0; i <= static_cast<unsigned>(steps); ++i)
  {
    trans = start_pos + step * i;
    q = start_q.slerp(slerp_ratio * i, stop_q);
    pose = (Eigen::Translation3d(trans) * q);
    result.push_back(pose);
  }
  return result;
}

Eigen::MatrixXd interpolate(const Eigen::Ref<const Eigen::VectorXd>& start,
                            const Eigen::Ref<const Eigen::VectorXd>& stop,
                            long steps)
{
  assert(start.size() == stop.size());

  Eigen::MatrixXd result(start.size(), steps + 1);

  for (int i = 0; i < start.size(); ++i)
    result.row(i) = Eigen::VectorXd::LinSpaced(steps + 1, start(i), stop(i));

  return result;
}

std::vector<WaypointPoly> interpolate_waypoint(const WaypointPoly& start, const WaypointPoly& stop, long steps)
{
  if (start.isCartesianWaypoint())
  {
    const auto& cwp1 = start.as<CartesianWaypointPoly>();
    const auto& cwp2 = stop.as<CartesianWaypointPoly>();
    tesseract_common::VectorIsometry3d eigen_poses = interpolate(cwp1.getTransform(), cwp2.getTransform(), steps);

    std::vector<WaypointPoly> result;
    result.reserve(eigen_poses.size());
    for (auto& eigen_pose : eigen_poses)
    {
      CartesianWaypointPoly copy(cwp2);
      copy.setTransform(eigen_pose);
      result.emplace_back(copy);
    }

    return result;
  }

  if (start.isJointWaypoint())
  {
    const auto& jwp1 = start.as<JointWaypointPoly>();
    const auto& jwp2 = stop.as<JointWaypointPoly>();

    // TODO: Should check joint names are in the same order
    Eigen::MatrixXd joint_poses = interpolate(jwp1.getPosition(), jwp2.getPosition(), steps);

    std::vector<WaypointPoly> result;
    result.reserve(static_cast<std::size_t>(joint_poses.cols()));
    for (int i = 0; i < joint_poses.cols(); ++i)
    {
      JointWaypointPoly copy(jwp2);
      copy.setPosition(joint_poses.col(i));
      result.emplace_back(copy);
    }

    return result;
  }

  CONSOLE_BRIDGE_logError("Interpolator for Waypoint type %d is currently not support!", start.getType().hash_code());
  return {};
}

std::vector<MoveInstructionPoly> getInterpolatedInstructions(const std::vector<std::string>& joint_names,
                                                             const Eigen::MatrixXd& states,
                                                             const MoveInstructionPoly& base_instruction)
{
  // Convert to MoveInstructions
  std::vector<MoveInstructionPoly> move_instructions;
  for (long i = 1; i < states.cols() - 1; ++i)
  {
    MoveInstructionPoly move_instruction = base_instruction.createChild();
    JointWaypointPoly jwp = move_instruction.createJointWaypoint();
    jwp.setNames(joint_names);
    jwp.setPosition(states.col(i));
    jwp.setIsConstrained(false);
    move_instruction.assignJointWaypoint(jwp);
    if (!base_instruction.getPathProfile().empty())
    {
      move_instruction.setProfile(base_instruction.getPathProfile());
      move_instruction.setPathProfile(base_instruction.getPathProfile());
    }
    move_instructions.push_back(move_instruction);
  }

  MoveInstructionPoly move_instruction{ base_instruction };
  if (base_instruction.getWaypoint().isCartesianWaypoint())
    move_instruction.getWaypoint().as<CartesianWaypointPoly>().setSeed(
        tesseract_common::JointState(joint_names, states.col(states.cols() - 1)));

  move_instructions.push_back(move_instruction);
  return move_instructions;
}

std::vector<MoveInstructionPoly> getInterpolatedInstructions(const tesseract_common::VectorIsometry3d& poses,
                                                             const std::vector<std::string>& joint_names,
                                                             const Eigen::MatrixXd& states,
                                                             const MoveInstructionPoly& base_instruction)
{
  // Convert to MoveInstructions
  std::vector<MoveInstructionPoly> move_instructions;
  if (base_instruction.getWaypoint().isCartesianWaypoint())
  {
    for (long i = 1; i < states.cols() - 1; ++i)
    {
      MoveInstructionPoly move_instruction = base_instruction.createChild();
      move_instruction.getWaypoint().as<CartesianWaypointPoly>().setTransform(poses[static_cast<std::size_t>(i)]);
      move_instruction.getWaypoint().as<CartesianWaypointPoly>().setSeed(
          tesseract_common::JointState(joint_names, states.col(i)));
      if (!base_instruction.getPathProfile().empty())
      {
        move_instruction.setProfile(base_instruction.getPathProfile());
        move_instruction.setPathProfile(base_instruction.getPathProfile());
      }
      move_instructions.push_back(move_instruction);
    }

    MoveInstructionPoly move_instruction = base_instruction;
    move_instruction.getWaypoint().as<CartesianWaypointPoly>().setSeed(
        tesseract_common::JointState(joint_names, states.col(states.cols() - 1)));
    move_instructions.push_back(move_instruction);
  }
  else
  {
    for (long i = 1; i < states.cols() - 1; ++i)
    {
      MoveInstructionPoly move_instruction = base_instruction.createChild();
      CartesianWaypointPoly cwp = move_instruction.createCartesianWaypoint();
      cwp.setTransform(poses[static_cast<std::size_t>(i)]);
      cwp.setSeed(tesseract_common::JointState(joint_names, states.col(i)));
      move_instruction.assignCartesianWaypoint(cwp);
      if (!base_instruction.getPathProfile().empty())
      {
        move_instruction.setProfile(base_instruction.getPathProfile());
        move_instruction.setPathProfile(base_instruction.getPathProfile());
      }
      move_instructions.push_back(move_instruction);
    }

    move_instructions.push_back(base_instruction);
  }

  return move_instructions;
}

Eigen::VectorXd getClosestJointSolution(const KinematicGroupInstructionInfo& info, const Eigen::VectorXd& seed)
{
  auto limits = info.manip->getLimits();
  auto redundancy_indices = info.manip->getRedundancyCapableJointIndices();

  if (!info.has_cartesian_waypoint)
    throw std::runtime_error("Instruction waypoint type is not a CartesianWaypoint, unable to extract cartesian pose!");

  Eigen::Isometry3d cwp =
      info.instruction.getWaypoint().as<CartesianWaypointPoly>().getTransform() * info.tcp_offset.inverse();

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
      if (tesseract_common::satisfiesPositionLimits<double>(solution, limits.joint_limits))
      {
        if (jp_final.rows() == 0)
        {
          jp_final = solution;
          dist = (solution - seed).norm();
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

std::array<Eigen::VectorXd, 2> getClosestJointSolution(const KinematicGroupInstructionInfo& info1,
                                                       const KinematicGroupInstructionInfo& info2,
                                                       const Eigen::VectorXd& seed)
{
  auto manip1_limits = info1.manip->getLimits();
  auto manip1_redundancy_indices = info1.manip->getRedundancyCapableJointIndices();

  auto manip2_limits = info2.manip->getLimits();
  auto manip2_redundancy_indices = info2.manip->getRedundancyCapableJointIndices();

  if (!info1.has_cartesian_waypoint || !info2.has_cartesian_waypoint)
    throw std::runtime_error("Instruction waypoint type is not a CartesianWaypoint, unable to extract cartesian pose!");

  Eigen::Isometry3d cwp1 =
      info1.instruction.getWaypoint().as<CartesianWaypointPoly>().getTransform() * info1.tcp_offset.inverse();
  Eigen::Isometry3d cwp2 =
      info2.instruction.getWaypoint().as<CartesianWaypointPoly>().getTransform() * info2.tcp_offset.inverse();

  std::array<Eigen::VectorXd, 2> results;

  // Calculate IK for start and end
  Eigen::VectorXd j1_final;
  tesseract_kinematics::IKSolutions j1;
  tesseract_kinematics::KinGroupIKInput ik_input1(cwp1, info1.working_frame, info1.tcp_frame);
  tesseract_kinematics::IKSolutions j1_solutions = info1.manip->calcInvKin({ ik_input1 }, seed);
  j1_solutions.erase(std::remove_if(j1_solutions.begin(),
                                    j1_solutions.end(),
                                    [&manip1_limits](const Eigen::VectorXd& solution) {
                                      return !tesseract_common::satisfiesPositionLimits<double>(
                                          solution, manip1_limits.joint_limits);
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
                                      // NOLINTNEXTLINE(clang-analyzer-core.uninitialized.UndefReturn)
                                      return !tesseract_common::satisfiesPositionLimits<double>(
                                          solution, manip2_limits.joint_limits);
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

CompositeInstruction generateInterpolatedProgram(const CompositeInstruction& instructions,
                                                 const tesseract_scene_graph::SceneState& current_state,
                                                 const tesseract_environment::Environment::ConstPtr& env,
                                                 double state_longest_valid_segment_length,
                                                 double translation_longest_valid_segment_length,
                                                 double rotation_longest_valid_segment_length,
                                                 int min_steps)
{
  // Fill out request and response
  PlannerRequest request;
  request.instructions = instructions;
  request.env_state = current_state;
  request.env = env;

  // Set up planner
  SimpleMotionPlanner planner("SimpleMotionPlannerTask");

  auto profile = std::make_shared<SimplePlannerLVSNoIKPlanProfile>(state_longest_valid_segment_length,
                                                                   translation_longest_valid_segment_length,
                                                                   rotation_longest_valid_segment_length,
                                                                   min_steps);

  // Create profile dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<SimplePlannerPlanProfile>(planner.getName(), instructions.getProfile(), profile);
  auto flat = instructions.flatten(&moveFilter);
  for (const auto& i : flat)
    profiles->addProfile<SimplePlannerPlanProfile>(
        planner.getName(), i.get().as<MoveInstructionPoly>().getProfile(), profile);

  // Assign profile dictionary
  request.profiles = profiles;

  // Solve
  PlannerResponse response = planner.solve(request);

  return response.results;
}

}  // namespace tesseract_planning
