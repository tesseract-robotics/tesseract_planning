/**
 * @file simple_planner_default_move_profile.cpp
 * @brief
 *
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

#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_move_profile.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/kinematic_limits.h>

#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>

namespace tesseract_planning
{
SimplePlannerFixedSizeAssignMoveProfile::SimplePlannerFixedSizeAssignMoveProfile(int freespace_steps, int linear_steps)
  : freespace_steps(freespace_steps), linear_steps(linear_steps)
{
}

std::vector<MoveInstructionPoly>
SimplePlannerFixedSizeAssignMoveProfile::generate(const MoveInstructionPoly& prev_instruction,
                                                  const MoveInstructionPoly& /*prev_seed*/,
                                                  const MoveInstructionPoly& base_instruction,
                                                  const InstructionPoly& /*next_instruction*/,
                                                  const std::shared_ptr<const tesseract_environment::Environment>& env,
                                                  const tesseract_common::ManipulatorInfo& global_manip_info) const
{
  KinematicGroupInstructionInfo info1(prev_instruction, *env, global_manip_info);
  KinematicGroupInstructionInfo info2(base_instruction, *env, global_manip_info);

  Eigen::MatrixXd states;
  if (!info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
  {
    const Eigen::VectorXd& jp = info2.extractJointPosition();
    if (info2.instruction.isLinear())
      states = jp.replicate(1, linear_steps + 1);
    else if (info2.instruction.isFreespace())
      states = jp.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");
  }
  else if (!info1.has_cartesian_waypoint && info2.has_cartesian_waypoint)
  {
    const Eigen::VectorXd& jp = info1.extractJointPosition();
    if (info2.instruction.isLinear())
      states = jp.replicate(1, linear_steps + 1);
    else if (info2.instruction.isFreespace())
      states = jp.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");
  }
  else if (info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
  {
    const Eigen::VectorXd& jp = info2.extractJointPosition();
    if (info2.instruction.isLinear())
      states = jp.replicate(1, linear_steps + 1);
    else if (info2.instruction.isFreespace())
      states = jp.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");
  }
  else
  {
    Eigen::VectorXd seed = env->getCurrentJointValues(info2.manip->getJointNames());
    tesseract_common::enforceLimits<double>(seed, info2.manip->getLimits().joint_limits);

    if (info2.instruction.isLinear())
      states = seed.replicate(1, linear_steps + 1);
    else if (info2.instruction.isFreespace())
      states = seed.replicate(1, freespace_steps + 1);
    else
      throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");
  }

  // Linearly interpolate in cartesian space if linear move
  if (base_instruction.isLinear())
  {
    Eigen::Isometry3d p1_world;
    if (info1.has_cartesian_waypoint)
      p1_world = info1.extractCartesianPose();
    else
      p1_world = info1.calcCartesianPose(info1.extractJointPosition());

    Eigen::Isometry3d p2_world;
    if (info2.has_cartesian_waypoint)
      p2_world = info2.extractCartesianPose();
    else
      p2_world = info2.calcCartesianPose(info2.extractJointPosition());

    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, linear_steps);
    for (auto& pose : poses)
      pose = info2.working_frame_transform.inverse() * pose;

    assert(static_cast<Eigen::Index>(poses.size()) == states.cols());
    return getInterpolatedInstructions(poses, info2.manip->getJointNames(), states, info2.instruction);
  }

  return getInterpolatedInstructions(info2.manip->getJointNames(), states, info2.instruction);
}

template <class Archive>
void SimplePlannerFixedSizeAssignMoveProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(SimplePlannerMoveProfile);
  ar& BOOST_SERIALIZATION_NVP(freespace_steps);
  ar& BOOST_SERIALIZATION_NVP(linear_steps);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::SimplePlannerFixedSizeAssignMoveProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::SimplePlannerFixedSizeAssignMoveProfile)
