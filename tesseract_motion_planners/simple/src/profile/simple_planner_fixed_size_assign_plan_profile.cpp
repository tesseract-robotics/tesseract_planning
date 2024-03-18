/**
 * @file simple_planner_fixed_size_assign_plan_profile.cpp
 * @brief
 *
 * @author Roelof Oomen
 * @date May 29, 2024
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2024, ROS Industrial Consortium
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

#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/kinematic_limits.h>
#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>

namespace tesseract_planning
{
SimplePlannerFixedSizeAssignPlanProfile::SimplePlannerFixedSizeAssignPlanProfile(int freespace_steps, int linear_steps)
  : freespace_steps(freespace_steps), linear_steps(linear_steps)
{
}

std::vector<MoveInstructionPoly>
SimplePlannerFixedSizeAssignPlanProfile::generate(const MoveInstructionPoly& prev_instruction,
                                                  const MoveInstructionPoly& /*prev_seed*/,
                                                  const MoveInstructionPoly& base_instruction,
                                                  const InstructionPoly& /*next_instruction*/,
                                                  const std::shared_ptr<const tesseract_environment::Environment>& env,
                                                  const tesseract_common::ManipulatorInfo& global_manip_info) const
{
  KinematicGroupInstructionInfo prev(prev_instruction, *env, global_manip_info);
  KinematicGroupInstructionInfo base(base_instruction, *env, global_manip_info);

  Eigen::VectorXd j2;
  if (!base.has_cartesian_waypoint)
  {
    j2 = base.extractJointPosition();
  }
  else
  {
    // Determine base_instruction joint position and replicate
    const auto& base_cwp = base.instruction.getWaypoint().as<CartesianWaypointPoly>();
    if (base_cwp.hasSeed())
    {
      // Use joint position of cartesian base_instruction
      j2 = base_cwp.getSeed().position;
    }
    else
    {
      if (prev.has_cartesian_waypoint)
      {
        const auto& prev_cwp = prev.instruction.getWaypoint().as<CartesianWaypointPoly>();
        if (prev_cwp.hasSeed())
        {
          // Use joint position of cartesian prev_instruction as seed
          j2 = getClosestJointSolution(base, prev_cwp.getSeed().position);
        }
        else
        {
          // Use current env_state as seed
          j2 = getClosestJointSolution(base, env->getCurrentJointValues(base.manip->getJointNames()));
        }
      }
      else
      {
        // Use prev_instruction as seed
        j2 = getClosestJointSolution(base, prev.extractJointPosition());
      }
    }
    tesseract_common::enforceLimits<double>(j2, base.manip->getLimits().joint_limits);
  }

  Eigen::MatrixXd states;
  // Replicate base_instruction joint position
  if (base.instruction.isLinear())
    states = j2.replicate(1, linear_steps + 1);
  else if (base.instruction.isFreespace())
    states = j2.replicate(1, freespace_steps + 1);
  else
    throw std::runtime_error("stateJointJointWaypointFixedSize: Unsupported MoveInstructionType!");

  // Linearly interpolate in cartesian space if linear move
  if (base_instruction.isLinear())
  {
    Eigen::Isometry3d p1_world;
    if (prev.has_cartesian_waypoint)
      p1_world = prev.extractCartesianPose();
    else
      p1_world = prev.calcCartesianPose(prev.extractJointPosition());

    Eigen::Isometry3d p2_world;
    if (base.has_cartesian_waypoint)
      p2_world = base.extractCartesianPose();
    else
      p2_world = base.calcCartesianPose(base.extractJointPosition());

    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, linear_steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(static_cast<Eigen::Index>(poses.size()) == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

template <class Archive>
void SimplePlannerFixedSizeAssignPlanProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(SimplePlannerPlanProfile);
  ar& BOOST_SERIALIZATION_NVP(freespace_steps);
  ar& BOOST_SERIALIZATION_NVP(linear_steps);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::SimplePlannerFixedSizeAssignPlanProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::SimplePlannerFixedSizeAssignPlanProfile)