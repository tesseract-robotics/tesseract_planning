/**
 * @file descartes_default_move_profile.hpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_MOVE_PROFILE_HPP
#define TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_MOVE_PROFILE_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_light/edge_evaluators/compound_edge_evaluator.h>
#include <descartes_light/samplers/fixed_joint_waypoint_sampler.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_move_profile.h>
#include <tesseract_motion_planners/descartes/impl/profile/descartes_profile.hpp>
#include <tesseract_motion_planners/descartes/descartes_robot_sampler.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/descartes_collision_edge_evaluator.h>
#include <tesseract_motion_planners/descartes/descartes_vertex_evaluator.h>

#include <tesseract_common/utils.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/utils.h>

namespace tesseract_planning
{
template <typename FloatType>
std::unique_ptr<DescartesVertexEvaluator> DescartesDefaultMoveProfile<FloatType>::createVertexEvaluator(
    const MoveInstructionPoly& /*move_instruction*/,
    const std::shared_ptr<const tesseract_kinematics::KinematicGroup>& manip,
    const std::shared_ptr<const tesseract_environment::Environment>& /*env*/) const
{
  return std::make_unique<DescartesJointLimitsVertexEvaluator>(manip->getLimits().joint_limits);
}

template <typename FloatType>
PoseSamplerFn DescartesDefaultMoveProfile<FloatType>::createPoseSampler(
    const MoveInstructionPoly& /*move_instruction*/,
    const std::shared_ptr<const tesseract_kinematics::KinematicGroup>& /*manip*/,
    const std::shared_ptr<const tesseract_environment::Environment>& /*env*/) const
{
  if (target_pose_fixed)
    return sampleFixed;

  return [axis = target_pose_sample_axis,
          resolution = target_pose_sample_resolution,
          minimum = target_pose_sample_min,
          maximum = target_pose_sample_max](const Eigen::Isometry3d& tool_pose) {
    return sampleToolAxis(tool_pose, axis, resolution, minimum, maximum);
  };
}

template <typename FloatType>
std::unique_ptr<descartes_light::WaypointSampler<FloatType>>
DescartesDefaultMoveProfile<FloatType>::createWaypointSampler(
    const MoveInstructionPoly& move_instruction,
    const tesseract_common::ManipulatorInfo& composite_manip_info,
    const std::shared_ptr<const tesseract_environment::Environment>& env) const
{
  // If plan instruction has manipulator information then use it over the one provided by the composite.
  tesseract_common::ManipulatorInfo manip_info =
      composite_manip_info.getCombined(move_instruction.getManipulatorInfo());
  if (!manipulator_ik_solver.empty())
    manip_info.manipulator_ik_solver = manipulator_ik_solver;

  if (manip_info.empty())
    throw std::runtime_error("Descartes, manipulator info is empty!");

  auto manip = DescartesMoveProfile<FloatType>::createKinematicGroup(manip_info, *env);

  if (!move_instruction.getWaypoint().isCartesianWaypoint())
  {
    assert(checkJointPositionFormat(manip->getJointNames(), move_instruction.getWaypoint()));
    const Eigen::VectorXd& joint_waypoint = getJointPosition(move_instruction.getWaypoint());
    auto state = std::make_shared<descartes_light::State<FloatType>>(joint_waypoint.cast<FloatType>());
    return std::make_unique<descartes_light::FixedJointWaypointSampler<FloatType>>(state);
  }

  Eigen::Isometry3d tcp_offset = env->findTCPOffset(manip_info);

  /* Check if this cartesian waypoint is dynamic
   * (i.e. defined relative to a frame that will move with the kinematic chain) */
  //  auto it = std::find(active_links.begin(), active_links.end(), prob.manip_inv_kin->getBaseLinkName());
  //  if (it != active_links.end() && prob.manip_inv_kin->getBaseLinkName() != mi.working_frame)
  //    throw std::runtime_error("DescartesDefaultMoveProfile: Assigned dynamic waypoint but parent instruction working
  //    is "
  //                             "not set to the base link of manipulator!");

  DescartesCollision::Ptr descartes_collision = nullptr;
  if (enable_collision)
    descartes_collision = std::make_shared<DescartesCollision>(
        *env, manip, vertex_contact_manager_config, vertex_collision_check_config, debug);

  auto ve = createVertexEvaluator(move_instruction, manip, env);
  auto pose_sampler = createPoseSampler(move_instruction, manip, env);
  return std::make_unique<DescartesRobotSampler<FloatType>>(
      manip_info.working_frame,
      move_instruction.getWaypoint().as<CartesianWaypointPoly>().getTransform(),
      pose_sampler,
      manip,
      descartes_collision,
      manip_info.tcp_frame,
      tcp_offset,
      allow_collision,
      std::move(ve),
      use_redundant_joint_solutions);
}

template <typename FloatType>
std::unique_ptr<descartes_light::EdgeEvaluator<FloatType>> DescartesDefaultMoveProfile<FloatType>::createEdgeEvaluator(
    const MoveInstructionPoly& move_instruction,
    const tesseract_common::ManipulatorInfo& composite_manip_info,
    const std::shared_ptr<const tesseract_environment::Environment>& env) const
{
  // If plan instruction has manipulator information then use it over the one provided by the composite.
  tesseract_common::ManipulatorInfo manip_info =
      composite_manip_info.getCombined(move_instruction.getManipulatorInfo());
  if (!manipulator_ik_solver.empty())
    manip_info.manipulator_ik_solver = manipulator_ik_solver;

  if (manip_info.empty())
    throw std::runtime_error("Descartes, manipulator info is empty!");

  auto manip = DescartesMoveProfile<FloatType>::createKinematicGroup(manip_info, *env);

  if (!enable_edge_collision)
    return std::make_unique<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>();

  auto compound_evaluator = std::make_unique<descartes_light::CompoundEdgeEvaluator<FloatType>>();
  compound_evaluator->evaluators.push_back(
      std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>());
  compound_evaluator->evaluators.push_back(std::make_shared<DescartesCollisionEdgeEvaluator<FloatType>>(
      *env, manip, edge_contact_manager_config, edge_collision_check_config, allow_collision, debug));

  return compound_evaluator;
}

template <typename FloatType>
std::unique_ptr<descartes_light::StateEvaluator<FloatType>>
DescartesDefaultMoveProfile<FloatType>::createStateEvaluator(
    const MoveInstructionPoly& /*move_instruction*/,
    const tesseract_common::ManipulatorInfo& /*composite_manip_info*/,
    const std::shared_ptr<const tesseract_environment::Environment>& /*env*/) const
{
  return std::make_unique<descartes_light::StateEvaluator<FloatType>>();
}

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_MOVE_PROFILE_HPP
