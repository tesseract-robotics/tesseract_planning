/**
 * @file descartes_default_plan_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_DEFAULT_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_DEFAULT_PLAN_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>

#include <tesseract_collision/core/types.h>

namespace tesseract_planning
{
class DescartesVertexEvaluator;

template <typename FloatType>
class DescartesDefaultPlanProfile : public DescartesPlanProfile<FloatType>
{
public:
  using Ptr = std::shared_ptr<DescartesDefaultPlanProfile<FloatType>>;
  using ConstPtr = std::shared_ptr<const DescartesDefaultPlanProfile<FloatType>>;

  DescartesDefaultPlanProfile() = default;

  bool target_pose_fixed{ true };
  Eigen::Vector3d target_pose_sample_axis{ 0, 0, 1 };
  double target_pose_sample_resolution{ M_PI_2 };

  /**
   * @brief Flag to indicate that collisions should not cause failures during state/edge evaluation
   * @details Sometimes it is beneficial to evaluate states and edges based on the distance of states from collision
   * without treating collisions as failures. In the case that the Descartes trajectory is used as a seed for a
   * subsequent planner, such as TrajOpt, the subsequent planner can adjust/optimize individual joint poses such that
   * they become collision-free.
   */
  bool allow_collision{ false };

  /** @brief Flag to apply collision checking during state sampling */
  bool enable_collision{ true };
  tesseract_collision::CollisionCheckConfig vertex_collision_check_config{ 0 };

  /** @brief Flag to apply collision checking during edge evaluation */
  bool enable_edge_collision{ false };
  tesseract_collision::CollisionCheckConfig edge_collision_check_config{ 0 };

  /**
   * @brief Flag for generating redundant solutions as additional vertices for the planning graph search
   */
  bool use_redundant_joint_solutions{ false };

  /** @brief Flag to produce debug information during planning */
  bool debug{ false };

  std::unique_ptr<descartes_light::WaypointSampler<FloatType>>
  createWaypointSampler(const MoveInstructionPoly& move_instruction,
                        const tesseract_common::ManipulatorInfo& manip_info,
                        const std::shared_ptr<const tesseract_environment::Environment>& env) const override;

  std::unique_ptr<descartes_light::EdgeEvaluator<FloatType>>
  createEdgeEvaluator(const MoveInstructionPoly& move_instruction,
                      const tesseract_common::ManipulatorInfo& manip_info,
                      const std::shared_ptr<const tesseract_environment::Environment>& env) const override;

  std::unique_ptr<descartes_light::StateEvaluator<FloatType>>
  createStateEvaluator(const MoveInstructionPoly& move_instruction,
                       const tesseract_common::ManipulatorInfo& manip_info,
                       const std::shared_ptr<const tesseract_environment::Environment>& env) const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT

  virtual std::unique_ptr<DescartesVertexEvaluator>
  createVertexEvaluator(const MoveInstructionPoly& move_instruction,
                        const std::shared_ptr<const tesseract_kinematics::KinematicGroup>& manip,
                        const std::shared_ptr<const tesseract_environment::Environment>& env) const;

  virtual PoseSamplerFn createPoseSampler(const MoveInstructionPoly& move_instruction,
                                          const std::shared_ptr<const tesseract_kinematics::KinematicGroup>& manip,
                                          const std::shared_ptr<const tesseract_environment::Environment>& env) const;
};

using DescartesDefaultPlanProfileF = DescartesDefaultPlanProfile<float>;
using DescartesDefaultPlanProfileD = DescartesDefaultPlanProfile<double>;
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::DescartesDefaultPlanProfile<float>)
BOOST_CLASS_EXPORT_KEY(tesseract_planning::DescartesDefaultPlanProfile<double>)

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_DEFAULT_PLAN_PROFILE_H
