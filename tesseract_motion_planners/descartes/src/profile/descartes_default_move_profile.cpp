/**
 * @file descartes_default_move_profile.cpp
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
#include <tesseract_motion_planners/descartes/impl/profile/descartes_default_move_profile.hpp>
#include <tesseract_motion_planners/descartes/descartes_vertex_evaluator.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
template <typename FloatType>
bool DescartesDefaultMoveProfile<FloatType>::operator==(const DescartesDefaultMoveProfile<FloatType>& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (target_pose_fixed == rhs.target_pose_fixed);
  equal &= tesseract_common::almostEqualRelativeAndAbs(target_pose_sample_axis, rhs.target_pose_sample_axis, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(
      target_pose_sample_resolution, rhs.target_pose_sample_resolution, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(target_pose_sample_min, rhs.target_pose_sample_min, max_diff);
  equal &= tesseract_common::almostEqualRelativeAndAbs(target_pose_sample_max, rhs.target_pose_sample_max, max_diff);
  equal &= (manipulator_ik_solver == rhs.manipulator_ik_solver);
  equal &= (allow_collision == rhs.allow_collision);
  equal &= (enable_collision == rhs.enable_collision);
  equal &= (vertex_contact_manager_config == rhs.vertex_contact_manager_config);
  equal &= (vertex_collision_check_config == rhs.vertex_collision_check_config);
  equal &= (enable_edge_collision == rhs.enable_edge_collision);
  equal &= (edge_contact_manager_config == rhs.edge_contact_manager_config);
  equal &= (edge_collision_check_config == rhs.edge_collision_check_config);
  equal &= (use_redundant_joint_solutions == rhs.use_redundant_joint_solutions);
  equal &= (debug == rhs.debug);
  return equal;
}

template <typename FloatType>
bool DescartesDefaultMoveProfile<FloatType>::operator!=(const DescartesDefaultMoveProfile<FloatType>& rhs) const
{
  return !operator==(rhs);
}

// Explicit template instantiation
template class DescartesDefaultMoveProfile<float>;
template class DescartesDefaultMoveProfile<double>;
}  // namespace tesseract_planning
