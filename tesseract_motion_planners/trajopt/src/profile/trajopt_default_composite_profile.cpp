/**
 * @file trajopt_default_composite_profile.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>

#include <tesseract_common/eigen_serialization.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_environment/environment.h>
#include <trajopt_common/utils.hpp>

static const double LONGEST_VALID_SEGMENT_FRACTION_DEFAULT = 0.01;

namespace tesseract_planning
{
TrajOptTermInfos
TrajOptDefaultCompositeProfile::create(const tesseract_common::ManipulatorInfo& composite_manip_info,
                                       const std::shared_ptr<const tesseract_environment::Environment>& env,
                                       const std::vector<int>& fixed_indices,
                                       int start_index,
                                       int end_index) const
{
  // -------- Construct the problem ------------
  // -------------------------------------------
  TrajOptTermInfos term_infos;
  tesseract_kinematics::JointGroup::ConstPtr joint_group = env->getJointGroup(composite_manip_info.manipulator);
  const Eigen::Index dof = joint_group->numJoints();
  const Eigen::MatrixX2d joint_limits = joint_group->getLimits().joint_limits;
  const double lvs_length = computeLongestValidSegmentLength(joint_limits);

  if (collision_constraint_config.enabled)
  {
    trajopt::TermInfo::Ptr ti = createCollisionTermInfo(start_index,
                                                        end_index,
                                                        collision_constraint_config.safety_margin,
                                                        collision_constraint_config.safety_margin_buffer,
                                                        collision_constraint_config.type,
                                                        collision_constraint_config.use_weighted_sum,
                                                        collision_constraint_config.coeff,
                                                        contact_test_type,
                                                        lvs_length,
                                                        trajopt::TermType::TT_CNT);

    // Update the term info with the (possibly) new start and end state indices for which to apply this cost
    std::shared_ptr<trajopt::CollisionTermInfo> ct = std::static_pointer_cast<trajopt::CollisionTermInfo>(ti);
    if (special_collision_constraint)
    {
      for (auto& info : ct->info)
      {
        info = special_collision_constraint;
      }
    }
    ct->fixed_steps = fixed_indices;

    term_infos.constraints.push_back(ct);
  }

  if (collision_cost_config.enabled)
  {
    // Create a default collision term info
    trajopt::TermInfo::Ptr ti = createCollisionTermInfo(start_index,
                                                        end_index,
                                                        collision_cost_config.safety_margin,
                                                        collision_cost_config.safety_margin_buffer,
                                                        collision_cost_config.type,
                                                        collision_cost_config.use_weighted_sum,
                                                        collision_cost_config.coeff,
                                                        contact_test_type,
                                                        lvs_length,
                                                        trajopt::TermType::TT_COST);

    // Update the term info with the (possibly) new start and end state indices for which to apply this cost
    std::shared_ptr<trajopt::CollisionTermInfo> ct = std::static_pointer_cast<trajopt::CollisionTermInfo>(ti);
    if (special_collision_cost)
    {
      for (auto& info : ct->info)
      {
        info = special_collision_cost;
      }
    }
    ct->fixed_steps = fixed_indices;

    term_infos.costs.push_back(ct);
  }

  if (smooth_velocities)
  {
    if (velocity_coeff.size() == 0)
      term_infos.costs.push_back(createSmoothVelocityTermInfo(start_index, end_index, static_cast<int>(dof)));
    else
      term_infos.costs.push_back(createSmoothVelocityTermInfo(start_index, end_index, velocity_coeff));
  }

  if (smooth_accelerations)
  {
    if (acceleration_coeff.size() == 0)
      term_infos.costs.push_back(createSmoothAccelerationTermInfo(start_index, end_index, static_cast<int>(dof)));
    else
      term_infos.costs.push_back(createSmoothAccelerationTermInfo(start_index, end_index, acceleration_coeff));
  }

  if (smooth_jerks)
  {
    if (jerk_coeff.size() == 0)
      term_infos.costs.push_back(createSmoothJerkTermInfo(start_index, end_index, static_cast<int>(dof)));
    else
      term_infos.costs.push_back(createSmoothJerkTermInfo(start_index, end_index, jerk_coeff));
  }

  if (avoid_singularity)
    term_infos.costs.push_back(createAvoidSingularityTermInfo(
        start_index, end_index, composite_manip_info.tcp_frame, avoid_singularity_coeff));

  return term_infos;
}

double TrajOptDefaultCompositeProfile::computeLongestValidSegmentLength(const Eigen::MatrixX2d& joint_limits) const
{
  // Calculate longest valid segment length
  double extent = (joint_limits.col(1) - joint_limits.col(0)).norm();
  if (longest_valid_segment_fraction > 0 && longest_valid_segment_length > 0)
    return std::min(longest_valid_segment_fraction * extent, longest_valid_segment_length);

  if (longest_valid_segment_fraction > 0)
    return longest_valid_segment_fraction * extent;

  if (longest_valid_segment_length > 0)
    return longest_valid_segment_length;

  return LONGEST_VALID_SEGMENT_FRACTION_DEFAULT * extent;
}

template <class Archive>
void TrajOptDefaultCompositeProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TrajOptCompositeProfile);
  ar& BOOST_SERIALIZATION_NVP(contact_test_type);
  ar& BOOST_SERIALIZATION_NVP(collision_cost_config);
  ar& BOOST_SERIALIZATION_NVP(collision_constraint_config);
  ar& BOOST_SERIALIZATION_NVP(smooth_velocities);
  ar& BOOST_SERIALIZATION_NVP(velocity_coeff);
  ar& BOOST_SERIALIZATION_NVP(smooth_accelerations);
  ar& BOOST_SERIALIZATION_NVP(acceleration_coeff);
  ar& BOOST_SERIALIZATION_NVP(smooth_jerks);
  ar& BOOST_SERIALIZATION_NVP(jerk_coeff);
  ar& BOOST_SERIALIZATION_NVP(avoid_singularity);
  ar& BOOST_SERIALIZATION_NVP(avoid_singularity_coeff);
  ar& BOOST_SERIALIZATION_NVP(longest_valid_segment_fraction);
  ar& BOOST_SERIALIZATION_NVP(longest_valid_segment_length);
  ar& BOOST_SERIALIZATION_NVP(special_collision_cost);
  ar& BOOST_SERIALIZATION_NVP(special_collision_constraint);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptDefaultCompositeProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptDefaultCompositeProfile)
