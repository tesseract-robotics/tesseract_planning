#ifndef TESSERACT_TASK_COMPOSER_PLANNING_CEREAL_SERIALIZATION_H
#define TESSERACT_TASK_COMPOSER_PLANNING_CEREAL_SERIALIZATION_H

#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract_task_composer/planning/profiles/fix_state_bounds_profile.h>
#include <tesseract_task_composer/planning/profiles/fix_state_collision_profile.h>
#include <tesseract_task_composer/planning/profiles/kinematic_limits_check_profile.h>
#include <tesseract_task_composer/planning/profiles/min_length_profile.h>
#include <tesseract_task_composer/planning/profiles/profile_switch_profile.h>
#include <tesseract_task_composer/planning/profiles/upsample_trajectory_profile.h>

#include <tesseract_collision/core/cereal_serialization.h>
#include <tesseract_motion_planners/trajopt/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract_planning
{
template <class Archive>
void serialize(Archive& ar, ContactCheckProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
  ar(cereal::make_nvp("contact_manager_config", obj.contact_manager_config));
  ar(cereal::make_nvp("collision_check_config", obj.collision_check_config));
}

template <class Archive>
void serialize(Archive& ar, FixStateBoundsProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
  ar(cereal::make_nvp("mode", obj.mode));
  ar(cereal::make_nvp("max_deviation_global", obj.max_deviation_global));
  ar(cereal::make_nvp("upper_bounds_reduction", obj.upper_bounds_reduction));
  ar(cereal::make_nvp("lower_bounds_reduction", obj.lower_bounds_reduction));
}

template <class Archive>
void serialize(Archive& ar, FixStateCollisionProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
  ar(cereal::make_nvp("mode", obj.mode));
  ar(cereal::make_nvp("correction_workflow", obj.correction_workflow));
  ar(cereal::make_nvp("jiggle_factor", obj.jiggle_factor));
  ar(cereal::make_nvp("contact_manager_config", obj.contact_manager_config));
  ar(cereal::make_nvp("collision_check_config", obj.collision_check_config));
  ar(cereal::make_nvp("sampling_attempts", obj.sampling_attempts));
  ar(cereal::make_nvp("trajopt_joint_constraint_config", obj.trajopt_joint_constraint_config));
  ar(cereal::make_nvp("trajopt_joint_cost_config", obj.trajopt_joint_cost_config));
  ar(cereal::make_nvp("collision_constraint_coeff", obj.collision_constraint_coeff));
  ar(cereal::make_nvp("collision_cost_coeff", obj.collision_cost_coeff));
  ar(cereal::make_nvp("opt_params", obj.opt_params));
  ar(cereal::make_nvp("osqp_settings", obj.osqp_settings));
  ar(cereal::make_nvp("update_workspace", obj.update_workspace));
}

template <class Archive>
void serialize(Archive& ar, KinematicLimitsCheckProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
  ar(cereal::make_nvp("check_position", obj.check_position));
  ar(cereal::make_nvp("check_velocity", obj.check_velocity));
  ar(cereal::make_nvp("check_acceleration", obj.check_acceleration));
}

template <class Archive>
void serialize(Archive& ar, MinLengthProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
  ar(cereal::make_nvp("min_length", obj.min_length));
}

template <class Archive>
void serialize(Archive& ar, ProfileSwitchProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
  ar(cereal::make_nvp("return_value", obj.return_value));
}

template <class Archive>
void serialize(Archive& ar, UpsampleTrajectoryProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
  ar(cereal::make_nvp("longest_valid_segment_length", obj.longest_valid_segment_length));
}

}  // namespace tesseract_planning

#endif  // TESSERACT_TASK_COMPOSER_PLANNING_CEREAL_SERIALIZATION_H
