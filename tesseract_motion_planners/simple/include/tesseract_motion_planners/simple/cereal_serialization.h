#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_CEREAL_SERIALIZATION_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_CEREAL_SERIALIZATION_H

#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_no_ik_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_assign_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_assign_no_ik_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_move_profile.h>

#include <tesseract/common/cereal_serialization.h>

#include <cereal/cereal.hpp>

namespace tesseract::motion_planners
{
template <class Archive>
void serialize(Archive& ar, SimplePlannerMoveProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
}

template <class Archive>
void serialize(Archive& ar, SimplePlannerCompositeProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
}

template <class Archive>
void serialize(Archive& ar, SimplePlannerFixedSizeAssignMoveProfile& obj)
{
  ar(cereal::base_class<SimplePlannerMoveProfile>(&obj));
  ar(cereal::make_nvp("freespace_steps", obj.freespace_steps));
  ar(cereal::make_nvp("linear_steps", obj.linear_steps));
}

template <class Archive>
void serialize(Archive& ar, SimplePlannerFixedSizeAssignNoIKMoveProfile& obj)
{
  ar(cereal::base_class<SimplePlannerMoveProfile>(&obj));
  ar(cereal::make_nvp("freespace_steps", obj.freespace_steps));
  ar(cereal::make_nvp("linear_steps", obj.linear_steps));
}

template <class Archive>
void serialize(Archive& ar, SimplePlannerFixedSizeMoveProfile& obj)
{
  ar(cereal::base_class<SimplePlannerMoveProfile>(&obj));
  ar(cereal::make_nvp("freespace_steps", obj.freespace_steps));
  ar(cereal::make_nvp("linear_steps", obj.linear_steps));
}

template <class Archive>
void serialize(Archive& ar, SimplePlannerLVSAssignMoveProfile& obj)
{
  ar(cereal::base_class<SimplePlannerMoveProfile>(&obj));
  ar(cereal::make_nvp("state_longest_valid_segment_length", obj.state_longest_valid_segment_length));
  ar(cereal::make_nvp("translation_longest_valid_segment_length", obj.translation_longest_valid_segment_length));
  ar(cereal::make_nvp("rotation_longest_valid_segment_length", obj.rotation_longest_valid_segment_length));
  ar(cereal::make_nvp("min_steps", obj.min_steps));
  ar(cereal::make_nvp("max_steps", obj.max_steps));
}

template <class Archive>
void serialize(Archive& ar, SimplePlannerLVSAssignNoIKMoveProfile& obj)
{
  ar(cereal::base_class<SimplePlannerMoveProfile>(&obj));
  ar(cereal::make_nvp("state_longest_valid_segment_length", obj.state_longest_valid_segment_length));
  ar(cereal::make_nvp("translation_longest_valid_segment_length", obj.translation_longest_valid_segment_length));
  ar(cereal::make_nvp("rotation_longest_valid_segment_length", obj.rotation_longest_valid_segment_length));
  ar(cereal::make_nvp("min_steps", obj.min_steps));
  ar(cereal::make_nvp("max_steps", obj.max_steps));
}

template <class Archive>
void serialize(Archive& ar, SimplePlannerLVSMoveProfile& obj)
{
  ar(cereal::base_class<SimplePlannerMoveProfile>(&obj));
  ar(cereal::make_nvp("state_longest_valid_segment_length", obj.state_longest_valid_segment_length));
  ar(cereal::make_nvp("translation_longest_valid_segment_length", obj.translation_longest_valid_segment_length));
  ar(cereal::make_nvp("rotation_longest_valid_segment_length", obj.rotation_longest_valid_segment_length));
  ar(cereal::make_nvp("min_steps", obj.min_steps));
  ar(cereal::make_nvp("max_steps", obj.max_steps));
}

template <class Archive>
void serialize(Archive& ar, SimplePlannerLVSNoIKMoveProfile& obj)
{
  ar(cereal::base_class<SimplePlannerMoveProfile>(&obj));
  ar(cereal::make_nvp("state_longest_valid_segment_length", obj.state_longest_valid_segment_length));
  ar(cereal::make_nvp("translation_longest_valid_segment_length", obj.translation_longest_valid_segment_length));
  ar(cereal::make_nvp("rotation_longest_valid_segment_length", obj.rotation_longest_valid_segment_length));
  ar(cereal::make_nvp("min_steps", obj.min_steps));
  ar(cereal::make_nvp("max_steps", obj.max_steps));
}
}  // namespace tesseract::motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_CEREAL_SERIALIZATION_H
