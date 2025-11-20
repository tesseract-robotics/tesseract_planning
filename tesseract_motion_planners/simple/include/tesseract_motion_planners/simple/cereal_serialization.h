#ifndef TESSERACT_MOTION_PLANNERS_SIMPLE_CEREAL_SERIALIZATION_H
#define TESSERACT_MOTION_PLANNERS_SIMPLE_CEREAL_SERIALIZATION_H

#include <tesseract_motion_planners/simple/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_no_ik_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_assign_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_assign_no_ik_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_move_profile.h>

#include <tesseract_common/cereal_serialization.h>

#include <cereal/cereal.hpp>

namespace tesseract_planning
{
template <class Archive>
void serialize(Archive& ar, SimplePlannerMoveProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
}

template <class Archive>
void serialize(Archive& ar, SimplePlannerCompositeProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
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
}  // namespace tesseract_planning

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerCompositeProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerFixedSizeAssignMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerFixedSizeAssignNoIKMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerFixedSizeMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerLVSAssignMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerLVSAssignNoIKMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerLVSMoveProfile)
CEREAL_REGISTER_TYPE(tesseract_planning::SimplePlannerLVSNoIKMoveProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::SimplePlannerMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::SimplePlannerCompositeProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile,
                                     tesseract_planning::SimplePlannerFixedSizeAssignMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile,
                                     tesseract_planning::SimplePlannerFixedSizeAssignNoIKMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::SimplePlannerFixedSizeMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::SimplePlannerLVSAssignMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile,
                                     tesseract_planning::SimplePlannerLVSAssignNoIKMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::SimplePlannerLVSMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile, tesseract_planning::SimplePlannerLVSNoIKMoveProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerFixedSizeAssignMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerFixedSizeAssignNoIKMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerFixedSizeMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerLVSAssignMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerLVSAssignNoIKMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerLVSMoveProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::SimplePlannerMoveProfile,
                                     tesseract_planning::SimplePlannerLVSNoIKMoveProfile)

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_CEREAL_SERIALIZATION_H
