#ifndef TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_CEREAL_SERIALIZATION_H
#define TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_CEREAL_SERIALIZATION_H

#include <tesseract_time_parameterization/ruckig/ruckig_trajectory_smoothing_profiles.h>

#include <tesseract_common/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract_planning
{
template <class Archive>
void serialize(Archive& ar, RuckigTrajectorySmoothingCompositeProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
  ar(cereal::make_nvp("duration_extension_fraction", obj.duration_extension_fraction));
  ar(cereal::make_nvp("max_duration_extension_factor", obj.max_duration_extension_factor));
  ar(cereal::make_nvp("override_limits", obj.override_limits));
  ar(cereal::make_nvp("velocity_limits", obj.velocity_limits));
  ar(cereal::make_nvp("acceleration_limits", obj.acceleration_limits));
  ar(cereal::make_nvp("jerk_limits", obj.jerk_limits));
}
}  // namespace tesseract_planning

#endif  // TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_CEREAL_SERIALIZATION_H
