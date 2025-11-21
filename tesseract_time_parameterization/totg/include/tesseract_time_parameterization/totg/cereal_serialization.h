#ifndef TESSERACT_TIME_PARAMETERIZATION_TIME_OPTIMAL_TRAJECTORY_GENERATION_CEREAL_SERIALIZATION_H
#define TESSERACT_TIME_PARAMETERIZATION_TIME_OPTIMAL_TRAJECTORY_GENERATION_CEREAL_SERIALIZATION_H

#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation_profiles.h>

#include <tesseract_common/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract_planning
{
template <class Archive>
void serialize(Archive& ar, TimeOptimalTrajectoryGenerationCompositeProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
  ar(cereal::make_nvp("path_tolerance", obj.path_tolerance));
  ar(cereal::make_nvp("min_angle_change", obj.min_angle_change));
  ar(cereal::make_nvp("override_limits", obj.override_limits));
  ar(cereal::make_nvp("velocity_limits", obj.velocity_limits));
  ar(cereal::make_nvp("acceleration_limits", obj.acceleration_limits));
  ar(cereal::make_nvp("max_velocity_scaling_factor", obj.max_velocity_scaling_factor));
  ar(cereal::make_nvp("max_acceleration_scaling_factor", obj.max_acceleration_scaling_factor));
}

}  // namespace tesseract_planning

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_planning::TimeOptimalTrajectoryGenerationCompositeProfile)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::Profile,
                                     tesseract_planning::TimeOptimalTrajectoryGenerationCompositeProfile)

#endif  // TESSERACT_TIME_PARAMETERIZATION_TIME_OPTIMAL_TRAJECTORY_GENERATION_CEREAL_SERIALIZATION_H
