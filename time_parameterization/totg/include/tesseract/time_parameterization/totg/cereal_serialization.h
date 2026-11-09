#ifndef TESSERACT_TIME_PARAMETERIZATION_TIME_OPTIMAL_TRAJECTORY_GENERATION_CEREAL_SERIALIZATION_H
#define TESSERACT_TIME_PARAMETERIZATION_TIME_OPTIMAL_TRAJECTORY_GENERATION_CEREAL_SERIALIZATION_H

#include <tesseract/time_parameterization/totg/time_optimal_trajectory_generation_profiles.h>

#include <tesseract/common/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract::time_parameterization
{
template <class Archive>
void serialize(Archive& ar, TimeOptimalTrajectoryGenerationCompositeProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
  ar(cereal::make_nvp("path_tolerance", obj.path_tolerance));
  ar(cereal::make_nvp("min_angle_change", obj.min_angle_change));
  ar(cereal::make_nvp("override_limits", obj.override_limits));
  ar(cereal::make_nvp("velocity_limits", obj.velocity_limits));
  ar(cereal::make_nvp("acceleration_limits", obj.acceleration_limits));
  ar(cereal::make_nvp("max_velocity_scaling_factor", obj.max_velocity_scaling_factor));
  ar(cereal::make_nvp("max_acceleration_scaling_factor", obj.max_acceleration_scaling_factor));
  ar(cereal::make_nvp("use_start_velocity", obj.use_start_velocity));
  ar(cereal::make_nvp("use_end_velocity", obj.use_end_velocity));
}

}  // namespace tesseract::time_parameterization

// On Windows and macOS the cereal polymorphic-type registration must be in the header,
// for other platforms registration is in the cpp.
#if defined(_WIN32) || defined(__APPLE__)
#include <tesseract/time_parameterization/totg/cereal_serialization_impl.hpp>
#else
CEREAL_FORCE_DYNAMIC_INIT(tesseract_time_parameterization_totg_cereal)
#endif

#endif  // TESSERACT_TIME_PARAMETERIZATION_TIME_OPTIMAL_TRAJECTORY_GENERATION_CEREAL_SERIALIZATION_H
