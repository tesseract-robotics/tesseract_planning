#ifndef TESSERACT_TIME_PARAMETERIZATION_ITERATIVE_SPLINE_PARAMETERIZATION_CEREAL_SERIALIZATION_H
#define TESSERACT_TIME_PARAMETERIZATION_ITERATIVE_SPLINE_PARAMETERIZATION_CEREAL_SERIALIZATION_H

#include <tesseract_time_parameterization/isp/iterative_spline_parameterization_profiles.h>

#include <tesseract_common/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract_planning
{
template <class Archive>
void serialize(Archive& ar, IterativeSplineParameterizationCompositeProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
  ar(cereal::make_nvp("add_points", obj.add_points));
  ar(cereal::make_nvp("override_limits", obj.override_limits));
  ar(cereal::make_nvp("velocity_limits", obj.velocity_limits));
  ar(cereal::make_nvp("max_velocity_scaling_factor", obj.max_velocity_scaling_factor));
  ar(cereal::make_nvp("max_acceleration_scaling_factor", obj.max_acceleration_scaling_factor));
}

template <class Archive>
void serialize(Archive& ar, IterativeSplineParameterizationMoveProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
  ar(cereal::make_nvp("max_velocity_scaling_factor", obj.max_velocity_scaling_factor));
  ar(cereal::make_nvp("max_acceleration_scaling_factor", obj.max_acceleration_scaling_factor));
}

}  // namespace tesseract_planning

#endif  // TESSERACT_TIME_PARAMETERIZATION_ITERATIVE_SPLINE_PARAMETERIZATION_CEREAL_SERIALIZATION_H
