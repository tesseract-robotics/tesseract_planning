#ifndef TESSERACT_TIME_PARAMETERIZATION_CONSTANT_TCP_SPEED_PARAMETERIZATION_CEREAL_SERIALIZATION_H
#define TESSERACT_TIME_PARAMETERIZATION_CONSTANT_TCP_SPEED_PARAMETERIZATION_CEREAL_SERIALIZATION_H

#include <tesseract_time_parameterization/kdl/constant_tcp_speed_parameterization_profiles.h>

#include <tesseract_common/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract_planning
{
template <class Archive>
void serialize(Archive& ar, ConstantTCPSpeedParameterizationCompositeProfile& obj)
{
  ar(cereal::base_class<tesseract_common::Profile>(&obj));
  ar(cereal::make_nvp("max_translational_velocity", obj.max_translational_velocity));
  ar(cereal::make_nvp("max_rotational_velocity", obj.max_rotational_velocity));
  ar(cereal::make_nvp("max_translational_acceleration", obj.max_translational_acceleration));
  ar(cereal::make_nvp("max_rotational_acceleration", obj.max_rotational_acceleration));
  ar(cereal::make_nvp("max_velocity_scaling_factor", obj.max_velocity_scaling_factor));
  ar(cereal::make_nvp("max_acceleration_scaling_factor", obj.max_acceleration_scaling_factor));
}

}  // namespace tesseract_planning

#endif  // TESSERACT_TIME_PARAMETERIZATION_CONSTANT_TCP_SPEED_PARAMETERIZATION_CEREAL_SERIALIZATION_H
