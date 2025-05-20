#include <tesseract_time_parameterization/kdl/constant_tcp_speed_parameterization_profiles.h>
#include <tesseract_common/serialization.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <typeindex>

namespace tesseract_planning
{
ConstantTCPSpeedParameterizationCompositeProfile::ConstantTCPSpeedParameterizationCompositeProfile()
  : Profile(ConstantTCPSpeedParameterizationCompositeProfile::getStaticKey())
{
}
ConstantTCPSpeedParameterizationCompositeProfile::ConstantTCPSpeedParameterizationCompositeProfile(
    double max_translational_velocity,
    double max_rotational_velocity,
    double max_translational_acceleration,
    double max_rotational_acceleration,
    double max_velocity_scaling_factor,
    double max_acceleration_scaling_factor)
  : Profile(ConstantTCPSpeedParameterizationCompositeProfile::getStaticKey())
  , max_translational_velocity(max_translational_velocity)
  , max_rotational_velocity(max_rotational_velocity)
  , max_translational_acceleration(max_translational_acceleration)
  , max_rotational_acceleration(max_rotational_acceleration)
  , max_velocity_scaling_factor(max_velocity_scaling_factor)
  , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
{
}

std::size_t ConstantTCPSpeedParameterizationCompositeProfile::getStaticKey()
{
  return std::type_index(typeid(ConstantTCPSpeedParameterizationCompositeProfile)).hash_code();
}

template <class Archive>
void ConstantTCPSpeedParameterizationCompositeProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Profile);
  ar& BOOST_SERIALIZATION_NVP(max_translational_velocity);
  ar& BOOST_SERIALIZATION_NVP(max_rotational_velocity);
  ar& BOOST_SERIALIZATION_NVP(max_translational_acceleration);
  ar& BOOST_SERIALIZATION_NVP(max_rotational_acceleration);
  ar& BOOST_SERIALIZATION_NVP(max_velocity_scaling_factor);
  ar& BOOST_SERIALIZATION_NVP(max_acceleration_scaling_factor);
}
}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::ConstantTCPSpeedParameterizationCompositeProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::ConstantTCPSpeedParameterizationCompositeProfile)
