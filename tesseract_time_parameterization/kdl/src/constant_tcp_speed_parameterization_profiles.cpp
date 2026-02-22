#include <tesseract_time_parameterization/kdl/constant_tcp_speed_parameterization_profiles.h>
#include <tesseract_common/utils.h>

namespace tesseract::time_parameterization
{
ConstantTCPSpeedParameterizationCompositeProfile::ConstantTCPSpeedParameterizationCompositeProfile()
  : Profile(createKey<ConstantTCPSpeedParameterizationCompositeProfile>())
{
}
ConstantTCPSpeedParameterizationCompositeProfile::ConstantTCPSpeedParameterizationCompositeProfile(
    double max_translational_velocity,
    double max_rotational_velocity,
    double max_translational_acceleration,
    double max_rotational_acceleration,
    double max_velocity_scaling_factor,
    double max_acceleration_scaling_factor)
  : Profile(createKey<ConstantTCPSpeedParameterizationCompositeProfile>())
  , max_translational_velocity(max_translational_velocity)
  , max_rotational_velocity(max_rotational_velocity)
  , max_translational_acceleration(max_translational_acceleration)
  , max_rotational_acceleration(max_rotational_acceleration)
  , max_velocity_scaling_factor(max_velocity_scaling_factor)
  , max_acceleration_scaling_factor(max_acceleration_scaling_factor)
{
}

bool ConstantTCPSpeedParameterizationCompositeProfile::operator==(
    const ConstantTCPSpeedParameterizationCompositeProfile& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      max_translational_velocity, rhs.max_translational_velocity, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(max_rotational_velocity, rhs.max_rotational_velocity, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      max_translational_acceleration, rhs.max_translational_acceleration, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      max_rotational_acceleration, rhs.max_rotational_acceleration, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      max_velocity_scaling_factor, rhs.max_velocity_scaling_factor, max_diff);
  equal &= tesseract::common::almostEqualRelativeAndAbs(
      max_acceleration_scaling_factor, rhs.max_acceleration_scaling_factor, max_diff);
  return equal;
}

bool ConstantTCPSpeedParameterizationCompositeProfile::operator!=(
    const ConstantTCPSpeedParameterizationCompositeProfile& rhs) const
{
  return !operator==(rhs);
}

}  // namespace tesseract::time_parameterization
