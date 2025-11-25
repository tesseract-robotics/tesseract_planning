/**
 * @file contant_tcp_speed_parameterization_profiles.h
 * @brief Constant Tool Center Point (TCP) speed time parameterization profiles
 *
 * @copyright Copyright (c) 2025, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TESSERACT_TIME_PARAMETERIZATION_CONSTANT_TCP_SPEED_PARAMETERIZATION_PROFILES_H
#define TESSERACT_TIME_PARAMETERIZATION_CONSTANT_TCP_SPEED_PARAMETERIZATION_PROFILES_H

#include <memory>
#include <tesseract_common/profile.h>

namespace tesseract_planning
{
struct ConstantTCPSpeedParameterizationCompositeProfile : public tesseract_common::Profile
{
  using Ptr = std::shared_ptr<ConstantTCPSpeedParameterizationCompositeProfile>;
  using ConstPtr = std::shared_ptr<const ConstantTCPSpeedParameterizationCompositeProfile>;

  ConstantTCPSpeedParameterizationCompositeProfile();
  ConstantTCPSpeedParameterizationCompositeProfile(double max_translational_velocity,
                                                   double max_rotational_velocity,
                                                   double max_translational_acceleration,
                                                   double max_rotational_acceleration,
                                                   double max_velocity_scaling_factor_ = 1.0,
                                                   double max_acceleration_scaling_factor = 1.0);

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  double max_translational_velocity{ 1.0 };
  double max_rotational_velocity{ 1.0 };
  double max_translational_acceleration{ 1.0 };
  double max_rotational_acceleration{ 1.0 };
  double max_velocity_scaling_factor{ 1.0 };
  double max_acceleration_scaling_factor{ 1.0 };

  bool operator==(const ConstantTCPSpeedParameterizationCompositeProfile& rhs) const;
  bool operator!=(const ConstantTCPSpeedParameterizationCompositeProfile& rhs) const;
};

}  // namespace tesseract_planning

#endif
