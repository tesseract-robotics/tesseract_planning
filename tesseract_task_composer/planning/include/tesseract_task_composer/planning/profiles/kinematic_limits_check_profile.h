/**
 * @file kinematic_limits_check_profile.h
 * @brief Profile for kinematic limits check task
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
#ifndef TESSERACT_TASK_COMPOSER_PLANNING_PROFILES_KINEMATIC_LIMITS_CHECK_PROFILE_H
#define TESSERACT_TASK_COMPOSER_PLANNING_PROFILES_KINEMATIC_LIMITS_CHECK_PROFILE_H

#include <memory>
#include <tesseract_common/profile.h>

namespace YAML
{
class Node;
}

namespace tesseract_common
{
class ProfilePluginFactory;
}
namespace tesseract_planning
{
struct KinematicLimitsCheckProfile : public tesseract_common::Profile
{
  using Ptr = std::shared_ptr<KinematicLimitsCheckProfile>;
  using ConstPtr = std::shared_ptr<const KinematicLimitsCheckProfile>;

  KinematicLimitsCheckProfile(bool check_position = true, bool check_velocity = true, bool check_acceleration = true);
  KinematicLimitsCheckProfile(std::string name, const YAML::Node& config, const tesseract_common::ProfilePluginFactory& plugin_factory);

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  bool check_position{ true };
  bool check_velocity{ true };
  bool check_acceleration{ true };

protected:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::KinematicLimitsCheckProfile)

#endif  // TESSERACT_TASK_COMPOSER_PLANNING_PROFILES_KINEMATIC_LIMITS_CHECK_PROFILE_H
