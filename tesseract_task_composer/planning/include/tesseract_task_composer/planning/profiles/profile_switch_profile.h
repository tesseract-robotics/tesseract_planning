/**
 * @file profile_switch_profile.h
 * @brief Profile for task that returns a value based on the profile
 *
 * @author Matthew Powelson
 * @date October 26. 2020
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_TASK_COMPOSER_PROFILE_SWITCH_PROFILE_H
#define TESSERACT_TASK_COMPOSER_PROFILE_SWITCH_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/profile.h>
#include <tesseract_common/fwd.h>

namespace YAML
{
class Node;
}


namespace tesseract_planning
{
struct ProfileSwitchProfile : public tesseract_common::Profile
{
  using Ptr = std::shared_ptr<ProfileSwitchProfile>;
  using ConstPtr = std::shared_ptr<const ProfileSwitchProfile>;

  ProfileSwitchProfile(int return_value = 1);
  ProfileSwitchProfile(std::string name, const YAML::Node& config, const tesseract_common::ProfilePluginFactory& plugin_factory);

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  int return_value;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::ProfileSwitchProfile)

#endif  // TESSERACT_TASK_COMPOSER_PROFILE_SWITCH_PROFILE_H
