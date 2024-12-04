/**
 * @file contact_check_profile.h
 * @brief Contact check trajectory profile
 *
 * @author Levi Armstrong
 * @date August 10. 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_TASK_COMPOSER_CONTACT_CHECK_PROFILE_H
#define TESSERACT_TASK_COMPOSER_CONTACT_CHECK_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>
#include <tesseract_command_language/profile.h>

namespace tesseract_planning
{
struct ContactCheckProfile : public Profile
{
  using Ptr = std::shared_ptr<ContactCheckProfile>;
  using ConstPtr = std::shared_ptr<const ContactCheckProfile>;

  ContactCheckProfile();
  ContactCheckProfile(double longest_valid_segment_length, double contact_distance);

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  /** @brief The contact manager config */
  tesseract_collision::CollisionCheckConfig config;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::ContactCheckProfile)

#endif  // TESSERACT_TASK_COMPOSER_CONTACT_CHECK_PROFILE_H
