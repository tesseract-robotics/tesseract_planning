/**
 * @file min_length_profile.h
 * @brief Profile for task that processing the program so it meets a minimum length. Planners like trajopt
 * need at least the user defined number of states in the trajectory to perform velocity, acceleration and jerk
 * smoothing.
 *
 * @author Levi Armstrong
 * @date November 2. 2020
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
#ifndef TESSERACT_TASK_COMPOSER_MIN_LENGTH_PROFILE_H
#define TESSERACT_TASK_COMPOSER_MIN_LENGTH_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/profile.h>

namespace tesseract_planning
{
struct MinLengthProfile : public Profile
{
  using Ptr = std::shared_ptr<MinLengthProfile>;
  using ConstPtr = std::shared_ptr<const MinLengthProfile>;

  MinLengthProfile();
  MinLengthProfile(long min_length);

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  long min_length{ 10 };

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::MinLengthProfile)

#endif  // TESSERACT_TASK_COMPOSER_MIN_LENGTH_PROFILE_H
