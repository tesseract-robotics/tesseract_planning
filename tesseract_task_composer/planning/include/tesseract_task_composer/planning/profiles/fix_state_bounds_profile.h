/**
 * @file fix_state_bounds_profile.h
 * @brief Profile for process that pushes plan instructions back within joint limits
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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
#ifndef TESSERACT_TASK_COMPOSER_FIX_STATE_BOUNDS_PROFILE_H
#define TESSERACT_TASK_COMPOSER_FIX_STATE_BOUNDS_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <limits>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/profile.h>

namespace tesseract_planning
{
struct FixStateBoundsProfile : public Profile
{
  using Ptr = std::shared_ptr<FixStateBoundsProfile>;
  using ConstPtr = std::shared_ptr<const FixStateBoundsProfile>;

  enum class Settings
  {
    START_ONLY,
    END_ONLY,
    ALL,
    DISABLED
  };

  FixStateBoundsProfile(Settings mode = Settings::ALL);

  /**
   * @brief A utility function for getting profile ID
   * @return The profile ID used when storing in profile dictionary
   */
  static std::size_t getStaticKey();

  /** @brief Sets which terms will be corrected  */
  Settings mode;

  /** @brief Maximum amount the process is allowed to correct. If deviation is further than this, it will fail */
  double max_deviation_global = std::numeric_limits<double>::max();

  /** @brief Amount to reduce the upper bounds before clamping limits. Should be > 1 */
  double upper_bounds_reduction{ std::numeric_limits<float>::epsilon() };

  /** @brief Amount to increase the lower bounds before clamping limits. Should be > 1 */
  double lower_bounds_reduction{ std::numeric_limits<float>::epsilon() };

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::FixStateBoundsProfile)

#endif  // TESSERACT_TASK_COMPOSER_FIX_STATE_BOUNDS_PROFILE_H
