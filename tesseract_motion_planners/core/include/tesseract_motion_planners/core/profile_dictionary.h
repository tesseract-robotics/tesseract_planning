/**
 * @file profile_dictionary.h
 * @brief This is a profile dictionary for storing all profiles
 *
 * @author Michael Ripperger
 * @date January 11, 2022
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
#ifndef TESSERACT_MOTION_PLANNERS_CORE_PROFILE_DICTIONARY_H
#define TESSERACT_MOTION_PLANNERS_CORE_PROFILE_DICTIONARY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <any>
#include <boost/serialization/serialization.hpp>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <shared_mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifdef SWIG
%shared_ptr(tesseract_planning::ProfileDictionary)
#endif  // SWIG
#include <tesseract_motion_planners/core/profiles.h>

namespace tesseract_planning::tmp
{
/**
 * @brief Stores profiles for motion planning and process planning
 * @details This class is thread-safe
 */
template <typename ProfileT>
class ProfileDictionary
{
public:
  using Ptr = std::shared_ptr<ProfileDictionary<ProfileT>>;
  using ConstPtr = std::shared_ptr<const ProfileDictionary<ProfileT>>;

  ProfileDictionary() = default;
  virtual ~ProfileDictionary() = default;

  ProfileDictionary(const ProfileDictionary& rhs);
  ProfileDictionary(ProfileDictionary&& rhs) noexcept;
  ProfileDictionary& operator=(const ProfileDictionary& rhs);
  ProfileDictionary& operator=(ProfileDictionary&& rhs) noexcept;

  /**
   * @brief Checks if a profile exists in the provided namespace
   * @return True if exists, false otherwise
   */
  bool hasProfile(const std::string& ns, const std::string& profile) const;

  /**
   * @brief Gets a profile by name under a given namespace
   * @throws Runtime exception if the namespace or profile do not exist
   */
  typename ProfileT::ConstPtr getProfile(const std::string& ns, const std::string& profile_name) const;

  /**
   * @brief Gets a profile entry from a given namespace
   */
  std::unordered_map<std::string, typename ProfileT::ConstPtr> getProfileEntry(const std::string& ns) const;

  /**
   * @brief Adds a profile with the input name under the given namespace
   */
  void addProfile(const std::string& ns, const std::string& profile_name, typename ProfileT::ConstPtr profile);

  /**
   * @brief Removes a profile entry from a given namespace
   */
  void removeProfile(const std::string& ns, const std::string& profile);

protected:
  std::unordered_map<std::string, std::unordered_map<std::string, typename ProfileT::ConstPtr>> profiles_;
  mutable std::shared_mutex mutex_;
};

using WaypointProfileDictionary = ProfileDictionary<WaypointProfile>;
using CompositeProfileDictionary = ProfileDictionary<CompositeProfile>;
using PlannerProfileDictionary = ProfileDictionary<PlannerProfile>;

}  // namespace tesseract_planning::tmp

#endif  // TESSERACT_MOTION_PLANNERS_CORE_PROFILE_DICTIONARY_H
