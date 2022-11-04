/**
 * @file profile_dictionary.h
 * @brief This is a profile dictionary for storing all profiles
 *
 * @author Levi Armstrong
 * @date December 2, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_PROFILE_DICTIONARY_H
#define TESSERACT_MOTION_PLANNERS_PROFILE_DICTIONARY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <any>
#include <iostream>
#include <typeindex>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <boost/core/demangle.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
/**
 * @brief This class is used to store profiles for motion planning and process planning
 * @details This is a thread safe class
 *    A ProfileEntry<T> is a std::unordered_map<std::string, std::shared_ptr<const T>>
 *      - The key is the profile name
 *      - Where std::shared_ptr<const T> is the profile
 *    The ProfleEntry<T> is also stored in std::unordered_map where the key here is the std::type_index(typeid(T))
 * @note When adding a profile entry the T should be the base class type.
 */
class ProfileDictionary
{
public:
  using Ptr = std::shared_ptr<ProfileDictionary>;
  using ConstPtr = std::shared_ptr<const ProfileDictionary>;

  /**
   * @brief Check if a profile entry exists
   * @param ns The namesspace to search under
   * @return True if exists, otherwise false
   */
  template <typename ProfileType>
  bool hasProfileEntry(const std::string& ns) const
  {
    std::shared_lock lock(mutex_);
    auto it = profiles_.find(ns);
    if (it == profiles_.end())
      return false;

    return (it->second.find(std::type_index(typeid(ProfileType))) != it->second.end());
  }

  /** @brief Remove a profile entry */
  template <typename ProfileType>
  void removeProfileEntry(const std::string& ns)
  {
    std::unique_lock lock(mutex_);

    auto it = profiles_.find(ns);
    if (it == profiles_.end())
      return;

    it->second.erase(std::type_index(typeid(ProfileType)));
  }

  /**
   * @brief Get a profile entry
   * @return The profile map associated with the profile entry
   */
  template <typename ProfileType>
  std::unordered_map<std::string, std::shared_ptr<const ProfileType>> getProfileEntry(const std::string& ns) const
  {
    std::shared_lock lock(mutex_);
    auto it = profiles_.find(ns);
    if (it == profiles_.end())
      throw std::runtime_error("Profile namespace does not exist for '" + ns + "'!");

    auto it2 = it->second.find(std::type_index(typeid(ProfileType)));
    if (it2 != it->second.end())
      return std::any_cast<const std::unordered_map<std::string, std::shared_ptr<const ProfileType>>&>(it2->second);

    throw std::runtime_error("Profile entry does not exist for type name '" +
                             std::string(std::type_index(typeid(ProfileType)).name()) + "' in namespace '" + ns + "'!");
  }

  /**
   * @brief Add a profile
   * @details If the profile entry does not exist it will create one
   * @param ns The profile namespace
   * @param profile_name The profile name
   * @param profile The profile to add
   */
  template <typename ProfileType>
  void addProfile(const std::string& ns, const std::string& profile_name, std::shared_ptr<const ProfileType> profile)
  {
    if (ns.empty())
      throw std::runtime_error("Adding profile with an empty namespace!");

    if (profile_name.empty())
      throw std::runtime_error("Adding profile with an empty string as the key!");

    if (profile == nullptr)
      throw std::runtime_error("Adding profile that is a nullptr");

    std::unique_lock lock(mutex_);
    auto it = profiles_.find(ns);
    if (it == profiles_.end())
    {
      std::unordered_map<std::string, std::shared_ptr<const ProfileType>> new_entry;
      new_entry[profile_name] = profile;
      profiles_[ns][std::type_index(typeid(ProfileType))] = new_entry;
    }
    else
    {
      auto it2 = it->second.find(std::type_index(typeid(ProfileType)));
      if (it2 != it->second.end())
      {
        std::any_cast<std::unordered_map<std::string, std::shared_ptr<const ProfileType>>&>(it2->second)[profile_name] =
            profile;
      }
      else
      {
        std::unordered_map<std::string, std::shared_ptr<const ProfileType>> new_entry;
        new_entry[profile_name] = profile;
        it->second[std::type_index(typeid(ProfileType))] = new_entry;
      }
    }
  }

  /**
   * @brief Get a profile by name
   * @details Check if the profile exist before calling this function, if missing an exception is thrown
   * @param profile_name The profile name
   * @return The profile
   */
  template <typename ProfileType>
  std::shared_ptr<const ProfileType> getProfile(const std::string& ns, const std::string& profile_name) const
  {
    std::shared_lock lock(mutex_);

    const std::unordered_map<std::type_index, std::any>* it = nullptr;
    try
    {
      it = &(profiles_.at(ns));
    }
    catch (const std::exception&)
    {
      std::stringstream ss;
      ss << "Failed to find entries for namespace '" << ns << "'. Available namespaces are: ";
      for (auto it = profiles_.begin(); it != profiles_.end(); ++it)
      {
        ss << "'" << it->first << "', ";
      }
      std::throw_with_nested(std::runtime_error(ss.str()));
    }

    const std::any* it2 = nullptr;
    try
    {
      it2 = &(it->at(std::type_index(typeid(ProfileType))));
    }
    catch (const std::exception&)
    {
      std::stringstream ss;
      ss << "No entries for profile base class type '" << boost::core::demangle(typeid(ProfileType).name()) << "'";
      std::throw_with_nested(std::runtime_error(ss.str()));
    }

    const auto& profile_map =
        std::any_cast<const std::unordered_map<std::string, std::shared_ptr<const ProfileType>>&>(*it2);
    try
    {
      return profile_map.at(profile_name);
    }
    catch (const std::exception&)
    {
      std::stringstream ss;
      ss << "No entries for profile '" << profile_name << "' in namespace '" << ns << "'. Available profiles are: ";
      for (auto it = profile_map.begin(); it != profile_map.end(); ++it)
        ss << "'" << it->first << "', ";

      std::throw_with_nested(std::runtime_error(ss.str()));
    }
  }

  /**
   * @brief Remove a profile
   * @param profile_name The profile to be removed
   */
  template <typename ProfileType>
  void removeProfile(const std::string& ns, const std::string& profile_name)
  {
    std::unique_lock lock(mutex_);
    auto it = profiles_.find(ns);
    if (it == profiles_.end())
      return;

    auto it2 = it->second.find(std::type_index(typeid(ProfileType)));
    if (it2 != it->second.end())
      std::any_cast<std::unordered_map<std::string, std::shared_ptr<const ProfileType>>&>(it2->second)
          .erase(profile_name);
  }

protected:
  std::unordered_map<std::string, std::unordered_map<std::type_index, std::any>> profiles_;
  mutable std::shared_mutex mutex_;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_PROFILE_DICTIONARY_H
