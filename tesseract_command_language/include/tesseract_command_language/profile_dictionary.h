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
#include <boost/serialization/serialization.hpp>
#include <boost/core/demangle.hpp>
#include <iostream>
#include <typeindex>
#include <unordered_map>
#include <memory>
#include <shared_mutex>
#include <sstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifdef SWIG
%shared_ptr(tesseract_planning::ProfileDictionary)
#endif  // SWIG

namespace tesseract_environment
{
class Environment;
}

namespace tesseract_planning
{
class Instruction;
class CompositeInstruction;

/**
 * @brief Struct to produce a planner-specific planning profile to apply to a single waypoint.
 * @details Examples of waypoint profiles might include costs/constraints for a waypoint or a waypoint sampler
 */
class WaypointProfile
{
public:
  using Ptr = std::shared_ptr<WaypointProfile>;
  using ConstPtr = std::shared_ptr<const WaypointProfile>;

  WaypointProfile() = default;
  WaypointProfile(const WaypointProfile&) = delete;
  WaypointProfile& operator=(const WaypointProfile&) = delete;
  WaypointProfile(WaypointProfile&&) = delete;
  WaypointProfile&& operator=(WaypointProfile&&) = delete;

  virtual ~WaypointProfile() = default;

  virtual std::any create(const Instruction& instruction, const tesseract_environment::Environment& env) const = 0;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int)  // NOLINT
  {
  }
};

/**
 * @brief Struct to produce a planner-specific planning profile to apply to a collection of waypoints defined in a
 * composite instruction.
 * @details Examples of composite profiles include costs/constraints that apply collectively to a group of waypoints
 */
class CompositeProfile
{
public:
  using Ptr = std::shared_ptr<CompositeProfile>;
  using ConstPtr = std::shared_ptr<const CompositeProfile>;

  CompositeProfile() = default;
  CompositeProfile(const CompositeProfile&) = delete;
  CompositeProfile& operator=(const CompositeProfile&) = delete;
  CompositeProfile(CompositeProfile&&) = delete;
  CompositeProfile&& operator=(CompositeProfile&&) = delete;

  virtual ~CompositeProfile() = default;
  virtual std::any create(const CompositeInstruction& instruction,
                          const tesseract_environment::Environment& env) const = 0;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int)  // NOLINT
  {
  }
};

/**
 * @brief Struct to produce configuration parameters for the motion planner
 */
struct PlannerProfile
{
public:
  using Ptr = std::shared_ptr<PlannerProfile>;
  using ConstPtr = std::shared_ptr<const PlannerProfile>;

  PlannerProfile() = default;
  PlannerProfile(const PlannerProfile&) = delete;
  PlannerProfile& operator=(const PlannerProfile&) = delete;
  PlannerProfile(PlannerProfile&&) = delete;
  PlannerProfile&& operator=(PlannerProfile&&) = delete;

  virtual ~PlannerProfile() = default;

  virtual std::any create() const = 0;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int)  // NOLINT
  {
  }
};

/**
 * @brief This class is used to store profiles for motion planning and process planning
 * @details This is a thread safe class
 *    A ProfileEntry<T> is a std::unordered_map<std::string, std::shared_ptr<const T>>
 *      - The key is the profile name
 *      - Where std::shared_ptr<const T> is the profile
 *    The ProfileEntry<T> is also stored in std::unordered_map where the key here is the std::type_index(typeid(T))
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
   * @brief Check if a profile exists
   * @details If profile entry does not exist it also returns false
   * @return True if profile exists, otherwise false
   */
  template <typename ProfileType>
  bool hasProfile(const std::string& ns, const std::string& profile_name) const
  {
    std::shared_lock lock(mutex_);
    auto it = profiles_.find(ns);
    if (it == profiles_.end())
      return false;

    auto it2 = it->second.find(std::type_index(typeid(ProfileType)));
    if (it2 != it->second.end())
    {
      const auto& profile_map =
          std::any_cast<const std::unordered_map<std::string, std::shared_ptr<const ProfileType>>&>(it2->second);
      auto it3 = profile_map.find(profile_name);
      if (it3 != profile_map.end())
        return true;
    }
    return false;
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
    const auto& it = profiles_.at(ns);
    const auto& it2 = it.at(std::type_index(typeid(ProfileType)));
    const auto& profile_map =
        std::any_cast<const std::unordered_map<std::string, std::shared_ptr<const ProfileType>>&>(it2);
    return profile_map.at(profile_name);
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

  std::unordered_map<std::string, std::unordered_map<std::string, WaypointProfile::ConstPtr>> waypoint_profiles;
  std::unordered_map<std::string, std::unordered_map<std::string, CompositeProfile::ConstPtr>> composite_profiles;
  std::unordered_map<std::string, std::unordered_map<std::string, PlannerProfile::ConstPtr>> planner_profiles;

  WaypointProfile::ConstPtr getWaypointProfile(const std::string& ns, const std::string& profile_name) const
  {
    return getProfile<WaypointProfile>(ns, profile_name, waypoint_profiles);
  }

  CompositeProfile::ConstPtr getCompositeProfile(const std::string& ns, const std::string& profile_name) const
  {
    return getProfile<CompositeProfile>(ns, profile_name, composite_profiles);
  }

  PlannerProfile::ConstPtr getPlannerProfile(const std::string& ns, const std::string& profile_name) const
  {
    return getProfile<PlannerProfile>(ns, profile_name, planner_profiles);
  }

protected:
  template <typename ProfileT>
  typename ProfileT::ConstPtr getProfile(
      const std::string& ns,
      const std::string& profile_name,
      const std::unordered_map<std::string, std::unordered_map<std::string, typename ProfileT::ConstPtr>>& map) const
  {
    try
    {
      return map.at(ns).at(profile_name);
    }
    catch (const std::out_of_range&)
    {
      std::stringstream ss;
      ss << "Failed to get " << boost::core::demangle(typeid(ProfileT).name()) << " for '" << ns << "/" << profile_name
         << "'\n";
      if (map.find(ns) == map.end())
      {
        ss << "Profile namespace '" << ns << "' does not exist";
      }
      else
      {
        ss << "Entries in profile namespace '" << ns << "' are:";
        for (auto it = map.at(ns).begin(); it != map.at(ns).end(); ++it)
        {
          ss << "\n\t" << it->first;
        }
      }

      throw std::out_of_range(ss.str());
    }
  }

  std::unordered_map<std::string, std::unordered_map<std::type_index, std::any>> profiles_;
  mutable std::shared_mutex mutex_;
};
}  // namespace tesseract_planning

BOOST_SERIALIZATION_ASSUME_ABSTRACT(tesseract_planning::WaypointProfile);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(tesseract_planning::CompositeProfile);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(tesseract_planning::PlannerProfile);

#endif  // TESSERACT_MOTION_PLANNERS_PROFILE_DICTIONARY_H
