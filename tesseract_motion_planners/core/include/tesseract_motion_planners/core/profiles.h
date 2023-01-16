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
#ifndef TESSERACT_MOTION_PLANNERS_CORE_PROFILES_H
#define TESSERACT_MOTION_PLANNERS_CORE_PROFILES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <any>
#include <iostream>
#include <typeindex>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <shared_mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
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

  virtual std::any create(const MoveInstruction& instruction,
                          tesseract_environment::Environment::ConstPtr env) const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
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
                          tesseract_environment::Environment::ConstPtr env) const = 0;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
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

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_SERIALIZATION_ASSUME_ABSTRACT(tesseract_planning::WaypointProfile);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(tesseract_planning::CompositeProfile);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(tesseract_planning::PlannerProfile);

#endif  // TESSERACT_MOTION_PLANNERS_CORE_PROFILES_H
