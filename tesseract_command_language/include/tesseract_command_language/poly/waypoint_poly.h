/**
 * @file waypoint.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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
#ifndef TESSERACT_COMMAND_LANGUAGE_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <boost/stacktrace.hpp>
#include <boost/core/demangle.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <string>
#include <memory>
#include <typeindex>
#include <tesseract_common/serialization.h>

namespace tesseract_planning
{
class WaypointInterface
{
public:
  virtual ~WaypointInterface() = default;

  virtual void setName(const std::string& name) = 0;
  virtual const std::string& getName() const = 0;
  virtual void print(const std::string& prefix = "") const = 0;
  virtual std::unique_ptr<WaypointInterface> clone() const = 0;

  // Operators
  bool operator==(const WaypointInterface& rhs) const;
  bool operator!=(const WaypointInterface& rhs) const;

protected:
  virtual bool equals(const WaypointInterface& other) const = 0;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

class WaypointPoly
{
public:
  WaypointPoly() = default;  // Default constructor
  WaypointPoly(const WaypointPoly& other);
  WaypointPoly& operator=(const WaypointPoly& other);
  WaypointPoly(WaypointPoly&& other) noexcept = default;
  WaypointPoly& operator=(WaypointPoly&& other) noexcept = default;

  WaypointPoly(const WaypointInterface& impl);

  void setName(const std::string& name);
  const std::string& getName() const;

  std::type_index getType() const;

  void print(const std::string& prefix = "") const;

  bool isNull() const;

  WaypointInterface& getWaypoint();
  const WaypointInterface& getWaypoint() const;

  bool isCartesianWaypoint() const;

  bool isJointWaypoint() const;

  bool isStateWaypoint() const;

  // Type Casting
  template <typename T>
  T& as()
  {
    if (getType() != typeid(T))
      throw std::runtime_error("WaypointPoly, tried to cast '" + boost::core::demangle(getType().name()) + "' to '" +
                               boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<T*>(impl_.get());
  }

  template <typename T>
  const T& as() const
  {
    if (getType() != typeid(T))
      throw std::runtime_error("WaypointPoly, tried to cast '" + boost::core::demangle(getType().name()) + "' to '" +
                               boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<const T*>(impl_.get());
  }

  // Operators
  bool operator==(const WaypointPoly& rhs) const;
  bool operator!=(const WaypointPoly& rhs) const;

private:
  std::unique_ptr<WaypointInterface> impl_;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::WaypointInterface)
BOOST_CLASS_TRACKING(tesseract_planning::WaypointInterface, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::WaypointPoly)
BOOST_CLASS_TRACKING(tesseract_planning::WaypointPoly, boost::serialization::track_never)

#endif  // TESSERACT_COMMAND_LANGUAGE_WAYPOINT_H
