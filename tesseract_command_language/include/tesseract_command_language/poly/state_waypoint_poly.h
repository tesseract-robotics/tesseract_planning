/**
 * @file state_waypoint_poly.h
 * @brief The state waypoint interface
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
#ifndef TESSERACT_COMMAND_LANGUAGE_STATE_WAYPOINT_POLY_H
#define TESSERACT_COMMAND_LANGUAGE_STATE_WAYPOINT_POLY_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <Eigen/Core>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_common/fwd.h>

namespace tesseract_planning
{
class StateWaypointInterface
{
public:
  virtual ~StateWaypointInterface() = default;

  // Waypoint
  virtual void setName(const std::string& name) = 0;
  virtual const std::string& getName() const = 0;
  virtual void print(const std::string& prefix = "") const = 0;
  virtual std::unique_ptr<StateWaypointInterface> clone() const = 0;

  // State Waypoint
  virtual void setNames(const std::vector<std::string>& names) = 0;
  virtual std::vector<std::string>& getNames() = 0;
  virtual const std::vector<std::string>& getNames() const = 0;

  virtual void setPosition(const Eigen::VectorXd& position) = 0;
  virtual Eigen::VectorXd& getPosition() = 0;
  virtual const Eigen::VectorXd& getPosition() const = 0;

  virtual void setVelocity(const Eigen::VectorXd& velocity) = 0;
  virtual Eigen::VectorXd& getVelocity() = 0;
  virtual const Eigen::VectorXd& getVelocity() const = 0;

  virtual void setAcceleration(const Eigen::VectorXd& acceleration) = 0;
  virtual Eigen::VectorXd& getAcceleration() = 0;
  virtual const Eigen::VectorXd& getAcceleration() const = 0;

  virtual void setEffort(const Eigen::VectorXd& effort) = 0;
  virtual Eigen::VectorXd& getEffort() = 0;
  virtual const Eigen::VectorXd& getEffort() const = 0;

  virtual void setTime(double time) = 0;
  virtual double getTime() const = 0;

  // Operators
  bool operator==(const StateWaypointInterface& rhs) const;
  bool operator!=(const StateWaypointInterface& rhs) const;

protected:
  virtual bool equals(const StateWaypointInterface& other) const = 0;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

class StateWaypointPoly final : public WaypointInterface
{
public:
  StateWaypointPoly() = default;  // Default constructor
  StateWaypointPoly(const StateWaypointPoly& other);
  StateWaypointPoly& operator=(const StateWaypointPoly& other);
  StateWaypointPoly(StateWaypointPoly&& other) noexcept = default;
  StateWaypointPoly& operator=(StateWaypointPoly&& other) noexcept = default;
  StateWaypointPoly(const StateWaypointInterface& impl);

  // Waypoint
  void setName(const std::string& name) override final;
  const std::string& getName() const override final;
  void print(const std::string& prefix = "") const override final;
  std::unique_ptr<WaypointInterface> clone() const override final;

  // State Waypoint
  void setNames(const std::vector<std::string>& names);
  std::vector<std::string>& getNames();
  const std::vector<std::string>& getNames() const;

  void setPosition(const Eigen::VectorXd& position);
  Eigen::VectorXd& getPosition();
  const Eigen::VectorXd& getPosition() const;

  void setVelocity(const Eigen::VectorXd& velocity);
  Eigen::VectorXd& getVelocity();
  const Eigen::VectorXd& getVelocity() const;

  void setAcceleration(const Eigen::VectorXd& acceleration);
  Eigen::VectorXd& getAcceleration();
  const Eigen::VectorXd& getAcceleration() const;

  void setEffort(const Eigen::VectorXd& effort);
  Eigen::VectorXd& getEffort();
  const Eigen::VectorXd& getEffort() const;

  void setTime(double time);
  double getTime() const;

  // Poly Methods

  /**
   * @brief Get the stored derived type
   * @return The derived type index
   */
  std::type_index getType() const;

  /**
   * @brief Check if the poly type is null
   * @return True if null, otherwise false
   */
  bool isNull() const;

  /**
   * @brief Get the interface object
   * @return The interface object
   */
  StateWaypointInterface& getStateWaypoint();
  const StateWaypointInterface& getStateWaypoint() const;

  template <typename T>
  T& as()
  {
    if (getType() != typeid(T))
      throw std::runtime_error("StateWaypointPoly, tried to cast '" + boost::core::demangle(getType().name()) +
                               "' to '" + boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<T*>(impl_.get());
  }

  template <typename T>
  const T& as() const
  {
    if (getType() != typeid(T))
      throw std::runtime_error("StateWaypointPoly, tried to cast '" + boost::core::demangle(getType().name()) +
                               "' to '" + boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<const T*>(impl_.get());
  }

  // Operators
  bool operator==(const StateWaypointPoly& rhs) const;
  bool operator!=(const StateWaypointPoly& rhs) const;

private:
  std::unique_ptr<StateWaypointInterface> impl_;

  bool equals(const WaypointInterface& other) const override final;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::StateWaypointInterface)
BOOST_CLASS_TRACKING(tesseract_planning::StateWaypointInterface, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::StateWaypointPoly)
BOOST_CLASS_TRACKING(tesseract_planning::StateWaypointPoly, boost::serialization::track_never)

#endif  // TESSERACT_COMMAND_LANGUAGE_STATE_WAYPOINT_POLY_H
