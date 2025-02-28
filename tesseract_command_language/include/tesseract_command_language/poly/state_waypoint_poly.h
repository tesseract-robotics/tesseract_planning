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
/**
 * @brief The StateWaypointInterface class
 */
class StateWaypointInterface
{
public:
  virtual ~StateWaypointInterface() = default;

  ///////////
  // Waypoint
  ///////////

  /**
   * @brief Set the name of the waypoint
   * @param name The name of the waypoint
   */
  virtual void setName(const std::string& name) = 0;

  /**
   * @brief Get the name of the waypoint
   * @return The name of the waypoint
   */
  virtual const std::string& getName() const = 0;

  /**
   * @brief Output the contents to std::cout
   * @param prefix The prefix to add to each variable
   */
  virtual void print(const std::string& prefix = "") const = 0;

  /**
   * @brief Make a deep copy of the object
   * @return A deep copy
   */
  virtual std::unique_ptr<StateWaypointInterface> clone() const = 0;

  /////////////////
  // State Waypoint
  /////////////////

  /**
   * @brief Set the joint names
   * @param names The joint names
   */
  virtual void setNames(const std::vector<std::string>& names) = 0;
  /**
   * @brief Get the joint names
   * @return The joint names
   */
  virtual std::vector<std::string>& getNames() = 0;
  virtual const std::vector<std::string>& getNames() const = 0;

  /**
   * @brief Set the joint positions
   * @param position The joint positions
   */
  virtual void setPosition(const Eigen::VectorXd& position) = 0;
  /**
   * @brief Get the joint positions
   * @return The joint positions
   */
  virtual Eigen::VectorXd& getPosition() = 0;
  virtual const Eigen::VectorXd& getPosition() const = 0;

  /**
   * @brief Set the joint velocity
   * @param position The joint velocity
   */
  virtual void setVelocity(const Eigen::VectorXd& velocity) = 0;
  /**
   * @brief Get the joint velocity
   * @return The joint velocity
   */
  virtual Eigen::VectorXd& getVelocity() = 0;
  virtual const Eigen::VectorXd& getVelocity() const = 0;

  /**
   * @brief Set the joint acceleration
   * @param position The joint acceleration
   */
  virtual void setAcceleration(const Eigen::VectorXd& acceleration) = 0;
  /**
   * @brief Get the joint acceleration
   * @return The joint acceleration
   */
  virtual Eigen::VectorXd& getAcceleration() = 0;
  virtual const Eigen::VectorXd& getAcceleration() const = 0;

  /**
   * @brief Set the joint effort
   * @param position The joint effort
   */
  virtual void setEffort(const Eigen::VectorXd& effort) = 0;
  /**
   * @brief Get the joint effort
   * @return The joint effort
   */
  virtual Eigen::VectorXd& getEffort() = 0;
  virtual const Eigen::VectorXd& getEffort() const = 0;

  /**
   * @brief Set the time from start
   * @param time The time from start
   */
  virtual void setTime(double time) = 0;
  /**
   * @brief Get the time from start
   * @return The time from start
   */
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

/**
 * @brief The StateWaypointPoly class
 */
class StateWaypointPoly final : public WaypointInterface
{
public:
  StateWaypointPoly() = default;  // Default constructor
  StateWaypointPoly(const StateWaypointPoly& other);
  StateWaypointPoly& operator=(const StateWaypointPoly& other);
  StateWaypointPoly(StateWaypointPoly&& other) noexcept = default;
  StateWaypointPoly& operator=(StateWaypointPoly&& other) noexcept = default;
  StateWaypointPoly(const StateWaypointInterface& impl);

  ///////////
  // Waypoint
  ///////////

  /**
   * @brief Set the name of the waypoint
   * @param name The name of the waypoint
   */
  void setName(const std::string& name) override final;
  /**
   * @brief Get the name of the waypoint
   * @return The name of the waypoint
   */
  const std::string& getName() const override final;
  /**
   * @brief Output the contents to std::cout
   * @param prefix The prefix to add to each variable
   */
  void print(const std::string& prefix = "") const override final;

  /**
   * @brief Make a deep copy of the object
   * @return A deep copy
   */
  std::unique_ptr<WaypointInterface> clone() const override final;

  /////////////////
  // State Waypoint
  /////////////////

  /**
   * @brief Set the joint names
   * @param names The joint names
   */
  void setNames(const std::vector<std::string>& names);
  /**
   * @brief Get the joint names
   * @return The joint names
   */
  std::vector<std::string>& getNames();
  const std::vector<std::string>& getNames() const;

  /**
   * @brief Set the joint positions
   * @param position The joint positions
   */
  void setPosition(const Eigen::VectorXd& position);
  /**
   * @brief Get the joint positions
   * @return The joint positions
   */
  Eigen::VectorXd& getPosition();
  const Eigen::VectorXd& getPosition() const;

  /**
   * @brief Set the joint velocity
   * @param position The joint velocity
   */
  void setVelocity(const Eigen::VectorXd& velocity);
  /**
   * @brief Get the joint velocity
   * @return The joint velocity
   */
  Eigen::VectorXd& getVelocity();
  const Eigen::VectorXd& getVelocity() const;

  /**
   * @brief Set the joint acceleration
   * @param position The joint acceleration
   */
  void setAcceleration(const Eigen::VectorXd& acceleration);
  /**
   * @brief Get the joint acceleration
   * @return The joint acceleration
   */
  Eigen::VectorXd& getAcceleration();
  const Eigen::VectorXd& getAcceleration() const;

  /**
   * @brief Set the joint effort
   * @param position The joint effort
   */
  void setEffort(const Eigen::VectorXd& effort);
  /**
   * @brief Get the joint effort
   * @return The joint effort
   */
  Eigen::VectorXd& getEffort();
  const Eigen::VectorXd& getEffort() const;

  /**
   * @brief Set the time from start
   * @param time The time from start
   */
  void setTime(double time);
  /**
   * @brief Get the time from start
   * @return The time from start
   */
  double getTime() const;

  ///////////////
  // Poly Methods
  ///////////////

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
   * @brief Get the state waypoint being stored
   * @return The state waypoint
   * @throws If null
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

  /**
   * @brief Check if two objects are equal
   * @param other The other object to compare with
   * @return True if equal, otherwise false
   */
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
