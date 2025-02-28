/**
 * @file joint_waypoint_poly.h
 * @brief The joint waypoint interface
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
#ifndef TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_POLY_H
#define TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_POLY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <memory>
#include <Eigen/Core>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_common/fwd.h>

namespace tesseract_planning
{
/**
 * @brief The JointWaypointInterface class
 */
class JointWaypointInterface
{
public:
  virtual ~JointWaypointInterface() = default;

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
  virtual std::unique_ptr<JointWaypointInterface> clone() const = 0;

  /////////////////
  // Joint Waypoint
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
   * @brief Set the upper tolerance
   * @param upper_tol The upper tolerance to assign
   */
  virtual void setUpperTolerance(const Eigen::VectorXd& upper_tol) = 0;
  /**
   * @brief Get the upper tolerance
   * @return The upper tolerance
   */
  virtual Eigen::VectorXd& getUpperTolerance() = 0;
  virtual const Eigen::VectorXd& getUpperTolerance() const = 0;

  /**
   * @brief Set the lower tolerance
   * @param lower_tol The lower tolerance to assign
   */
  virtual void setLowerTolerance(const Eigen::VectorXd& lower_tol) = 0;
  /**
   * @brief Get the lower tolerance
   * @return The lower tolerance
   */
  virtual Eigen::VectorXd& getLowerTolerance() = 0;
  virtual const Eigen::VectorXd& getLowerTolerance() const = 0;

  /**
   * @brief Set if the waypoint should be considered constrained
   * @param value True if constrained, otherwise false
   */
  virtual void setIsConstrained(bool value) = 0;
  /**
   * @brief Check if constrained
   * @return True if constrained, otherwise false
   */
  virtual bool isConstrained() const = 0;

  // Operators
  bool operator==(const JointWaypointInterface& rhs) const;
  bool operator!=(const JointWaypointInterface& rhs) const;

protected:
  virtual bool equals(const JointWaypointInterface& other) const = 0;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/**
 * @brief The JointWaypointPoly class
 */
class JointWaypointPoly final : public WaypointInterface
{
public:
  JointWaypointPoly() = default;  // Default constructor
  JointWaypointPoly(const JointWaypointPoly& other);
  JointWaypointPoly& operator=(const JointWaypointPoly& other);
  JointWaypointPoly(JointWaypointPoly&& other) noexcept = default;
  JointWaypointPoly& operator=(JointWaypointPoly&& other) noexcept = default;
  JointWaypointPoly(const JointWaypointInterface& impl);

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
  // Joint Waypoint
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
   * @brief Set the upper tolerance
   * @param upper_tol The upper tolerance to assign
   */
  void setUpperTolerance(const Eigen::VectorXd& upper_tol);
  /**
   * @brief Get the upper tolerance
   * @return The upper tolerance
   */
  Eigen::VectorXd& getUpperTolerance();
  const Eigen::VectorXd& getUpperTolerance() const;

  /**
   * @brief Set the lower tolerance
   * @param lower_tol The lower tolerance to assign
   */
  void setLowerTolerance(const Eigen::VectorXd& lower_tol);
  /**
   * @brief Get the lower tolerance
   * @return The lower tolerance
   */
  Eigen::VectorXd& getLowerTolerance();
  const Eigen::VectorXd& getLowerTolerance() const;

  /**
   * @brief Set if the waypoint should be considered constrained
   * @param value True if constrained, otherwise false
   */
  void setIsConstrained(bool value);
  /**
   * @brief Check if constrained
   * @return True if constrained, otherwise false
   */
  bool isConstrained() const;

  /**
   * @brief Returns true if waypoint has tolerances
   * @return True if waypoint has tolerances
   */
  bool isToleranced() const;

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
   * @brief Get the joint waypoint being stored
   * @return The joint waypoint
   * @throws If null
   */
  JointWaypointInterface& getJointWaypoint();
  const JointWaypointInterface& getJointWaypoint() const;

  template <typename T>
  T& as()
  {
    if (getType() != typeid(T))
      throw std::runtime_error("JointWaypointPoly, tried to cast '" + boost::core::demangle(getType().name()) +
                               "' to '" + boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<T*>(impl_.get());
  }

  template <typename T>
  const T& as() const
  {
    if (getType() != typeid(T))
      throw std::runtime_error("JointWaypointPoly, tried to cast '" + boost::core::demangle(getType().name()) +
                               "' to '" + boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<const T*>(impl_.get());
  }

  // Operators
  bool operator==(const JointWaypointPoly& rhs) const;
  bool operator!=(const JointWaypointPoly& rhs) const;

private:
  std::unique_ptr<JointWaypointInterface> impl_;

  /**
   * @brief Check if two objects are equal
   * @param other The other object to compare with
   * @return True if equal, otherwise false
   */
  bool equals(const WaypointInterface& other) const override final;

  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::JointWaypointInterface)
BOOST_CLASS_TRACKING(tesseract_planning::JointWaypointInterface, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::JointWaypointPoly)
BOOST_CLASS_TRACKING(tesseract_planning::JointWaypointPoly, boost::serialization::track_never)

#endif  // TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_POLY_H
