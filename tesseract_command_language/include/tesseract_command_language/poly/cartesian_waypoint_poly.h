/**
 * @file cartesian_waypoint_poly.h
 * @brief The cartesian waypoint interface
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
#ifndef TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_POLY_H
#define TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_POLY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <Eigen/Geometry>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_common/fwd.h>

namespace tesseract_planning
{
/**
 * @brief The CartesianWaypointInterface class
 */
class CartesianWaypointInterface
{
public:
  virtual ~CartesianWaypointInterface() = default;

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
  virtual std::unique_ptr<CartesianWaypointInterface> clone() const = 0;

  /////////////////////
  // Cartesian Waypoint
  /////////////////////

  /**
   * @brief Set transform
   * @param transform The transform to assign
   */
  virtual void setTransform(const Eigen::Isometry3d& transform) = 0;
  /**
   * @brief Get transform
   * @return The transform
   */
  virtual Eigen::Isometry3d& getTransform() = 0;
  virtual const Eigen::Isometry3d& getTransform() const = 0;

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
   * @brief Set the seed
   * @param seed The seed
   */
  virtual void setSeed(const tesseract_common::JointState& seed) = 0;
  /**
   * @brief Get the seed
   * @return The seed
   */
  virtual tesseract_common::JointState& getSeed() = 0;
  virtual const tesseract_common::JointState& getSeed() const = 0;

  // Operators
  bool operator==(const CartesianWaypointInterface& rhs) const;
  bool operator!=(const CartesianWaypointInterface& rhs) const;

protected:
  /**
   * @brief Check if two objects are equal
   * @param other The other object to compare with
   * @return True if equal, otherwise false
   */
  virtual bool equals(const CartesianWaypointInterface& other) const = 0;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

/**
 * @brief The CartesianWaypointPoly class
 */
class CartesianWaypointPoly final : public WaypointInterface
{
public:
  CartesianWaypointPoly() = default;  // Default constructor
  CartesianWaypointPoly(const CartesianWaypointPoly& other);
  CartesianWaypointPoly& operator=(const CartesianWaypointPoly& other);
  CartesianWaypointPoly(CartesianWaypointPoly&& other) noexcept = default;
  CartesianWaypointPoly& operator=(CartesianWaypointPoly&& other) noexcept = default;
  CartesianWaypointPoly(const CartesianWaypointInterface& impl);

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

  /////////////////////
  // Cartesian Waypoint
  /////////////////////

  /**
   * @brief Set transform
   * @param transform The transform to assign
   */
  void setTransform(const Eigen::Isometry3d& transform);
  /**
   * @brief Get transform
   * @return The transform
   */
  Eigen::Isometry3d& getTransform();
  const Eigen::Isometry3d& getTransform() const;

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
   * @brief Set the seed
   * @param seed The seed
   */
  void setSeed(const tesseract_common::JointState& seed);
  /**
   * @brief Get the seed
   * @return The seed
   */
  tesseract_common::JointState& getSeed();
  const tesseract_common::JointState& getSeed() const;

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
   * @brief Get the cartesian waypoint being stored
   * @return The cartesian waypoint
   * @throws If null
   */
  CartesianWaypointInterface& getCartesianWaypoint();
  const CartesianWaypointInterface& getCartesianWaypoint() const;

  /**
   * @brief Check if it has a seed. If the position or joint names is empty this returns false
   * @return True if it has a seed, otherwise false
   */
  bool hasSeed() const;

  /** @brief Clear the seed to empty data structures */
  void clearSeed();

  /**
   * @brief Returns true if waypoint has tolerances
   * @return True if waypoint has tolerances
   */
  bool isToleranced() const;

  template <typename T>
  T& as()
  {
    if (getType() != typeid(T))
      throw std::runtime_error("CartesianWaypointPoly, tried to cast '" + boost::core::demangle(getType().name()) +
                               "' to '" + boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<T*>(impl_.get());
  }

  template <typename T>
  const T& as() const
  {
    if (getType() != typeid(T))
      throw std::runtime_error("CartesianWaypointPoly, tried to cast '" + boost::core::demangle(getType().name()) +
                               "' to '" + boost::core::demangle(typeid(T).name()) + "'\nBacktrace:\n" +
                               boost::stacktrace::to_string(boost::stacktrace::stacktrace()) + "\n");

    return *dynamic_cast<const T*>(impl_.get());
  }

  // Operators
  bool operator==(const CartesianWaypointPoly& rhs) const;
  bool operator!=(const CartesianWaypointPoly& rhs) const;

private:
  std::unique_ptr<CartesianWaypointInterface> impl_;

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

BOOST_CLASS_EXPORT_KEY(tesseract_planning::CartesianWaypointInterface)
BOOST_CLASS_TRACKING(tesseract_planning::CartesianWaypointInterface, boost::serialization::track_never)

BOOST_CLASS_EXPORT_KEY(tesseract_planning::CartesianWaypointPoly)
BOOST_CLASS_TRACKING(tesseract_planning::CartesianWaypointPoly, boost::serialization::track_never)

#endif  // TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_POLY_H
