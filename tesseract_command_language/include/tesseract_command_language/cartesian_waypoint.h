/**
 * @file cartesian_waypoint.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/waypoint_type.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
class CartesianWaypoint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CartesianWaypoint() = default;

  void print(const std::string& prefix = "") const
  {
    std::cout << prefix << "Cart WP: xyz=" << this->translation().x() << ", " << this->translation().y()
              << ", "                                   // NOLINT
              << this->translation().z() << std::endl;  // NOLINT
    // TODO: Add rotation
  }

  /////////////////////
  // Eigen Container //
  /////////////////////

  using ConstLinearPart = Eigen::Isometry3d::ConstLinearPart;
  using LinearPart = Eigen::Isometry3d::LinearPart;
  using ConstTranslationPart = Eigen::Isometry3d::ConstTranslationPart;
  using TranslationPart = Eigen::Isometry3d::TranslationPart;

  ////////////////////////
  // Eigen Constructors //
  ////////////////////////

  // This constructor allows you to construct from Eigen expressions
  template <typename OtherDerived>
  CartesianWaypoint(const Eigen::MatrixBase<OtherDerived>& other) : waypoint(other)
  {
  }

  CartesianWaypoint(const Eigen::Isometry3d& other) : waypoint(other) {}

  ///////////////////
  // Eigen Methods //
  ///////////////////

#ifndef SWIG
  /** @returns a read-only expression of the linear part of the transformation */
  inline ConstLinearPart linear() const { return waypoint.linear(); }  // NOLINT

  /** @returns a writable expression of the linear part of the transformation */
  inline LinearPart linear() { return waypoint.linear(); }

  /** @returns a read-only expression of the translation vector of the transformation */
  inline ConstTranslationPart translation() const { return waypoint.translation(); }  // NOLINT

  /** @returns a writable expression of the translation vector of the transformation */
  inline TranslationPart translation() { return waypoint.translation(); }
#else   // SWIG
  Eigen::Matrix3d linear() const;
  Eigen::Matrix3d linear();
  Eigen::Vector3d translation() const;
  Eigen::Vector3d translation();
#endif  // SWIG

  /** @returns true if two are approximate */
  inline bool isApprox(const Eigen::Isometry3d& other, double prec = 1e-12) const
  {
    return waypoint.isApprox(other, prec);
  }  // NOLINT

  /////////////////////
  // Eigen Operators //
  /////////////////////

  // This method allows you to assign Eigen expressions to MyVectorType
  template <typename OtherDerived>
  inline CartesianWaypoint& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    waypoint = other;
    return *this;
  }

  template <typename OtherDerived>
  inline CartesianWaypoint& operator*=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    waypoint = waypoint * other;
    return *this;
  }

  template <typename OtherDerived>
  inline CartesianWaypoint operator*(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    CartesianWaypoint cwp = *this;
    cwp.waypoint = cwp.waypoint * other;
    return cwp;
  }

  inline CartesianWaypoint& operator=(const Eigen::Isometry3d& other)
  {
    waypoint = other;
    return *this;
  }

  inline CartesianWaypoint& operator*=(const Eigen::Isometry3d& other) { return *this = waypoint * other; }

  inline CartesianWaypoint operator*(const Eigen::Isometry3d& other) const
  {
    CartesianWaypoint cwp = *this;
    cwp.waypoint = cwp.waypoint * other;
    return cwp;
  }

  //////////////////////////
  // Implicit Conversions //
  //////////////////////////

  /** @return Implicit Conversions to read-only Eigen::Isometry3d */
  inline operator const Eigen::Isometry3d&() const { return waypoint; }

  /** @return Implicit Conversions to writable Eigen::Isometry3d */
  inline operator Eigen::Isometry3d&() { return waypoint; }

  //////////////////////////////////
  // Cartesian Waypoint Container //
  //////////////////////////////////

  /** @brief The Cartesian Waypoint */
  Eigen::Isometry3d waypoint{ Eigen::Isometry3d::Identity() };
  /** @brief Distance below waypoint that is allowed. Should be size = 6. First 3 elements are dx, dy, dz. The last 3
   * elements are angle axis error allowed (Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle()) */
  Eigen::VectorXd lower_tolerance;
  /** @brief Distance above waypoint that is allowed. Should be size = 6. First 3 elements are dx, dy, dz. The last 3
   * elements are angle axis error allowed (Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle())*/
  Eigen::VectorXd upper_tolerance;

  /**
   * @brief Returns true if waypoint is toleranced
   * @return True if waypoint is toleranced
   */
  bool isToleranced() const
  {
    // Check if they are empty
    if (lower_tolerance.size() == 0 || upper_tolerance.size() == 0)
      return false;

    // Check if they are the same
    static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

    if ((lower_tolerance.array() > max_diff).any())
      throw std::runtime_error("CartesianWaypoint: lower tolerance was provided but must be <= 0,");

    if ((upper_tolerance.array() < -max_diff).any())
      throw std::runtime_error("CartesianWaypoint: upper tolerance was provided but must be >= 0,");

    return !tesseract_common::almostEqualRelativeAndAbs(lower_tolerance, upper_tolerance, max_diff);
  }
  bool operator==(const CartesianWaypoint& rhs) const
  {
    static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

    bool equal = true;
    equal &= waypoint.isApprox(rhs.waypoint);
    equal &= tesseract_common::almostEqualRelativeAndAbs(lower_tolerance, rhs.lower_tolerance, max_diff);
    equal &= tesseract_common::almostEqualRelativeAndAbs(upper_tolerance, rhs.upper_tolerance, max_diff);
    return equal;
  }
  bool operator!=(const CartesianWaypoint& rhs) const { return !operator==(rhs); }

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_waypoint_type(CartesianWaypoint)
#else
TESSERACT_WAYPOINT_EXPORT_KEY(tesseract_planning::CartesianWaypoint);
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_H
