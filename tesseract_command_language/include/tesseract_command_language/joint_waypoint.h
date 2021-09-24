/**
 * @file joint_waypoint.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
class JointWaypoint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  JointWaypoint() = default;

  void print(const std::string& prefix = "") const;

#ifndef SWIG

  /////////////////////
  // Eigen Container //
  /////////////////////

  using ConstTransposeReturnType = Eigen::VectorXd::ConstTransposeReturnType;

  ////////////////////////
  // Eigen Constructors //
  ////////////////////////

  // This constructor allows you to construct MyVectorType from Eigen expressions
  template <typename OtherDerived>
  JointWaypoint(std::vector<std::string> joint_names, const Eigen::MatrixBase<OtherDerived>& other)
    : waypoint(other), joint_names(std::move(joint_names))
  {
    if (static_cast<Eigen::Index>(this->joint_names.size()) != this->waypoint.rows())
      throw std::runtime_error("JointWaypoint: joint_names is not the same size as position!");
  }

  JointWaypoint(std::vector<std::string> joint_names, std::initializer_list<double> l)
    : joint_names(std::move(joint_names))
  {
    waypoint.resize(static_cast<Eigen::Index>(l.size()));
    Eigen::Index i = 0;
    for (const auto& v : l)
      waypoint(i++) = v;

    if (static_cast<Eigen::Index>(this->joint_names.size()) != this->waypoint.rows())
      throw std::runtime_error("JointWaypoint: joint_names is not the same size as position!");
  }

  ///////////////////
  // Eigen Methods //
  ///////////////////

  /** @returns true if two are approximate */
  inline Eigen::Index size() const { return waypoint.size(); }
  /** @returns norm of vector */
  inline double norm() const { return waypoint.norm(); }
  /** @returns true if two are approximate */
  inline bool isApprox(const Eigen::VectorXd& other, double prec = 1e-12) { return waypoint.isApprox(other, prec); }
  /** @returns the transpose of the joint positions */
  inline ConstTransposeReturnType transpose() const { return waypoint.transpose(); }  // NOLINT

  /////////////////////
  // Eigen Operators //
  /////////////////////

  // This method allows you to assign Eigen expressions to MyVectorType
  template <typename OtherDerived>
  inline JointWaypoint& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    waypoint = other;
    return *this;
  }

  template <typename OtherDerived>
  inline JointWaypoint& operator*=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    waypoint = waypoint * other;
    return *this;
  }

  inline JointWaypoint& operator=(std::initializer_list<double> l)
  {
    waypoint.resize(static_cast<Eigen::Index>(l.size()));
    Eigen::Index i = 0;
    for (const auto& v : l)
      waypoint(i++) = v;

    return *this;
  }

  template <typename OtherDerived>
  inline JointWaypoint operator*(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    JointWaypoint jwp = *this;
    jwp.waypoint = jwp.waypoint * other;
    return jwp;
  }

  template <typename OtherDerived>
  inline JointWaypoint operator-(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    JointWaypoint jwp = *this;
    jwp.waypoint = jwp.waypoint - other;
    return jwp;
  }

  template <typename OtherDerived>
  inline JointWaypoint operator+(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    JointWaypoint jwp = *this;
    jwp.waypoint = jwp.waypoint + other;
    return jwp;
  }

  inline JointWaypoint operator-(const JointWaypoint& other) const
  {
    JointWaypoint jwp = *this;
    jwp.waypoint = jwp.waypoint - other.waypoint;
    return jwp;
  }

  template <typename OtherDerived>
  inline JointWaypoint operator+(const JointWaypoint& other) const
  {
    JointWaypoint jwp = *this;
    jwp.waypoint = jwp.waypoint + other.waypoint;
    return jwp;
  }

  inline Eigen::CommaInitializer<Eigen::VectorXd> operator<<(const double& s)
  {
    return Eigen::CommaInitializer<Eigen::VectorXd>(waypoint, s);
  }

  inline double& operator[](Eigen::Index i) { return waypoint[i]; }
  inline double operator[](Eigen::Index i) const { return waypoint[i]; }
  inline double& operator()(Eigen::Index i) { return waypoint(i); }
  inline double operator()(Eigen::Index i) const { return waypoint(i); }

  //////////////////////////
  // Implicit Conversions //
  //////////////////////////

  /** @return Implicit Conversions to read-only Eigen::VectorXd */
  inline operator const Eigen::VectorXd&() const { return waypoint; }

  /** @return Implicit Conversions to writable Eigen::VectorXd */
  inline operator Eigen::VectorXd&() { return waypoint; }

  /** @return Implicit Conversions to read-only Eigen::VectorXd */
  inline operator Eigen::Ref<const Eigen::VectorXd>() const { return Eigen::Ref<const Eigen::VectorXd>(waypoint); }

  /** @return Implicit Conversions to writable Eigen::VectorXd */
  inline operator Eigen::Ref<Eigen::VectorXd>() { return Eigen::Ref<Eigen::VectorXd>(waypoint); }

  //////////////////////////////////
  // Joint Waypoint Container //
  //////////////////////////////////

#endif  // SWIG

  Eigen::VectorXd waypoint;
  std::vector<std::string> joint_names;
  /** @brief Joint distance below waypoint that is allowed. Each element should be <= 0 */
  Eigen::VectorXd lower_tolerance;
  /** @brief Joint distance above waypoint that is allowed. Each element should be >= 0 */
  Eigen::VectorXd upper_tolerance;

  /**
   * @brief Returns true if waypoint is toleranced
   * @return True if waypoint is toleranced
   */
  bool isToleranced() const;

  bool operator==(const JointWaypoint& rhs) const;
  bool operator!=(const JointWaypoint& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_planning

#ifdef SWIG
%extend tesseract_planning::JointWaypoint {
  JointWaypoint(std::vector<std::string> joint_names, const Eigen::VectorXd& other)
  {
    return new tesseract_planning::JointWaypoint(joint_names, other);
  }
}

%tesseract_command_language_add_waypoint_type(JointWaypoint)
#else
TESSERACT_WAYPOINT_EXPORT_KEY(tesseract_planning::JointWaypoint);
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H
