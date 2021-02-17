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
#include <tinyxml2.h>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/waypoint_type.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
class JointWaypoint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  JointWaypoint() = default;

  // This constructs a Joint Waypoint from xml element
  JointWaypoint(const tinyxml2::XMLElement& xml_element)
  {
    const tinyxml2::XMLElement* names_element = xml_element.FirstChildElement("Names");
    const tinyxml2::XMLElement* position_element = xml_element.FirstChildElement("Position");
    const tinyxml2::XMLElement* upper_tolerance_element = xml_element.FirstChildElement("UpperTolerance");
    const tinyxml2::XMLElement* lower_tolerance_element = xml_element.FirstChildElement("LowerTolerance");

    if (!names_element)
      throw std::runtime_error("JointWaypoint: Must have Names element.");

    if (!position_element)
      throw std::runtime_error("JointWaypoint: Must have Position element.");

    std::vector<std::string> names_tokens, position_tokens, upper_tolerance_tokens, lower_tolerance_tokens;
    std::string names_string;
    tinyxml2::XMLError status = tesseract_common::QueryStringText(names_element, names_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("JointWaypoint: Error parsing Names string");

    std::string position_string;
    status = tesseract_common::QueryStringText(position_element, position_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("JointWaypoint: Error parsing Position string");

    std::string upper_tolerance_string;
    if (upper_tolerance_element)
    {
      status = tesseract_common::QueryStringText(upper_tolerance_element, upper_tolerance_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("JointWaypoint: Error parsing UpperTolerance string");
    }

    std::string lower_tolerance_string;
    if (lower_tolerance_element)
    {
      status = tesseract_common::QueryStringText(lower_tolerance_element, lower_tolerance_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("JointWaypoint: Error parsing LowerTolerance string");
    }

    boost::split(names_tokens, names_string, boost::is_any_of(" "), boost::token_compress_on);
    boost::split(position_tokens, position_string, boost::is_any_of(" "), boost::token_compress_on);

    if (names_tokens.empty() || position_tokens.empty())
      throw std::runtime_error("JointWaypoint: Names or Position elements are empty.");

    if (names_tokens.size() != position_tokens.size())
      throw std::runtime_error("JointWaypoint: Names and Position are not the same size.");

    if (!tesseract_common::isNumeric(position_tokens))
      throw std::runtime_error("JointWaypoint: Positions are not all numeric values.");

    joint_names = names_tokens;
    waypoint.resize(static_cast<long>(names_tokens.size()));
    for (long i = 0; i < static_cast<long>(position_tokens.size()); ++i)
      tesseract_common::toNumeric<double>(position_tokens[static_cast<std::size_t>(i)], waypoint[i]);

    if (!upper_tolerance_string.empty() && !lower_tolerance_string.empty())
    {
      boost::split(lower_tolerance_tokens, lower_tolerance_string, boost::is_any_of(" "), boost::token_compress_on);
      boost::split(upper_tolerance_tokens, upper_tolerance_string, boost::is_any_of(" "), boost::token_compress_on);

      if (upper_tolerance_tokens.size() != lower_tolerance_tokens.size())
        throw std::runtime_error("JointWaypoint: UpperTolerance and LowerTolerance are not the same size.");

      if (!tesseract_common::isNumeric(upper_tolerance_tokens))
        throw std::runtime_error("JointWaypoint: UpperTolerance are not all numeric values.");

      if (!tesseract_common::isNumeric(lower_tolerance_tokens))
        throw std::runtime_error("JointWaypoint: LowerTolerance are not all numeric values.");

      upper_tolerance.resize(static_cast<Eigen::Index>(upper_tolerance_tokens.size()));
      lower_tolerance.resize(static_cast<Eigen::Index>(lower_tolerance_tokens.size()));
      for (long i = 0; i < static_cast<long>(upper_tolerance_tokens.size()); ++i)
      {
        tesseract_common::toNumeric<double>(upper_tolerance_tokens[static_cast<std::size_t>(i)], upper_tolerance[i]);
        tesseract_common::toNumeric<double>(lower_tolerance_tokens[static_cast<std::size_t>(i)], lower_tolerance[i]);
      }
    }
  }

  int getType() const { return static_cast<int>(WaypointType::JOINT_WAYPOINT); }

  void print(const std::string& prefix = "") const
  {
    std::cout << prefix << "Joint WP: " << this->transpose() << std::endl;
  }

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const
  {
    Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");
    tinyxml2::XMLElement* xml_waypoint = doc.NewElement("Waypoint");
    xml_waypoint->SetAttribute("type", std::to_string(getType()).c_str());

    tinyxml2::XMLElement* xml_joint_waypoint = doc.NewElement("JointWaypoint");
    tinyxml2::XMLElement* xml_joint_names = doc.NewElement("Names");
    if (!joint_names.empty())
    {
      std::string jn = joint_names[0];
      for (std::size_t i = 1; i < joint_names.size(); ++i)
        jn += " " + joint_names[i];

      xml_joint_names->SetText(jn.c_str());
    }
    xml_joint_waypoint->InsertEndChild(xml_joint_names);

    // Write position
    {
      std::stringstream position_string;
      position_string << waypoint.format(eigen_format);

      tinyxml2::XMLElement* xml_joint_position = doc.NewElement("Position");
      xml_joint_position->SetText(position_string.str().c_str());
      xml_joint_waypoint->InsertEndChild(xml_joint_position);
    }

    // Write upper tolerance
    {
      std::stringstream upper_string;
      upper_string << upper_tolerance.format(eigen_format);

      tinyxml2::XMLElement* xml_upper = doc.NewElement("UpperTolerance");
      xml_upper->SetText(upper_string.str().c_str());
      xml_joint_waypoint->InsertEndChild(xml_upper);
    }

    // Write lower tolerance
    {
      std::stringstream lower_string;
      lower_string << lower_tolerance.format(eigen_format);

      tinyxml2::XMLElement* xml_lower = doc.NewElement("LowerTolerance");
      xml_lower->SetText(lower_string.str().c_str());
      xml_joint_waypoint->InsertEndChild(xml_lower);
    }

    xml_waypoint->InsertEndChild(xml_joint_waypoint);
    return xml_waypoint;
  }

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
    for (auto& v : l)
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
  inline ConstTransposeReturnType transpose() const { return waypoint.transpose(); }

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
    for (auto& v : l)
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
  bool isToleranced() const
  {
    // Check if they are empty
    if (lower_tolerance.size() == 0 || upper_tolerance.size() == 0)
      return false;

    // Check if they are the same
    static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

    if ((lower_tolerance.array() > max_diff).any())
      throw std::runtime_error("JointWaypoint: lower tolerance was provided but must be <= 0,");

    if ((upper_tolerance.array() < -max_diff).any())
      throw std::runtime_error("JointWaypoint: upper tolerance was provided but must be >= 0,");

    return !tesseract_common::almostEqualRelativeAndAbs(lower_tolerance, upper_tolerance, max_diff);
  }
  bool operator==(const JointWaypoint& rhs) const
  {
    static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

    bool equal = true;
    equal &= tesseract_common::almostEqualRelativeAndAbs(waypoint, rhs.waypoint, max_diff);
    equal &= tesseract_common::isIdentical(joint_names, rhs.joint_names);
    equal &= tesseract_common::almostEqualRelativeAndAbs(lower_tolerance, rhs.lower_tolerance, max_diff);
    equal &= tesseract_common::almostEqualRelativeAndAbs(upper_tolerance, rhs.upper_tolerance, max_diff);
    return equal;
  }
  bool operator!=(const JointWaypoint& rhs) const { return !operator==(rhs); }
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
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H
