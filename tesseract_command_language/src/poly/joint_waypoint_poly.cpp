/**
 * @file joint_waypoint_poly.cpp
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>

template <class Archive>
void tesseract_planning::detail_joint_waypoint::JointWaypointInterface::serialize(
    Archive& ar,
    const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base",
                                     boost::serialization::base_object<tesseract_common::TypeErasureInterface>(*this));
}

void tesseract_planning::JointWaypointPoly::setNames(const std::vector<std::string>& names)
{
  getInterface().setNames(names);
}
std::vector<std::string>& tesseract_planning::JointWaypointPoly::getNames() { return getInterface().getNames(); }
const std::vector<std::string>& tesseract_planning::JointWaypointPoly::getNames() const
{
  return getInterface().getNames();
}

void tesseract_planning::JointWaypointPoly::setPosition(const Eigen::VectorXd& position)
{
  getInterface().setPosition(position);
}
Eigen::VectorXd& tesseract_planning::JointWaypointPoly::getPosition() { return getInterface().getPosition(); }
const Eigen::VectorXd& tesseract_planning::JointWaypointPoly::getPosition() const
{
  return getInterface().getPosition();
}

void tesseract_planning::JointWaypointPoly::setUpperTolerance(const Eigen::VectorXd& upper_tol)
{
  getInterface().setUpperTolerance(upper_tol);
}
Eigen::VectorXd& tesseract_planning::JointWaypointPoly::getUpperTolerance()
{
  return getInterface().getUpperTolerance();
}
const Eigen::VectorXd& tesseract_planning::JointWaypointPoly::getUpperTolerance() const
{
  return getInterface().getUpperTolerance();
}

void tesseract_planning::JointWaypointPoly::setLowerTolerance(const Eigen::VectorXd& lower_tol)
{
  getInterface().setLowerTolerance(lower_tol);
}
Eigen::VectorXd& tesseract_planning::JointWaypointPoly::getLowerTolerance()
{
  return getInterface().getLowerTolerance();
}
const Eigen::VectorXd& tesseract_planning::JointWaypointPoly::getLowerTolerance() const
{
  return getInterface().getLowerTolerance();
}

void tesseract_planning::JointWaypointPoly::setIsConstrained(bool value) { getInterface().setIsConstrained(value); }
bool tesseract_planning::JointWaypointPoly::isConstrained() const { return getInterface().isConstrained(); }

void tesseract_planning::JointWaypointPoly::setName(const std::string& name) { getInterface().setName(name); }
const std::string& tesseract_planning::JointWaypointPoly::getName() const { return getInterface().getName(); }

void tesseract_planning::JointWaypointPoly::print(const std::string& prefix) const { getInterface().print(prefix); }

bool tesseract_planning::JointWaypointPoly::isToleranced() const
{
  const auto& lower_tolerance = getInterface().getLowerTolerance();
  const auto& upper_tolerance = getInterface().getUpperTolerance();

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

template <class Archive>
void tesseract_planning::JointWaypointPoly::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base", boost::serialization::base_object<JointWaypointPolyBase>(*this));
}

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::detail_joint_waypoint::JointWaypointInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::JointWaypointPolyBase)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::JointWaypointPoly)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::detail_joint_waypoint::JointWaypointInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::JointWaypointPolyBase)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::JointWaypointPoly)

TESSERACT_WAYPOINT_EXPORT_IMPLEMENT(tesseract_planning::JointWaypointPoly)
