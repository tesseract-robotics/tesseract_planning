/**
 * @file cartesian_waypoint_poly.cpp
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>

template <class Archive>
void tesseract_planning::detail_cartesian_waypoint::CartesianWaypointInterface::serialize(
    Archive& ar,
    const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base",
                                     boost::serialization::base_object<tesseract_common::TypeErasureInterface>(*this));
}

void tesseract_planning::CartesianWaypointPoly::setTransform(const Eigen::Isometry3d& transform)
{
  getInterface().setTransform(transform);
}
Eigen::Isometry3d& tesseract_planning::CartesianWaypointPoly::getTransform() { return getInterface().getTransform(); }
const Eigen::Isometry3d& tesseract_planning::CartesianWaypointPoly::getTransform() const
{
  return getInterface().getTransform();
}

void tesseract_planning::CartesianWaypointPoly::setUpperTolerance(const Eigen::VectorXd& upper_tol)
{
  getInterface().setUpperTolerance(upper_tol);
}
Eigen::VectorXd& tesseract_planning::CartesianWaypointPoly::getUpperTolerance()
{
  return getInterface().getUpperTolerance();
}
const Eigen::VectorXd& tesseract_planning::CartesianWaypointPoly::getUpperTolerance() const
{
  return getInterface().getUpperTolerance();
}

void tesseract_planning::CartesianWaypointPoly::setLowerTolerance(const Eigen::VectorXd& lower_tol)
{
  getInterface().setLowerTolerance(lower_tol);
}
Eigen::VectorXd& tesseract_planning::CartesianWaypointPoly::getLowerTolerance()
{
  return getInterface().getLowerTolerance();
}
const Eigen::VectorXd& tesseract_planning::CartesianWaypointPoly::getLowerTolerance() const
{
  return getInterface().getLowerTolerance();
}

void tesseract_planning::CartesianWaypointPoly::setSeed(const tesseract_common::JointState& seed)
{
  getInterface().setSeed(seed);
}
tesseract_common::JointState& tesseract_planning::CartesianWaypointPoly::getSeed() { return getInterface().getSeed(); }
const tesseract_common::JointState& tesseract_planning::CartesianWaypointPoly::getSeed() const
{
  return getInterface().getSeed();
}

void tesseract_planning::CartesianWaypointPoly::setName(const std::string& name) { getInterface().setName(name); }
const std::string& tesseract_planning::CartesianWaypointPoly::getName() const { return getInterface().getName(); }

void tesseract_planning::CartesianWaypointPoly::print(const std::string& prefix) const { getInterface().print(prefix); }

bool tesseract_planning::CartesianWaypointPoly::hasSeed() const
{
  const auto& seed = getInterface().getSeed();
  return (seed.position.size() != 0 && !seed.joint_names.empty() && seed.position.size() == seed.joint_names.size());
}

void tesseract_planning::CartesianWaypointPoly::clearSeed() { getInterface().setSeed(tesseract_common::JointState()); }

bool tesseract_planning::CartesianWaypointPoly::isToleranced() const
{
  const auto& lower_tolerance = getInterface().getLowerTolerance();
  const auto& upper_tolerance = getInterface().getUpperTolerance();

  // Check if they are empty
  if (lower_tolerance.size() == 0 || upper_tolerance.size() == 0)
    return false;

  // Check if they are the same
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  if ((lower_tolerance.array() > max_diff).any())
    throw std::runtime_error("CartesianWaypointPoly: lower tolerance was provided but must be <= 0,");

  if ((upper_tolerance.array() < -max_diff).any())
    throw std::runtime_error("CartesianWaypointPoly: upper tolerance was provided but must be >= 0,");

  return !tesseract_common::almostEqualRelativeAndAbs(lower_tolerance, upper_tolerance, max_diff);
}

template <class Archive>
void tesseract_planning::CartesianWaypointPoly::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base", boost::serialization::base_object<CartesianWaypointPolyBase>(*this));
}

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::detail_cartesian_waypoint::CartesianWaypointInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::CartesianWaypointPolyBase)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::CartesianWaypointPoly)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::detail_cartesian_waypoint::CartesianWaypointInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::CartesianWaypointPolyBase)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::CartesianWaypointPoly)

TESSERACT_WAYPOINT_EXPORT_IMPLEMENT(tesseract_planning::CartesianWaypointPoly);
