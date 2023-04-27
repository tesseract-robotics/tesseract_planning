/**
 * @file state_waypoint_poly.cpp
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>

template <class Archive>
void tesseract_planning::detail_state_waypoint::StateWaypointInterface::serialize(
    Archive& ar,
    const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base",
                                     boost::serialization::base_object<tesseract_common::TypeErasureInterface>(*this));
}

void tesseract_planning::StateWaypointPoly::setNames(const std::vector<std::string>& names)
{
  getInterface().setNames(names);
}
std::vector<std::string>& tesseract_planning::StateWaypointPoly::getNames() { return getInterface().getNames(); }
const std::vector<std::string>& tesseract_planning::StateWaypointPoly::getNames() const
{
  return getInterface().getNames();
}

void tesseract_planning::StateWaypointPoly::setPosition(const Eigen::VectorXd& position)
{
  getInterface().setPosition(position);
}
Eigen::VectorXd& tesseract_planning::StateWaypointPoly::getPosition() { return getInterface().getPosition(); }
const Eigen::VectorXd& tesseract_planning::StateWaypointPoly::getPosition() const
{
  return getInterface().getPosition();
}

void tesseract_planning::StateWaypointPoly::setVelocity(const Eigen::VectorXd& velocity)
{
  getInterface().setVelocity(velocity);
}
Eigen::VectorXd& tesseract_planning::StateWaypointPoly::getVelocity() { return getInterface().getVelocity(); }
const Eigen::VectorXd& tesseract_planning::StateWaypointPoly::getVelocity() const
{
  return getInterface().getVelocity();
}

void tesseract_planning::StateWaypointPoly::setAcceleration(const Eigen::VectorXd& acceleration)
{
  getInterface().setAcceleration(acceleration);
}
Eigen::VectorXd& tesseract_planning::StateWaypointPoly::getAcceleration() { return getInterface().getAcceleration(); }
const Eigen::VectorXd& tesseract_planning::StateWaypointPoly::getAcceleration() const
{
  return getInterface().getAcceleration();
}

void tesseract_planning::StateWaypointPoly::setEffort(const Eigen::VectorXd& effort)
{
  getInterface().setEffort(effort);
}
Eigen::VectorXd& tesseract_planning::StateWaypointPoly::getEffort() { return getInterface().getEffort(); }
const Eigen::VectorXd& tesseract_planning::StateWaypointPoly::getEffort() const { return getInterface().getEffort(); }

void tesseract_planning::StateWaypointPoly::setTime(double time) { getInterface().setTime(time); }
double tesseract_planning::StateWaypointPoly::getTime() const { return getInterface().getTime(); }

void tesseract_planning::StateWaypointPoly::setName(const std::string& name) { getInterface().setName(name); }
const std::string& tesseract_planning::StateWaypointPoly::getName() const { return getInterface().getName(); }

void tesseract_planning::StateWaypointPoly::print(const std::string& prefix) const { getInterface().print(prefix); }

template <class Archive>
void tesseract_planning::StateWaypointPoly::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base", boost::serialization::base_object<StateWaypointPolyBase>(*this));
}

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::detail_state_waypoint::StateWaypointInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::StateWaypointPolyBase)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::StateWaypointPoly)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::detail_state_waypoint::StateWaypointInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::StateWaypointPolyBase)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::StateWaypointPoly)

TESSERACT_WAYPOINT_EXPORT_IMPLEMENT(tesseract_planning::StateWaypointPoly);
