/**
 * @file state_waypoint.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
#include <boost/serialization/vector.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/eigen_serialization.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/types.h>

namespace tesseract_planning
{
StateWaypoint::StateWaypoint(std::vector<std::string> joint_names, const Eigen::Ref<const Eigen::VectorXd>& position)
  : joint_names_(std::move(joint_names)), position_(position)
{
  if (static_cast<Eigen::Index>(joint_names_.size()) != position_.size())
    throw std::runtime_error("StateWaypoint: parameters are not the same size!");
}
StateWaypoint::StateWaypoint(const std::vector<std::string>& names,
                             const Eigen::VectorXd& position,      // NOLINT(modernize-pass-by-value)
                             const Eigen::VectorXd& velocity,      // NOLINT(modernize-pass-by-value)
                             const Eigen::VectorXd& acceleration,  // NOLINT(modernize-pass-by-value)
                             double time)
  : joint_names_(names), position_(position), velocity_(velocity), acceleration_(acceleration), time_(time)
{
  if (static_cast<Eigen::Index>(joint_names_.size()) != position_.size() || position_.size() != velocity_.size() ||
      position_.size() != acceleration_.size())
    throw std::runtime_error("StateWaypoint: parameters are not the same size!");
}

StateWaypoint::StateWaypoint(std::initializer_list<std::string> names, std::initializer_list<double> position)
  : StateWaypoint(names,
                  Eigen::Map<const Eigen::VectorXd>(position.begin(), static_cast<Eigen::Index>(position.size())))
{
}

StateWaypoint::StateWaypoint(std::initializer_list<std::string> names,
                             std::initializer_list<double> position,
                             std::initializer_list<double> velocity,
                             std::initializer_list<double> acceleration,
                             double time)
  : StateWaypoint(
        names,
        Eigen::Map<const Eigen::VectorXd>(position.begin(), static_cast<Eigen::Index>(position.size())),
        Eigen::Map<const Eigen::VectorXd>(velocity.begin(), static_cast<Eigen::Index>(velocity.size())),
        Eigen::Map<const Eigen::VectorXd>(acceleration.begin(), static_cast<Eigen::Index>(acceleration.size())),
        time)
{
}

// Waypoint
void StateWaypoint::setName(const std::string& name) { name_ = name; }
const std::string& StateWaypoint::getName() const { return name_; }
void StateWaypoint::print(const std::string& prefix) const
{
  std::cout << prefix << "State WP: Pos=" << position_.transpose() << std::endl;  // NOLINT
}

std::unique_ptr<StateWaypointInterface> StateWaypoint::clone() const { return std::make_unique<StateWaypoint>(*this); }

// State Waypoint
void StateWaypoint::setNames(const std::vector<std::string>& names) { joint_names_ = names; }
std::vector<std::string>& StateWaypoint::getNames() { return joint_names_; }
const std::vector<std::string>& StateWaypoint::getNames() const { return joint_names_; }

void StateWaypoint::setPosition(const Eigen::VectorXd& position) { this->position_ = position; }
Eigen::VectorXd& StateWaypoint::getPosition() { return position_; }
const Eigen::VectorXd& StateWaypoint::getPosition() const { return position_; }

void StateWaypoint::setVelocity(const Eigen::VectorXd& velocity) { this->velocity_ = velocity; }
Eigen::VectorXd& StateWaypoint::getVelocity() { return velocity_; }
const Eigen::VectorXd& StateWaypoint::getVelocity() const { return velocity_; }

void StateWaypoint::setAcceleration(const Eigen::VectorXd& acceleration) { this->acceleration_ = acceleration; }
Eigen::VectorXd& StateWaypoint::getAcceleration() { return acceleration_; }
const Eigen::VectorXd& StateWaypoint::getAcceleration() const { return acceleration_; }

void StateWaypoint::setEffort(const Eigen::VectorXd& effort) { this->effort_ = effort; }
Eigen::VectorXd& StateWaypoint::getEffort() { return effort_; }
const Eigen::VectorXd& StateWaypoint::getEffort() const { return effort_; }

void StateWaypoint::setTime(double time) { this->time_ = time; }
double StateWaypoint::getTime() const { return time_; }

bool StateWaypoint::equals(const StateWaypointInterface& other) const
{
  const auto* rhs = dynamic_cast<const StateWaypoint*>(&other);
  if (rhs == nullptr)
    return false;

  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (name_ == rhs->name_);
  equal &= tesseract_common::almostEqualRelativeAndAbs(position_, rhs->position_, max_diff);
  equal &= tesseract_common::isIdentical(joint_names_, rhs->joint_names_);
  return equal;
}

template <class Archive>
void StateWaypoint::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(StateWaypointInterface);
  ar& BOOST_SERIALIZATION_NVP(name_);
  ar& BOOST_SERIALIZATION_NVP(joint_names_);
  ar& BOOST_SERIALIZATION_NVP(position_);
  ar& BOOST_SERIALIZATION_NVP(velocity_);
  ar& BOOST_SERIALIZATION_NVP(acceleration_);
  ar& BOOST_SERIALIZATION_NVP(effort_);
  ar& BOOST_SERIALIZATION_NVP(time_);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::StateWaypoint)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::StateWaypoint)
