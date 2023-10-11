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

#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
StateWaypoint::StateWaypoint(std::vector<std::string> joint_names, const Eigen::Ref<const Eigen::VectorXd>& position)
  : tesseract_common::JointState(std::move(joint_names), position)
{
  if (static_cast<Eigen::Index>(this->joint_names.size()) != this->position.size())
    throw std::runtime_error("StateWaypoint: parameters are not the same size!");
}
StateWaypoint::StateWaypoint(const std::vector<std::string>& names,
                             const Eigen::VectorXd& position,
                             const Eigen::VectorXd& velocity,
                             const Eigen::VectorXd& acceleration,
                             double time)
{
  this->joint_names = names;
  this->position = position;
  this->velocity = velocity;
  this->acceleration = acceleration;
  this->time = time;

  if (static_cast<Eigen::Index>(joint_names.size()) != this->position.size() ||
      this->position.size() != this->velocity.size() || this->position.size() != this->acceleration.size())
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

void StateWaypoint::setNames(const std::vector<std::string>& names) { joint_names = names; }
std::vector<std::string>& StateWaypoint::getNames() { return joint_names; }
const std::vector<std::string>& StateWaypoint::getNames() const { return joint_names; }

void StateWaypoint::setPosition(const Eigen::VectorXd& position) { this->position = position; }
Eigen::VectorXd& StateWaypoint::getPosition() { return position; }
const Eigen::VectorXd& StateWaypoint::getPosition() const { return position; }

void StateWaypoint::setVelocity(const Eigen::VectorXd& velocity) { this->velocity = velocity; }
Eigen::VectorXd& StateWaypoint::getVelocity() { return velocity; }
const Eigen::VectorXd& StateWaypoint::getVelocity() const { return velocity; }

void StateWaypoint::setAcceleration(const Eigen::VectorXd& acceleration) { this->acceleration = acceleration; }
Eigen::VectorXd& StateWaypoint::getAcceleration() { return acceleration; }
const Eigen::VectorXd& StateWaypoint::getAcceleration() const { return acceleration; }

void StateWaypoint::setEffort(const Eigen::VectorXd& effort) { this->effort = effort; }
Eigen::VectorXd& StateWaypoint::getEffort() { return effort; }
const Eigen::VectorXd& StateWaypoint::getEffort() const { return effort; }

void StateWaypoint::setTime(double time) { this->time = time; }
double StateWaypoint::getTime() const { return time; }

void StateWaypoint::setName(const std::string& name) { name_ = name; }
const std::string& StateWaypoint::getName() const { return name_; }

void StateWaypoint::print(const std::string& prefix) const
{
  std::cout << prefix << "State WP: Pos=" << position.transpose() << std::endl;  // NOLINT
}

bool StateWaypoint::operator==(const StateWaypoint& rhs) const
{
  static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

  bool equal = true;
  equal &= (name_ == rhs.name_);
  equal &= tesseract_common::almostEqualRelativeAndAbs(position, rhs.position, max_diff);
  equal &= tesseract_common::isIdentical(joint_names, rhs.joint_names);
  return equal;
}
// LCOV_EXCL_START
bool StateWaypoint::operator!=(const StateWaypoint& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void StateWaypoint::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(name_);
  ar& boost::serialization::make_nvp("base", boost::serialization::base_object<tesseract_common::JointState>(*this));
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::StateWaypoint)
TESSERACT_STATE_WAYPOINT_EXPORT_IMPLEMENT(tesseract_planning::StateWaypoint)
