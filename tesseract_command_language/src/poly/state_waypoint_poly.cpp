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
#include <boost/serialization/unique_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_common/serialization.h>

namespace tesseract_planning
{
// Operators
bool StateWaypointInterface::operator==(const StateWaypointInterface& rhs) const { return equals(rhs); }

// LCOV_EXCL_START
bool StateWaypointInterface::operator!=(const StateWaypointInterface& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void StateWaypointInterface::serialize(Archive& /*ar*/, const unsigned int /*version*/)  // NOLINT
{
}

StateWaypointPoly::StateWaypointPoly(const StateWaypointPoly& other)
{
  if (other.impl_)
  {
    impl_ = other.impl_->clone();  // Deep copy
  }
}

StateWaypointPoly& StateWaypointPoly::operator=(const StateWaypointPoly& other)
{
  if (this != &other)
  {
    impl_ = other.impl_ ? other.impl_->clone() : nullptr;
  }
  return *this;
}

StateWaypointPoly::StateWaypointPoly(const StateWaypointInterface& impl) : impl_(impl.clone()) {}

void StateWaypointPoly::setName(const std::string& name) { impl_->setName(name); }
const std::string& StateWaypointPoly::getName() const { return impl_->getName(); }

std::type_index StateWaypointPoly::getType() const
{
  if (impl_ == nullptr)
    return typeid(nullptr);

  const StateWaypointInterface& value = *impl_;
  return typeid(value);
}

bool StateWaypointPoly::equals(const WaypointInterface& other) const
{
  if (impl_ == nullptr)
    return false;

  const auto* derived_other = dynamic_cast<const StateWaypointPoly*>(&other);
  if (derived_other == nullptr)
    return false;

  return (*this == *derived_other);
}

void StateWaypointPoly::print(const std::string& prefix) const { impl_->print(prefix); }

void StateWaypointPoly::setNames(const std::vector<std::string>& names) { impl_->setNames(names); }
std::vector<std::string>& StateWaypointPoly::getNames() { return impl_->getNames(); }
const std::vector<std::string>& StateWaypointPoly::getNames() const { return impl_->getNames(); }

void StateWaypointPoly::setPosition(const Eigen::VectorXd& position) { impl_->setPosition(position); }
Eigen::VectorXd& StateWaypointPoly::getPosition() { return impl_->getPosition(); }
const Eigen::VectorXd& StateWaypointPoly::getPosition() const { return impl_->getPosition(); }

void StateWaypointPoly::setVelocity(const Eigen::VectorXd& velocity) { impl_->setVelocity(velocity); }
Eigen::VectorXd& StateWaypointPoly::getVelocity() { return impl_->getVelocity(); }
const Eigen::VectorXd& StateWaypointPoly::getVelocity() const { return impl_->getVelocity(); }

void StateWaypointPoly::setAcceleration(const Eigen::VectorXd& acceleration) { impl_->setAcceleration(acceleration); }
Eigen::VectorXd& StateWaypointPoly::getAcceleration() { return impl_->getAcceleration(); }
const Eigen::VectorXd& StateWaypointPoly::getAcceleration() const { return impl_->getAcceleration(); }

void StateWaypointPoly::setEffort(const Eigen::VectorXd& effort) { impl_->setEffort(effort); }
Eigen::VectorXd& StateWaypointPoly::getEffort() { return impl_->getEffort(); }
const Eigen::VectorXd& StateWaypointPoly::getEffort() const { return impl_->getEffort(); }

void StateWaypointPoly::setTime(double time) { impl_->setTime(time); }
double StateWaypointPoly::getTime() const { return impl_->getTime(); }

std::unique_ptr<WaypointInterface> StateWaypointPoly::clone() const
{
  return (impl_ == nullptr) ? nullptr : std::make_unique<StateWaypointPoly>(*impl_);
}

bool StateWaypointPoly::isNull() const { return (impl_ == nullptr); }
StateWaypointInterface& StateWaypointPoly::getStateWaypoint() { return *impl_; }
const StateWaypointInterface& StateWaypointPoly::getStateWaypoint() const { return *impl_; }

bool StateWaypointPoly::operator==(const StateWaypointPoly& rhs) const
{
  if (impl_ == nullptr && rhs.impl_ == nullptr)
    return true;

  if (impl_ == nullptr || rhs.impl_ == nullptr)
    return false;

  if (getType() != rhs.getType())
    return false;

  return (*impl_ == *rhs.impl_);
}

// LCOV_EXCL_START
bool StateWaypointPoly::operator!=(const StateWaypointPoly& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void StateWaypointPoly::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(WaypointInterface);
  ar& boost::serialization::make_nvp("impl", impl_);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::StateWaypointInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::StateWaypointPoly)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::StateWaypointInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::StateWaypointPoly)
