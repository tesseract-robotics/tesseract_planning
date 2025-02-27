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
#include <boost/serialization/unique_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
// Operators
bool JointWaypointInterface::operator==(const JointWaypointInterface& rhs) const { return equals(rhs); }

// LCOV_EXCL_START
bool JointWaypointInterface::operator!=(const JointWaypointInterface& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void JointWaypointInterface::serialize(Archive& /*ar*/, const unsigned int /*version*/)  // NOLINT
{
}

JointWaypointPoly::JointWaypointPoly(const JointWaypointPoly& other)
{
  if (other.impl_)
  {
    impl_ = other.impl_->clone();  // Deep copy
  }
}

JointWaypointPoly& JointWaypointPoly::operator=(const JointWaypointPoly& other)
{
  if (this != &other)
  {
    impl_ = other.impl_ ? other.impl_->clone() : nullptr;
  }
  return *this;
}

JointWaypointPoly::JointWaypointPoly(const JointWaypointInterface& impl) : impl_(impl.clone()) {}

void JointWaypointPoly::setName(const std::string& name) { impl_->setName(name); }
const std::string& JointWaypointPoly::getName() const { return impl_->getName(); }

std::type_index JointWaypointPoly::getType() const
{
  if (impl_ == nullptr)
    return typeid(nullptr);

  const JointWaypointInterface& value = *impl_;
  return typeid(value);
}

bool JointWaypointPoly::equals(const WaypointInterface& other) const
{
  if (impl_ == nullptr)
    return false;

  const auto* derived_other = dynamic_cast<const JointWaypointPoly*>(&other);
  if (derived_other == nullptr)
    return false;

  return (*this == *derived_other);
}

std::unique_ptr<WaypointInterface> JointWaypointPoly::clone() const
{
  return (impl_ == nullptr) ? nullptr : std::make_unique<JointWaypointPoly>(*impl_);
}

void JointWaypointPoly::print(const std::string& prefix) const { impl_->print(prefix); }

void JointWaypointPoly::setNames(const std::vector<std::string>& names) { impl_->setNames(names); }
std::vector<std::string>& JointWaypointPoly::getNames() { return impl_->getNames(); }
const std::vector<std::string>& JointWaypointPoly::getNames() const { return impl_->getNames(); }

void JointWaypointPoly::setPosition(const Eigen::VectorXd& position) { impl_->setPosition(position); }
Eigen::VectorXd& JointWaypointPoly::getPosition() { return impl_->getPosition(); }
const Eigen::VectorXd& JointWaypointPoly::getPosition() const { return impl_->getPosition(); }

void JointWaypointPoly::setUpperTolerance(const Eigen::VectorXd& upper_tol) { impl_->setUpperTolerance(upper_tol); }
Eigen::VectorXd& JointWaypointPoly::getUpperTolerance() { return impl_->getUpperTolerance(); }
const Eigen::VectorXd& JointWaypointPoly::getUpperTolerance() const { return impl_->getUpperTolerance(); }

void JointWaypointPoly::setLowerTolerance(const Eigen::VectorXd& lower_tol) { impl_->setLowerTolerance(lower_tol); }
Eigen::VectorXd& JointWaypointPoly::getLowerTolerance() { return impl_->getLowerTolerance(); }
const Eigen::VectorXd& JointWaypointPoly::getLowerTolerance() const { return impl_->getLowerTolerance(); }

void JointWaypointPoly::setIsConstrained(bool value) { impl_->setIsConstrained(value); }
bool JointWaypointPoly::isConstrained() const { return impl_->isConstrained(); }

bool JointWaypointPoly::isToleranced() const
{
  const auto& lower_tolerance = impl_->getLowerTolerance();
  const auto& upper_tolerance = impl_->getUpperTolerance();

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

bool JointWaypointPoly::isNull() const { return (impl_ == nullptr); }
JointWaypointInterface& JointWaypointPoly::getJointWaypoint() { return *impl_; }
const JointWaypointInterface& JointWaypointPoly::getJointWaypoint() const { return *impl_; }

bool JointWaypointPoly::operator==(const JointWaypointPoly& rhs) const
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
bool JointWaypointPoly::operator!=(const JointWaypointPoly& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void JointWaypointPoly::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(WaypointInterface);
  ar& boost::serialization::make_nvp("impl", impl_);
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::JointWaypointInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::JointWaypointPoly)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::JointWaypointInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::JointWaypointPoly)
