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
#include <boost/serialization/unique_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
// Operators
bool CartesianWaypointInterface::operator==(const CartesianWaypointInterface& rhs) const { return equals(rhs); }

// LCOV_EXCL_START
bool CartesianWaypointInterface::operator!=(const CartesianWaypointInterface& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void CartesianWaypointInterface::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}

CartesianWaypointPoly::CartesianWaypointPoly(const CartesianWaypointPoly& other)
{
  if (other.impl_)
  {
    impl_ = other.impl_->clone();  // Deep copy
  }
}

CartesianWaypointPoly& CartesianWaypointPoly::operator=(const CartesianWaypointPoly& other)
{
  if (this != &other)
  {
    impl_ = other.impl_ ? other.impl_->clone() : nullptr;
  }
  return *this;
}

CartesianWaypointPoly::CartesianWaypointPoly(const CartesianWaypointInterface& impl) : impl_(impl.clone()) {}

void CartesianWaypointPoly::setName(const std::string& name) { impl_->setName(name); }
const std::string& CartesianWaypointPoly::getName() const { return impl_->getName(); }

std::type_index CartesianWaypointPoly::getType() const
{
  if (impl_ == nullptr)
    return typeid(nullptr);

  const CartesianWaypointInterface& value = *impl_;
  return typeid(value);
}

bool CartesianWaypointPoly::equals(const WaypointInterface& other) const
{
  if (impl_ == nullptr)
    return false;

  const auto* derived_other = dynamic_cast<const CartesianWaypointPoly*>(&other);
  if (derived_other == nullptr)
    return false;

  return (*this == *derived_other);
}

std::unique_ptr<WaypointInterface> CartesianWaypointPoly::clone() const
{
  return (impl_ == nullptr) ? nullptr : std::make_unique<CartesianWaypointPoly>(*impl_);
}

void CartesianWaypointPoly::print(const std::string& prefix) const { impl_->print(prefix); }

void CartesianWaypointPoly::setTransform(const Eigen::Isometry3d& transform) { impl_->setTransform(transform); }
Eigen::Isometry3d& CartesianWaypointPoly::getTransform() { return impl_->getTransform(); }
const Eigen::Isometry3d& CartesianWaypointPoly::getTransform() const { return impl_->getTransform(); }

void CartesianWaypointPoly::setUpperTolerance(const Eigen::VectorXd& upper_tol) { impl_->setUpperTolerance(upper_tol); }
Eigen::VectorXd& CartesianWaypointPoly::getUpperTolerance() { return impl_->getUpperTolerance(); }
const Eigen::VectorXd& CartesianWaypointPoly::getUpperTolerance() const { return impl_->getUpperTolerance(); }

void CartesianWaypointPoly::setLowerTolerance(const Eigen::VectorXd& lower_tol) { impl_->setLowerTolerance(lower_tol); }
Eigen::VectorXd& CartesianWaypointPoly::getLowerTolerance() { return impl_->getLowerTolerance(); }
const Eigen::VectorXd& CartesianWaypointPoly::getLowerTolerance() const { return impl_->getLowerTolerance(); }

void CartesianWaypointPoly::setSeed(const tesseract_common::JointState& seed) { impl_->setSeed(seed); }
tesseract_common::JointState& CartesianWaypointPoly::getSeed() { return impl_->getSeed(); }
const tesseract_common::JointState& CartesianWaypointPoly::getSeed() const { return impl_->getSeed(); }

bool tesseract_planning::CartesianWaypointPoly::hasSeed() const
{
  const auto& seed = impl_->getSeed();
  return (seed.position.size() != 0 && !seed.joint_names.empty() &&
          static_cast<std::size_t>(seed.position.size()) == seed.joint_names.size());
}

void tesseract_planning::CartesianWaypointPoly::clearSeed() { impl_->setSeed(tesseract_common::JointState()); }

bool tesseract_planning::CartesianWaypointPoly::isToleranced() const
{
  const auto& lower_tolerance = impl_->getLowerTolerance();
  const auto& upper_tolerance = impl_->getUpperTolerance();

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

bool CartesianWaypointPoly::isNull() const { return (impl_ == nullptr); }
CartesianWaypointInterface& CartesianWaypointPoly::getCartesianWaypoint() { return *impl_; }
const CartesianWaypointInterface& CartesianWaypointPoly::getCartesianWaypoint() const { return *impl_; }

bool CartesianWaypointPoly::operator==(const CartesianWaypointPoly& rhs) const
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
bool CartesianWaypointPoly::operator!=(const CartesianWaypointPoly& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void CartesianWaypointPoly::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(WaypointInterface);
  ar& boost::serialization::make_nvp("impl", impl_);
}
}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::CartesianWaypointInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::CartesianWaypointPoly)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::CartesianWaypointInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::CartesianWaypointPoly)
