
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>

namespace tesseract::command_language
{
// Operators
bool WaypointInterface::operator==(const WaypointInterface& rhs) const { return equals(rhs); }

// LCOV_EXCL_START
bool WaypointInterface::operator!=(const WaypointInterface& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

WaypointPoly::WaypointPoly(const WaypointPoly& other)
{
  if (other.impl_)
    impl_ = other.impl_->clone();  // Deep copy
}

WaypointPoly& WaypointPoly::operator=(const WaypointPoly& other)
{
  if (this != &other)
    impl_ = other.impl_ ? other.impl_->clone() : nullptr;

  return *this;
}

WaypointPoly::WaypointPoly(const WaypointInterface& impl) : impl_(impl.clone()) {}
WaypointPoly::WaypointPoly(const CartesianWaypointInterface& impl)
  : impl_(std::make_unique<CartesianWaypointPoly>(impl))
{
}
WaypointPoly::WaypointPoly(const JointWaypointInterface& impl) : impl_(std::make_unique<JointWaypointPoly>(impl)) {}
WaypointPoly::WaypointPoly(const StateWaypointInterface& impl) : impl_(std::make_unique<StateWaypointPoly>(impl)) {}

void WaypointPoly::setName(const std::string& name) { impl_->setName(name); }
const std::string& WaypointPoly::getName() const { return std::as_const(*impl_).getName(); }

std::type_index WaypointPoly::getType() const
{
  if (impl_ == nullptr)
    return typeid(nullptr);

  const WaypointInterface& value = *impl_;
  return typeid(value);
}

void WaypointPoly::print(const std::string& prefix) const { std::as_const(*impl_).print(prefix); }

bool WaypointPoly::isNull() const { return (impl_ == nullptr); }
WaypointInterface& WaypointPoly::getWaypoint() { return *impl_; }
const WaypointInterface& WaypointPoly::getWaypoint() const { return std::as_const(*impl_); }

bool WaypointPoly::isCartesianWaypoint() const
{
  return ((impl_ == nullptr) ? false : (getType() == std::type_index(typeid(CartesianWaypointPoly))));
}

bool WaypointPoly::isJointWaypoint() const
{
  return ((impl_ == nullptr) ? false : (getType() == std::type_index(typeid(JointWaypointPoly))));
}

bool WaypointPoly::isStateWaypoint() const
{
  return ((impl_ == nullptr) ? false : (getType() == std::type_index(typeid(StateWaypointPoly))));
}

bool WaypointPoly::operator==(const WaypointPoly& rhs) const
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
bool WaypointPoly::operator!=(const WaypointPoly& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

}  // namespace tesseract::command_language
