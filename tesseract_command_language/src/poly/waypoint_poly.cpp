#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unique_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_common/serialization.h>

namespace tesseract_planning
{
// Operators
bool WaypointInterface::operator==(const WaypointInterface& rhs) const { return equals(rhs); }

// LCOV_EXCL_START
bool WaypointInterface::operator!=(const WaypointInterface& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void WaypointInterface::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}

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

void WaypointPoly::setName(const std::string& name) { impl_->setName(name); }
const std::string& WaypointPoly::getName() const { return impl_->getName(); }

std::type_index WaypointPoly::getType() const
{
  if (impl_ == nullptr)
    return typeid(nullptr);

  const WaypointInterface& value = *impl_;
  return typeid(value);
}

void WaypointPoly::print(const std::string& prefix) const { impl_->print(prefix); }

bool WaypointPoly::isNull() const { return (impl_ == nullptr); }
WaypointInterface& WaypointPoly::getWaypoint() { return *impl_; }
const WaypointInterface& WaypointPoly::getWaypoint() const { return *impl_; }

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

template <class Archive>
void WaypointPoly::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("impl", impl_);
}
}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::WaypointInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::WaypointPoly)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::WaypointInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::WaypointPoly)
