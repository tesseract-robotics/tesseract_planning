#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <boost/uuid/uuid.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/serialization.h>

namespace tesseract_planning
{
// Operators
bool MoveInstructionInterface::operator==(const MoveInstructionInterface& rhs) const { return equals(rhs); }

// LCOV_EXCL_START
bool MoveInstructionInterface::operator!=(const MoveInstructionInterface& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void MoveInstructionInterface::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}

MoveInstructionPoly::MoveInstructionPoly(const MoveInstructionPoly& other)
{
  if (other.impl_)
  {
    impl_ = other.impl_->clone();  // Deep copy
  }
}

MoveInstructionPoly& MoveInstructionPoly::operator=(const MoveInstructionPoly& other)
{
  if (this != &other)
  {
    impl_ = other.impl_ ? other.impl_->clone() : nullptr;
  }
  return *this;
}

MoveInstructionPoly::MoveInstructionPoly(const MoveInstructionInterface& impl) : impl_(impl.clone()) {}

const boost::uuids::uuid& MoveInstructionPoly::getUUID() const { return impl_->getUUID(); }

void MoveInstructionPoly::setUUID(const boost::uuids::uuid& uuid) { impl_->setUUID(uuid); }

void MoveInstructionPoly::regenerateUUID() { impl_->regenerateUUID(); }

const boost::uuids::uuid& MoveInstructionPoly::getParentUUID() const { return impl_->getParentUUID(); }

void MoveInstructionPoly::setParentUUID(const boost::uuids::uuid& uuid) { impl_->setParentUUID(uuid); }

WaypointPoly& MoveInstructionPoly::getWaypoint() { return impl_->getWaypoint(); }
const WaypointPoly& MoveInstructionPoly::getWaypoint() const { return impl_->getWaypoint(); }

void MoveInstructionPoly::setManipulatorInfo(tesseract_common::ManipulatorInfo info)
{
  impl_->setManipulatorInfo(std::move(info));
}
const tesseract_common::ManipulatorInfo& MoveInstructionPoly::getManipulatorInfo() const
{
  return impl_->getManipulatorInfo();
}
tesseract_common::ManipulatorInfo& MoveInstructionPoly::getManipulatorInfo() { return impl_->getManipulatorInfo(); }

void MoveInstructionPoly::setProfile(const std::string& profile) { impl_->setProfile(profile); }
const std::string& MoveInstructionPoly::getProfile(const std::string& ns) const { return impl_->getProfile(ns); }

void MoveInstructionPoly::setPathProfile(const std::string& profile) { impl_->setPathProfile(profile); }
const std::string& MoveInstructionPoly::getPathProfile(const std::string& ns) const
{
  return impl_->getPathProfile(ns);
}

void MoveInstructionPoly::setProfileOverrides(ProfileOverrides profile_overrides)
{
  impl_->setProfileOverrides(std::move(profile_overrides));
}
const ProfileOverrides& MoveInstructionPoly::getProfileOverrides() const { return impl_->getProfileOverrides(); }

void MoveInstructionPoly::setPathProfileOverrides(ProfileOverrides profile_overrides)
{
  impl_->setPathProfileOverrides(std::move(profile_overrides));
}
const ProfileOverrides& MoveInstructionPoly::getPathProfileOverrides() const
{
  return impl_->getPathProfileOverrides();
}

void MoveInstructionPoly::setMoveType(MoveInstructionType move_type) { impl_->setMoveType(move_type); }
MoveInstructionType MoveInstructionPoly::getMoveType() const { return impl_->getMoveType(); }

const std::string& MoveInstructionPoly::getDescription() const { return impl_->getDescription(); }

void MoveInstructionPoly::setDescription(const std::string& description) { impl_->setDescription(description); }

std::type_index MoveInstructionPoly::getType() const
{
  if (impl_ == nullptr)
    return typeid(nullptr);

  const MoveInstructionInterface& value = *impl_;
  return typeid(value);
}

bool MoveInstructionPoly::equals(const InstructionInterface& other) const
{
  if (impl_ == nullptr)
    return false;

  const auto* derived_other = dynamic_cast<const MoveInstructionPoly*>(&other);
  if (derived_other == nullptr)
    return false;

  return (*this == *derived_other);
}

std::unique_ptr<InstructionInterface> MoveInstructionPoly::clone() const
{
  return (impl_ == nullptr) ? nullptr : std::make_unique<MoveInstructionPoly>(*impl_);
}

void MoveInstructionPoly::print(const std::string& prefix) const { impl_->print(prefix); }

CartesianWaypointPoly MoveInstructionPoly::createCartesianWaypoint() const { return impl_->createCartesianWaypoint(); }
JointWaypointPoly MoveInstructionPoly::createJointWaypoint() const { return impl_->createJointWaypoint(); }
StateWaypointPoly MoveInstructionPoly::createStateWaypoint() const { return impl_->createStateWaypoint(); }

MoveInstructionPoly MoveInstructionPoly::createChild() const
{
  MoveInstructionPoly child(*this);
  child.setParentUUID(getUUID());
  child.regenerateUUID();
  if (!impl_->getWaypoint().getName().empty())
    child.getWaypoint().setName(impl_->getWaypoint().getName() + " (child)");
  return child;
}

bool MoveInstructionPoly::isLinear() const { return (impl_->getMoveType() == MoveInstructionType::LINEAR); }

bool MoveInstructionPoly::isFreespace() const { return (impl_->getMoveType() == MoveInstructionType::FREESPACE); }

bool MoveInstructionPoly::isCircular() const { return (impl_->getMoveType() == MoveInstructionType::CIRCULAR); }

bool MoveInstructionPoly::isChild() const { return (!impl_->getParentUUID().is_nil()); }

bool MoveInstructionPoly::isNull() const { return (impl_ == nullptr); }
MoveInstructionInterface& MoveInstructionPoly::getMoveInstruction() { return *impl_; }
const MoveInstructionInterface& MoveInstructionPoly::getMoveInstruction() const { return *impl_; }

bool MoveInstructionPoly::operator==(const MoveInstructionPoly& rhs) const
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
bool MoveInstructionPoly::operator!=(const MoveInstructionPoly& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void MoveInstructionPoly::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(InstructionInterface);
  ar& boost::serialization::make_nvp("impl", impl_);
}
}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MoveInstructionInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MoveInstructionPoly)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::MoveInstructionInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::MoveInstructionPoly)
