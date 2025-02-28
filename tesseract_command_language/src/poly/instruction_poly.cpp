#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <boost/uuid/uuid.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_common/serialization.h>

namespace tesseract_planning
{
// Operators
bool InstructionInterface::operator==(const InstructionInterface& rhs) const { return equals(rhs); }

// LCOV_EXCL_START
bool InstructionInterface::operator!=(const InstructionInterface& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void InstructionInterface::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}

InstructionPoly::InstructionPoly(const InstructionPoly& other)
{
  if (other.impl_)
    impl_ = other.impl_->clone();  // Deep copy
}

InstructionPoly& InstructionPoly::operator=(const InstructionPoly& other)
{
  if (this != &other)
    impl_ = other.impl_ ? other.impl_->clone() : nullptr;

  return *this;
}

InstructionPoly::InstructionPoly(const InstructionInterface& impl) : impl_(impl.clone()) {}

const boost::uuids::uuid& InstructionPoly::getUUID() const { return impl_->getUUID(); }

void InstructionPoly::setUUID(const boost::uuids::uuid& uuid) { impl_->setUUID(uuid); }

void InstructionPoly::regenerateUUID() { impl_->regenerateUUID(); }

const boost::uuids::uuid& InstructionPoly::getParentUUID() const { return impl_->getParentUUID(); }

void InstructionPoly::setParentUUID(const boost::uuids::uuid& uuid) { impl_->setParentUUID(uuid); }

const std::string& InstructionPoly::getDescription() const { return impl_->getDescription(); }

void InstructionPoly::setDescription(const std::string& description) { impl_->setDescription(description); }

void InstructionPoly::print(const std::string& prefix) const { impl_->print(prefix); }

std::type_index InstructionPoly::getType() const
{
  if (impl_ == nullptr)
    return typeid(nullptr);

  const InstructionInterface& value = *impl_;
  return typeid(value);
}

bool InstructionPoly::isNull() const { return (impl_ == nullptr); }
InstructionInterface& InstructionPoly::getInstruction() { return *impl_; }
const InstructionInterface& InstructionPoly::getInstruction() const { return *impl_; }

bool InstructionPoly::isCompositeInstruction() const
{
  return ((impl_ == nullptr) ? false : (getType() == std::type_index(typeid(CompositeInstruction))));
}

bool InstructionPoly::isMoveInstruction() const
{
  return ((impl_ == nullptr) ? false : (getType() == std::type_index(typeid(MoveInstructionPoly))));
}

bool InstructionPoly::operator==(const InstructionPoly& rhs) const
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
bool InstructionPoly::operator!=(const InstructionPoly& rhs) const { return !operator==(rhs); }
// LCOV_EXCL_STOP

template <class Archive>
void InstructionPoly::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& boost::serialization::make_nvp("impl", impl_);
}
}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::InstructionInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::InstructionPoly)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::InstructionInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::InstructionPoly)
