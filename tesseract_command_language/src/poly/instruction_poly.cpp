#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/composite_instruction.h>

template <class Archive>
void tesseract_planning::detail_instruction::InstructionInterface::serialize(Archive& ar,
                                                                             const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base",
                                     boost::serialization::base_object<tesseract_common::TypeErasureInterface>(*this));
}

const boost::uuids::uuid& tesseract_planning::InstructionPoly::getUUID() const { return getInterface().getUUID(); }

void tesseract_planning::InstructionPoly::setUUID(const boost::uuids::uuid& uuid) { getInterface().setUUID(uuid); }

void tesseract_planning::InstructionPoly::regenerateUUID() { getInterface().regenerateUUID(); }

const boost::uuids::uuid& tesseract_planning::InstructionPoly::getParentUUID() const
{
  return getInterface().getParentUUID();
}

void tesseract_planning::InstructionPoly::setParentUUID(const boost::uuids::uuid& uuid)
{
  getInterface().setParentUUID(uuid);
}

const std::string& tesseract_planning::InstructionPoly::getDescription() const
{
  return getInterface().getDescription();
}

void tesseract_planning::InstructionPoly::setDescription(const std::string& description)
{
  getInterface().setDescription(description);
}

void tesseract_planning::InstructionPoly::print(const std::string& prefix) const { getInterface().print(prefix); }

bool tesseract_planning::InstructionPoly::isCompositeInstruction() const
{
  return (isNull() ? false : (getInterface().getType() == std::type_index(typeid(CompositeInstruction))));
}

bool tesseract_planning::InstructionPoly::isMoveInstruction() const
{
  return (isNull() ? false : (getInterface().getType() == std::type_index(typeid(MoveInstructionPoly))));
}

template <class Archive>
void tesseract_planning::InstructionPoly::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base", boost::serialization::base_object<InstructionPolyBase>(*this));
}

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::detail_instruction::InstructionInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::InstructionPolyBase)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::InstructionPoly)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::detail_instruction::InstructionInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::InstructionPolyBase)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::InstructionPoly)
