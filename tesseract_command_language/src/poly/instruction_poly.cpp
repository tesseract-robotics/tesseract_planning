#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/instruction_poly.h>

template <class Archive>
void tesseract_planning::detail_instruction::InstructionInterface::serialize(Archive& ar,
                                                                             const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base",
                                     boost::serialization::base_object<tesseract_common::TypeErasureInterface>(*this));
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
