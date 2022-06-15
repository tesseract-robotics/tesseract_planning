#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_command_language/core/instruction.h>

template <class Archive>
void tesseract_planning::detail_instruction::InstructionInterface::serialize(Archive& ar,
                                                                             const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base",
                                     boost::serialization::base_object<tesseract_common::TypeErasureInterface>(*this));
}

const std::string& tesseract_planning::Instruction::getDescription() const { return interface().getDescription(); }

void tesseract_planning::Instruction::setDescription(const std::string& description)
{
  interface().setDescription(description);
}

void tesseract_planning::Instruction::print(const std::string& prefix) const { interface().print(prefix); }

template <class Archive>
void tesseract_planning::Instruction::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base", boost::serialization::base_object<InstructionBase>(*this));
}

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::detail_instruction::InstructionInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::InstructionBase)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::Instruction)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::detail_instruction::InstructionInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::InstructionBase)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::Instruction)
