#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_command_language/core/instruction.h>

tesseract_planning::Instruction::Instruction()  // NOLINT
  : instruction_(nullptr)
{
}

// Copy constructor
tesseract_planning::Instruction::Instruction(const Instruction& other) { instruction_ = other.instruction_->clone(); }

// Move ctor.
tesseract_planning::Instruction::Instruction(Instruction&& other) noexcept { instruction_.swap(other.instruction_); }

// Move assignment.
tesseract_planning::Instruction& tesseract_planning::Instruction::operator=(Instruction&& other) noexcept
{
  instruction_.swap(other.instruction_);
  return (*this);
}

tesseract_planning::Instruction& tesseract_planning::Instruction::operator=(const Instruction& other)
{
  (*this) = Instruction(other);
  return (*this);
}

std::type_index tesseract_planning::Instruction::getType() const { return instruction_->getType(); }

const std::string& tesseract_planning::Instruction::getDescription() const { return instruction_->getDescription(); }

void tesseract_planning::Instruction::setDescription(const std::string& description)
{
  instruction_->setDescription(description);
}

void tesseract_planning::Instruction::print(const std::string& prefix) const { instruction_->print(prefix); }

bool tesseract_planning::Instruction::operator==(const Instruction& rhs) const
{
  return instruction_->operator==(*rhs.instruction_);
}

bool tesseract_planning::Instruction::operator!=(const Instruction& rhs) const { return !operator==(rhs); }

template <class Archive>
void tesseract_planning::Instruction::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("instruction", instruction_);
}

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::Instruction);
